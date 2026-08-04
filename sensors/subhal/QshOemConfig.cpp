/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#undef LOG_TAG
#define LOG_TAG "QshOemConfig"

#include "QshOemConfig.h"

#include <android-base/logging.h>
#include <android-base/stringprintf.h>

#include <chrono>
#include <condition_variable>
#include <cstring>
#include <functional>
#include <vector>

struct sensor_uid {
    uint64_t low;
    uint64_t high;
};

enum qsh_connection_type {
    QSH_CONNECTION_QMI = 0,
};

enum qsh_interface_error {
    QSH_INTERFACE_ERROR_NONE = 0,
};

struct qsh_conn_config {
    uint64_t opaque;
};

class qsh_interface {
  public:
    static qsh_interface* create(qsh_connection_type type, qsh_conn_config config);
};

class qsh_qmi {
  public:
    ~qsh_qmi();
    bool send_request(sensor_uid suid, bool sync, std::string payload);
    void register_cb(sensor_uid suid, std::function<void(unsigned int, unsigned long)> onResp,
                     std::function<void(qsh_interface_error)> onError,
                     std::function<void(const unsigned char*, unsigned long, unsigned long)> onEvent);
};

class suid_lookup {
  public:
    suid_lookup(std::function<void(const std::string&, const std::vector<sensor_uid>&)> cb);
    ~suid_lookup();
    void request_suid(std::string datatype, bool register_updates);

  private:
    char opaque_[256];
};

namespace {
constexpr uint32_t kMsgIdOemConfig = 0x800;
constexpr uint32_t kClientProcTypeApss = 1;
constexpr uint32_t kDeliveryTypeNoWakeup = 1;
constexpr char kDataType[] = "ambient_light";
constexpr auto kSuidTimeout = std::chrono::seconds(2);

void putVarint(std::string* out, uint64_t value) {
    do {
        uint8_t byte = value & 0x7f;
        value >>= 7;
        if (value) {
            byte |= 0x80;
        }
        out->push_back(static_cast<char>(byte));
    } while (value);
}

void putTag(std::string* out, uint32_t field, uint32_t wireType) {
    putVarint(out, (field << 3) | wireType);
}

void putVarintField(std::string* out, uint32_t field, uint64_t value) {
    putTag(out, field, 0 /* varint */);
    putVarint(out, value);
}

void putFixed64Field(std::string* out, uint32_t field, uint64_t value) {
    putTag(out, field, 1 /* 64-bit */);
    for (int i = 0; i < 8; i++) {
        out->push_back(static_cast<char>((value >> (i * 8)) & 0xff));
    }
}

void putFixed32Field(std::string* out, uint32_t field, uint32_t value) {
    putTag(out, field, 5 /* 32-bit */);
    for (int i = 0; i < 4; i++) {
        out->push_back(static_cast<char>((value >> (i * 8)) & 0xff));
    }
}

void putBytesField(std::string* out, uint32_t field, const std::string& value) {
    putTag(out, field, 2 /* length-delimited */);
    putVarint(out, value.size());
    out->append(value);
}

}  // namespace

QshOemConfig::QshOemConfig() = default;

QshOemConfig::~QshOemConfig() {
    delete reinterpret_cast<suid_lookup*>(mLookup);
}

QshOemConfig& QshOemConfig::getInstance() {
    static QshOemConfig instance;
    return instance;
}

bool QshOemConfig::ensureReady() {
    if (mReady) {
        return true;
    }

    if (!lookupSuid()) {
        return false;
    }

    if (mConnection == nullptr) {
        qsh_conn_config config = {};
        mConnection = qsh_interface::create(QSH_CONNECTION_QMI, config);
        if (mConnection == nullptr) {
            LOG(ERROR) << "failed to open QSH connection";
            return false;
        }
        sensor_uid uid = {mSuidLow, mSuidHigh};
        reinterpret_cast<qsh_qmi*>(mConnection)
                ->register_cb(
                        uid,
                        [](unsigned int resp, unsigned long ts) {
                            LOG(INFO) << "qsh resp=" << resp << " ts=" << ts;
                        },
                        [](qsh_interface_error error) {
                            LOG(ERROR) << "qsh error=" << static_cast<int>(error);
                        },
                        [](const unsigned char* data, unsigned long len, unsigned long ts) {
                            std::string hex;
                            for (unsigned long i = 0; i < len && i < 64; i++) {
                                android::base::StringAppendF(&hex, "%02x", data[i]);
                            }
                            LOG(INFO) << "qsh event len=" << len << " ts=" << ts << " " << hex;
                        });
    }

    mReady = true;
    return true;
}

bool QshOemConfig::lookupSuid() {
    std::mutex suidMutex;
    std::condition_variable suidCv;
    bool resolved = false;

    auto onSuid = [&](const std::string& datatype, const std::vector<sensor_uid>& suids) {
        if (datatype != kDataType || suids.empty()) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(suidMutex);
            mSuidLow = suids[0].low;
            mSuidHigh = suids[0].high;
            resolved = true;
        }
        suidCv.notify_one();
    };

    suid_lookup lookup(onSuid);
    lookup.request_suid(kDataType, false /* register_updates */);

    std::unique_lock<std::mutex> lock(suidMutex);
    if (!suidCv.wait_for(lock, kSuidTimeout, [&] { return resolved; })) {
        LOG(ERROR) << "timed out resolving SUID for " << kDataType;
        return false;
    }

    LOG(INFO) << "resolved " << kDataType << " suid low=" << mSuidLow << " high=" << mSuidHigh;
    return true;
}

bool QshOemConfig::send(int32_t type, const std::string& oemConfig) {
    std::string suid;
    putFixed64Field(&suid, 1, mSuidLow);
    putFixed64Field(&suid, 2, mSuidHigh);
    std::string suspendConfig;
    putVarintField(&suspendConfig, 1, kClientProcTypeApss);
    putVarintField(&suspendConfig, 2, kDeliveryTypeNoWakeup);
    std::string request;
    putBytesField(&request, 2, oemConfig);
    std::string encoded;
    putBytesField(&encoded, 1, suid);
    putFixed32Field(&encoded, 2, kMsgIdOemConfig);
    putBytesField(&encoded, 3, suspendConfig);
    putBytesField(&encoded, 4, request);
    sensor_uid uid = {mSuidLow, mSuidHigh};
    bool sent = reinterpret_cast<qsh_qmi*>(mConnection)->send_request(uid, true, encoded);
    if (!sent) {
        LOG(ERROR) << "failed to send config type " << type;
        mReady = false;
    }
    return sent;
}

static void putFloatField(std::string* out, uint32_t field, float value) {
    putTag(out, field, 5 /* 32-bit */);
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    for (int i = 0; i < 4; i++) {
        out->push_back(static_cast<char>((bits >> (i * 8)) & 0xff));
    }
}

bool QshOemConfig::reportValue(float value, float aux) {
    std::lock_guard<std::mutex> lock(mMutex);
    if (!ensureReady()) {
        return false;
    }

    std::string config;
    putVarintField(&config, 1, BOLED_DATA);
    putFloatField(&config, 11, value);
    putFloatField(&config, 12, aux);

    LOG(VERBOSE) << "report value=" << value << " aux=" << aux;
    return send(BOLED_DATA, config);
}

bool QshOemConfig::notifyBacklight(int32_t brightness) {
    std::lock_guard<std::mutex> lock(mMutex);
    if (!ensureReady()) {
        return false;
    }

    std::string config;
    putVarintField(&config, 1, BACKLIGHT);
    putVarintField(&config, 9, static_cast<uint64_t>(static_cast<int64_t>(brightness)));

    LOG(VERBOSE) << "backlight " << brightness;
    return send(BACKLIGHT, config);
}

bool QshOemConfig::notifyDcState(int32_t state) {
    std::lock_guard<std::mutex> lock(mMutex);
    if (!ensureReady()) {
        return false;
    }

    std::string config;
    putVarintField(&config, 1, DC_STATE);
    putVarintField(&config, 10, static_cast<uint64_t>(static_cast<int64_t>(state)));

    LOG(VERBOSE) << "dc state " << state;
    return send(DC_STATE, config);
}

bool QshOemConfig::notifyDisplayFreq(uint32_t freq) {
    std::lock_guard<std::mutex> lock(mMutex);
    if (!ensureReady()) {
        return false;
    }

    std::string config;
    putVarintField(&config, 1, DISPLAY_FREQ);
    putVarintField(&config, 15, freq);

    LOG(VERBOSE) << "display freq " << freq;
    return send(DISPLAY_FREQ, config);
}
