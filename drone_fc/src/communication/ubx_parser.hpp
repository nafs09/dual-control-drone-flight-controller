#pragma once

#include <cstdint>

namespace FC { namespace Communication {

/* Fixed-memory UBX parser for the 92-byte NAV-PVT message. */
class UBXParser {
public:
    struct NavPvt {
        double latitude_deg = 0.0;
        double longitude_deg = 0.0;
        float altitude_msl_m = 0.0f;
        float velocity_n_mps = 0.0f;
        float velocity_e_mps = 0.0f;
        float velocity_d_mps = 0.0f;
        float horizontal_accuracy_m = 0.0f;
        float vertical_accuracy_m = 0.0f;
        float speed_accuracy_mps = 0.0f;
        float position_dop = 0.0f;
        uint8_t satellites = 0;
        uint8_t fix_type = 0;
        bool fix_valid = false;
    };

    bool feed(uint8_t byte, NavPvt& message);
    void reset();

private:
    enum class State : uint8_t {
        Sync1, Sync2, Class, Id, LengthLow, LengthHigh,
        Payload, ChecksumA, ChecksumB
    };
    static constexpr uint16_t kMaxPayload = 128;
    static constexpr uint16_t kNavPvtLength = 92;

    State state_ = State::Sync1;
    uint8_t messageClass_ = 0, messageId_ = 0;
    uint8_t ckA_ = 0, ckB_ = 0;
    uint8_t receivedCkA_ = 0;
    uint16_t payloadLength_ = 0, payloadIndex_ = 0;
    uint8_t payload_[kMaxPayload] = {};

    void checksum(uint8_t byte);
    bool decodeNavPvt(NavPvt& message) const;
};

}} // namespace FC::Communication
