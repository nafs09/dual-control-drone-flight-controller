#include "ubx_parser.hpp"

namespace FC { namespace Communication {
namespace {
uint16_t readU16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8);
}
uint32_t readU32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}
int32_t readI32(const uint8_t* p) { return static_cast<int32_t>(readU32(p)); }
}

void UBXParser::reset() {
    state_ = State::Sync1;
    payloadLength_ = payloadIndex_ = 0;
    ckA_ = ckB_ = receivedCkA_ = 0;
}

void UBXParser::checksum(uint8_t byte) {
    ckA_ = static_cast<uint8_t>(ckA_ + byte);
    ckB_ = static_cast<uint8_t>(ckB_ + ckA_);
}

bool UBXParser::feed(uint8_t byte, NavPvt& message) {
    switch (state_) {
    case State::Sync1:
        if (byte == 0xB5u) state_ = State::Sync2;
        break;
    case State::Sync2:
        state_ = (byte == 0x62u) ? State::Class :
                 (byte == 0xB5u) ? State::Sync2 : State::Sync1;
        ckA_ = ckB_ = 0;
        break;
    case State::Class:
        messageClass_ = byte; checksum(byte); state_ = State::Id; break;
    case State::Id:
        messageId_ = byte; checksum(byte); state_ = State::LengthLow; break;
    case State::LengthLow:
        payloadLength_ = byte; checksum(byte); state_ = State::LengthHigh; break;
    case State::LengthHigh:
        payloadLength_ |= static_cast<uint16_t>(byte) << 8;
        checksum(byte);
        if (payloadLength_ > kMaxPayload) { reset(); break; }
        payloadIndex_ = 0;
        state_ = payloadLength_ == 0 ? State::ChecksumA : State::Payload;
        break;
    case State::Payload:
        payload_[payloadIndex_++] = byte;
        checksum(byte);
        if (payloadIndex_ == payloadLength_) state_ = State::ChecksumA;
        break;
    case State::ChecksumA:
        receivedCkA_ = byte;
        state_ = State::ChecksumB;
        break;
    case State::ChecksumB: {
        const bool checksumOk = receivedCkA_ == ckA_ && byte == ckB_;
        const bool isNavPvt = checksumOk && messageClass_ == 0x01u &&
                              messageId_ == 0x07u &&
                              payloadLength_ == kNavPvtLength;
        const bool decoded = isNavPvt && decodeNavPvt(message);
        reset();
        return decoded;
    }
    }
    return false;
}

bool UBXParser::decodeNavPvt(NavPvt& out) const {
    /* NAV-PVT layout: lon@24, lat@28, hMSL@36, hAcc@40, vAcc@44,
     * velN/E/D@48/52/56, sAcc@68, pDOP@76, fixType@20, flags@21, numSV@23. */
    out.fix_type = payload_[20];
    const uint8_t flags = payload_[21];
    out.satellites = payload_[23];
    out.longitude_deg = static_cast<double>(readI32(&payload_[24])) * 1.0e-7;
    out.latitude_deg = static_cast<double>(readI32(&payload_[28])) * 1.0e-7;
    out.altitude_msl_m = static_cast<float>(readI32(&payload_[36])) * 0.001f;
    out.horizontal_accuracy_m = static_cast<float>(readU32(&payload_[40])) * 0.001f;
    out.vertical_accuracy_m = static_cast<float>(readU32(&payload_[44])) * 0.001f;
    out.velocity_n_mps = static_cast<float>(readI32(&payload_[48])) * 0.001f;
    out.velocity_e_mps = static_cast<float>(readI32(&payload_[52])) * 0.001f;
    out.velocity_d_mps = static_cast<float>(readI32(&payload_[56])) * 0.001f;
    out.speed_accuracy_mps = static_cast<float>(readU32(&payload_[68])) * 0.001f;
    out.position_dop = static_cast<float>(readU16(&payload_[76])) * 0.01f;
    out.fix_valid = out.fix_type >= 3u && (flags & 0x01u) != 0u;
    return true;
}

}} // namespace FC::Communication
