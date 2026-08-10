/**
 * @file iso_tp.hpp
 * @brief ISO 15765-2 (ISO-TP) multi-frame transport protocol — C++ header
 *
 * Splits UDS payloads >7 bytes into CAN frame sequences and reassembles
 * them on the receiving end. Required for DIDs like VIN (0xF190, 17 bytes)
 * that exceed a single CAN frame.
 *
 * Frame types (N_PCI byte high nibble):
 *   Single Frame (SF):      0x0 + payload_length (4 bits)
 *   First Frame (FF):       0x1 + total_length (12 bits: 4+8)
 *   Consecutive Frame (CF): 0x2 + sequence_number (4 bits)
 *   Flow Control (FC):      0x3 + FC_flag (CTS=0 / WAIT=1 / Overflow=2)
 */

#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace iso_tp {

// ═══════════════════════════════════════════════
// 协议常量
// ═══════════════════════════════════════════════

constexpr int CAN_FRAME_DATA_LEN = 8;
constexpr int SF_PAYLOAD_MAX     = 7;   // Single Frame: 1B PCI + 7B data
constexpr int FF_PAYLOAD_MAX     = 6;   // First Frame: 2B PCI + 6B data
constexpr int CF_PAYLOAD_MAX     = 7;   // Consecutive Frame: 1B PCI + 7B data
constexpr int ISO_TP_MAX_LENGTH  = 4095;

/// N_PCI frame type (high nibble of first byte)
enum class FrameType : uint8_t {
    SINGLE       = 0x0,
    FIRST        = 0x1,
    CONSECUTIVE  = 0x2,
    FLOW_CONTROL = 0x3,
};

/// Flow Control flag (low nibble of FC frame's first byte)
enum class FlowControlFlag : uint8_t {
    CTS      = 0,  // Continue To Send
    WAIT     = 1,  // Wait
    OVERFLOW = 2,  // Overflow
};

// ═══════════════════════════════════════════════
// 帧构造 (encoding)
// ═══════════════════════════════════════════════

/// [0x0N, data...] — N = payload.size()
std::vector<uint8_t> build_sf(const std::vector<uint8_t>& payload);

/// [0x1Y, ZZ, data...] — YZZ = total_length (12-bit)
std::vector<uint8_t> build_ff(const std::vector<uint8_t>& payload,
                               int total_length);

/// [0x2X, data...] — X = sequence_number (0–15)
std::vector<uint8_t> build_cf(const std::vector<uint8_t>& payload,
                               int sequence);

/// [0x3X, block_size, st_min_ms]
std::vector<uint8_t> build_fc(FlowControlFlag flag,
                               uint8_t block_size = 0,
                               uint8_t st_min_ms = 0);

// ═══════════════════════════════════════════════
// 帧解析
// ═══════════════════════════════════════════════

struct ParsedFrame {
    FrameType type;
    std::vector<uint8_t> payload;
    int  total_length    = 0;       // FF only
    int  sequence_number = 0;       // CF only
    FlowControlFlag fc_flag = FlowControlFlag::CTS;  // FC only
    uint8_t block_size = 0;
    uint8_t st_min_ms  = 0;
};

ParsedFrame parse_frame(const std::vector<uint8_t>& data);

// ═══════════════════════════════════════════════
// 分段 (segmentation)
// ═══════════════════════════════════════════════

/// Split a full payload into ISO-TP frame sequence.
/// ≤7 bytes → Single Frame; >7 → First Frame + Consecutive Frames.
std::vector<std::vector<uint8_t>> segment_payload(
    const std::vector<uint8_t>& data);

// ═══════════════════════════════════════════════
// 接收端状态机
// ═══════════════════════════════════════════════

class IsoTPReceiver {
public:
    IsoTPReceiver() = default;

    /// Feed one CAN frame payload (raw 8 bytes).
    /// Returns (assembled_payload, flow_control_frame).
    /// - assembled_payload: complete data when done, empty otherwise
    /// - flow_control_frame: FC frame to send back (CTS/WAIT/Overflow)
    struct FeedResult {
        std::vector<uint8_t> assembled;
        std::vector<uint8_t> flow_control;
        bool complete = false;
    };

    FeedResult feed(const std::vector<uint8_t>& raw_frame);

    /// Reset internal state for a new transfer.
    void reset();

    bool is_complete() const { return complete_; }

private:
    std::vector<uint8_t> buffer_;
    int expected_length_  = 0;
    int received_length_  = 0;
    int expected_seq_     = 0;
    bool complete_        = false;
};

// ═══════════════════════════════════════════════
// 实用函数 — VIN
// ═══════════════════════════════════════════════

/// Encode a 17-character VIN string to bytes (ASCII).
inline std::vector<uint8_t> encode_vin(const std::string& vin) {
    if (vin.size() != 17) {
        throw std::invalid_argument("VIN must be 17 characters, got " +
                                    std::to_string(vin.size()));
    }
    return std::vector<uint8_t>(vin.begin(), vin.end());
}

/// Decode bytes back to VIN string.
inline std::string decode_vin(const std::vector<uint8_t>& data) {
    return std::string(data.begin(), data.end());
}

}  // namespace iso_tp
