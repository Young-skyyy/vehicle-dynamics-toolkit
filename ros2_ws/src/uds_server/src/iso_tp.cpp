/**
 * @file iso_tp.cpp
 * @brief ISO 15765-2 (ISO-TP) implementation — frame construction, parsing,
 *        payload segmentation, and receiver state machine.
 */

#include "iso_tp.hpp"
#include <algorithm>
#include <stdexcept>

namespace iso_tp {

// ═══════════════════════════════════════════════
// 帧构造
// ═══════════════════════════════════════════════

std::vector<uint8_t> build_sf(const std::vector<uint8_t>& payload) {
    auto n = static_cast<int>(payload.size());
    if (n == 0) {
        throw std::invalid_argument("SF payload must not be empty");
    }
    if (n > SF_PAYLOAD_MAX) {
        throw std::invalid_argument("SF payload too long: " +
                                    std::to_string(n));
    }
    std::vector<uint8_t> frame;
    frame.push_back(static_cast<uint8_t>(n));  // 0x0N
    frame.insert(frame.end(), payload.begin(), payload.end());
    return frame;
}

std::vector<uint8_t> build_ff(const std::vector<uint8_t>& payload,
                               int total_length) {
    if (total_length <= 0) {
        throw std::invalid_argument("Total length must be positive");
    }
    if (total_length > ISO_TP_MAX_LENGTH) {
        throw std::invalid_argument("Total length too large: " +
                                    std::to_string(total_length));
    }
    std::vector<uint8_t> frame;
    uint8_t high = (total_length >> 8) & 0x0F;
    uint8_t low  = total_length & 0xFF;
    frame.push_back(0x10 | high);
    frame.push_back(low);

    int copy_len = std::min(FF_PAYLOAD_MAX,
                            static_cast<int>(payload.size()));
    frame.insert(frame.end(), payload.begin(), payload.begin() + copy_len);
    return frame;
}

std::vector<uint8_t> build_cf(const std::vector<uint8_t>& payload,
                               int sequence) {
    std::vector<uint8_t> frame;
    frame.push_back(0x20 | (sequence & 0x0F));
    int copy_len = std::min(CF_PAYLOAD_MAX,
                            static_cast<int>(payload.size()));
    frame.insert(frame.end(), payload.begin(), payload.begin() + copy_len);
    return frame;
}

std::vector<uint8_t> build_fc(FlowControlFlag flag,
                               uint8_t block_size,
                               uint8_t st_min_ms) {
    return {static_cast<uint8_t>(0x30 | static_cast<uint8_t>(flag)),
            block_size, st_min_ms};
}

// ═══════════════════════════════════════════════
// 帧解析
// ═══════════════════════════════════════════════

ParsedFrame parse_frame(const std::vector<uint8_t>& data) {
    if (data.empty()) {
        throw std::invalid_argument("ISO-TP frame requires at least 1 byte");
    }

    uint8_t pci = data[0];
    auto type = static_cast<FrameType>((pci >> 4) & 0x0F);

    ParsedFrame result;
    result.type = type;

    switch (type) {
    case FrameType::SINGLE: {
        int length = pci & 0x0F;
        if (length == 0 || length > SF_PAYLOAD_MAX) {
            throw std::invalid_argument("Invalid SF length: " +
                                        std::to_string(length));
        }
        result.payload.assign(data.begin() + 1,
                              data.begin() + 1 + length);
        break;
    }
    case FrameType::FIRST: {
        if (data.size() < 2) {
            throw std::invalid_argument("FF requires at least 2 bytes");
        }
        result.total_length = ((pci & 0x0F) << 8) | data[1];
        if (result.total_length == 0 ||
            result.total_length > ISO_TP_MAX_LENGTH) {
            throw std::invalid_argument("Invalid FF total length: " +
                                        std::to_string(result.total_length));
        }
        int payload_end = std::min(static_cast<int>(data.size()),
                                   2 + FF_PAYLOAD_MAX);
        result.payload.assign(data.begin() + 2, data.begin() + payload_end);
        break;
    }
    case FrameType::CONSECUTIVE: {
        result.sequence_number = pci & 0x0F;
        int payload_end = std::min(static_cast<int>(data.size()),
                                   1 + CF_PAYLOAD_MAX);
        result.payload.assign(data.begin() + 1, data.begin() + payload_end);
        break;
    }
    case FrameType::FLOW_CONTROL: {
        if (data.size() < 3) {
            throw std::invalid_argument("FC requires at least 3 bytes");
        }
        result.fc_flag   = static_cast<FlowControlFlag>(pci & 0x0F);
        result.block_size = data[1];
        result.st_min_ms  = data[2];
        result.payload    = data;  // raw copy
        break;
    }
    }
    return result;
}

// ═══════════════════════════════════════════════
// 分段
// ═══════════════════════════════════════════════

std::vector<std::vector<uint8_t>> segment_payload(
    const std::vector<uint8_t>& data) {

    std::vector<std::vector<uint8_t>> frames;
    int total = static_cast<int>(data.size());

    if (total <= SF_PAYLOAD_MAX) {
        frames.push_back(build_sf(data));
        return frames;
    }

    // First Frame: first 6 bytes
    std::vector<uint8_t> ff_payload(data.begin(),
                                     data.begin() + FF_PAYLOAD_MAX);
    frames.push_back(build_ff(ff_payload, total));

    // Consecutive Frames: 7 bytes each
    int consumed = FF_PAYLOAD_MAX;
    int seq = 1;
    while (consumed < total) {
        int chunk_len = std::min(CF_PAYLOAD_MAX, total - consumed);
        std::vector<uint8_t> chunk(data.begin() + consumed,
                                    data.begin() + consumed + chunk_len);
        frames.push_back(build_cf(chunk, seq));
        seq = (seq + 1) & 0x0F;
        consumed += chunk_len;
    }

    return frames;
}

// ═══════════════════════════════════════════════
// 接收端状态机
// ═══════════════════════════════════════════════

IsoTPReceiver::FeedResult IsoTPReceiver::feed(
    const std::vector<uint8_t>& raw_frame) {

    FeedResult result;
    ParsedFrame frame = parse_frame(raw_frame);

    if (frame.type == FrameType::SINGLE) {
        complete_ = true;
        result.assembled = frame.payload;
        result.complete  = true;
        return result;
    }

    if (frame.type == FrameType::FIRST) {
        // Start new multi-frame reception
        buffer_          = frame.payload;
        expected_length_ = frame.total_length;
        received_length_ = static_cast<int>(frame.payload.size());
        expected_seq_    = 1;
        complete_        = false;

        result.flow_control = build_fc(FlowControlFlag::CTS);
        return result;
    }

    if (frame.type == FrameType::CONSECUTIVE) {
        if (frame.sequence_number != expected_seq_) {
            result.flow_control = build_fc(FlowControlFlag::OVERFLOW);
            complete_ = true;
            return result;
        }

        buffer_.insert(buffer_.end(),
                       frame.payload.begin(), frame.payload.end());
        received_length_ += static_cast<int>(frame.payload.size());
        expected_seq_ = (expected_seq_ + 1) & 0x0F;

        if (received_length_ >= expected_length_) {
            complete_ = true;
            result.assembled.assign(buffer_.begin(),
                                     buffer_.begin() + expected_length_);
            result.complete = true;
            return result;
        }

        // Still waiting — reply CTS
        result.flow_control = build_fc(FlowControlFlag::CTS);
        return result;
    }

    // Flow Control: should not be received by this side
    return result;
}

void IsoTPReceiver::reset() {
    buffer_.clear();
    expected_length_ = 0;
    received_length_ = 0;
    expected_seq_    = 0;
    complete_        = false;
}

}  // namespace iso_tp
