#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

namespace hmi::device
{
struct TelemetryRecord
{
    std::string text;
    std::string json; // Original validated JSON, preserving integer spelling.
    std::string event;
    bool duplicate = false; // Still returned so query acknowledgements are seen.
    bool recovered_prefix = false; // Preserve a damaged physical prefix in text even for a repeated JSON payload.
};

struct DecoderStatistics
{
    std::uint64_t bytes_received = 0;
    std::uint64_t lines_received = 0;
    std::uint64_t telemetry_records = 0; // Validated payloads, including repeats.
    std::uint64_t duplicate_records = 0;
    std::uint64_t malformed_records = 0;
    std::uint64_t oversized_lines = 0;
    std::uint64_t partial_lines = 0;
};

// Single-worker stream decoder. Text lines are limited to 8192 bytes. Only
// exact, identified telemetry payloads are deduplicated; warnings and errors
// remain individual records. Both entry count and cached bytes are bounded.
class TelemetryDecoder
{
public:
    std::vector<TelemetryRecord> feed(std::string_view bytes);
    // End one connection. Discard an unfinished line, retaining seen payloads
    // so a reconnect does not write the same cached snapshot again.
    void finish();
    // A storage failure invalidates delivery assumptions, but not the stream
    // framing or cumulative diagnostics. Retransmitted payloads may be saved.
    void clear_duplicates() noexcept;
    DecoderStatistics statistics() const noexcept;

private:
    void complete_line(std::vector<TelemetryRecord> &out);
    bool seen_payload(const std::string &payload);

    std::string pending_;
    bool discarding_ = false;
    DecoderStatistics statistics_;
    std::deque<std::string> recent_;
    std::unordered_set<std::string> seen_;
    std::size_t cached_bytes_ = 0;
};
}
