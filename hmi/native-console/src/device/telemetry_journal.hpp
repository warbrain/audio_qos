#pragma once

#include "device/telemetry_record.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace hmi::device
{
struct JournalOptions
{
    std::filesystem::path directory;
    std::string stem;
    std::string device_id;
    std::string port;
    std::string host_session;
    std::size_t max_bytes = 5 * 1024 * 1024;
    std::size_t max_files = 3;
};

// One worker owns each journal. Text and JSONL rotate independently, preserve
// existing records on open, and are flushed periodically by the worker.
class TelemetryJournal
{
public:
    TelemetryJournal();
    ~TelemetryJournal();
    TelemetryJournal(const TelemetryJournal &) = delete;
    TelemetryJournal &operator=(const TelemetryJournal &) = delete;

    bool open(const JournalOptions &options, std::string *error = nullptr);
    bool write(const TelemetryRecord &record, std::int64_t received_at_ms,
               std::uint64_t connection, std::string *error = nullptr);
    bool flush(std::string *error = nullptr);
    void close() noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
