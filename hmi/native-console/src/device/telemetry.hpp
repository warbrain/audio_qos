#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace hmi::base { class Logger; }

namespace hmi::device
{
struct TelemetryOptions
{
    std::filesystem::path directory;
    std::size_t max_bytes = 5 * 1024 * 1024;
    std::size_t max_files = 3;
    int poll_interval_seconds = 10;
};

struct TelemetryStatistics
{
    bool running = false;
    std::size_t connected_devices = 0;
    std::size_t devices_with_telemetry = 0;
    std::uint64_t connections = 0;
    std::uint64_t bytes_received = 0;
    std::uint64_t records_saved = 0; // Accepted journal records; flush failures are reported separately.
    std::uint64_t telemetry_records_saved = 0;
    std::uint64_t scepter_telemetry_records = 0;
    std::uint64_t dongle_telemetry_records = 0;
    std::uint64_t duplicate_records = 0;
    std::uint64_t rejected_lines = 0;
    std::uint64_t query_failures = 0;
    std::uint64_t storage_failures = 0;
    std::uint64_t worker_failures = 0;
    std::uint64_t partial_lines = 0;
    std::uint64_t snapshots_completed = 0;
    std::uint64_t dongle_snapshots_completed = 0;
    std::uint64_t mixed_snapshots = 0;
    std::uint64_t snapshot_retries = 0;
    std::uint64_t snapshot_failures = 0;
    std::string last_error;
};

// Owns serial connections and bounded journals on one background worker.
// No microphone, DSP settings, pairing or firmware update operations occur here.
class Telemetry
{
public:
    explicit Telemetry(base::Logger *logger = nullptr);
    ~Telemetry();
    Telemetry(const Telemetry &) = delete;
    Telemetry &operator=(const Telemetry &) = delete;

    bool start(const TelemetryOptions &options, std::string *error = nullptr);
    void stop() noexcept;
    TelemetryStatistics statistics() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
