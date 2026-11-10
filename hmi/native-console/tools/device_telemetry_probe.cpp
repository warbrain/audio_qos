#include "device/telemetry.hpp"
#include "base/logger.hpp"
#include "base/project_paths.hpp"
#include "device_telemetry_validation.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <thread>

int wmain(int argc, wchar_t **argv)
{
    try
    {
        int seconds = 25;
        int required = 0;
        const auto base = hmi::base::process_directory();
        std::filesystem::path directory = base / L"logs/devices";
        for (int index = 1; index < argc; ++index)
        {
            const std::wstring option = argv[index];
            if (++index >= argc) throw std::runtime_error("missing argument");
            if (option == L"--output") directory = argv[index];
            else if (option == L"--seconds" || option == L"--require")
            {
                std::size_t used = 0;
                const std::wstring value = argv[index];
                const int number = std::stoi(value, &used);
                if (used != value.size()) throw std::runtime_error("invalid numeric argument");
                if (option == L"--seconds") seconds = number;
                else required = number;
            }
            else throw std::runtime_error("unknown argument");
        }
        if (seconds < 1 || seconds > 300 || required < 0 || required > 16)
            throw std::runtime_error("argument outside allowed range");
        std::string error;
        std::filesystem::path local;
        if (!hmi::base::resolve_project_path(directory, base, local, &error)) throw std::runtime_error(error);
        hmi::base::Logger logger;
        hmi::base::LoggerOptions logging;
        logging.file = local.parent_path() / L"device-telemetry-probe.log";
        if (!logger.configure(logging, &error)) throw std::runtime_error(error);
        hmi::device::Telemetry collector(&logger);
        hmi::device::TelemetryOptions options;
        options.directory = local;
        if (!collector.start(options, &error)) throw std::runtime_error(error);
        const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
        std::size_t peak_devices = 0;
        while (std::chrono::steady_clock::now() < end)
        {
            peak_devices = (std::max)(peak_devices, collector.statistics().connected_devices);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        const bool healthy_before_stop = collector.statistics().running;
        collector.stop();
        const auto measured = collector.statistics();
        logger.flush();
        std::printf("devices=%zu connections=%llu bytes=%llu saved=%llu duplicates=%llu rejected=%llu query_failures=%llu storage_failures=%llu "
                    "healthy_before_stop=%d worker_failures=%llu telemetry_records_saved=%llu devices_with_telemetry=%zu "
                    "scepter_telemetry_records=%llu dongle_telemetry_records=%llu snapshots_completed=%llu "
                    "dongle_snapshots_completed=%llu mixed_snapshots=%llu snapshot_retries=%llu snapshot_failures=%llu\n",
            peak_devices, static_cast<unsigned long long>(measured.connections),
            static_cast<unsigned long long>(measured.bytes_received), static_cast<unsigned long long>(measured.records_saved),
            static_cast<unsigned long long>(measured.duplicate_records), static_cast<unsigned long long>(measured.rejected_lines),
            static_cast<unsigned long long>(measured.query_failures), static_cast<unsigned long long>(measured.storage_failures),
            healthy_before_stop ? 1 : 0, static_cast<unsigned long long>(measured.worker_failures),
            static_cast<unsigned long long>(measured.telemetry_records_saved), measured.devices_with_telemetry,
            static_cast<unsigned long long>(measured.scepter_telemetry_records),
            static_cast<unsigned long long>(measured.dongle_telemetry_records),
            static_cast<unsigned long long>(measured.snapshots_completed),
            static_cast<unsigned long long>(measured.dongle_snapshots_completed),
            static_cast<unsigned long long>(measured.mixed_snapshots),
            static_cast<unsigned long long>(measured.snapshot_retries),
            static_cast<unsigned long long>(measured.snapshot_failures));
        const auto validation = hmi::device::validate_probe(measured, healthy_before_stop,
                                                           static_cast<std::size_t>(required));
        if (!validation.ok) std::fprintf(stderr, "validation_failed: %s\n", validation.reason.c_str());
        return validation.ok ? 0 : 1;
    }
    catch (const std::exception &failure)
    {
        std::fprintf(stderr, "%s\n", failure.what());
        return 2;
    }
}
