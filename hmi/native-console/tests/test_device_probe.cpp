#include "check.hpp"
#include "../tools/device_telemetry_validation.hpp"

namespace
{
hmi::device::TelemetryStatistics successful_probe()
{
    hmi::device::TelemetryStatistics statistics;
    statistics.connections = 2;
    statistics.bytes_received = 4096;
    statistics.records_saved = 100;
    statistics.telemetry_records_saved = 90;
    statistics.devices_with_telemetry = 2;
    statistics.scepter_telemetry_records = 60;
    statistics.dongle_telemetry_records = 30;
    statistics.snapshots_completed = 4;
    statistics.dongle_snapshots_completed = 1;
    return statistics;
}
}

HMI_TEST(device_probe_requires_healthy_worker_before_stop)
{
    auto statistics = successful_probe();
    CHECK(!hmi::device::validate_probe(statistics, false, 2).ok);
    statistics.worker_failures = 1;
    CHECK(!hmi::device::validate_probe(statistics, true, 2).ok);
}

HMI_TEST(device_probe_connection_acknowledgements_are_not_telemetry)
{
    hmi::device::TelemetryStatistics statistics;
    statistics.connections = 2;
    statistics.connected_devices = 2;
    statistics.records_saved = 4;
    statistics.bytes_received = 1024;
    CHECK(!hmi::device::validate_probe(statistics, true, 2).ok);
}

HMI_TEST(device_probe_rejects_queries_storage_and_snapshot_failures)
{
    auto statistics = successful_probe();
    statistics.query_failures = 10;
    CHECK(!hmi::device::validate_probe(statistics, true, 2).ok);
    statistics = successful_probe();
    statistics.storage_failures = 1;
    CHECK(!hmi::device::validate_probe(statistics, true, 2).ok);
    statistics = successful_probe();
    statistics.snapshot_failures = 1;
    CHECK(!hmi::device::validate_probe(statistics, true, 2).ok);
}

HMI_TEST(device_probe_requires_complete_multipart_records)
{
    auto statistics = successful_probe();
    statistics.snapshots_completed = 0;
    CHECK(!hmi::device::validate_probe(statistics, true, 1).ok);
    statistics = successful_probe();
    statistics.dongle_snapshots_completed = 0;
    CHECK(!hmi::device::validate_probe(statistics, true, 2).ok);
}

HMI_TEST(device_probe_requires_telemetry_from_each_device_and_role)
{
    auto statistics = successful_probe();
    statistics.devices_with_telemetry = 1;
    CHECK(!hmi::device::validate_probe(statistics, true, 2).ok);
    statistics = successful_probe();
    statistics.telemetry_records_saved = 0;
    statistics.duplicate_records = 100;
    CHECK(!hmi::device::validate_probe(statistics, true, 2).ok);
    statistics = successful_probe();
    statistics.scepter_telemetry_records = 0;
    CHECK(!hmi::device::validate_probe(statistics, true, 2).ok);
    statistics = successful_probe();
    statistics.dongle_telemetry_records = 0;
    CHECK(!hmi::device::validate_probe(statistics, true, 2).ok);
}

HMI_TEST(device_probe_accepts_complete_healthy_capture_after_stop)
{
    const auto statistics = successful_probe();
    CHECK(!statistics.running);
    const auto result = hmi::device::validate_probe(statistics, true, 2);
    CHECK(result.ok);
    CHECK(result.reason.empty());
}

HMI_TEST(device_probe_successful_mixed_snapshot_retry_is_not_failure)
{
    auto statistics = successful_probe();
    statistics.mixed_snapshots = 3;
    statistics.snapshot_retries = 3;
    CHECK(hmi::device::validate_probe(statistics, true, 2).ok);
}

HMI_TEST(device_probe_no_device_smoke_check_still_requires_health)
{
    hmi::device::TelemetryStatistics statistics;
    CHECK(hmi::device::validate_probe(statistics, true, 0).ok);
    CHECK(!hmi::device::validate_probe(statistics, false, 0).ok);
    statistics.query_failures = 1;
    CHECK(!hmi::device::validate_probe(statistics, true, 0).ok);
}
