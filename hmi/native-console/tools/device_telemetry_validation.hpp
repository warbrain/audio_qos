#pragma once

#include "device/telemetry.hpp"

#include <cstddef>
#include <string>

namespace hmi::device
{
struct ProbeValidation
{
    bool ok = false;
    std::string reason;
};

// Evaluate final counters after stop(), using the worker health observed just
// before stop. Connection acknowledgements and text-only records are not proof
// that useful telemetry or complete multipart snapshots reached the journal.
inline ProbeValidation validate_probe(const TelemetryStatistics &statistics,
                                      bool healthy_before_stop,
                                      std::size_t required_devices)
{
    if (!healthy_before_stop || statistics.worker_failures != 0)
        return {false, "device collector was not healthy before shutdown"};
    if (statistics.query_failures != 0)
        return {false, "device queries failed"};
    if (statistics.storage_failures != 0)
        return {false, "device journals reported storage failures"};
    if (statistics.snapshot_failures != 0)
        return {false, "multipart snapshots could not be completed"};
    if (required_devices > 0)
    {
        if (statistics.devices_with_telemetry < required_devices ||
            statistics.telemetry_records_saved < required_devices)
            return {false, "required devices did not save valid telemetry"};
        if (statistics.snapshots_completed == 0)
            return {false, "no complete multipart snapshot was saved"};
    }
    if (required_devices >= 2)
    {
        if (statistics.scepter_telemetry_records == 0 || statistics.dongle_telemetry_records == 0)
            return {false, "both device roles must save telemetry"};
        if (statistics.dongle_snapshots_completed == 0)
            return {false, "no complete Dongle diagnostic snapshot was saved"};
    }
    // A mixed read that was retried successfully is not a final failure.
    return {true, {}};
}
}
