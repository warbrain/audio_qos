#include "check.hpp"
#include "device/telemetry_protocol.hpp"
#include "util/json_reader.hpp"
#include "util/json_writer.hpp"

#include <cstdint>
#include <string>
#include <string_view>
#include <utility>

namespace
{
using hmi::device::SnapshotStatus;
using hmi::device::SnapshotValidator;
using Value = hmi::util::json::Value;

std::string dongle(unsigned part, unsigned sequence = 2530)
{
    return "{\"e\":\"dongle.diag\",\"v\":2,\"p\":" + std::to_string(part) +
        ",\"n\":14,\"b\":\"2e4523d4\",\"r\":1,\"q\":" + std::to_string(sequence) +
        ",\"run\":0,\"arm\":0,\"cx\":0,\"s\":878848382,\"t\":\"1206683117\",\"counter\":7}";
}

std::string resource(unsigned part, unsigned sequence = 239)
{
    return "{\"event\":\"audio.device_resources\",\"schema\":1,\"boot_id\":\"2e4523d4\",\"role\":1,\"seq\":" +
        std::to_string(sequence) + ",\"part\":" + std::to_string(part) +
        (part == 0 ? ",\"parts\":10" : "") + ",\"value\":" + std::to_string(part * 10U) + "}";
}

std::string dispatch(unsigned part, unsigned sequence = 77)
{
    const std::string extra = part == 15 ? ",\"le\":2" :
        (part == 16 ? ",\"late_events\":3,\"la\":1" : "");
    return "{\"event\":\"audio.dispatch\",\"schema\":1,\"boot_id\":\"2e4523d4\",\"role\":0,\"session_id\":42,\"seq\":" +
        std::to_string(sequence) + ",\"part\":" + std::to_string(part) +
        ",\"c\":4,\"m\":0" + (part == 0 ? ",\"part_count\":17" : "") + extra + "}";
}

std::string changed(std::string_view json, std::string key, Value value)
{
    Value parsed;
    CHECK(hmi::util::json::parse(json, parsed));
    std::get<Value::Object>(parsed.data).insert_or_assign(std::move(key), std::move(value));
    return hmi::util::json::stringify(parsed);
}

std::string without(std::string_view json, const std::string &key)
{
    Value parsed;
    CHECK(hmi::util::json::parse(json, parsed));
    std::get<Value::Object>(parsed.data).erase(key);
    return hmi::util::json::stringify(parsed);
}
}

HMI_TEST(device_protocol_accepts_complete_ordered_dongle_snapshot)
{
    SnapshotValidator validator;
    for (unsigned part = 0; part < 14; ++part)
        CHECK(validator.observe("DONGLE_TELEMETRY_V2", 11, 14, part, dongle(part)) ==
              (part == 13 ? SnapshotStatus::Complete : SnapshotStatus::Incomplete));
    CHECK(validator.observe("DONGLE_TELEMETRY_V2", 11, 14, 13, dongle(13)) == SnapshotStatus::Mismatch);
}

HMI_TEST(device_protocol_rejects_real_mixed_dongle_snapshot)
{
    SnapshotValidator validator;
    for (unsigned part = 0; part < 5; ++part)
        CHECK(validator.observe("DONGLE_TELEMETRY_V2", 11, 14, part, dongle(part, 2530)) == SnapshotStatus::Incomplete);
    CHECK(validator.observe("DONGLE_TELEMETRY_V2", 11, 14, 5, dongle(5, 2531)) == SnapshotStatus::Mismatch);
    // The remaining parts cannot complete a group whose part zero was absent.
    CHECK(validator.observe("DONGLE_TELEMETRY_V2", 11, 14, 6, dongle(6, 2531)) == SnapshotStatus::Mismatch);
    for (unsigned part = 0; part < 14; ++part)
        CHECK(validator.observe("DONGLE_TELEMETRY_V2", 12, 14, part, dongle(part, 2531)) ==
              (part == 13 ? SnapshotStatus::Complete : SnapshotStatus::Incomplete));
}

HMI_TEST(device_protocol_checks_every_dongle_identity_field)
{
    const std::pair<std::string, Value> changes[] = {
        {"b", Value("aabbccdd")}, {"q", Value(2531.0)}, {"run", Value(1.0)},
        {"arm", Value(2.0)}, {"cx", Value(3.0)}, {"s", Value(4.0)}, {"t", Value("1206683118")}};
    for (const auto &[key, value] : changes)
    {
        SnapshotValidator validator;
        CHECK(validator.observe("DONGLE_TELEMETRY_V2", 1, 14, 0, dongle(0)) == SnapshotStatus::Incomplete);
        CHECK(validator.observe("DONGLE_TELEMETRY_V2", 1, 14, 1, changed(dongle(1), key, value)) == SnapshotStatus::Mismatch);
    }
    // Dongle schema 2 is role 1; role 0 is invalid instead of a new identity.
    SnapshotValidator validator;
    CHECK(validator.observe("DONGLE_TELEMETRY_V2", 1, 14, 0, dongle(0)) == SnapshotStatus::Incomplete);
    CHECK(validator.observe("DONGLE_TELEMETRY_V2", 1, 14, 1,
        changed(dongle(1), "r", Value(0.0))) == SnapshotStatus::Invalid);
}

HMI_TEST(device_protocol_checks_resource_window_boot_role_and_completion)
{
    SnapshotValidator validator;
    for (unsigned part = 0; part < 10; ++part)
        CHECK(validator.observe("TELEMETRY", 1, 10, part, resource(part)) ==
              (part == 9 ? SnapshotStatus::Complete : SnapshotStatus::Incomplete));
    CHECK(validator.observe("TELEMETRY", 2, 10, 0, resource(0)) == SnapshotStatus::Incomplete);
    CHECK(validator.observe("TELEMETRY", 2, 10, 1, resource(1, 240)) == SnapshotStatus::Mismatch);
    const std::pair<std::string, Value> changes[] = {{"boot_id", Value("aabbccdd")}, {"role", Value(0.0)}};
    for (const auto &[key, value] : changes)
    {
        validator.reset();
        CHECK(validator.observe("TELEMETRY", 3, 10, 0, resource(0)) == SnapshotStatus::Incomplete);
        CHECK(validator.observe("TELEMETRY", 3, 10, 1, changed(resource(1), key, value)) == SnapshotStatus::Mismatch);
    }
}

HMI_TEST(device_protocol_accepts_complete_ordered_scepter_dispatch_snapshot)
{
    SnapshotValidator validator;
    for (unsigned part = 0; part < 17; ++part)
        CHECK(validator.observe("SCEPTER_TELEMETRY", 41, 17, part, dispatch(part)) ==
              (part == 16 ? SnapshotStatus::Complete : SnapshotStatus::Incomplete));
    CHECK(validator.observe("SCEPTER_TELEMETRY", 41, 17, 16, dispatch(16)) == SnapshotStatus::Mismatch);
}

HMI_TEST(device_protocol_accepts_unknown_codec_dispatch_snapshot)
{
    SnapshotValidator validator;
    auto first = dispatch(0);
    first.replace(first.find("\"c\":4"), 5, "\"c\":null");
    auto second = dispatch(1);
    second.replace(second.find("\"c\":4"), 5, "\"c\":null");
    CHECK(validator.observe("SCEPTER_TELEMETRY", 42, 17, 0, first) == SnapshotStatus::Incomplete);
    CHECK(validator.observe("SCEPTER_TELEMETRY", 42, 17, 1, second) == SnapshotStatus::Incomplete);
}

HMI_TEST(device_protocol_rejects_mixed_scepter_dispatch_identity)
{
    const std::pair<std::string, Value> changes[] = {
        {"boot_id", Value("aabbccdd")},
        {"session_id", Value(43.0)}, {"seq", Value(78.0)},
        {"c", Value(5.0)}, {"m", Value(3.0)}};
    for (const auto &[key, value] : changes)
    {
        SnapshotValidator validator;
        CHECK(validator.observe("SCEPTER_TELEMETRY", 9, 17, 0, dispatch(0)) == SnapshotStatus::Incomplete);
        CHECK(validator.observe("SCEPTER_TELEMETRY", 9, 17, 1,
            changed(dispatch(1), key, value)) == SnapshotStatus::Mismatch);
    }
    SnapshotValidator validator;
    CHECK(validator.observe("SCEPTER_TELEMETRY", 10, 17, 0, dispatch(0)) == SnapshotStatus::Incomplete);
    CHECK(validator.observe("SCEPTER_TELEMETRY", 10, 17, 1,
        without(dispatch(1), "c")) == SnapshotStatus::Invalid);
    SnapshotValidator invalid_role;
    CHECK(invalid_role.observe("SCEPTER_TELEMETRY", 11, 17, 0, dispatch(0)) == SnapshotStatus::Incomplete);
    CHECK(invalid_role.observe("SCEPTER_TELEMETRY", 11, 17, 1,
        changed(dispatch(1), "role", Value(1.0))) == SnapshotStatus::Invalid);
}

HMI_TEST(device_protocol_requires_order_and_new_token_part_zero)
{
    SnapshotValidator validator;
    CHECK(validator.observe("TELEMETRY", 1, 10, 1, resource(1)) == SnapshotStatus::Mismatch);
    CHECK(validator.observe("TELEMETRY", 1, 10, 0, resource(0)) == SnapshotStatus::Incomplete);
    CHECK(validator.observe("TELEMETRY", 2, 10, 1, resource(1)) == SnapshotStatus::Mismatch);
    CHECK(validator.observe("TELEMETRY", 3, 10, 0, resource(0)) == SnapshotStatus::Incomplete);
    CHECK(validator.observe("TELEMETRY", 3, 10, 0, resource(0)) == SnapshotStatus::Mismatch);
    CHECK(validator.observe("TELEMETRY", 4, 10, 0, resource(0)) == SnapshotStatus::Incomplete);
    CHECK(validator.observe("TELEMETRY", 4, 10, 2, resource(2)) == SnapshotStatus::Mismatch);
    CHECK(validator.observe("TELEMETRY", 5, 10, 0, resource(0)) == SnapshotStatus::Incomplete);
    // A new token at part zero intentionally replaces an unfinished group.
    CHECK(validator.observe("TELEMETRY", 6, 10, 0, resource(0, 240)) == SnapshotStatus::Incomplete);
    for (unsigned part = 1; part < 10; ++part)
        CHECK(validator.observe("TELEMETRY", 6, 10, part, resource(part, 240)) ==
              (part == 9 ? SnapshotStatus::Complete : SnapshotStatus::Incomplete));
}

HMI_TEST(device_protocol_rejects_missing_or_invalid_identity_fields)
{
    const char *dongle_fields[] = {"e", "v", "b", "r", "q", "run", "arm", "cx", "s", "t", "p"};
    for (const char *field : dongle_fields)
    {
        SnapshotValidator validator;
        CHECK(validator.observe("DONGLE_TELEMETRY_V2", 1, 14, 0, without(dongle(0), field)) == SnapshotStatus::Invalid);
    }
    const char *resource_fields[] = {"event", "schema", "boot_id", "role", "seq", "part"};
    for (const char *field : resource_fields)
    {
        SnapshotValidator validator;
        CHECK(validator.observe("TELEMETRY", 1, 10, 0, without(resource(0), field)) == SnapshotStatus::Invalid);
    }
    const std::pair<std::string, Value> invalid[] = {
        {"v", Value(1.0)}, {"v", Value("2")}, {"q", Value(-1.0)}, {"q", Value(1.5)},
        {"q", Value(4294967296.0)}, {"run", Value(true)}, {"b", Value("short")},
        {"b", Value("zzzzzzzz")}, {"r", Value(2.0)}, {"p", Value(1.0)}, {"n", Value(13.0)},
        {"e", Value("other.diag")}, {"t", Value(123.0)}};
    for (const auto &[field, value] : invalid)
    {
        SnapshotValidator validator;
        CHECK(validator.observe("DONGLE_TELEMETRY_V2", 1, 14, 0, changed(dongle(0), field, value)) == SnapshotStatus::Invalid);
    }
}

HMI_TEST(device_protocol_preserves_full_u64_timestamp_identity)
{
    SnapshotValidator validator;
    for (unsigned part = 0; part < 14; ++part)
        CHECK(validator.observe("DONGLE_TELEMETRY_V2", 1, 14, part,
            changed(dongle(part), "t", Value("18446744073709551615"))) ==
              (part == 13 ? SnapshotStatus::Complete : SnapshotStatus::Incomplete));
    const char *invalid[] = {"", "-1", "+1", "1.5", "1e9", "18446744073709551616", "000000000000000000001"};
    for (const char *timestamp : invalid)
    {
        validator.reset();
        CHECK(validator.observe("DONGLE_TELEMETRY_V2", 2, 14, 0,
            changed(dongle(0), "t", Value(timestamp))) == SnapshotStatus::Invalid);
    }
}

HMI_TEST(device_protocol_rejects_bad_arguments_and_reset_abandons_partial_group)
{
    SnapshotValidator validator;
    CHECK(validator.observe("TELEMETRY", 0, 10, 0, resource(0)) == SnapshotStatus::Invalid);
    CHECK(validator.observe("TELEMETRY", 1, 9, 0, resource(0)) == SnapshotStatus::Invalid);
    CHECK(validator.observe("TELEMETRY", 1, 10, 10, resource(9)) == SnapshotStatus::Invalid);
    CHECK(validator.observe("OTHER", 1, 10, 0, resource(0)) == SnapshotStatus::Invalid);
    CHECK(validator.observe("TELEMETRY", 1, 10, 0, "[]") == SnapshotStatus::Invalid);
    CHECK(validator.observe("TELEMETRY", 1, 10, 0, "{broken") == SnapshotStatus::Invalid);
    CHECK(validator.observe("TELEMETRY", 1, 10, 0, resource(0)) == SnapshotStatus::Incomplete);
    validator.reset();
    CHECK(validator.observe("TELEMETRY", 1, 10, 1, resource(1)) == SnapshotStatus::Mismatch);
    CHECK(validator.observe("TELEMETRY", 1, 10, 0, resource(0)) == SnapshotStatus::Incomplete);
    CHECK(validator.observe("DONGLE_TELEMETRY_V2", 1, 14, 1, dongle(1)) == SnapshotStatus::Mismatch);
    CHECK(validator.observe("DONGLE_TELEMETRY_V2", 2, 14, 0, dongle(0)) == SnapshotStatus::Incomplete);
}
