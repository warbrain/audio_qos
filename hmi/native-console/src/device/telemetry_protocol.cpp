#include "device/telemetry_protocol.hpp"
#include "util/json_reader.hpp"
#include "util/json_writer.hpp"

#include <charconv>
#include <cmath>
#include <limits>
#include <utility>

namespace hmi::device
{
namespace
{
using Value = util::json::Value;

bool unsigned_value(const Value *value, std::uint32_t &out) noexcept
{
    const auto *number = value ? std::get_if<double>(&value->data) : nullptr;
    if (!number || !std::isfinite(*number) || *number < 0.0 ||
        *number > static_cast<double>(std::numeric_limits<std::uint32_t>::max()) ||
        std::floor(*number) != *number) return false;
    out = static_cast<std::uint32_t>(*number);
    return true;
}

bool exact_unsigned(const Value *value, std::uint32_t expected) noexcept
{
    std::uint32_t number = 0;
    return unsigned_value(value, number) && number == expected;
}

bool exact_text(const Value *value, std::string_view expected) noexcept
{
    const auto *text = value ? std::get_if<std::string>(&value->data) : nullptr;
    return text && *text == expected;
}

bool boot_value(const Value *value) noexcept
{
    const auto *text = value ? std::get_if<std::string>(&value->data) : nullptr;
    if (!text || text->size() != 8) return false;
    for (const char ch : *text)
        if (!((ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f') ||
              (ch >= 'A' && ch <= 'F'))) return false;
    return true;
}

bool timestamp_value(const Value *value) noexcept
{
    const auto *text = value ? std::get_if<std::string>(&value->data) : nullptr;
    if (!text || text->empty() || text->size() > 20) return false;
    for (const char ch : *text) if (ch < '0' || ch > '9') return false;
    std::uint64_t number = 0;
    const auto converted = std::from_chars(text->data(), text->data() + text->size(), number);
    return converted.ec == std::errc{} && converted.ptr == text->data() + text->size();
}

bool unsigned_identity(const Value &payload, std::string_view name, Value::Object &identity)
{
    const auto *value = payload.find(name);
    std::uint32_t ignored = 0;
    if (!unsigned_value(value, ignored)) return false;
    identity.emplace(std::string(name), *value);
    return true;
}

bool identity_for(std::string_view family, unsigned expected_parts, unsigned part,
                  const Value &payload, std::string &out)
{
    Value::Object identity;
    if (family == "TELEMETRY")
    {
        if (expected_parts != 10 || !exact_text(payload.find("event"), "audio.device_resources") ||
            !exact_unsigned(payload.find("schema"), 1) || !boot_value(payload.find("boot_id")) ||
            !exact_unsigned(payload.find("part"), part)) return false;
        std::uint32_t role = 0;
        if (!unsigned_value(payload.find("role"), role) || role > 1 ||
            !unsigned_identity(payload, "role", identity) ||
            !unsigned_identity(payload, "seq", identity)) return false;
        if (const auto *parts = payload.find("parts"); parts && !exact_unsigned(parts, expected_parts)) return false;
        for (const char *name : {"event", "schema", "boot_id"}) identity.emplace(name, *payload.find(name));
    }
    else if (family == "SCEPTER_TELEMETRY")
    {
        const auto *event_value = payload.find("event");
        const auto *event_text = event_value ? std::get_if<std::string>(&event_value->data) : nullptr;
        unsigned event_parts = 0;
        if (event_text)
        {
            if (*event_text == "audio.pipeline") event_parts = 32;
            else if (*event_text == "audio.capture_health") event_parts = 18;
            else if (*event_text == "audio.owner_timing") event_parts = 17;
            else if (*event_text == "audio.dispatch") event_parts = 17;
            else if (*event_text == "audio.mvdr") event_parts = 8;
        }
        if (!event_parts || expected_parts != event_parts ||
            !exact_unsigned(payload.find("schema"), 1) || !boot_value(payload.find("boot_id")) ||
            !exact_unsigned(payload.find("part"), part)) return false;
        std::uint32_t role = 0, mode = 0;
        if (!unsigned_value(payload.find("role"), role) || role != 0 ||
            !unsigned_value(payload.find("m"), mode) || mode > 6 ||
            !unsigned_identity(payload, "role", identity) ||
            !unsigned_identity(payload, "session_id", identity) ||
            !unsigned_identity(payload, "seq", identity) ||
            !unsigned_identity(payload, "m", identity)) return false;
        const auto *codec = payload.find("c");
        if (!codec) return false;
        if (const auto *number = std::get_if<double>(&codec->data))
        {
            if (!std::isfinite(*number) || std::floor(*number) != *number ||
                (*number != 3.0 && *number != 4.0 && *number != 5.0)) return false;
        }
        else if (!std::holds_alternative<std::nullptr_t>(codec->data)) return false;
        identity.emplace("c", *codec);
        if (const auto *parts = payload.find("part_count");
            parts && !exact_unsigned(parts, expected_parts)) return false;
        for (const char *name : {"event", "schema", "boot_id"})
            identity.emplace(name, *payload.find(name));
    }
    else if (family == "DONGLE_TELEMETRY_V2")
    {
        if (expected_parts != 14 || !exact_text(payload.find("e"), "dongle.diag") ||
            !exact_unsigned(payload.find("v"), 2) || !boot_value(payload.find("b")) ||
            !exact_unsigned(payload.find("r"), 1) || !exact_unsigned(payload.find("p"), part) ||
            !timestamp_value(payload.find("t"))) return false;
        if (const auto *parts = payload.find("n"); parts && !exact_unsigned(parts, expected_parts)) return false;
        for (const char *name : {"r", "q", "run", "arm", "cx", "s"})
            if (!unsigned_identity(payload, name, identity)) return false;
        for (const char *name : {"e", "v", "b", "t"}) identity.emplace(name, *payload.find(name));
    }
    else return false;
    out = util::json::stringify(Value(std::move(identity)));
    return true;
}
}

SnapshotStatus SnapshotValidator::observe(std::string_view family, std::uint32_t token,
                                           unsigned expected_parts, unsigned part,
                                           std::string_view payload)
{
    Value parsed;
    std::string identity;
    if (!token || !expected_parts || part >= expected_parts || payload.size() > 8192 ||
        !util::json::parse(payload, parsed) || !std::holds_alternative<Value::Object>(parsed.data) ||
        !identity_for(family, expected_parts, part, parsed, identity))
    {
        reset();
        return SnapshotStatus::Invalid;
    }

    if (part == 0 && token != token_)
    {
        family_.assign(family);
        identity_ = std::move(identity);
        token_ = token;
        expected_parts_ = expected_parts;
        next_part_ = 1;
        return SnapshotStatus::Incomplete;
    }
    if (!token_ || token != token_ || family != family_ || expected_parts != expected_parts_ ||
        part != next_part_ || identity != identity_)
    {
        reset();
        return SnapshotStatus::Mismatch;
    }
    ++next_part_;
    return next_part_ == expected_parts_ ? SnapshotStatus::Complete : SnapshotStatus::Incomplete;
}

void SnapshotValidator::reset() noexcept
{
    family_.clear();
    identity_.clear();
    token_ = 0;
    expected_parts_ = 0;
    next_part_ = 0;
}
}
