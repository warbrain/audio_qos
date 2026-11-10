#include "device/telemetry_record.hpp"
#include "util/json_reader.hpp"

#include <cmath>
#include <initializer_list>
#include <limits>
#include <utility>

namespace hmi::device
{
namespace
{
constexpr std::size_t maximum_line_bytes = 8192;
constexpr std::size_t maximum_cached_records = 4096;
constexpr std::size_t maximum_cached_bytes = 2 * 1024 * 1024;

bool whitespace(char ch) noexcept
{
    return ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n';
}

std::string_view trim(std::string_view value) noexcept
{
    while (!value.empty() && whitespace(value.front())) value.remove_prefix(1);
    while (!value.empty() && whitespace(value.back())) value.remove_suffix(1);
    return value;
}

std::string without_ansi(std::string_view value)
{
    std::string out;
    out.reserve(value.size());
    for (std::size_t i = 0; i < value.size();)
    {
        if (value[i] != '\x1b')
        {
            out.push_back(value[i++]);
            continue;
        }
        if (++i == value.size()) break;
        if (value[i] == '[')
        {
            ++i;
            while (i < value.size())
            {
                const auto ch = static_cast<unsigned char>(value[i++]);
                if (ch >= 0x40 && ch <= 0x7e) break;
            }
        }
        else if (value[i] == ']')
        {
            ++i;
            while (i < value.size())
            {
                if (value[i++] == '\a') break;
                if (value[i - 1] == '\x1b' && i < value.size() && value[i] == '\\')
                {
                    ++i;
                    break;
                }
            }
        }
        else ++i;
    }
    return out;
}

bool warning_or_error(std::string_view line) noexcept
{
    line = trim(line);
    if (line.size() > 1 && (line.front() == 'W' || line.front() == 'E' || line.front() == 'F') &&
        (whitespace(line[1]) || line[1] == '(')) return true;
    return line.starts_with("WARN") || line.starts_with("ERROR") || line.starts_with("ERR ") ||
        line.starts_with("FATAL") || line.starts_with("[warning]") || line.starts_with("[error]");
}

bool known_event(std::string_view event) noexcept
{
    return (event.starts_with("audio.") && event.size() > 6) || event == "dongle.diag";
}

const std::string *string_value(const util::json::Value *value) noexcept
{
    return value ? std::get_if<std::string>(&value->data) : nullptr;
}

const util::json::Value *field(const util::json::Value &value,
                               std::string_view primary, std::string_view alternate)
{
    const auto *selected = value.find(primary);
    return selected ? selected : value.find(alternate);
}

bool identified(const util::json::Value &value)
{
    const auto *boot = string_value(field(value, "boot_id", "b"));
    const auto *part = field(value, "part", "p");
    const auto *number = part ? std::get_if<double>(&part->data) : nullptr;
    return boot && !boot->empty() && boot->size() <= 256 && number &&
        std::isfinite(*number) && *number >= 0 && *number <= 65535 && std::floor(*number) == *number;
}

// This is used only after validation failed. Do not classify arbitrary JSON
// text mentioning audio as a broken firmware event.
bool telemetry_marker(std::string_view json) noexcept
{
    for (const auto key : {std::string_view("\"event\""), std::string_view("\"e\"")})
    {
        std::size_t position = 0;
        while ((position = json.find(key, position)) != std::string_view::npos)
        {
            position += key.size();
            auto rest = trim(json.substr(position));
            if (rest.empty() || rest.front() != ':') continue;
            rest.remove_prefix(1);
            rest = trim(rest);
            if (rest.starts_with("\"audio.") || rest.starts_with("\"dongle.diag\"")) return true;
        }
    }
    return false;
}

// A truncated CDC mirror can be followed by a command reply without a line
// boundary. Resynchronize only at a recognized reply with a valid token,
// never by trying arbitrary braces from device-controlled text.
std::size_t reply_payload_start(std::string_view line) noexcept
{
    std::size_t latest_reply = std::string_view::npos;
    std::size_t opening = std::string_view::npos;
    for (const auto prefix : {std::string_view("OK TELEMETRY token="),
                             std::string_view("OK PERF_SNAPSHOT token="),
                             std::string_view("OK DONGLE_TELEMETRY_V2 token="),
                             std::string_view("OK SCEPTER_TELEMETRY token=")})
    {
        const auto position = line.rfind(prefix);
        if (position == std::string_view::npos) continue;
        std::size_t cursor = position + prefix.size();
        const auto token_start = cursor;
        std::uint64_t token = 0;
        while (cursor < line.size() && line[cursor] >= '0' && line[cursor] <= '9')
        {
            token = token * 10U + static_cast<unsigned>(line[cursor++] - '0');
            if (token > std::numeric_limits<std::uint32_t>::max()) break;
        }
        if (cursor == token_start || token == 0 ||
            token > std::numeric_limits<std::uint32_t>::max() ||
            cursor == line.size() || !whitespace(line[cursor])) continue;
        const auto candidate = line.find('{', cursor);
        if (candidate != std::string_view::npos &&
            (latest_reply == std::string_view::npos || position > latest_reply))
        {
            latest_reply = position;
            opening = candidate;
        }
    }
    return opening;
}

void add_bytes(std::uint64_t &total, std::size_t bytes) noexcept
{
    const auto count = static_cast<std::uint64_t>(bytes);
    const auto maximum = std::numeric_limits<std::uint64_t>::max();
    total = count > maximum - total ? maximum : total + count;
}
}

std::vector<TelemetryRecord> TelemetryDecoder::feed(std::string_view bytes)
{
    add_bytes(statistics_.bytes_received, bytes.size());
    std::vector<TelemetryRecord> records;
    for (const char ch : bytes)
    {
        if (ch == '\n')
        {
            ++statistics_.lines_received;
            if (!discarding_) complete_line(records);
            pending_.clear();
            discarding_ = false;
            continue;
        }
        if (discarding_) continue;
        // One CR may follow an exactly full line before its LF terminator.
        if (pending_.size() >= maximum_line_bytes &&
            !(pending_.size() == maximum_line_bytes && ch == '\r'))
        {
            pending_.clear();
            discarding_ = true;
            ++statistics_.oversized_lines;
            continue;
        }
        pending_.push_back(ch);
    }
    return records;
}

void TelemetryDecoder::complete_line(std::vector<TelemetryRecord> &out)
{
    if (!pending_.empty() && pending_.back() == '\r') pending_.pop_back();
    TelemetryRecord record;
    record.text = without_ansi(pending_);
    if (trim(record.text).empty()) return;
    const auto opening = record.text.find('{');
    if (opening != std::string::npos)
    {
        auto json = trim(std::string_view(record.text).substr(opening));
        util::json::Value parsed;
        bool valid = util::json::parse(json, parsed);
        if (valid && opening > 0 && !trim(record.text).starts_with("OK ") && reply_payload_start(record.text) == opening)
            record.recovered_prefix = true;
        if (!valid)
        {
            const auto recovered_opening = reply_payload_start(record.text);
            if (recovered_opening != std::string_view::npos && recovered_opening > opening)
            {
                const auto recovered_json = trim(std::string_view(record.text).substr(recovered_opening));
                if (util::json::parse(recovered_json, parsed))
                {
                    if (telemetry_marker(std::string_view(record.text).substr(opening, recovered_opening - opening)))
                        ++statistics_.malformed_records;
                    json = recovered_json;
                    valid = true;
                    record.recovered_prefix = true;
                }
            }
        }
        if (valid)
        {
            const auto *event = string_value(field(parsed, "event", "e"));
            if (event && known_event(*event))
            {
                if (identified(parsed))
                {
                    record.event = *event;
                    record.json.assign(json);
                    ++statistics_.telemetry_records;
                    if (!warning_or_error(record.text) && seen_payload(record.json))
                    {
                        record.duplicate = true;
                        ++statistics_.duplicate_records;
                    }
                }
                else ++statistics_.malformed_records;
            }
        }
        else if (telemetry_marker(json)) ++statistics_.malformed_records;
    }
    out.push_back(std::move(record));
}

bool TelemetryDecoder::seen_payload(const std::string &payload)
{
    if (seen_.contains(payload)) return true;
    while (!recent_.empty() && (recent_.size() >= maximum_cached_records ||
           payload.size() > maximum_cached_bytes - cached_bytes_))
    {
        cached_bytes_ -= recent_.front().size();
        seen_.erase(recent_.front());
        recent_.pop_front();
    }
    // Every payload also obeys the line bound, which is smaller than this cap.
    recent_.push_back(payload);
    seen_.insert(payload);
    cached_bytes_ += payload.size();
    return false;
}

void TelemetryDecoder::finish()
{
    if (!pending_.empty() || discarding_) ++statistics_.partial_lines;
    pending_.clear();
    discarding_ = false;
}

void TelemetryDecoder::clear_duplicates() noexcept
{
    recent_.clear();
    seen_.clear();
    cached_bytes_ = 0;
}

DecoderStatistics TelemetryDecoder::statistics() const noexcept
{
    return statistics_;
}
}
