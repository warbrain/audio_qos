#include "check.hpp"
#include "device/telemetry_record.hpp"

#include <string>
#include <string_view>

namespace
{
std::string telemetry(unsigned sequence = 1, unsigned part = 0,
                      std::string_view boot = "12345678", unsigned value = 7)
{
    return "{\"event\":\"audio.pipeline\",\"boot_id\":\"" + std::string(boot) +
        "\",\"role\":0,\"session_id\":42,\"seq\":" + std::to_string(sequence) +
        ",\"part\":" + std::to_string(part) + ",\"value\":" + std::to_string(value) + "}";
}
}

HMI_TEST(device_telemetry_decodes_fragmented_unicode_and_crlf)
{
    hmi::device::TelemetryDecoder decoder;
    const std::string payload = R"({"event":"audio.device_boot","boot_id":"12345678","part":0,"name":"双麦"})";
    const auto split = payload.find("双") + 1;
    CHECK(decoder.feed("\x1b[32mI (123) runtime: " + payload.substr(0, split)).empty());
    CHECK(decoder.feed(payload.substr(split) + "\x1b[0m\r").empty());
    const auto records = decoder.feed("\n\r\nplain text\n");
    CHECK(records.size() == 2);
    if (records.size() != 2) return;
    CHECK(records[0].json == payload);
    CHECK(records[0].event == "audio.device_boot");
    CHECK(records[0].text == "I (123) runtime: " + payload);
    CHECK(records[0].text.find('\x1b') == std::string::npos);
    CHECK(records[1].text == "plain text");
    CHECK(records[1].json.empty());
    CHECK(decoder.statistics().lines_received == 3);
    CHECK(decoder.statistics().telemetry_records == 1);
}

HMI_TEST(device_telemetry_repeated_payload_keeps_query_acknowledgements)
{
    hmi::device::TelemetryDecoder decoder;
    const auto payload = telemetry();
    const auto records = decoder.feed("I (12) mic: " + payload +
        "\nOK TELEMETRY token=31 " + payload + "\nOK TELEMETRY token=32 " + payload + "\n");
    CHECK(records.size() == 3);
    if (records.size() != 3) return;
    CHECK(!records[0].duplicate);
    CHECK(records[1].duplicate);
    CHECK(records[2].duplicate);
    CHECK(records[1].text.starts_with("OK TELEMETRY token=31 "));
    CHECK(records[2].json == payload);
    CHECK(decoder.statistics().telemetry_records == 3);
    CHECK(decoder.statistics().duplicate_records == 2);
}

HMI_TEST(device_telemetry_retains_distinct_parts_boots_windows_and_conflicts)
{
    hmi::device::TelemetryDecoder decoder;
    const auto records = decoder.feed(telemetry(1, 0) + "\n" + telemetry(1, 1) + "\n" +
        telemetry(2, 0) + "\n" + telemetry(1, 0, "87654321") + "\n" +
        telemetry(1, 0, "12345678", 8) + "\n" + telemetry(1, 0) + "\n");
    CHECK(records.size() == 6);
    if (records.size() != 6) return;
    for (std::size_t i = 0; i < 5; ++i) CHECK(!records[i].duplicate);
    CHECK(records[5].duplicate);
    CHECK(decoder.statistics().duplicate_records == 1);
}

HMI_TEST(device_telemetry_supports_compact_dongle_identity)
{
    hmi::device::TelemetryDecoder decoder;
    const std::string payload = R"({"e":"dongle.diag","v":2,"b":"aabbccdd","q":88,"r":1,"s":9,"p":13,"n":14,"rxt":"7722219513"})";
    const auto records = decoder.feed("OK DONGLE_TELEMETRY_V2 token=8 " + payload +
        "\nOK DONGLE_TELEMETRY_V2 token=9 " + payload + "\n");
    CHECK(records.size() == 2);
    if (records.size() != 2) return;
    CHECK(records[0].event == "dongle.diag");
    CHECK(records[0].json == payload);
    CHECK(!records[0].duplicate);
    CHECK(records[1].duplicate);
}

HMI_TEST(device_telemetry_warning_and_error_records_are_never_suppressed)
{
    hmi::device::TelemetryDecoder decoder;
    const auto payload = telemetry();
    const auto records = decoder.feed("W (12) mic: " + payload + "\nW (12) mic: " + payload +
        "\nE (12) mic: " + payload + "\nE (12) mic: " + payload +
        "\nwarning text\nwarning text\nERR TELEMETRY unavailable\nERR TELEMETRY unavailable\n");
    CHECK(records.size() == 8);
    for (const auto &record : records) CHECK(!record.duplicate);
    CHECK(decoder.statistics().duplicate_records == 0);
}

HMI_TEST(device_telemetry_keeps_json_integer_spelling_and_precision)
{
    hmi::device::TelemetryDecoder decoder;
    const std::string first = R"({"event":"audio.pipeline","boot_id":"12345678","part":0,"wide":9007199254740992})";
    const std::string second = R"({"event":"audio.pipeline","boot_id":"12345678","part":0,"wide":9007199254740993})";
    const std::string maximum = R"({"event":"audio.pipeline","boot_id":"12345678","part":0,"wide":18446744073709551615})";
    const auto records = decoder.feed(first + "\n" + second + "\n" + maximum + "\n" + second + "\n");
    CHECK(records.size() == 4);
    if (records.size() != 4) return;
    CHECK(records[0].json == first);
    CHECK(records[1].json == second);
    CHECK(records[2].json == maximum);
    CHECK(!records[0].duplicate && !records[1].duplicate && !records[2].duplicate);
    CHECK(records[3].duplicate);
}

HMI_TEST(device_telemetry_flags_invalid_payloads_and_keeps_unknown_text)
{
    hmi::device::TelemetryDecoder decoder;
    const std::string lines =
        R"({"event":"audio.pipeline","boot_id":"12345678","part":0,})" "\n"
        R"({"event":"audio.pipeline","boot_id":"12345678"})" "\n"
        R"({"event":"audio.pipeline","boot_id":"12345678","part":0.5})" "\n"
        R"({"event":"audio.pipeline","boot_id":"12345678","part":-1})" "\n"
        R"({"event":"audio.pipeline","boot_id":"12345678","part":0,"part":0})" "\n"
        R"({"event":"other.event","boot_id":"12345678","part":0})" "\n"
        R"({"message":"audio.pipeline"})" "\n"
        R"({"event":"audio.pipeline","boot_id":"12345678","part":0})" " trailing\n";
    const auto records = decoder.feed(lines);
    CHECK(records.size() == 8);
    for (const auto &record : records) CHECK(record.json.empty());
    CHECK(decoder.statistics().malformed_records == 6);
    CHECK(decoder.statistics().telemetry_records == 0);
}

HMI_TEST(device_telemetry_recovers_after_oversized_lines_and_accepts_boundary)
{
    hmi::device::TelemetryDecoder decoder;
    CHECK(decoder.feed(std::string(8192, 'x')).empty());
    CHECK(decoder.feed("xmore discarded").empty());
    auto records = decoder.feed("\n" + telemetry() + "\n");
    CHECK(records.size() == 1);
    CHECK(decoder.statistics().oversized_lines == 1);
    CHECK(decoder.statistics().lines_received == 2);
    records = decoder.feed(std::string(8192, 'y') + "\r\n");
    CHECK(records.size() == 1);
    if (!records.empty()) CHECK(records.front().text.size() == 8192);
    CHECK(decoder.statistics().oversized_lines == 1);
}

HMI_TEST(device_telemetry_disconnect_discards_partial_and_retains_dedup_history)
{
    hmi::device::TelemetryDecoder decoder;
    const auto payload = telemetry();
    CHECK(decoder.feed(payload + "\n").size() == 1);
    CHECK(decoder.feed("unfinished ").empty());
    decoder.finish();
    decoder.finish();
    CHECK(decoder.statistics().partial_lines == 1);
    const auto records = decoder.feed("OK TELEMETRY token=99 " + payload + "\n");
    CHECK(records.size() == 1);
    if (!records.empty())
    {
        CHECK(records.front().duplicate);
        CHECK(!records.front().text.starts_with("unfinished"));
    }
    CHECK(decoder.feed(std::string(8194, 'z')).empty());
    decoder.finish();
    CHECK(decoder.statistics().partial_lines == 2);
    CHECK(decoder.statistics().oversized_lines == 1);
    CHECK(decoder.feed("connected again\n").size() == 1);
}

HMI_TEST(device_telemetry_clear_duplicates_preserves_framing_and_counters)
{
    hmi::device::TelemetryDecoder decoder;
    const auto payload = telemetry();
    const auto first = decoder.feed(payload + "\n" + payload + "\n");
    CHECK(first.size() == 2);
    CHECK(decoder.statistics().duplicate_records == 1);
    const std::string prefix = "OK TELEMETRY token=77 ";
    const auto split = payload.size() / 2;
    CHECK(decoder.feed(prefix + payload.substr(0, split)).empty());
    const auto before = decoder.statistics();
    decoder.clear_duplicates();
    CHECK(decoder.statistics().bytes_received == before.bytes_received);
    CHECK(decoder.statistics().lines_received == before.lines_received);
    CHECK(decoder.statistics().telemetry_records == before.telemetry_records);
    CHECK(decoder.statistics().duplicate_records == before.duplicate_records);
    const auto recovered = decoder.feed(payload.substr(split) + "\n");
    CHECK(recovered.size() == 1);
    if (recovered.empty()) return;
    CHECK(recovered.front().text == prefix + payload);
    CHECK(recovered.front().json == payload);
    CHECK(!recovered.front().duplicate);
    decoder.finish();
    const auto repeated = decoder.feed(payload + "\n");
    CHECK(repeated.size() == 1);
    if (!repeated.empty()) CHECK(repeated.front().duplicate);
    CHECK(decoder.statistics().partial_lines == 0);
    CHECK(decoder.statistics().duplicate_records == 2);
}

HMI_TEST(device_telemetry_removes_terminal_osc_and_handles_plain_controls)
{
    hmi::device::TelemetryDecoder decoder;
    const auto payload = telemetry();
    const auto records = decoder.feed("\x1b]title\a\x1b[0;32m" + payload + "\x1b[0m\n" +
        std::string("text\0tail\n", 10));
    CHECK(records.size() == 2);
    if (records.size() != 2) return;
    CHECK(records[0].json == payload);
    CHECK(records[1].text == std::string("text\0tail", 9));
    CHECK(records[1].json.empty());
}

HMI_TEST(device_telemetry_cache_bounds_record_count)
{
    hmi::device::TelemetryDecoder decoder;
    for (unsigned sequence = 0; sequence < 4097; ++sequence)
    {
        const auto records = decoder.feed(telemetry(sequence) + "\n");
        CHECK(records.size() == 1);
        if (!records.empty()) CHECK(!records.front().duplicate);
    }
    auto records = decoder.feed(telemetry(0) + "\n");
    CHECK(records.size() == 1);
    if (!records.empty()) CHECK(!records.front().duplicate);
    records = decoder.feed(telemetry(4096) + "\n");
    CHECK(records.size() == 1);
    if (!records.empty()) CHECK(records.front().duplicate);
}

HMI_TEST(device_telemetry_cache_bounds_retained_payload_bytes)
{
    hmi::device::TelemetryDecoder decoder;
    std::string first;
    for (unsigned sequence = 0; sequence < 300; ++sequence)
    {
        auto payload = telemetry(sequence);
        payload.pop_back();
        payload += ",\"padding\":\"" + std::string(7900, 'x') + "\"}";
        if (sequence == 0) first = payload;
        const auto records = decoder.feed(payload + "\n");
        CHECK(records.size() == 1);
        if (!records.empty()) CHECK(!records.front().json.empty());
    }
    const auto records = decoder.feed(first + "\n");
    CHECK(records.size() == 1);
    if (!records.empty()) CHECK(!records.front().duplicate);
    CHECK(decoder.statistics().oversized_lines == 0);
}

HMI_TEST(device_telemetry_preserves_truncated_text_before_command_reply)
{
    hmi::device::TelemetryDecoder decoder;
    const auto payload = telemetry();
    const std::string prefix = "I (123) audio: recovery nack_ac";
    const auto physical = prefix + "OK TELEMETRY token=2 " + payload;
    const auto records = decoder.feed(physical + "\n");
    CHECK(records.size() == 1);
    if (records.empty()) return;
    CHECK(records.front().text == physical);
    CHECK(records.front().json == payload);
    CHECK(records.front().recovered_prefix);
    CHECK(!records.front().duplicate);
    CHECK(decoder.statistics().malformed_records == 0);
    const auto repeated = decoder.feed(prefix + "OK TELEMETRY token=3 " + payload + "\n");
    CHECK(repeated.size() == 1);
    if (!repeated.empty())
    {
        CHECK(repeated.front().duplicate);
        CHECK(repeated.front().recovered_prefix);
        CHECK(repeated.front().text.find("token=3 ") != std::string::npos);
        CHECK(repeated.front().json == payload);
    }
}

HMI_TEST(device_telemetry_recovers_known_replies_after_truncated_json)
{
    hmi::device::TelemetryDecoder decoder;
    const std::string truncated = R"(I (123) audio: {"event":"audio.rx_telemetry","boot_id":"old","part":0,"nack_ac)";
    const std::string prefixes[] = {
        "OK TELEMETRY token=7 ",
        "OK PERF_SNAPSHOT token=4294967295 ",
        "OK DONGLE_TELEMETRY_V2 token=9 ",
        "OK SCEPTER_TELEMETRY token=10 kind=1 event=audio.pipeline cache=1 active=1 "};
    for (unsigned i = 0; i < 4; ++i)
    {
        const auto payload = telemetry(i);
        const auto physical = truncated + prefixes[i] + payload;
        CHECK(decoder.feed(truncated + prefixes[i].substr(0, 8)).empty());
        const auto records = decoder.feed(prefixes[i].substr(8) + payload + "\r\n");
        CHECK(records.size() == 1);
        if (!records.empty())
        {
            CHECK(records.front().text == physical);
            CHECK(records.front().json == payload);
            CHECK(records.front().event == "audio.pipeline");
            CHECK(!records.front().duplicate);
        }
    }
    CHECK(decoder.statistics().malformed_records == 4);
    CHECK(decoder.statistics().telemetry_records == 4);
}

HMI_TEST(device_telemetry_does_not_resynchronize_at_arbitrary_braces_or_tokens)
{
    hmi::device::TelemetryDecoder decoder;
    const std::string truncated = R"({"event":"audio.pipeline","boot_id":"old","part":0,"bad":)";
    const auto payload = telemetry();
    const std::string invalid[] = {
        truncated + payload,
        truncated + "OK OTHER token=2 " + payload,
        truncated + "OK TELEMETRY token=0 " + payload,
        truncated + "OK TELEMETRY token=4294967296 " + payload,
        truncated + "OK TELEMETRY token=2x " + payload,
        truncated + "OK TELEMETRY token=2" + payload};
    for (const auto &line : invalid)
    {
        const auto records = decoder.feed(line + "\n");
        CHECK(records.size() == 1);
        if (!records.empty())
        {
            CHECK(records.front().text == line);
            CHECK(records.front().json.empty());
        }
    }
    CHECK(decoder.statistics().malformed_records == 6);
    CHECK(decoder.statistics().telemetry_records == 0);
}

HMI_TEST(device_telemetry_valid_json_reply_text_is_not_resynchronized)
{
    hmi::device::TelemetryDecoder decoder;
    const std::string payload = R"({"event":"other.event","message":"OK TELEMETRY token=5 {not a reply}"})";
    const auto records = decoder.feed(payload + "\n");
    CHECK(records.size() == 1);
    if (!records.empty())
    {
        CHECK(records.front().text == payload);
        CHECK(records.front().json.empty());
    }
    CHECK(decoder.statistics().malformed_records == 0);
}
