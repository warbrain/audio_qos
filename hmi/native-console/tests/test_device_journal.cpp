#include "check.hpp"
#include "device/telemetry_journal.hpp"
#include "util/json_reader.hpp"

#include <windows.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
class JournalDirectory
{
public:
    JournalDirectory()
    {
        wchar_t executable[32768]{};
        const DWORD count = GetModuleFileNameW(nullptr, executable, 32768);
        if (!count || count == 32768) throw std::runtime_error("cannot locate journal test executable");
        parent_ = std::filesystem::path(std::wstring(executable, count)).parent_path();
        for (int attempt = 0; attempt < 100; ++attempt)
        {
            const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
            path = parent_ / (L"hmi-device-journal-" + std::to_wstring(GetCurrentProcessId()) +
                             L"-" + std::to_wstring(stamp) + L"-" + std::to_wstring(attempt));
            std::error_code code;
            if (std::filesystem::create_directory(path, code)) return;
        }
        throw std::runtime_error("cannot create device journal fixture");
    }

    ~JournalDirectory()
    {
        const auto checked = std::filesystem::absolute(path).lexically_normal();
        if (checked.parent_path() == parent_ && checked.filename().wstring().starts_with(L"hmi-device-journal-"))
        {
            std::error_code code;
            std::filesystem::remove_all(checked, code);
        }
    }

    std::filesystem::path path;

private:
    std::filesystem::path parent_;
};

std::string read_file(const std::filesystem::path &file)
{
    std::ifstream input(file, std::ios::binary);
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

hmi::device::JournalOptions journal_options(const std::filesystem::path &directory)
{
    hmi::device::JournalOptions options;
    options.directory = directory;
    options.stem = "scepter_aca7042f1bf0";
    options.device_id = "SCEPTER-aca7042f1bf0";
    options.port = "COM6";
    options.host_session = "host-12345";
    return options;
}

std::filesystem::path json_file(const hmi::device::JournalOptions &options, int archive = 0)
{
    return options.directory / (options.stem + ".telemetry" +
        (archive ? "." + std::to_string(archive) : std::string{}) + ".jsonl");
}

hmi::device::TelemetryRecord numbered_record(int number, std::size_t padding = 0)
{
    hmi::device::TelemetryRecord record;
    record.text = "I (1234) device: record=" + std::to_string(number) + " " + std::string(padding, 'x');
    record.json = "{\"event\":\"audio.test\",\"seq\":" + std::to_string(number) +
        ",\"text\":\"" + std::string(padding, 'x') + "\"}";
    record.event = "audio.test";
    return record;
}
}

HMI_TEST(device_journal_unicode_jsonl_and_exact_u64)
{
    JournalDirectory directory;
    auto options = journal_options(directory.path / L"设备记录");
    options.device_id = "设备\"编号\\一\n二";
    options.port = "COM6\t标签";
    options.host_session = "会话\"\\标识";
    hmi::device::TelemetryJournal journal;
    std::string error = "stale";
    CHECK(journal.open(options, &error));
    CHECK(error.empty());
    auto record = numbered_record(1);
    record.text = "W (1) mic: 中文文字 {literal} %s";
    record.json = R"({"event":"audio.test","counter":18446744073709551615,"text":"中文"})";
    CHECK(journal.write(record, INT64_C(1700000123456), std::numeric_limits<std::uint64_t>::max(), &error));
    CHECK(error.empty());
    CHECK(journal.flush(&error));
    const auto json = read_file(json_file(options));
    CHECK(std::count(json.begin(), json.end(), '\n') == 1);
    CHECK(json.find("\"received_at_ms\":1700000123456") != std::string::npos);
    CHECK(json.find("\"connection\":18446744073709551615") != std::string::npos);
    CHECK(json.find("\"payload\":" + record.json) != std::string::npos);
    hmi::util::json::Value parsed;
    CHECK(hmi::util::json::parse(json, parsed));
    CHECK(parsed.find("device_id") != nullptr);
    if (parsed.find("device_id")) CHECK(std::get<std::string>(parsed.find("device_id")->data) == options.device_id);
    if (parsed.find("host_session")) CHECK(std::get<std::string>(parsed.find("host_session")->data) == options.host_session);
    const auto text = read_file(options.directory / (options.stem + ".log"));
    CHECK(text.find("[warning]") != std::string::npos);
    CHECK(text.find("中文文字 {literal} %s") != std::string::npos);
    CHECK(text.find("[COM6\\t标签]") != std::string::npos);
}

HMI_TEST(device_journal_text_only_duplicates_append_and_reconnect_metadata)
{
    JournalDirectory directory;
    auto options = journal_options(directory.path);
    hmi::device::TelemetryJournal journal;
    CHECK(journal.open(options));
    auto record = numbered_record(1);
    CHECK(journal.write(record, 100, 1));
    CHECK(journal.flush());
    const auto before = read_file(json_file(options));
    const auto text_before = read_file(options.directory / (options.stem + ".log"));
    record.duplicate = true;
    CHECK(journal.write(record, 101, 1));
    CHECK(journal.flush());
    CHECK(read_file(json_file(options)) == before);
    CHECK(read_file(options.directory / (options.stem + ".log")) == text_before);
    record.recovered_prefix = true;
    record.text = "I (2) cut-prefixOK TELEMETRY token=3 " + record.json;
    CHECK(journal.write(record, 101, 1));
    CHECK(journal.flush());
    CHECK(read_file(json_file(options)) == before);
    CHECK(read_file(options.directory / (options.stem + ".log")).find("cut-prefixOK TELEMETRY") != std::string::npos);
    hmi::device::TelemetryRecord text;
    text.text = "E (2) device: retained error\nline";
    CHECK(journal.write(text, 102, 1));
    CHECK(journal.flush());
    CHECK(read_file(json_file(options)) == before);
    CHECK(read_file(options.directory / (options.stem + ".log")).find("retained error\\nline") != std::string::npos);
    journal.close();
    options.port = "COM16";
    options.host_session = "next-host";
    CHECK(journal.open(options));
    CHECK(journal.write(numbered_record(2), 200, 2));
    CHECK(journal.flush());
    journal.close();
    const auto after = read_file(json_file(options));
    CHECK(after.starts_with(before));
    CHECK(std::count(after.begin(), after.end(), '\n') == 2);
    CHECK(after.find("\"port\":\"COM16\"") != std::string::npos);
    CHECK(after.find("\"host_session\":\"next-host\"") != std::string::npos);
    std::string error;
    CHECK(!journal.write(numbered_record(3), 201, 3, &error));
    CHECK(!error.empty());
    CHECK(journal.flush());
}

HMI_TEST(device_journal_rotation_bounds_both_outputs_and_preserves_valid_rows)
{
    JournalDirectory directory;
    auto options = journal_options(directory.path);
    options.max_bytes = 1024;
    options.max_files = 2;
    hmi::device::TelemetryJournal journal;
    CHECK(journal.open(options));
    for (int number = 0; number < 24; ++number)
        CHECK(journal.write(numbered_record(number, 300), number, 1));
    CHECK(journal.flush());
    journal.close();
    CHECK(std::filesystem::exists(json_file(options, 1)));
    CHECK(std::filesystem::exists(json_file(options, 2)));
    CHECK(!std::filesystem::exists(json_file(options, 3)));
    CHECK(std::filesystem::exists(options.directory / (options.stem + ".1.log")));
    CHECK(std::filesystem::exists(options.directory / (options.stem + ".2.log")));
    CHECK(!std::filesystem::exists(options.directory / (options.stem + ".3.log")));
    CHECK(read_file(json_file(options)).find("\"seq\":23") != std::string::npos);
    std::size_t files = 0;
    for (const auto &entry : std::filesystem::directory_iterator(options.directory))
    {
        ++files;
        CHECK(entry.file_size() <= options.max_bytes);
    }
    CHECK(files == 6);
    for (int archive = 0; archive <= 2; ++archive)
    {
        std::ifstream input(json_file(options, archive), std::ios::binary);
        std::string line;
        while (std::getline(input, line))
        {
            hmi::util::json::Value parsed;
            CHECK(hmi::util::json::parse(line, parsed));
            CHECK(parsed.find("payload") != nullptr);
        }
    }
}

HMI_TEST(device_journal_failed_open_retains_existing_journal_and_rejects_paths)
{
    JournalDirectory directory;
    const auto options = journal_options(directory.path);
    hmi::device::TelemetryJournal journal;
    CHECK(journal.open(options));
    CHECK(journal.write(numbered_record(1), 100, 1));
    const std::vector<std::string> invalid{
        "", ".", "..", "../escape", "a/b", "a\\b", "a.json", "CON", "com1", "LPT9", std::string(97, 'a')};
    std::string error;
    for (const auto &stem : invalid)
    {
        auto rejected = options;
        rejected.stem = stem;
        CHECK(!journal.open(rejected, &error));
        CHECK(!error.empty());
    }
    auto rejected = options;
    rejected.max_bytes = 1023;
    CHECK(!journal.open(rejected, &error));
    rejected = options;
    rejected.max_files = 0;
    CHECK(!journal.open(rejected, &error));
    rejected = options;
    rejected.directory = directory.path / "not-a-directory";
    { std::ofstream sentinel(rejected.directory); sentinel << "preserved"; }
    CHECK(!journal.open(rejected, &error));
    CHECK(read_file(rejected.directory) == "preserved");
    CHECK(journal.write(numbered_record(2), 101, 1));
    CHECK(journal.flush());
    const auto text = read_file(json_file(options));
    CHECK(std::count(text.begin(), text.end(), '\n') == 2);
}

HMI_TEST(device_journal_rejects_invalid_payload_without_corrupting_jsonl)
{
    JournalDirectory directory;
    const auto options = journal_options(directory.path);
    hmi::device::TelemetryJournal journal;
    CHECK(journal.open(options));
    std::string error;
    const std::vector<std::string> invalid{"{broken", "[]", "{\"a\":1}\n{\"a\":2}", "{\"a\":1,\"a\":2}"};
    for (const auto &payload : invalid)
    {
        auto record = numbered_record(1);
        record.json = payload;
        CHECK(!journal.write(record, 100, 1, &error));
        CHECK(!error.empty());
    }
    CHECK(journal.write(numbered_record(2), 101, 1));
    CHECK(journal.flush());
    const auto text = read_file(json_file(options));
    CHECK(std::count(text.begin(), text.end(), '\n') == 1);
    CHECK(text.find("\"seq\":2") != std::string::npos);
}

HMI_TEST(device_journal_rotation_failure_is_visible_and_stops_further_writes)
{
    JournalDirectory directory;
    auto options = journal_options(directory.path);
    options.max_bytes = 1024;
    options.max_files = 2;
    hmi::device::TelemetryJournal journal;
    CHECK(journal.open(options));
    CHECK(journal.write(numbered_record(1, 620), 100, 1));
    CHECK(journal.flush());
    const auto original = read_file(json_file(options));
    CHECK(std::filesystem::create_directory(json_file(options, 1)));
    std::string error;
    CHECK(!journal.write(numbered_record(2, 620), 101, 1, &error));
    CHECK(!error.empty());
    CHECK(read_file(json_file(options)) == original);
    CHECK(!journal.write(numbered_record(3), 102, 1, &error));
    CHECK(!journal.flush(&error));
    journal.close();
}

HMI_TEST(device_journal_failed_write_retransmission_is_saved_after_duplicate_reset)
{
    JournalDirectory directory;
    auto options = journal_options(directory.path);
    options.max_bytes = 1024;
    options.max_files = 2;
    hmi::device::TelemetryJournal journal;
    hmi::device::TelemetryDecoder decoder;
    CHECK(journal.open(options));
    const auto payload = [](int sequence) {
        return "{\"event\":\"audio.pipeline\",\"boot_id\":\"12345678\",\"role\":0,\"part\":0,\"seq\":" +
            std::to_string(sequence) + ",\"padding\":\"" + std::string(620, 'x') + "\"}";
    };
    const auto first_payload = payload(1);
    const auto second_payload = payload(2);
    auto records = decoder.feed(first_payload + "\n");
    CHECK(records.size() == 1);
    if (records.empty()) return;
    CHECK(journal.write(records.front(), 100, 1));
    CHECK(journal.flush());
    const auto original = read_file(json_file(options));
    CHECK(original.size() <= options.max_bytes);
    // Block the next rotation after the decoder has received a new payload.
    CHECK(std::filesystem::create_directory(json_file(options, 1)));
    records = decoder.feed(second_payload + "\n");
    CHECK(records.size() == 1);
    if (records.empty()) return;
    CHECK(!records.front().duplicate);
    std::string error;
    CHECK(!journal.write(records.front(), 101, 1, &error));
    CHECK(!error.empty());
    CHECK(read_file(json_file(options)) == original);
    journal.close();
    decoder.finish();
    decoder.clear_duplicates();
    CHECK(std::filesystem::remove(json_file(options, 1)));
    CHECK(journal.open(options, &error));
    records = decoder.feed("OK TELEMETRY token=88 " + second_payload + "\n");
    CHECK(records.size() == 1);
    if (records.empty()) return;
    CHECK(!records.front().duplicate);
    CHECK(journal.write(records.front(), 200, 2, &error));
    CHECK(journal.flush(&error));
    const auto recovered = read_file(json_file(options));
    CHECK(recovered.find("\"seq\":2") != std::string::npos);
    CHECK(std::count(recovered.begin(), recovered.end(), '\n') == 1);
    CHECK(read_file(json_file(options, 1)) == original);
    // Successful reconnects still retain committed payload history.
    decoder.finish();
    records = decoder.feed("OK TELEMETRY token=89 " + second_payload + "\n");
    CHECK(records.size() == 1);
    if (records.empty()) return;
    CHECK(records.front().duplicate);
    CHECK(journal.write(records.front(), 201, 3, &error));
    CHECK(journal.flush(&error));
    CHECK(read_file(json_file(options)) == recovered);
}

HMI_TEST(device_journal_recovers_incomplete_tail_after_lossless_warning_preservation)
{
    JournalDirectory directory;
    const auto options = journal_options(directory.path);
    const std::string complete = "{\"old\":1}\n{\"old\":2}\n";
    // The tail ends inside a UTF-8 code point and includes a literal NUL.
    const std::string fragment = "{\"unfinished\":\"" + std::string("\xe4\xb8\0", 3);
    {
        std::ofstream output(json_file(options), std::ios::binary);
        output.write(complete.data(), static_cast<std::streamsize>(complete.size()));
        output.write(fragment.data(), static_cast<std::streamsize>(fragment.size()));
    }
    hmi::device::TelemetryJournal journal;
    std::string error;
    CHECK(journal.open(options, &error));
    CHECK(error.empty());
    CHECK(read_file(json_file(options)) == complete);
    const auto preserved = read_file(options.directory / (options.stem + ".log"));
    CHECK(preserved.find("[warning]") != std::string::npos);
    CHECK(preserved.find("event=telemetry_incomplete_tail") != std::string::npos);
    CHECK(preserved.find("preserved_bytes=" + std::to_string(fragment.size())) != std::string::npos);
    CHECK(preserved.find("encoding=hex data=7b22756e66696e6973686564223a22e4b800") != std::string::npos);
    CHECK(journal.write(numbered_record(3), 100, 1));
    CHECK(journal.flush());
    journal.close();
    const auto content = read_file(json_file(options));
    CHECK(content.starts_with(complete));
    CHECK(std::count(content.begin(), content.end(), '\n') == 3);
    std::ifstream input(json_file(options), std::ios::binary);
    std::string line;
    while (std::getline(input, line))
    {
        hmi::util::json::Value parsed;
        CHECK(hmi::util::json::parse(line, parsed));
    }
}

HMI_TEST(device_journal_tail_recovery_is_bounded_and_preserves_oversized_history)
{
    JournalDirectory directory;
    const auto options = journal_options(directory.path);
    const std::string oversized = "{\"old\":1}\n" + std::string(64 * 1024, 'x');
    { std::ofstream output(json_file(options), std::ios::binary); output << oversized; }
    hmi::device::TelemetryJournal journal;
    std::string error;
    CHECK(!journal.open(options, &error));
    CHECK(error.find("64 KiB") != std::string::npos);
    CHECK(read_file(json_file(options)) == oversized);
    CHECK(read_file(options.directory / (options.stem + ".log")).find("telemetry_incomplete_tail") == std::string::npos);

    // A whole file of exactly 64 KiB is within the bounded recovery limit.
    const std::string bounded(64 * 1024, 'x');
    { std::ofstream output(json_file(options), std::ios::binary | std::ios::trunc); output << bounded; }
    CHECK(journal.open(options, &error));
    CHECK(read_file(json_file(options)).empty());
    const auto preserved = read_file(options.directory / (options.stem + ".log"));
    CHECK(preserved.find("preserved_bytes=65536") != std::string::npos);
    CHECK(preserved.find("encoding=hex data=7878787878787878") != std::string::npos);
    CHECK(journal.write(numbered_record(1), 100, 1));
    CHECK(journal.flush());
    const auto content = read_file(json_file(options));
    CHECK(std::count(content.begin(), content.end(), '\n') == 1);
    hmi::util::json::Value parsed;
    CHECK(hmi::util::json::parse(content, parsed));
}

HMI_TEST(device_journal_does_not_remove_tail_when_preservation_log_cannot_open)
{
    JournalDirectory directory;
    const auto options = journal_options(directory.path);
    const std::string original = "{\"old\":1}\n{\"unfinished\":";
    { std::ofstream output(json_file(options), std::ios::binary); output << original; }
    CHECK(std::filesystem::create_directory(options.directory / (options.stem + ".log")));
    hmi::device::TelemetryJournal journal;
    std::string error;
    CHECK(!journal.open(options, &error));
    CHECK(!error.empty());
    CHECK(read_file(json_file(options)) == original);
}
