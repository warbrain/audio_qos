#include "device/telemetry.hpp"
#include "device/telemetry_journal.hpp"
#include "device/telemetry_record.hpp"
#include "device/telemetry_protocol.hpp"
#include "base/logger.hpp"
#include "util/json_reader.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <iterator>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <thread>
#include <utility>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#include <setupapi.h>
#include <devguid.h>
#include <cfgmgr32.h>
#endif

namespace hmi::device
{
namespace
{
using Clock = std::chrono::steady_clock;

bool contains_header_token(std::string_view header, std::string_view expected)
{
    std::size_t position = 0;
    while (position < header.size())
    {
        while (position < header.size() && (header[position] == ' ' || header[position] == '\t')) ++position;
        const auto end = header.find_first_of(" \t", position);
        const auto token = header.substr(position, end == std::string_view::npos ? end : end - position);
        if (token == expected) return true;
        if (end == std::string_view::npos) break;
        position = end;
    }
    return false;
}

std::int64_t epoch_ms()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string safe_stem(std::string_view text)
{
    std::string result;
    for (const unsigned char value : text)
    {
        if (result.size() >= 90) break;
        result += (value >= 'a' && value <= 'z') || (value >= 'A' && value <= 'Z') ||
            (value >= '0' && value <= '9') || value == '-' || value == '_' ? static_cast<char>(value) : '_';
    }
    return result.empty() ? "device" : result;
}

#ifdef _WIN32
std::string utf8(std::wstring_view text)
{
    if (text.empty()) return {};
    const int size = static_cast<int>(text.size());
    const int count = WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, text.data(), size,
                                         nullptr, 0, nullptr, nullptr);
    if (count <= 0) return {};
    std::string result(static_cast<std::size_t>(count), '\0');
    WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, text.data(), size,
                        result.data(), count, nullptr, nullptr);
    return result;
}

struct Endpoint
{
    std::wstring port;
    std::string id;
    std::string role;
    std::string stem;
};

std::vector<Endpoint> endpoints()
{
    std::vector<Endpoint> result;
    const HDEVINFO devices = SetupDiGetClassDevsW(&GUID_DEVCLASS_PORTS, nullptr, nullptr, DIGCF_PRESENT);
    if (devices == INVALID_HANDLE_VALUE) return result;
    struct DeviceOwner
    {
        HDEVINFO value;
        ~DeviceOwner() { SetupDiDestroyDeviceInfoList(value); }
    } owner{devices};
    for (DWORD index = 0; result.size() < 16; ++index)
    {
        SP_DEVINFO_DATA info{};
        info.cbSize = sizeof(info);
        if (!SetupDiEnumDeviceInfo(devices, index, &info)) break;
        wchar_t hardware[2048]{};
        if (!SetupDiGetDeviceRegistryPropertyW(devices, &info, SPDRP_HARDWAREID, nullptr,
            reinterpret_cast<PBYTE>(hardware), sizeof(hardware), nullptr)) continue;
        hardware[std::size(hardware) - 1] = L'\0';
        std::wstring identity(hardware);
        std::transform(identity.begin(), identity.end(), identity.begin(), [](wchar_t c)
        { return c >= L'a' && c <= L'z' ? static_cast<wchar_t>(c - L'a' + L'A') : c; });
        const bool dongle = identity.find(L"VID_303A&PID_8001") != std::wstring::npos;
        const bool scepter = identity.find(L"VID_303A&PID_1001") != std::wstring::npos;
        if (!dongle && !scepter) continue;
        const HKEY key = SetupDiOpenDevRegKey(devices, &info, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
        if (key == INVALID_HANDLE_VALUE) continue;
        wchar_t name[128]{};
        DWORD bytes = sizeof(name), type = 0;
        const LSTATUS status = RegQueryValueExW(key, L"PortName", nullptr, &type,
                                               reinterpret_cast<LPBYTE>(name), &bytes);
        RegCloseKey(key);
        name[std::size(name) - 1] = L'\0';
        const std::wstring port(name);
        if (status != ERROR_SUCCESS || type != REG_SZ || port.size() < 4 || port.size() > 12 ||
            !port.starts_with(L"COM") || !std::all_of(port.begin() + 3, port.end(),
                [](wchar_t c) { return c >= L'0' && c <= L'9'; })) continue;
        wchar_t parent_name[MAX_DEVICE_ID_LEN]{};
        DEVINST parent = 0;
        if (CM_Get_Parent(&parent, info.DevInst, 0) != CR_SUCCESS ||
            CM_Get_Device_IDW(parent, parent_name, MAX_DEVICE_ID_LEN, 0) != CR_SUCCESS)
            SetupDiGetDeviceInstanceIdW(devices, &info, parent_name, MAX_DEVICE_ID_LEN, nullptr);
        std::wstring serial(parent_name);
        const auto separator = serial.find_last_of(L'\\');
        if (separator != std::wstring::npos) serial.erase(0, separator + 1);
        const std::string role = dongle ? "dongle" : "scepter";
        const auto id = role + ":" + utf8(serial.empty() ? port : serial);
        result.push_back({port, id, role, safe_stem(id)});
    }
    return result;
}

class Serial
{
public:
    ~Serial() { close(); }
    Serial() = default;
    Serial(const Serial &) = delete;
    Serial &operator=(const Serial &) = delete;

    bool open(const std::wstring &port, DWORD &error)
    {
        close();
        value_ = CreateFileW((L"\\\\.\\" + port).c_str(), GENERIC_READ | GENERIC_WRITE, 0,
                             nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (value_ == INVALID_HANDLE_VALUE) { error = GetLastError(); return false; }
        DCB state{};
        state.DCBlength = sizeof(state);
        if (!GetCommState(value_, &state)) return failed(error);
        state.BaudRate = CBR_115200;
        state.ByteSize = 8;
        state.Parity = NOPARITY;
        state.StopBits = ONESTOPBIT;
        state.fBinary = TRUE;
        state.fParity = FALSE;
        state.fOutxCtsFlow = FALSE;
        state.fOutxDsrFlow = FALSE;
        state.fDtrControl = DTR_CONTROL_DISABLE;
        state.fDsrSensitivity = FALSE;
        state.fOutX = FALSE;
        state.fInX = FALSE;
        state.fRtsControl = RTS_CONTROL_DISABLE;
        state.fAbortOnError = FALSE;
        if (!SetCommState(value_, &state)) return failed(error);
        COMMTIMEOUTS timeouts{};
        timeouts.ReadIntervalTimeout = MAXDWORD;
        timeouts.WriteTotalTimeoutConstant = 50;
        if (!SetCommTimeouts(value_, &timeouts)) return failed(error);
        EscapeCommFunction(value_, CLRDTR);
        EscapeCommFunction(value_, CLRRTS);
        return true;
    }

    bool read(std::string &out, DWORD &error)
    {
        char bytes[8192];
        DWORD count = 0;
        if (!ReadFile(value_, bytes, sizeof(bytes), &count, nullptr)) { error = GetLastError(); return false; }
        out.assign(bytes, count);
        return true;
    }

    bool write(std::string_view text, DWORD &error)
    {
        DWORD count = 0;
        if (!WriteFile(value_, text.data(), static_cast<DWORD>(text.size()), &count, nullptr))
        { error = GetLastError(); return false; }
        if (count != text.size()) { error = ERROR_WRITE_FAULT; return false; }
        return true;
    }

    bool mirror(bool enabled) { return EscapeCommFunction(value_, enabled ? SETDTR : CLRDTR) != FALSE; }
    bool valid() const noexcept { return value_ != INVALID_HANDLE_VALUE; }
    void close() noexcept
    {
        if (valid()) { CloseHandle(value_); value_ = INVALID_HANDLE_VALUE; }
    }

private:
    bool failed(DWORD &error) { error = GetLastError(); close(); return false; }
    HANDLE value_ = INVALID_HANDLE_VALUE;
};

struct Command
{
    std::string name;
    std::string text;
    std::uint32_t token = 0;
    int part = -1;
    unsigned group_size = 0;
    int stage = -1;
    unsigned retry = 0;
};

bool matches(const Command &command, const TelemetryRecord &record)
{
    const auto &line = record.text;
    if (!command.token)
        return line.starts_with("OK " + command.name + " ") || line == "OK " + command.name ||
            (command.name == "PING" && line.starts_with("OK PONG "));
    const auto suffix = command.name + " token=" + std::to_string(command.token) + " ";
    const auto prefix = "OK " + suffix;
    const auto response_at = line.find(prefix);
    const auto json_at = record.json.empty() ? std::string::npos : line.rfind(record.json);
    if (response_at == std::string::npos || json_at == std::string::npos ||
        response_at + prefix.size() > json_at) return false;
    const auto reply_header = std::string_view(line).substr(response_at + prefix.size(),
        json_at - (response_at + prefix.size()));
    if (command.name == "SCEPTER_TELEMETRY")
    {
        if (!contains_header_token(reply_header, "kind=" + std::to_string(command.stage)) ||
            !contains_header_token(reply_header, "event=audio.dispatch")) return false;
    }
    else if (!std::all_of(reply_header.begin(), reply_header.end(),
                          [](char c) { return c == ' ' || c == '\t'; })) return false;
    util::json::Value payload;
    if (!util::json::parse(record.json, payload)) return false;
    const auto *part = payload.find("part");
    if (!part) part = payload.find("p");
    const auto *number = part ? std::get_if<double>(&part->data) : nullptr;
    return number && *number == command.part;
}

bool rejected(const Command &command, std::string_view line)
{
    if (line.starts_with("ERR COMMAND ")) return true;
    if (!command.token) return line.starts_with("ERR " + command.name + " ");
    if (line.starts_with("ERR " + command.name + " ") && line.find("token=") == std::string_view::npos) return true;
    return line.starts_with("ERR " + command.name + " token=" + std::to_string(command.token) + " ");
}

struct Connection
{
    Endpoint endpoint;
    Serial serial;
    TelemetryDecoder decoder;
    DecoderStatistics accounted;
    SnapshotValidator snapshot;
    TelemetryJournal journal;
    std::deque<Command> commands;
    std::optional<Command> pending;
    std::set<std::string> unsupported;
    std::deque<std::pair<TelemetryRecord, std::int64_t>> identity_records;
    std::size_t identity_bytes = 0;
    Clock::time_point next_open{}, deadline{}, next_poll{}, last_query_warning{}, last_rx{};
    std::uint64_t connection = 0;
    DWORD last_open_error = 0;
    bool verified = false;
    bool restore_mirror_off = false;
    bool storage_failed = false;
    bool journal_open = false;
};
#endif
}

struct Telemetry::Impl
{
    base::Logger *logger;
    TelemetryOptions options;
    mutable std::mutex mutex;
    std::condition_variable wake;
    TelemetryStatistics measured;
    std::thread worker;
    std::atomic<bool> stopping{false};
    std::string host_session;

    explicit Impl(base::Logger *value) : logger(value) {}

    void log(base::LogLevel level, const std::string &text)
    {
        if (logger) logger->write(level, "device_telemetry", text);
    }

    void error(std::string text, bool storage = false)
    {
        { std::lock_guard lock(mutex); measured.last_error = text; if (storage) ++measured.storage_failures; }
        log(base::LogLevel::Warning, text);
    }

#ifdef _WIN32
    std::uint32_t token = 1;
    std::set<std::string> telemetry_devices;

    void append_group(Connection &node, std::string name, unsigned count, int stage = -1,
                      unsigned retry = 0, bool front = false)
    {
        if (node.unsupported.contains(name)) return;
        if (++token == 0) ++token;
        std::deque<Command> group;
        for (unsigned part = 0; part < count; ++part)
        {
            const auto text = name + " " + std::to_string(token) +
                (stage >= 0 ? " " + std::to_string(stage) : "") + " " + std::to_string(part);
            group.push_back({name, text, token, static_cast<int>(part), count, stage, retry});
        }
        node.commands.insert(front ? node.commands.begin() : node.commands.end(),
            std::make_move_iterator(group.begin()), std::make_move_iterator(group.end()));
    }

    void poll(Connection &node)
    {
        append_group(node, "TELEMETRY", 10);
        const int first = node.endpoint.role == "scepter" ? 0 : 2;
        append_group(node, "PERF_SNAPSHOT", 13, first);
        append_group(node, "PERF_SNAPSHOT", 13, first + 1);
        if (node.endpoint.role == "scepter")
            append_group(node, "SCEPTER_TELEMETRY", 17, 6);
        if (node.endpoint.role == "dongle") append_group(node, "DONGLE_TELEMETRY_V2", 14);
        node.next_poll = Clock::now() + std::chrono::seconds(options.poll_interval_seconds);
    }

    void store(Connection &node, const TelemetryRecord &record, std::int64_t received_at = epoch_ms())
    {
        if ((record.duplicate && !record.recovered_prefix) || node.storage_failed) return;
        if (!node.journal_open)
        {
            if (record.text.size() + node.identity_bytes <= 64 * 1024)
            {
                node.identity_records.emplace_back(record, received_at);
                node.identity_bytes += record.text.size();
            }
            else node.decoder.clear_duplicates();
            return;
        }
        std::string detail;
        if (!node.journal.write(record, received_at, node.connection, &detail))
        {
            node.storage_failed = true;
            node.decoder.clear_duplicates();
            error("event=storage_failed device=" + node.endpoint.id + " detail=" + detail, true);
        }
        else
        {
            const bool telemetry = !record.json.empty() && !record.duplicate;
            if (telemetry && telemetry_devices.size() < 16) telemetry_devices.insert(node.endpoint.id);
            std::lock_guard lock(mutex);
            ++measured.records_saved;
            if (telemetry)
            {
                ++measured.telemetry_records_saved;
                if (node.endpoint.role == "scepter") ++measured.scepter_telemetry_records;
                else ++measured.dongle_telemetry_records;
                measured.devices_with_telemetry = telemetry_devices.size();
            }
        }
    }

    void account(Connection &node)
    {
        const auto current = node.decoder.statistics();
        std::lock_guard lock(mutex);
        measured.bytes_received += current.bytes_received - node.accounted.bytes_received;
        measured.duplicate_records += current.duplicate_records - node.accounted.duplicate_records;
        measured.rejected_lines += current.malformed_records - node.accounted.malformed_records +
            current.oversized_lines - node.accounted.oversized_lines;
        measured.partial_lines += current.partial_lines - node.accounted.partial_lines;
        node.accounted = current;
    }

    void close(Connection &node, const std::string &reason)
    {
        const bool was_open = node.serial.valid();
        if (was_open && node.restore_mirror_off)
        {
            DWORD ignored = 0;
            if (node.serial.write("LOG 0\n", ignored))
            {
                // Give the device a bounded opportunity to restore its console
                // setting before closing the CDC connection invalidates RX.
                const auto until = Clock::now() + std::chrono::milliseconds(200);
                bool restored = false;
                while (Clock::now() < until && !restored)
                {
                    std::string bytes;
                    if (!node.serial.read(bytes, ignored)) break;
                    for (const auto &record : node.decoder.feed(bytes))
                    {
                        store(node, record);
                        if (record.text == "OK LOG mirror=0") restored = true;
                    }
                    if (!restored) std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                if (!restored) log(base::LogLevel::Warning, "event=mirror_restore_unconfirmed device=" + node.endpoint.id);
            }
        }
        if (was_open && node.endpoint.role == "dongle") node.serial.mirror(false);
        node.decoder.finish();
        if (was_open)
        {
            if (node.journal_open) store(node, {"event=disconnected reason=" + reason, {}, {}});
            std::string detail;
            if (node.journal_open && !node.storage_failed && !node.journal.flush(&detail))
            { node.storage_failed = true; error(detail, true); }
            log(base::LogLevel::Info, "event=disconnected device=" + node.endpoint.id + " reason=" + reason);
            if (node.verified) { std::lock_guard lock(mutex); if (measured.connected_devices) --measured.connected_devices; }
            log(base::LogLevel::Info, "event=decoder_statistics device=" + node.endpoint.id +
                " partial_lines=" + std::to_string(node.decoder.statistics().partial_lines));
        }
        node.journal.close();
        // Keep the exclusive serial handle until the device's journal is
        // flushed and closed; another collector must not race its rotation.
        node.serial.close();
        if (node.storage_failed || !node.verified) node.decoder.clear_duplicates();
        account(node);
        node.journal_open = false;
        node.identity_records.clear();
        node.identity_bytes = 0;
        const bool identity_failed = !node.verified && (reason == "identity_timeout" ||
            reason == "unexpected_firmware_role" || reason == "unsupported_firmware" || reason.starts_with("write_failed_"));
        node.verified = false;
        node.restore_mirror_off = false;
        node.pending.reset();
        node.commands.clear();
        node.snapshot.reset();
        node.next_open = Clock::now() + std::chrono::seconds(identity_failed ? 60 : 5);
    }

    bool open_journal(Connection &node)
    {
        JournalOptions journal_options;
        journal_options.directory = options.directory;
        journal_options.stem = node.endpoint.stem;
        journal_options.device_id = node.endpoint.id;
        journal_options.port = utf8(node.endpoint.port);
        journal_options.host_session = host_session;
        journal_options.max_bytes = options.max_bytes;
        journal_options.max_files = options.max_files;
        std::string detail;
        if (!node.journal.open(journal_options, &detail))
        {
            error("event=journal_open_failed device=" + node.endpoint.id + " detail=" + detail, true);
            return false;
        }
        node.journal_open = true;
        for (const auto &[record, received_at] : node.identity_records) store(node, record, received_at);
        node.identity_records.clear();
        node.identity_bytes = 0;
        return !node.storage_failed;
    }

    void open(Connection &node)
    {
        DWORD code = 0;
        if (!node.serial.open(node.endpoint.port, code))
        {
            if (code != node.last_open_error)
                error("event=port_unavailable device=" + node.endpoint.id + " port=" + utf8(node.endpoint.port) +
                      " win32=" + std::to_string(code));
            node.last_open_error = code;
            node.next_open = Clock::now() + std::chrono::seconds(5);
            return;
        }
        node.last_open_error = 0;
        node.storage_failed = false;
        node.unsupported.clear();
        ++node.connection;
        store(node, {"event=connected port=" + utf8(node.endpoint.port) + " identity=pending", {}, {}});
        node.commands.push_back({"PING", "PING", 0, -1});
        node.next_poll = Clock::now();
        node.last_rx = Clock::now();
    }

    void query_failed(Connection &node, bool unsupported)
    {
        const auto failed = *node.pending;
        { std::lock_guard lock(mutex); ++measured.query_failures; }
        store(node, {"event=query_unavailable command=" + failed.name + " token=" + std::to_string(failed.token) +
            " part=" + std::to_string(failed.part) + " reason=" + (unsupported ? "unsupported" : "timeout_or_not_ready"), {}, {}});
        if (unsupported) node.unsupported.insert(failed.name);
        std::erase_if(node.commands, [&](const Command &queued)
        { return unsupported || !failed.token ? queued.name == failed.name : queued.token == failed.token; });
        const auto now = Clock::now();
        if (node.last_query_warning == Clock::time_point{} || now - node.last_query_warning >= std::chrono::seconds(30))
        {
            log(base::LogLevel::Warning, "event=query_unavailable device=" + node.endpoint.id + " command=" +
                failed.name + " reason=" + (unsupported ? "unsupported" : "timeout_or_not_ready"));
            node.last_query_warning = now;
        }
        node.pending.reset();
    }

    void validate_snapshot(Connection &node, const Command &command, const TelemetryRecord &record)
    {
        if (command.name != "TELEMETRY" && command.name != "DONGLE_TELEMETRY_V2" &&
            command.name != "SCEPTER_TELEMETRY") return;
        const auto status = node.snapshot.observe(command.name, command.token, command.group_size,
            static_cast<unsigned>(command.part), record.json);
        if (status == SnapshotStatus::Incomplete) return;
        if (status == SnapshotStatus::Complete)
        {
            {
                std::lock_guard lock(mutex);
                ++measured.snapshots_completed;
                if (command.name == "DONGLE_TELEMETRY_V2") ++measured.dongle_snapshots_completed;
            }
            store(node, {"event=snapshot_complete command=" + command.name +
                " token=" + std::to_string(command.token), {}, {}});
            return;
        }
        {
            std::lock_guard lock(mutex);
            if (status == SnapshotStatus::Mismatch) ++measured.mixed_snapshots;
            if (command.retry < 2) ++measured.snapshot_retries;
            else ++measured.snapshot_failures;
        }
        std::erase_if(node.commands, [&](const Command &queued) { return queued.token == command.token; });
        node.snapshot.reset();
        store(node, {"event=snapshot_incomplete command=" + command.name + " token=" +
            std::to_string(command.token) + " reason=" + (status == SnapshotStatus::Mismatch ? "mixed_identity" : "invalid_payload") +
            " retry=" + std::to_string(command.retry), {}, {}});
        if (command.retry < 2)
            append_group(node, command.name, command.group_size, command.stage, command.retry + 1, true);
        else
            log(base::LogLevel::Warning, "event=snapshot_failed device=" + node.endpoint.id + " command=" + command.name);
    }

    void receive(Connection &node, const TelemetryRecord &record)
    {
        if (node.pending && matches(*node.pending, record))
        {
            const auto command = *node.pending;
            const auto &name = command.name;
            node.pending.reset();
            validate_snapshot(node, command, record);
            if (name == "PING")
            {
                const auto role = "role=" + node.endpoint.role;
                const auto at = record.text.find(role);
                if (at == std::string::npos || (at + role.size() != record.text.size() && record.text[at + role.size()] != ' '))
                { close(node, "unexpected_firmware_role"); return; }
                if (!open_journal(node))
                { close(node, "storage_failed"); node.next_open = Clock::now() + std::chrono::seconds(30); return; }
                node.verified = true;
                { std::lock_guard lock(mutex); ++measured.connections; ++measured.connected_devices; }
                log(base::LogLevel::Info, "event=connected device=" + node.endpoint.id + " port=" + utf8(node.endpoint.port));
                node.commands.push_back({"STATUS", "STATUS", 0, -1});
                if (node.endpoint.role == "dongle")
                {
                    if (!node.serial.mirror(true)) error("event=mirror_unavailable device=" + node.endpoint.id);
                    node.commands.push_back({"LOG", "LOG", 0, -1});
                }
                poll(node);
            }
            else if (name == "LOG" && record.text == "OK LOG mirror=0" && !node.restore_mirror_off)
            {
                node.restore_mirror_off = true;
                node.commands.push_front({"LOG", "LOG 1", 0, -1});
            }
        }
        else if (node.pending && rejected(*node.pending, record.text))
        {
            if (!node.verified) { close(node, "unsupported_firmware"); return; }
            query_failed(node, record.text.starts_with("ERR COMMAND "));
        }
        store(node, record);
    }

    void run()
    {
        { std::lock_guard lock(mutex); measured.running = true; }
        std::map<std::string, std::unique_ptr<Connection>> nodes;
        auto next_discovery = Clock::time_point{};
        auto next_flush = Clock::now() + std::chrono::seconds(2);
        auto next_statistics = Clock::now() + std::chrono::seconds(30);
        try
        {
            while (!stopping.load())
            {
                const auto now = Clock::now();
                if (now >= next_discovery)
                {
                    std::set<std::string> present;
                    for (auto endpoint : endpoints())
                    {
                        present.insert(endpoint.id);
                        if (!nodes.contains(endpoint.id) && nodes.size() >= 16)
                        {
                            auto unused = std::find_if(nodes.begin(), nodes.end(), [](const auto &entry)
                            { return !entry.second->serial.valid(); });
                            if (unused == nodes.end()) continue;
                            nodes.erase(unused);
                        }
                        auto &node = nodes[endpoint.id];
                        if (!node) node = std::make_unique<Connection>();
                        if (node->serial.valid() && node->endpoint.port != endpoint.port) close(*node, "port_changed");
                        node->endpoint = std::move(endpoint);
                        if (!node->serial.valid() && now >= node->next_open) open(*node);
                    }
                    for (auto &[id, node] : nodes)
                        if (!present.contains(id) && node->serial.valid()) close(*node, "device_removed");
                    next_discovery = now + std::chrono::seconds(2);
                }
                for (auto &[id, owner] : nodes)
                {
                    (void)id;
                    auto &node = *owner;
                    if (!node.serial.valid()) continue;
                    if (node.storage_failed) { close(node, "storage_failed"); node.next_open = now + std::chrono::seconds(30); continue; }
                    std::string bytes;
                    DWORD code = 0;
                    if (!node.serial.read(bytes, code)) { close(node, "read_failed_" + std::to_string(code)); continue; }
                    if (!bytes.empty()) node.last_rx = now;
                    if (node.verified && now - node.last_rx >= std::chrono::seconds((std::max)(30, options.poll_interval_seconds * 3)))
                    { close(node, "device_unresponsive"); continue; }
                    for (const auto &record : node.decoder.feed(bytes))
                    {
                        receive(node, record);
                        if (!node.serial.valid() || node.storage_failed) break;
                    }
                    account(node);
                    if (!node.serial.valid()) continue;
                    if (node.storage_failed) { close(node, "storage_failed"); node.next_open = now + std::chrono::seconds(30); continue; }
                    if (node.pending && now >= node.deadline)
                    {
                        if (!node.verified) { close(node, "identity_timeout"); continue; }
                        query_failed(node, false);
                    }
                    if (node.verified && !node.pending && node.commands.empty() && now >= node.next_poll) poll(node);
                    if (!node.pending && !node.commands.empty())
                    {
                        node.pending = std::move(node.commands.front());
                        node.commands.pop_front();
                        if (!node.serial.write(node.pending->text + "\n", code))
                        { close(node, "write_failed_" + std::to_string(code)); continue; }
                        node.deadline = Clock::now() + std::chrono::seconds(2);
                    }
                    if (now >= next_flush && !node.storage_failed)
                    {
                        std::string detail;
                        if (!node.journal.flush(&detail))
                        { node.storage_failed = true; node.decoder.clear_duplicates();
                          error("event=storage_failed device=" + node.endpoint.id + " detail=" + detail, true); }
                    }
                }
                if (now >= next_flush) next_flush = now + std::chrono::seconds(2);
                if (now >= next_statistics)
                {
                    TelemetryStatistics snapshot;
                    { std::lock_guard lock(mutex); snapshot = measured; }
                    log(base::LogLevel::Info, "event=statistics connected=" + std::to_string(snapshot.connected_devices) +
                        " bytes=" + std::to_string(snapshot.bytes_received) + " saved=" + std::to_string(snapshot.records_saved) +
                        " duplicates=" + std::to_string(snapshot.duplicate_records) + " rejected=" + std::to_string(snapshot.rejected_lines) +
                        " query_failures=" + std::to_string(snapshot.query_failures) + " storage_failures=" + std::to_string(snapshot.storage_failures) +
                        " telemetry=" + std::to_string(snapshot.telemetry_records_saved) + " snapshots=" + std::to_string(snapshot.snapshots_completed) +
                        " mixed_snapshots=" + std::to_string(snapshot.mixed_snapshots) + " snapshot_retries=" + std::to_string(snapshot.snapshot_retries) +
                        " snapshot_failures=" + std::to_string(snapshot.snapshot_failures));
                    next_statistics = now + std::chrono::seconds(30);
                }
                std::unique_lock lock(mutex);
                wake.wait_for(lock, std::chrono::milliseconds(20), [&] { return stopping.load(); });
            }
        }
        catch (const std::exception &failure)
        { { std::lock_guard lock(mutex); ++measured.worker_failures; } error("event=worker_failed detail=" + std::string(failure.what())); }
        catch (...)
        { { std::lock_guard lock(mutex); ++measured.worker_failures; } error("event=worker_failed detail=unknown"); }
        for (auto &[id, node] : nodes) { (void)id; close(*node, "collector_stopped"); }
        { std::lock_guard lock(mutex); measured.running = false; }
    }
#endif
};

Telemetry::Telemetry(base::Logger *logger) : impl_(std::make_unique<Impl>(logger)) {}
Telemetry::~Telemetry() { stop(); }

bool Telemetry::start(const TelemetryOptions &options, std::string *error)
{
    stop();
    if (options.directory.empty() || options.poll_interval_seconds < 5 || options.poll_interval_seconds > 60 ||
        options.max_bytes < 1024 || options.max_bytes > 1024ULL * 1024 * 1024 ||
        options.max_files < 1 || options.max_files > 32)
    { if (error) *error = "Invalid device telemetry options"; return false; }
#ifdef _WIN32
    try
    {
        impl_->options = options;
        impl_->host_session = std::to_string(epoch_ms()) + "-" + std::to_string(GetCurrentProcessId());
        { std::lock_guard lock(impl_->mutex); impl_->measured = {}; }
        impl_->telemetry_devices.clear();
        impl_->stopping.store(false);
        impl_->worker = std::thread([this] { impl_->run(); });
        impl_->log(base::LogLevel::Info, "event=started output=" + utf8(options.directory.wstring()) +
                   " poll_seconds=" + std::to_string(options.poll_interval_seconds));
        if (error) error->clear();
        return true;
    }
    catch (const std::exception &failure) { stop(); if (error) *error = failure.what(); return false; }
#else
    if (error) *error = "Device serial telemetry is unavailable on this platform";
    return false;
#endif
}

void Telemetry::stop() noexcept
{
    impl_->stopping.store(true);
    impl_->wake.notify_all();
    if (impl_->worker.joinable()) impl_->worker.join();
}

TelemetryStatistics Telemetry::statistics() const
{
    std::lock_guard lock(impl_->mutex);
    return impl_->measured;
}
}
