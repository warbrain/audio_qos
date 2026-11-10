#include "device/telemetry_journal.hpp"
#include "base/logger.hpp"
#include "util/json_reader.hpp"
#include "util/json_writer.hpp"

#include <fstream>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace hmi::device
{
namespace
{
std::string path_utf8(const std::filesystem::path &path)
{
    const auto text = path.u8string();
    return {reinterpret_cast<const char *>(text.data()), text.size()};
}

bool valid_stem(std::string_view stem)
{
    if (stem.empty() || stem.size() > 96) return false;
    std::string upper;
    upper.reserve(stem.size());
    for (const char ch : stem)
    {
        if ((ch < 'a' || ch > 'z') && (ch < 'A' || ch > 'Z') &&
            (ch < '0' || ch > '9') && ch != '_' && ch != '-') return false;
        upper += ch >= 'a' && ch <= 'z' ? static_cast<char>(ch - ('a' - 'A')) : ch;
    }
    if (upper == "CON" || upper == "PRN" || upper == "AUX" || upper == "NUL") return false;
    if (upper.size() == 4 && (upper.starts_with("COM") || upper.starts_with("LPT")) &&
        upper[3] >= '1' && upper[3] <= '9') return false;
    return true;
}

void require_regular_or_absent(const std::filesystem::path &path)
{
    std::error_code code;
    const auto status = std::filesystem::symlink_status(path, code);
    if (code && code != std::errc::no_such_file_or_directory)
        throw std::runtime_error("cannot inspect device journal: " + path_utf8(path) + ": " + code.message());
    if (!std::filesystem::exists(status)) return;
    if (!std::filesystem::is_regular_file(status))
        throw std::runtime_error("device journal target must be a regular file: " + path_utf8(path));
}

std::string json_quote(std::string_view text)
{
    return util::json::stringify(util::json::Value(std::string(text)));
}

std::string hex_bytes(std::string_view bytes)
{
    constexpr char digits[] = "0123456789abcdef";
    std::string encoded;
    encoded.reserve(bytes.size() * 2);
    for (const unsigned char byte : bytes)
    {
        encoded.push_back(digits[byte >> 4]);
        encoded.push_back(digits[byte & 15]);
    }
    return encoded;
}

base::LogLevel record_level(std::string_view text)
{
    if (text.starts_with("Guru Meditation") || text.starts_with("panic") || text.starts_with("abort("))
        return base::LogLevel::Critical;
    if (text.starts_with("E ") || text.starts_with("E(") || text.starts_with("ERR "))
        return base::LogLevel::Error;
    if (text.starts_with("W ") || text.starts_with("W(")) return base::LogLevel::Warning;
    if (text.starts_with("D ") || text.starts_with("D(")) return base::LogLevel::Debug;
    if (text.starts_with("V ") || text.starts_with("V(")) return base::LogLevel::Trace;
    return base::LogLevel::Info;
}
}

struct TelemetryJournal::Impl
{
    struct State
    {
        JournalOptions options;
        base::Logger text;
        std::ofstream telemetry;
        std::uintmax_t telemetry_bytes = 0;
        std::string failed;

        std::filesystem::path telemetry_path(std::size_t archive = 0) const
        {
            return options.directory / (options.stem + ".telemetry" +
                (archive ? "." + std::to_string(archive) : std::string{}) + ".jsonl");
        }

        void close() noexcept
        {
            try
            {
                text.shutdown();
                if (telemetry.is_open()) { telemetry.flush(); telemetry.close(); }
            }
            catch (...) {}
        }

        ~State() { close(); }

        bool fail(std::string detail, std::string *error)
        {
            if (failed.empty()) failed = std::move(detail);
            if (error) *error = failed;
            return false;
        }

        void recover_incomplete_tail()
        {
            constexpr std::uintmax_t tail_limit = 64 * 1024;
            if (!telemetry_bytes) return;
            if (telemetry_bytes > static_cast<std::uintmax_t>((std::numeric_limits<std::streamoff>::max)()))
                throw std::runtime_error("device telemetry file exceeds the supported seek range");
            const std::uintmax_t read_bytes = telemetry_bytes < tail_limit ? telemetry_bytes : tail_limit;
            const std::uintmax_t offset = telemetry_bytes - read_bytes;
            std::ifstream input(telemetry_path(), std::ios::binary);
            if (!input) throw std::runtime_error("cannot inspect device telemetry tail");
            input.seekg(static_cast<std::streamoff>(offset));
            std::string tail(static_cast<std::size_t>(read_bytes), '\0');
            input.read(tail.data(), static_cast<std::streamsize>(tail.size()));
            if (!input || input.gcount() != static_cast<std::streamsize>(tail.size()))
                throw std::runtime_error("cannot read device telemetry tail");
            input.close();
            if (tail.back() == '\n') return;
            const auto newline = tail.rfind('\n');
            if (newline == std::string::npos && offset != 0)
                throw std::runtime_error("device telemetry has no complete line boundary within its final 64 KiB");
            const auto fragment_start = newline == std::string::npos ? 0 : newline + 1;
            const std::uintmax_t keep_bytes = offset + fragment_start;
            const std::string_view fragment(tail.data() + fragment_start, tail.size() - fragment_start);
            // Hex is lossless even when the interrupted write ends mid-UTF-8
            // or contains NUL/control bytes. Preserve and flush before resizing.
            text.write(base::LogLevel::Warning, options.port,
                "event=telemetry_incomplete_tail preserved_bytes=" + std::to_string(fragment.size()) +
                " file_bytes=" + std::to_string(telemetry_bytes) +
                " complete_bytes=" + std::to_string(keep_bytes) +
                " encoding=hex data=" + hex_bytes(fragment));
            text.flush();
            const auto detail = text.last_error();
            if (!detail.empty()) throw std::runtime_error("cannot preserve incomplete telemetry tail: " + detail);
            require_regular_or_absent(telemetry_path());
            std::error_code code;
            const auto current_bytes = std::filesystem::file_size(telemetry_path(), code);
            if (code || current_bytes != telemetry_bytes)
                throw std::runtime_error("device telemetry changed while preserving its incomplete tail");
            std::filesystem::resize_file(telemetry_path(), keep_bytes, code);
            if (code) throw std::runtime_error("cannot remove preserved incomplete telemetry tail: " + code.message());
            telemetry_bytes = keep_bytes;
        }

        bool rotate(std::size_t incoming, std::string *error)
        {
            if (!telemetry_bytes || (telemetry_bytes < options.max_bytes &&
                incoming <= options.max_bytes - telemetry_bytes)) return true;
            telemetry.flush();
            if (!telemetry) return fail("cannot flush device telemetry before rotation", error);
            telemetry.close();
            if (telemetry.fail()) return fail("cannot close device telemetry before rotation", error);
            // Every rename is restricted to this journal's validated fixed names.
            for (std::size_t index = options.max_files; index > 0; --index)
            {
                const auto target = telemetry_path(index);
                require_regular_or_absent(target);
                const auto source = telemetry_path(index - 1);
                require_regular_or_absent(source);
                std::error_code code;
                if (!std::filesystem::exists(source, code))
                {
                    if (code) return fail("cannot inspect device telemetry archive: " + code.message(), error);
                    continue;
                }
                std::filesystem::remove(target, code);
                if (code) return fail("cannot replace device telemetry archive: " + code.message(), error);
                std::filesystem::rename(source, target, code);
                if (code) return fail("cannot rotate device telemetry: " + code.message(), error);
            }
            telemetry.clear();
            telemetry.open(telemetry_path(), std::ios::binary | std::ios::out | std::ios::app);
            if (!telemetry) return fail("cannot reopen device telemetry after rotation", error);
            telemetry_bytes = 0;
            return true;
        }
    };

    std::unique_ptr<State> state;
};

TelemetryJournal::TelemetryJournal() : impl_(std::make_unique<Impl>()) {}
TelemetryJournal::~TelemetryJournal() { close(); }

bool TelemetryJournal::open(const JournalOptions &options, std::string *error)
{
    try
    {
        if (!valid_stem(options.stem)) throw std::runtime_error("invalid device journal filename stem");
        if (options.directory.empty() || options.directory.native().find(std::filesystem::path::value_type{}) !=
            std::filesystem::path::string_type::npos)
            throw std::runtime_error("device journal directory must be nonempty and contain no NUL");
        if (options.max_bytes < 1024 || options.max_bytes > 1024ULL * 1024 * 1024)
            throw std::runtime_error("device journal max_bytes must be 1024..1073741824");
        if (options.max_files < 1 || options.max_files > 32)
            throw std::runtime_error("device journal max_files must be 1..32");
        if (options.device_id.size() > 1024 || options.port.size() > 1024 || options.host_session.size() > 1024)
            throw std::runtime_error("device journal identity metadata is too long");
        for (const auto &identity : {options.device_id, options.port, options.host_session})
        {
            util::json::Value parsed;
            if (!util::json::parse(json_quote(identity), parsed))
                throw std::runtime_error("device journal identity metadata must be valid UTF-8");
        }

        auto replacement = std::make_unique<Impl::State>();
        replacement->options = options;
        replacement->options.directory = std::filesystem::absolute(options.directory).lexically_normal();
        std::error_code code;
        std::filesystem::create_directories(replacement->options.directory, code);
        if (code) throw std::runtime_error("cannot create device journal directory: " + code.message());
        const auto text_file = replacement->options.directory / (options.stem + ".log");
        require_regular_or_absent(text_file);
        for (std::size_t index = 0; index <= options.max_files; ++index)
        {
            require_regular_or_absent(replacement->telemetry_path(index));
            if (index) require_regular_or_absent(replacement->options.directory /
                (options.stem + "." + std::to_string(index) + ".log"));
        }
        // Flush old handles before inspecting a same-file append transaction.
        if (impl_->state && !impl_->state->failed.empty()) impl_->state.reset();
        if (impl_->state && !flush(error)) return false;
        base::LoggerOptions text_options;
        text_options.file = text_file;
        text_options.level = base::LogLevel::Trace;
        text_options.max_bytes = options.max_bytes;
        text_options.max_files = options.max_files;
        std::string detail;
        if (!replacement->text.configure(text_options, &detail))
            throw std::runtime_error("cannot open device text log: " + detail);
        if (std::filesystem::exists(replacement->telemetry_path(), code))
        {
            replacement->telemetry_bytes = std::filesystem::file_size(replacement->telemetry_path(), code);
            if (code) throw std::runtime_error("cannot inspect device telemetry size: " + code.message());
            replacement->recover_incomplete_tail();
        }
        else if (code) throw std::runtime_error("cannot inspect device telemetry: " + code.message());
        replacement->telemetry.open(replacement->telemetry_path(), std::ios::binary | std::ios::out | std::ios::app);
        if (!replacement->telemetry) throw std::runtime_error("cannot open device telemetry file");
        impl_->state.swap(replacement);
        if (error) error->clear();
        return true;
    }
    catch (const std::exception &failure)
    {
        if (error) *error = failure.what();
    }
    catch (...)
    {
        if (error) *error = "unexpected device journal open failure";
    }
    return false;
}

bool TelemetryJournal::write(const TelemetryRecord &record, std::int64_t received_at_ms,
                             std::uint64_t connection, std::string *error)
{
    try
    {
        if (!impl_->state)
        {
            if (error) *error = "device journal is not open";
            return false;
        }
        auto &state = *impl_->state;
        if (!state.failed.empty()) { if (error) *error = state.failed; return false; }
        if (record.duplicate && !record.recovered_prefix) { if (error) error->clear(); return true; }
        if (record.text.size() > 8192)
        {
            if (error) *error = "device text record exceeds the line limit";
            return false;
        }
        // Validate the boundary once more, but preserve the original numeric text.
        // JSONL must remain one physical line even if a caller bypasses the decoder.
        std::string line;
        if (!record.json.empty() && !record.duplicate)
        {
            util::json::Value parsed;
            if (record.json.size() > 8192 || record.json.find_first_of("\r\n") != std::string::npos ||
                !util::json::parse(record.json, parsed) ||
                !std::holds_alternative<util::json::Value::Object>(parsed.data))
            {
                if (error) *error = "device telemetry payload must be one valid JSON object line";
                return false;
            }
            line = "{\"schema\":1,\"received_at_ms\":" + std::to_string(received_at_ms) +
                ",\"device_id\":" + json_quote(state.options.device_id) +
                ",\"port\":" + json_quote(state.options.port) +
                ",\"host_session\":" + json_quote(state.options.host_session) +
                ",\"connection\":" + std::to_string(connection) +
                ",\"payload\":" + record.json + "}\n";
            if (!state.rotate(line.size(), error)) return false;
            state.telemetry.write(line.data(), static_cast<std::streamsize>(line.size()));
            if (!state.telemetry) return state.fail("cannot write device telemetry", error);
            state.telemetry_bytes += line.size();
        }
        state.text.write(record_level(record.text), state.options.port, record.text);
        const auto detail = state.text.last_error();
        if (!detail.empty()) return state.fail("cannot write device text log: " + detail, error);
        if (error) error->clear();
        return true;
    }
    catch (const std::exception &failure)
    {
        if (impl_->state) return impl_->state->fail(failure.what(), error);
        if (error) *error = failure.what();
    }
    catch (...)
    {
        if (impl_->state) return impl_->state->fail("unexpected device journal write failure", error);
        if (error) *error = "unexpected device journal write failure";
    }
    return false;
}

bool TelemetryJournal::flush(std::string *error)
{
    try
    {
        if (!impl_->state) { if (error) error->clear(); return true; }
        auto &state = *impl_->state;
        if (!state.failed.empty()) { if (error) *error = state.failed; return false; }
        state.telemetry.flush();
        if (!state.telemetry) return state.fail("cannot flush device telemetry", error);
        state.text.flush();
        const auto detail = state.text.last_error();
        if (!detail.empty()) return state.fail("cannot flush device text log: " + detail, error);
        if (error) error->clear();
        return true;
    }
    catch (const std::exception &failure)
    {
        if (impl_->state) return impl_->state->fail(failure.what(), error);
        if (error) *error = failure.what();
    }
    catch (...)
    {
        if (impl_->state) return impl_->state->fail("unexpected device journal flush failure", error);
        if (error) *error = "unexpected device journal flush failure";
    }
    return false;
}

void TelemetryJournal::close() noexcept
{
    impl_->state.reset();
}
}
