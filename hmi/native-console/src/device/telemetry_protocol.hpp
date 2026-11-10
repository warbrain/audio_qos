#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace hmi::device
{
enum class SnapshotStatus
{
    Incomplete,
    Complete,
    Mismatch,
    Invalid
};

// Validates one ordered multipart reply without combining its measurements.
// Each new group starts at part zero with a new nonzero request token. A
// mismatched or invalid reply abandons the group; no missing part becomes zero.
class SnapshotValidator
{
public:
    SnapshotStatus observe(std::string_view family, std::uint32_t token,
                           unsigned expected_parts, unsigned part,
                           std::string_view payload);
    void reset() noexcept;

private:
    std::string family_;
    std::string identity_;
    std::uint32_t token_ = 0;
    unsigned expected_parts_ = 0;
    unsigned next_part_ = 0;
};
}
