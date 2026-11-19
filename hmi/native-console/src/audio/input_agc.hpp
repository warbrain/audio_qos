#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

namespace hmi::audio
{
// Microphone input enhancement applied to the fused mono stream before the
// vocoder, the VAD and the ASR see it.
//
// This mirrors the WhisPrompt client's AudioEnhancementLevel so the same
// microphone produces the same level in the console as it does in the
// shipping product: the firmware runs a fixed 1.0x gain, and the host AGC is
// what brings quiet speech up to the ASR's working level.
enum class AgcLevel
{
    Off,      // no AGC; HMI_ASR_INPUT_GAIN supplies one fixed gain
    Standard, // default
    Strong,
};

struct InputAgcConfig
{
    float target_rms = 0.07f;
    float noise_floor = 0.0008f;
    float min_gain = 1.0f;
    float max_gain = 6.0f;
    float attack = 0.45f;
    float release = 0.08f;
    float peak_limit = 0.92f;
};

// std::nullopt for AgcLevel::Off, matching the client's Option<Self>.
std::optional<InputAgcConfig> input_agc_config(AgcLevel level) noexcept;

bool parse_agc_level(std::string_view text, AgcLevel &out) noexcept;
std::string_view agc_level_name(AgcLevel level) noexcept;

// Fixed gain used when the AGC is off (HMI_ASR_INPUT_GAIN, 1.0 by default).
float environment_manual_gain() noexcept;

struct InputAgcStatistics
{
    std::uint64_t frames = 0;
    std::uint64_t limited_samples = 0;
    float last_gain = 1.0f;
    float min_gain = 1.0f;
    float max_gain = 1.0f;
};

// Stateful AGC + peak limiter. All methods run on the capture thread.
class InputAgc
{
public:
    void reset() noexcept;

    // Boosts or attenuates one block and returns the gain that was applied.
    float process(float *samples, std::size_t frames, const InputAgcConfig &config) noexcept;

    // Manual path for AgcLevel::Off: one fixed gain, clamped to full scale.
    static void apply_gain(float *samples, std::size_t frames, float gain) noexcept;

    InputAgcStatistics statistics() const noexcept { return stats_; }

private:
    float smoothed_gain_ = 1.0f;
    InputAgcStatistics stats_{};
};
}
