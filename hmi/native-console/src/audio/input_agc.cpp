#include "audio/input_agc.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>

namespace hmi::audio
{
std::optional<InputAgcConfig> input_agc_config(AgcLevel level) noexcept
{
    switch (level)
    {
    case AgcLevel::Off:
        return std::nullopt;
    case AgcLevel::Strong:
        return InputAgcConfig{0.10f, 0.0008f, 1.0f, 10.0f, 0.55f, 0.10f, 0.95f};
    case AgcLevel::Standard:
    default:
        return InputAgcConfig{};
    }
}

bool parse_agc_level(std::string_view text, AgcLevel &out) noexcept
{
    while (!text.empty() && (text.front() == ' ' || text.front() == '\t')) text.remove_prefix(1);
    while (!text.empty() && (text.back() == ' ' || text.back() == '\t')) text.remove_suffix(1);
    if (text == "off" || text == "0" || text == "false")
    {
        out = AgcLevel::Off;
        return true;
    }
    if (text == "standard" || text == "normal" || text == "default" || text == "1")
    {
        out = AgcLevel::Standard;
        return true;
    }
    if (text == "strong" || text == "high" || text == "2")
    {
        out = AgcLevel::Strong;
        return true;
    }
    return false;
}

std::string_view agc_level_name(AgcLevel level) noexcept
{
    switch (level)
    {
    case AgcLevel::Off: return "off";
    case AgcLevel::Strong: return "strong";
    case AgcLevel::Standard:
    default: return "standard";
    }
}

float environment_manual_gain() noexcept
{
    char *value = nullptr;
    std::size_t size = 0;
    _dupenv_s(&value, &size, "HMI_ASR_INPUT_GAIN");
    const std::unique_ptr<char, decltype(&std::free)> owner(value, &std::free);
    if (!value) return 1.0f;
    char *end = nullptr;
    const float gain = std::strtof(value, &end);
    if (end == value || !std::isfinite(gain) || gain <= 0.0f) return 1.0f;
    return std::clamp(gain, 0.01f, 32.0f);
}

void InputAgc::reset() noexcept
{
    smoothed_gain_ = 1.0f;
    stats_ = {};
}

float InputAgc::process(float *samples, std::size_t frames, const InputAgcConfig &config) noexcept
{
    if (!samples || frames == 0) return smoothed_gain_;

    double sum = 0.0;
    for (std::size_t index = 0; index < frames; ++index)
    {
        const float sample = samples[index];
        sum += static_cast<double>(sample) * sample;
    }
    const float rms = static_cast<float>(std::sqrt(sum / static_cast<double>(frames)));

    // Below the noise floor the gain returns to unity instead of lifting hiss.
    const float desired = (rms < config.noise_floor || !std::isfinite(rms))
        ? 1.0f
        : std::clamp(config.target_rms / rms, config.min_gain, config.max_gain);

    const float factor = desired > smoothed_gain_ ? config.attack : config.release;
    smoothed_gain_ += (desired - smoothed_gain_) * std::clamp(factor, 0.0f, 1.0f);
    if (!std::isfinite(smoothed_gain_)) smoothed_gain_ = 1.0f;

    const float ceiling = std::max(config.peak_limit, 1.0f / 32768.0f);
    std::uint64_t limited = 0;
    for (std::size_t index = 0; index < frames; ++index)
    {
        float value = samples[index] * smoothed_gain_;
        if (!std::isfinite(value))
        {
            value = 0.0f;
        }
        else if (value > ceiling || value < -ceiling)
        {
            value = std::clamp(value, -ceiling, ceiling);
            ++limited;
        }
        samples[index] = value;
    }

    stats_.frames += frames;
    stats_.limited_samples += limited;
    stats_.last_gain = smoothed_gain_;
    if (stats_.frames == frames)
    {
        stats_.min_gain = smoothed_gain_;
        stats_.max_gain = smoothed_gain_;
    }
    else
    {
        stats_.min_gain = std::min(stats_.min_gain, smoothed_gain_);
        stats_.max_gain = std::max(stats_.max_gain, smoothed_gain_);
    }
    return smoothed_gain_;
}

void InputAgc::apply_gain(float *samples, std::size_t frames, float gain) noexcept
{
    if (!samples || frames == 0) return;
    if (!std::isfinite(gain) || gain <= 0.0f) gain = 1.0f;
    if ((gain - 1.0f) < 1.0e-6f && (1.0f - gain) < 1.0e-6f) return;
    for (std::size_t index = 0; index < frames; ++index)
    {
        const float value = samples[index] * gain;
        samples[index] = std::isfinite(value) ? std::clamp(value, -1.0f, 1.0f) : 0.0f;
    }
}
}
