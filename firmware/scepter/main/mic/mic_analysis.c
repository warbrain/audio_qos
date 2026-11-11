#include "mic_analysis.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#define MIC_ANALYSIS_CHANNELS 2
#define MIC_ANALYSIS_SAMPLE_RATE_HZ 48000
#define MIC_ANALYSIS_FFT_SIZE 1024
#define MIC_ANALYSIS_FFT_FLOATS (MIC_ANALYSIS_FFT_SIZE * 2)
#define MIC_ANALYSIS_NYQUIST_BIN (MIC_ANALYSIS_FFT_SIZE / 2)
#define MIC_ANALYSIS_SPECTRAL_STORAGE (MIC_ANALYSIS_NYQUIST_BIN + 4)
#define MIC_ANALYSIS_CAL_FIR_TAPS 5
#define MIC_ANALYSIS_GCC_LOW_HZ 300
#define MIC_ANALYSIS_GCC_HIGH_HZ 7000
#define MIC_ANALYSIS_COHERENCE_HIGH_HZ 4000
/* The production array is 20 mm.  Its acoustic limit is about 2.8 samples
 * at 48 kHz; searching +/-3 samples enforces the physical boundary. */
#define MIC_ANALYSIS_MAX_TDOA_SAMPLES 3
#define MIC_ANALYSIS_CLIP_THRESHOLD 32700
#define MIC_ANALYSIS_SPECTRAL_EMA_ALPHA 0.80f
#define MIC_ANALYSIS_PI 3.14159265358979323846f

_Static_assert(MIC_ANALYSIS_INPUT_FRAMES < MIC_ANALYSIS_FFT_SIZE,
               "analysis frame must fit FFT");

/* Fixed calibration lives only in this parallel analysis path.  A controlled
 * raw 48 kHz fixture measurement is required before introducing differential
 * EQ or delay.  The current safe production calibration therefore preserves
 * matched frequency response and zero relative delay. */
static const float s_calibration_gain[MIC_ANALYSIS_CHANNELS] = {1.0f, 1.0f};
static const float s_calibration_fir[MIC_ANALYSIS_CHANNELS][MIC_ANALYSIS_CAL_FIR_TAPS] = {
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
};
static const int16_t s_hardware_delay_samples_q8_8 = 0;
static const bool s_fixed_calibration_enabled = false;

/* Diagnostics are optional and never borrow scarce INTERNAL/DMA memory.
 * One aligned PSRAM workspace is owned by the diagnostic task until reboot. */
typedef struct {
    _Alignas(16) float hann[MIC_ANALYSIS_INPUT_FRAMES];
    _Alignas(16) float fft_left[MIC_ANALYSIS_FFT_FLOATS];
    _Alignas(16) float fft_right[MIC_ANALYSIS_FFT_FLOATS];
    /* Explicit 516-float storage removes implicit padding; only bins 0..512
     * participate in analysis. No spectrum or FFT dimensions change. */
    _Alignas(16) float cross_ema_re[MIC_ANALYSIS_SPECTRAL_STORAGE];
    _Alignas(16) float cross_ema_im[MIC_ANALYSIS_SPECTRAL_STORAGE];
    _Alignas(16) float left_power_ema[MIC_ANALYSIS_SPECTRAL_STORAGE];
    _Alignas(16) float right_power_ema[MIC_ANALYSIS_SPECTRAL_STORAGE];
} mic_analysis_workspace_t;
#define MIC_ANALYSIS_ALIGNED(member) _Static_assert(offsetof(mic_analysis_workspace_t, member) % 16 == 0, "diagnostic member alignment")
MIC_ANALYSIS_ALIGNED(hann);
MIC_ANALYSIS_ALIGNED(fft_left);
MIC_ANALYSIS_ALIGNED(fft_right);
MIC_ANALYSIS_ALIGNED(cross_ema_re);
MIC_ANALYSIS_ALIGNED(cross_ema_im);
MIC_ANALYSIS_ALIGNED(left_power_ema);
MIC_ANALYSIS_ALIGNED(right_power_ema);
static mic_analysis_workspace_t *s_workspace;
#define s_hann (s_workspace->hann)
#define s_fft_left (s_workspace->fft_left)
#define s_fft_right (s_workspace->fft_right)
#define s_cross_ema_re (s_workspace->cross_ema_re)
#define s_cross_ema_im (s_workspace->cross_ema_im)
#define s_left_power_ema (s_workspace->left_power_ema)
#define s_right_power_ema (s_workspace->right_power_ema)
static float s_spectral_ema_alpha = MIC_ANALYSIS_SPECTRAL_EMA_ALPHA;
static bool s_spectral_ema_valid;

static float clamp_unit(float value)
{
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

static uint16_t unit_to_q15(float value)
{
    return (uint16_t)(clamp_unit(value) * 32767.0f + 0.5f);
}

static int16_t float_to_q8_8(float value)
{
    float scaled = value * 256.0f;
    if (scaled > 32767.0f) {
        scaled = 32767.0f;
    } else if (scaled < -32768.0f) {
        scaled = -32768.0f;
    }
    return (int16_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static size_t bounded_input_index(int index)
{
    if (index < 0) {
        return 0;
    }
    if (index >= MIC_ANALYSIS_INPUT_FRAMES) {
        return MIC_ANALYSIS_INPUT_FRAMES - 1;
    }
    return (size_t)index;
}

static float calibrated_sample(const int16_t *raw,
                               size_t frame,
                               size_t channel,
                               float mean)
{
    if (!s_fixed_calibration_enabled) {
        return (float)raw[frame * MIC_ANALYSIS_CHANNELS + channel] - mean;
    }

    float value = 0.0f;
    const int centre = MIC_ANALYSIS_CAL_FIR_TAPS / 2;
    for (int tap = 0; tap < MIC_ANALYSIS_CAL_FIR_TAPS; tap++) {
        const size_t source_frame = bounded_input_index((int)frame + tap - centre);
        value += s_calibration_fir[channel][tap] *
                 ((float)raw[source_frame * MIC_ANALYSIS_CHANNELS + channel] - mean);
    }
    return value * s_calibration_gain[channel];
}

/* One complex FFT carries both real microphone signals: z[n]=L[n]+jR[n].
 * The conjugate symmetry identities below recover their independent positive
 * spectra, saving one complete 1024-point FFT every 20 ms. */
static void unpack_two_real_spectra(void)
{
    const float dc_left = s_fft_left[0];
    const float dc_right = s_fft_left[1];
    s_fft_left[0] = dc_left;
    s_fft_left[1] = 0.0f;
    s_fft_right[0] = dc_right;
    s_fft_right[1] = 0.0f;

    for (size_t bin = 1; bin < MIC_ANALYSIS_NYQUIST_BIN; bin++) {
        const size_t mirror = MIC_ANALYSIS_FFT_SIZE - bin;
        const float a_re = s_fft_left[2 * bin];
        const float a_im = s_fft_left[2 * bin + 1];
        const float b_re = s_fft_left[2 * mirror];
        const float b_im = -s_fft_left[2 * mirror + 1];

        s_fft_left[2 * bin] = 0.5f * (a_re + b_re);
        s_fft_left[2 * bin + 1] = 0.5f * (a_im + b_im);
        s_fft_right[2 * bin] = 0.5f * (a_im - b_im);
        s_fft_right[2 * bin + 1] = 0.5f * (b_re - a_re);
    }

    const size_t nyquist = MIC_ANALYSIS_NYQUIST_BIN;
    const float nyquist_left = s_fft_left[2 * nyquist];
    const float nyquist_right = s_fft_left[2 * nyquist + 1];
    s_fft_left[2 * nyquist] = nyquist_left;
    s_fft_left[2 * nyquist + 1] = 0.0f;
    s_fft_right[2 * nyquist] = nyquist_right;
    s_fft_right[2 * nyquist + 1] = 0.0f;
}

static void complex_fft(float *data, bool inverse)
{
    for (size_t i = 1, j = 0; i < MIC_ANALYSIS_FFT_SIZE; i++) {
        size_t bit = MIC_ANALYSIS_FFT_SIZE >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            const float tmp_re = data[2 * i];
            const float tmp_im = data[2 * i + 1];
            data[2 * i] = data[2 * j];
            data[2 * i + 1] = data[2 * j + 1];
            data[2 * j] = tmp_re;
            data[2 * j + 1] = tmp_im;
        }
    }

    for (size_t length = 2; length <= MIC_ANALYSIS_FFT_SIZE; length <<= 1) {
        const float angle = (inverse ? 2.0f : -2.0f) * MIC_ANALYSIS_PI /
                            (float)length;
        const float step_re = cosf(angle);
        const float step_im = sinf(angle);
        const size_t half = length >> 1;
        for (size_t base = 0; base < MIC_ANALYSIS_FFT_SIZE; base += length) {
            float twiddle_re = 1.0f;
            float twiddle_im = 0.0f;
            for (size_t offset = 0; offset < half; offset++) {
                const size_t even = base + offset;
                const size_t odd = even + half;
                const float odd_re = data[2 * odd] * twiddle_re -
                                     data[2 * odd + 1] * twiddle_im;
                const float odd_im = data[2 * odd] * twiddle_im +
                                     data[2 * odd + 1] * twiddle_re;
                const float even_re = data[2 * even];
                const float even_im = data[2 * even + 1];
                data[2 * even] = even_re + odd_re;
                data[2 * even + 1] = even_im + odd_im;
                data[2 * odd] = even_re - odd_re;
                data[2 * odd + 1] = even_im - odd_im;

                const float next_re = twiddle_re * step_re - twiddle_im * step_im;
                twiddle_im = twiddle_re * step_im + twiddle_im * step_re;
                twiddle_re = next_re;
            }
        }
    }

    if (inverse) {
        const float scale = 1.0f / (float)MIC_ANALYSIS_FFT_SIZE;
        for (size_t i = 0; i < MIC_ANALYSIS_FFT_FLOATS; i++) {
            data[i] *= scale;
        }
    }
}

static float bin_frequency_hz(size_t bin)
{
    const size_t folded = bin <= MIC_ANALYSIS_NYQUIST_BIN ?
                          bin : MIC_ANALYSIS_FFT_SIZE - bin;
    return (float)folded * MIC_ANALYSIS_SAMPLE_RATE_HZ /
           (float)MIC_ANALYSIS_FFT_SIZE;
}

static float correlation_abs_at_lag(int lag)
{
    const size_t index = lag >= 0 ? (size_t)lag :
                         MIC_ANALYSIS_FFT_SIZE - (size_t)(-lag);
    return fabsf(s_fft_left[2 * index]);
}

esp_err_t mic_analysis_init(void)
{
    if (!s_workspace) {
        s_workspace = heap_caps_aligned_alloc(16, sizeof(*s_workspace),
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_workspace) return ESP_ERR_NO_MEM;
    }
    for (size_t i = 0; i < MIC_ANALYSIS_INPUT_FRAMES; i++) {
        s_hann[i] = 0.5f - 0.5f *
                    cosf(2.0f * MIC_ANALYSIS_PI * (float)i /
                         (float)(MIC_ANALYSIS_INPUT_FRAMES - 1));
    }
    mic_analysis_reset();
    return ESP_OK;
}

void mic_analysis_release(void)
{
    /* Startup-only failure cleanup; never free an active task's workspace. */
    free(s_workspace);
    s_workspace = NULL;
    s_spectral_ema_valid = false;
}

void mic_analysis_set_frame_gap(uint32_t frame_gap)
{
    /* Preserve the original 20 ms EMA time constant across sampled frames.
     * No intermediate audio is synthesized or treated as continuously analysed. */
    s_spectral_ema_alpha = powf(MIC_ANALYSIS_SPECTRAL_EMA_ALPHA,
                               (float)(frame_gap ? frame_gap : 1U));
}

void mic_analysis_reset(void)
{
    s_spectral_ema_valid = false;
    s_spectral_ema_alpha = MIC_ANALYSIS_SPECTRAL_EMA_ALPHA;
    if (!s_workspace) return;
    memset(s_cross_ema_re, 0, sizeof(s_cross_ema_re));
    memset(s_cross_ema_im, 0, sizeof(s_cross_ema_im));
    memset(s_left_power_ema, 0, sizeof(s_left_power_ema));
    memset(s_right_power_ema, 0, sizeof(s_right_power_ema));
    s_spectral_ema_valid = false;
}

uint32_t mic_analysis_process_frame(
    const int16_t *interleaved_raw,
    size_t frame_count,
    bool motor_active,
    dual_end_audio_analysis_metadata_t *metadata)
{
    const int64_t start_us = esp_timer_get_time();
    if (!s_workspace || !interleaved_raw || frame_count != MIC_ANALYSIS_INPUT_FRAMES || !metadata) {
        if (metadata) {
            memset(metadata, 0, sizeof(*metadata));
        }
        return 0;
    }

    memset(metadata, 0, sizeof(*metadata));
    metadata->version = DUAL_END_AUDIO_ANALYSIS_VERSION;
    metadata->flags = s_fixed_calibration_enabled ?
                      DUAL_END_AUDIO_ANALYSIS_FLAG_CALIBRATED : 0;

    /* PCM16 sums/squares over 960 samples fit below 2^40. The former double
     * sums were exact too; integer accumulation avoids software-double work,
     * then converts at the unchanged mean/RMS division boundary. */
    int64_t sum[MIC_ANALYSIS_CHANNELS] = {0, 0};
    uint64_t sum_sq[MIC_ANALYSIS_CHANNELS] = {0, 0};
    float peak[MIC_ANALYSIS_CHANNELS] = {0.0f, 0.0f};
    uint16_t clip_count[MIC_ANALYSIS_CHANNELS] = {0, 0};
    for (size_t frame = 0; frame < frame_count; frame++) {
        for (size_t channel = 0; channel < MIC_ANALYSIS_CHANNELS; channel++) {
            const int16_t sample = interleaved_raw[frame * MIC_ANALYSIS_CHANNELS + channel];
            const float magnitude = fabsf((float)sample);
            sum[channel] += sample;
            const int32_t wide_sample = sample;
            sum_sq[channel] += (uint64_t)(wide_sample * wide_sample);
            if (magnitude > peak[channel]) {
                peak[channel] = magnitude;
            }
            if (magnitude >= MIC_ANALYSIS_CLIP_THRESHOLD) {
                clip_count[channel]++;
            }
        }
    }

    const float mean[MIC_ANALYSIS_CHANNELS] = {
        (float)((double)sum[0] / frame_count),
        (float)((double)sum[1] / frame_count),
    };
    const float rms[MIC_ANALYSIS_CHANNELS] = {
        sqrtf((float)((double)sum_sq[0] / frame_count)),
        sqrtf((float)((double)sum_sq[1] / frame_count)),
    };

    memset(s_fft_left, 0, sizeof(s_fft_left));
    for (size_t frame = 0; frame < frame_count; frame++) {
        s_fft_left[2 * frame] = calibrated_sample(interleaved_raw, frame, 0, mean[0]) *
                                s_hann[frame];
        s_fft_left[2 * frame + 1] = calibrated_sample(interleaved_raw, frame, 1, mean[1]) *
                                    s_hann[frame];
    }
    complex_fft(s_fft_left, false);
    unpack_two_real_spectra();

    double voice_energy[MIC_ANALYSIS_CHANNELS] = {0.0, 0.0};
    double high_frequency_energy[MIC_ANALYSIS_CHANNELS] = {0.0, 0.0};
    double motor_energy[MIC_ANALYSIS_CHANNELS] = {0.0, 0.0};
    double coherence_sum = 0.0;
    double coherence_weight = 0.0;
    const float ema_new_weight = 1.0f - s_spectral_ema_alpha;

    for (size_t bin = 1; bin <= MIC_ANALYSIS_NYQUIST_BIN; bin++) {
        const float frequency = bin_frequency_hz(bin);
        const float left_re = s_fft_left[2 * bin];
        const float left_im = s_fft_left[2 * bin + 1];
        const float right_re = s_fft_right[2 * bin];
        const float right_im = s_fft_right[2 * bin + 1];
        const float left_power = left_re * left_re + left_im * left_im;
        const float right_power = right_re * right_re + right_im * right_im;
        /* Right * conj(left): a positive TDOA means the right channel arrives later. */
        const float cross_re = right_re * left_re + right_im * left_im;
        const float cross_im = right_im * left_re - right_re * left_im;

        if (frequency >= MIC_ANALYSIS_GCC_LOW_HZ &&
            frequency <= MIC_ANALYSIS_COHERENCE_HIGH_HZ) {
            if (!s_spectral_ema_valid) {
                s_cross_ema_re[bin] = cross_re;
                s_cross_ema_im[bin] = cross_im;
                s_left_power_ema[bin] = left_power;
                s_right_power_ema[bin] = right_power;
            } else {
                s_cross_ema_re[bin] = s_spectral_ema_alpha *
                                      s_cross_ema_re[bin] + ema_new_weight * cross_re;
                s_cross_ema_im[bin] = s_spectral_ema_alpha *
                                      s_cross_ema_im[bin] + ema_new_weight * cross_im;
                s_left_power_ema[bin] = s_spectral_ema_alpha *
                                        s_left_power_ema[bin] + ema_new_weight * left_power;
                s_right_power_ema[bin] = s_spectral_ema_alpha *
                                         s_right_power_ema[bin] + ema_new_weight * right_power;
            }
            const double denominator = (double)s_left_power_ema[bin] *
                                       s_right_power_ema[bin] + 1.0e-12;
            const double numerator =
                (double)s_cross_ema_re[bin] * s_cross_ema_re[bin] +
                (double)s_cross_ema_im[bin] * s_cross_ema_im[bin];
            const double bin_coherence = numerator / denominator;
            const double weight = sqrt(denominator);
            coherence_sum += (bin_coherence > 1.0 ? 1.0 : bin_coherence) * weight;
            coherence_weight += weight;
        }

        if (frequency >= MIC_ANALYSIS_GCC_LOW_HZ &&
            frequency <= MIC_ANALYSIS_GCC_HIGH_HZ) {
            voice_energy[0] += left_power;
            voice_energy[1] += right_power;
        } else if (frequency >= 8000.0f && frequency <= 20000.0f) {
            high_frequency_energy[0] += left_power;
            high_frequency_energy[1] += right_power;
        }

        if ((frequency >= 850.0f && frequency <= 1150.0f) ||
            (frequency >= 1850.0f && frequency <= 2150.0f) ||
            (frequency >= 2850.0f && frequency <= 3150.0f)) {
            motor_energy[0] += left_power;
            motor_energy[1] += right_power;
        }
    }
    s_spectral_ema_valid = true;

    const float coherence = coherence_weight > 0.0 ?
                            clamp_unit((float)(coherence_sum / coherence_weight)) : 0.0f;
    float hf_ratio[MIC_ANALYSIS_CHANNELS];
    float motor_ratio[MIC_ANALYSIS_CHANNELS];
    for (size_t channel = 0; channel < MIC_ANALYSIS_CHANNELS; channel++) {
        const double spectral_total = voice_energy[channel] +
                                      high_frequency_energy[channel] + 1.0e-12;
        hf_ratio[channel] = clamp_unit((float)(high_frequency_energy[channel] /
                                               spectral_total));
        motor_ratio[channel] = clamp_unit((float)(motor_energy[channel] /
                                                  (voice_energy[channel] + 1.0e-12)));
    }
    const float hf_score = 0.5f * (hf_ratio[0] + hf_ratio[1]);
    const float acoustic_motor_ratio = fmaxf(motor_ratio[0], motor_ratio[1]);
    const float motor_score = motor_active ? 1.0f :
                              clamp_unit((acoustic_motor_ratio - 0.08f) / 0.35f);
    const float crest = fmaxf(peak[0] / fmaxf(rms[0], 1.0f),
                              peak[1] / fmaxf(rms[1], 1.0f));
    const bool high_frequency_glitch =
        (hf_score > 0.28f && crest > 5.5f) ||
        (hf_score > 0.65f && crest > 3.0f);
    const bool acoustic_motor_noise = acoustic_motor_ratio > 0.30f;

    /* Build a conjugate-symmetric PHAT spectrum in the left scratch buffer. */
    for (size_t bin = 0; bin <= MIC_ANALYSIS_NYQUIST_BIN; bin++) {
        const float frequency = bin_frequency_hz(bin);
        const float left_re = s_fft_left[2 * bin];
        const float left_im = s_fft_left[2 * bin + 1];
        const float right_re = s_fft_right[2 * bin];
        const float right_im = s_fft_right[2 * bin + 1];
        const float cross_re = right_re * left_re + right_im * left_im;
        const float cross_im = right_im * left_re - right_re * left_im;
        const float magnitude = sqrtf(cross_re * cross_re + cross_im * cross_im);
        if (frequency >= MIC_ANALYSIS_GCC_LOW_HZ &&
            frequency <= MIC_ANALYSIS_GCC_HIGH_HZ && magnitude > 1.0e-9f) {
            const float phat_re = cross_re / magnitude;
            const float phat_im = cross_im / magnitude;
            s_fft_left[2 * bin] = phat_re;
            s_fft_left[2 * bin + 1] = phat_im;
            if (bin > 0 && bin < MIC_ANALYSIS_NYQUIST_BIN) {
                const size_t mirror = MIC_ANALYSIS_FFT_SIZE - bin;
                s_fft_left[2 * mirror] = phat_re;
                s_fft_left[2 * mirror + 1] = -phat_im;
            }
        } else {
            s_fft_left[2 * bin] = 0.0f;
            s_fft_left[2 * bin + 1] = 0.0f;
            if (bin > 0 && bin < MIC_ANALYSIS_NYQUIST_BIN) {
                const size_t mirror = MIC_ANALYSIS_FFT_SIZE - bin;
                s_fft_left[2 * mirror] = 0.0f;
                s_fft_left[2 * mirror + 1] = 0.0f;
            }
        }
    }
    complex_fft(s_fft_left, true);

    int best_lag = 0;
    float best_peak = -1.0f;
    for (int lag = -MIC_ANALYSIS_MAX_TDOA_SAMPLES;
         lag <= MIC_ANALYSIS_MAX_TDOA_SAMPLES;
         lag++) {
        const float candidate = correlation_abs_at_lag(lag);
        if (candidate > best_peak) {
            best_peak = candidate;
            best_lag = lag;
        }
    }

    float second_peak = 0.0f;
    for (int lag = -MIC_ANALYSIS_MAX_TDOA_SAMPLES;
         lag <= MIC_ANALYSIS_MAX_TDOA_SAMPLES;
         lag++) {
        if (abs(lag - best_lag) <= 1) {
            continue;
        }
        second_peak = fmaxf(second_peak, correlation_abs_at_lag(lag));
    }

    float fractional_offset = 0.0f;
    if (best_lag > -MIC_ANALYSIS_MAX_TDOA_SAMPLES &&
        best_lag < MIC_ANALYSIS_MAX_TDOA_SAMPLES) {
        const float before = correlation_abs_at_lag(best_lag - 1);
        const float centre = correlation_abs_at_lag(best_lag);
        const float after = correlation_abs_at_lag(best_lag + 1);
        const float denominator = before - 2.0f * centre + after;
        if (fabsf(denominator) > 1.0e-9f) {
            fractional_offset = 0.5f * (before - after) / denominator;
            if (fractional_offset > 0.5f) {
                fractional_offset = 0.5f;
            } else if (fractional_offset < -0.5f) {
                fractional_offset = -0.5f;
            }
        }
    }

    const float peak_ratio = best_peak / fmaxf(second_peak, 1.0e-6f);
    const float peak_ratio_score = clamp_unit((peak_ratio - 1.0f) / 0.75f);
    const float absolute_peak_score = clamp_unit(best_peak * 3.0f);
    const float confidence = clamp_unit(sqrtf(coherence * peak_ratio_score) *
                                        absolute_peak_score);
    const float calibrated_tdoa = (float)best_lag + fractional_offset -
                                  (float)s_hardware_delay_samples_q8_8 / 256.0f;

    float quality[MIC_ANALYSIS_CHANNELS];
    const float max_rms = fmaxf(fmaxf(rms[0], rms[1]), 1.0f);
    for (size_t channel = 0; channel < MIC_ANALYSIS_CHANNELS; channel++) {
        const float relative_level = rms[channel] / max_rms;
        quality[channel] = (0.35f + 0.65f * relative_level) *
                           (0.50f + 0.50f * coherence);
        if (clip_count[channel] > 0) {
            quality[channel] *= fmaxf(0.20f, 1.0f - (float)clip_count[channel] / 10.0f);
        }
        if (high_frequency_glitch) {
            quality[channel] *= 0.75f;
        }
        if (motor_active || acoustic_motor_noise) {
            quality[channel] *= 0.80f;
        }
        quality[channel] = clamp_unit(quality[channel]);
    }

    if (clip_count[0] > 0) {
        metadata->flags |= DUAL_END_AUDIO_ANALYSIS_FLAG_CLIP_LEFT;
    }
    if (clip_count[1] > 0) {
        metadata->flags |= DUAL_END_AUDIO_ANALYSIS_FLAG_CLIP_RIGHT;
    }
    if (high_frequency_glitch) {
        metadata->flags |= DUAL_END_AUDIO_ANALYSIS_FLAG_HF_GLITCH;
    }
    if (motor_active) {
        metadata->flags |= DUAL_END_AUDIO_ANALYSIS_FLAG_MOTOR_ACTIVE;
    }
    if (acoustic_motor_noise) {
        metadata->flags |= DUAL_END_AUDIO_ANALYSIS_FLAG_MOTOR_NOISE;
    }
    if (confidence >= 0.35f) {
        metadata->flags |= DUAL_END_AUDIO_ANALYSIS_FLAG_VALID;
    }
    if (confidence < 0.55f) {
        metadata->flags |= DUAL_END_AUDIO_ANALYSIS_FLAG_LOW_CONFIDENCE;
    }

    metadata->tdoa_samples_q8_8 = float_to_q8_8(calibrated_tdoa);
    metadata->gcc_peak_q15 = unit_to_q15(best_peak);
    metadata->confidence_q15 = unit_to_q15(confidence);
    metadata->coherence_q15 = unit_to_q15(coherence);
    metadata->left_quality_q15 = unit_to_q15(quality[0]);
    metadata->right_quality_q15 = unit_to_q15(quality[1]);
    metadata->high_frequency_q15 = unit_to_q15(hf_score);
    metadata->motor_q15 = unit_to_q15(motor_score);
    metadata->clip_left_count = clip_count[0];
    metadata->clip_right_count = clip_count[1];

    const int64_t elapsed_us = esp_timer_get_time() - start_us;
    return elapsed_us > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed_us;
}
