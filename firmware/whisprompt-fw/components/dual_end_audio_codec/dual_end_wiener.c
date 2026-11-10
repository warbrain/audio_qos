#include "dual_end_wiener.h"

#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979323846f
#define POWER_FLOOR 1.0e-12f
#define MIN_GAIN 0.15f
#define NOISE_DOWN_ALPHA 0.85f
#define NOISE_UP_ALPHA 0.995f
#define GAIN_SMOOTH_ALPHA 0.70f

_Static_assert((DUAL_END_WIENER_FFT_SIZE & (DUAL_END_WIENER_FFT_SIZE - 1U)) == 0,
               "Wiener FFT size must be a power of two");
_Static_assert(DUAL_END_WIENER_OUTPUT_CAPACITY >= 2U * DUAL_END_WIENER_FFT_SIZE,
               "Wiener output queue must absorb two complete hops");

static float clampf(float value, float low, float high)
{
    return value < low ? low : value > high ? high : value;
}

static void fft(float *data, bool inverse)
{
    const size_t n = DUAL_END_WIENER_FFT_SIZE;
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1U;
        for (; j & bit; bit >>= 1U) j ^= bit;
        j ^= bit;
        if (i < j) {
            float re = data[2U * i], im = data[2U * i + 1U];
            data[2U * i] = data[2U * j];
            data[2U * i + 1U] = data[2U * j + 1U];
            data[2U * j] = re;
            data[2U * j + 1U] = im;
        }
    }

    for (size_t length = 2U; length <= n; length <<= 1U) {
        const float angle = (inverse ? 2.0f : -2.0f) * PI_F / (float)length;
        const float step_re = cosf(angle), step_im = sinf(angle);
        const size_t half = length >> 1U;
        for (size_t base = 0; base < n; base += length) {
            float twiddle_re = 1.0f, twiddle_im = 0.0f;
            for (size_t offset = 0; offset < half; ++offset) {
                const size_t even = base + offset, odd = even + half;
                const float odd_re = data[2U * odd] * twiddle_re -
                                     data[2U * odd + 1U] * twiddle_im;
                const float odd_im = data[2U * odd] * twiddle_im +
                                     data[2U * odd + 1U] * twiddle_re;
                const float even_re = data[2U * even], even_im = data[2U * even + 1U];
                data[2U * even] = even_re + odd_re;
                data[2U * even + 1U] = even_im + odd_im;
                data[2U * odd] = even_re - odd_re;
                data[2U * odd + 1U] = even_im - odd_im;
                const float next_re = twiddle_re * step_re - twiddle_im * step_im;
                twiddle_im = twiddle_re * step_im + twiddle_im * step_re;
                twiddle_re = next_re;
            }
        }
    }

    if (inverse) {
        const float scale = 1.0f / (float)n;
        for (size_t i = 0; i < 2U * n; ++i) data[i] *= scale;
    }
}

static bool output_push(dual_end_wiener_t *state, int16_t sample)
{
    if (state->output_count >= DUAL_END_WIENER_OUTPUT_CAPACITY) {
        ++state->queue_overflows;
        return false;
    }
    const size_t index = (state->output_head + state->output_count) %
                         DUAL_END_WIENER_OUTPUT_CAPACITY;
    state->output[index] = sample;
    ++state->output_count;
    return true;
}

static int16_t pcm16(float value)
{
    if (!isfinite(value)) return 0;
    value = clampf(value, -32768.0f, 32767.0f);
    return (int16_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static bool process_hop(dual_end_wiener_t *state)
{
    for (size_t i = 0; i < DUAL_END_WIENER_FFT_SIZE; ++i) {
        state->fft[2U * i] = state->frame[i] * state->window[i];
        state->fft[2U * i + 1U] = 0.0f;
    }
    fft(state->fft, false);

    for (size_t bin = 0; bin < DUAL_END_WIENER_BINS; ++bin) {
        const float re = state->fft[2U * bin], im = state->fft[2U * bin + 1U];
        const float power = re * re + im * im;
        if (!isfinite(power)) {
            state->gain[bin] = 1.0f;
            continue;
        }
        if (!state->noise_valid) {
            state->noise_power[bin] = power * 0.25f;
        } else {
            const float noise = fmaxf(state->noise_power[bin], POWER_FLOOR);
            /* Fast downward tracking follows quieter noise; slow upward
             * tracking avoids learning a single speech burst as the floor. */
            const float alpha = power < noise * 1.5f ? NOISE_DOWN_ALPHA : NOISE_UP_ALPHA;
            state->noise_power[bin] = alpha * noise + (1.0f - alpha) * power;
        }
        const float residual = fmaxf(power - state->noise_power[bin], 0.0f);
        const float target = power > POWER_FLOOR ? residual / power : 0.0f;
        const float previous = state->noise_valid ? state->gain[bin] : 1.0f;
        state->gain[bin] = clampf(GAIN_SMOOTH_ALPHA * previous +
                                  (1.0f - GAIN_SMOOTH_ALPHA) * target,
                                  MIN_GAIN, 1.0f);
        const float gain = state->gain[bin];
        state->fft[2U * bin] *= gain;
        state->fft[2U * bin + 1U] *= gain;
        if (bin > 0U && bin < DUAL_END_WIENER_FFT_SIZE / 2U) {
            const size_t mirror = DUAL_END_WIENER_FFT_SIZE - bin;
            state->fft[2U * mirror] *= gain;
            state->fft[2U * mirror + 1U] *= gain;
        }
    }
    state->noise_valid = true;
    fft(state->fft, true);

    float synthesized[DUAL_END_WIENER_FFT_SIZE];
    for (size_t i = 0; i < DUAL_END_WIENER_FFT_SIZE; ++i)
        synthesized[i] = state->fft[2U * i] * state->window[i];

    /* Count every completed spectral hop, including the startup-padding hop.
     * This is processing telemetry, not a count of emitted PCM samples. */
    ++state->processed_hops;
    if (state->first_frame) {
        /* The first frame is left padding; retain its valid second half for
         * overlap-add, but do not emit samples before the first input sample. */
        memcpy(state->overlap, synthesized + DUAL_END_WIENER_HOP_SIZE,
               sizeof(state->overlap));
        state->first_frame = false;
        return true;
    }

    for (size_t i = 0; i < DUAL_END_WIENER_HOP_SIZE; ++i) {
        const float norm = fmaxf(state->overlap_norm[i], POWER_FLOOR);
        const float sample = (state->overlap[i] + synthesized[i]) / norm * 32768.0f;
        if (!output_push(state, pcm16(sample))) return false;
        state->overlap[i] = synthesized[DUAL_END_WIENER_HOP_SIZE + i];
    }
    return true;
}

void dual_end_wiener_reset(dual_end_wiener_t *state)
{
    if (!state) return;
    memset(state, 0, sizeof(*state));
    for (size_t i = 0; i < DUAL_END_WIENER_FFT_SIZE; ++i) {
        state->window[i] = 0.5f - 0.5f * cosf(2.0f * PI_F * (float)i /
                                               (float)DUAL_END_WIENER_FFT_SIZE);
    }
    for (size_t i = 0; i < DUAL_END_WIENER_HOP_SIZE; ++i) {
        const float first = state->window[i], second = state->window[i + DUAL_END_WIENER_HOP_SIZE];
        state->overlap_norm[i] = first * first + second * second;
    }
    /* Keep the STFT overlap and 20 ms packet cadence continuous from startup. */
    state->output_count = 2U * DUAL_END_WIENER_HOP_SIZE;
    state->first_frame = true;
}

bool dual_end_wiener_process(dual_end_wiener_t *state, int16_t *interleaved,
                             size_t frames)
{
    if (!state || !interleaved || frames != 320U) return false;
    for (size_t i = 0; i < frames; ++i) {
        state->pending[state->pending_count++] = (float)interleaved[2U * i] / 32768.0f;
        if (state->pending_count != DUAL_END_WIENER_HOP_SIZE) continue;

        memmove(state->frame, state->frame + DUAL_END_WIENER_HOP_SIZE,
                DUAL_END_WIENER_HOP_SIZE * sizeof(state->frame[0]));
        memcpy(state->frame + DUAL_END_WIENER_HOP_SIZE, state->pending,
               sizeof(state->pending));
        state->pending_count = 0;
        if (!process_hop(state)) return false;
    }

    for (size_t i = 0; i < frames; ++i) {
        int16_t sample = 0;
        if (state->output_count) {
            sample = state->output[state->output_head];
            state->output_head = (uint16_t)((state->output_head + 1U) %
                                             DUAL_END_WIENER_OUTPUT_CAPACITY);
            --state->output_count;
        } else {
            ++state->queue_underflows;
        }
        interleaved[2U * i] = sample;
        interleaved[2U * i + 1U] = sample;
    }
    return true;
}
