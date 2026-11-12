#include "dual_end_mvdr.h"

#include <math.h>
#include <string.h>
#include <limits.h>

#define PI_F 3.14159265358979323846f
#define POWER_FLOOR 1.0e-10f
#define COVARIANCE_FLOOR 1.0e-8f
#define NOISE_DOWN_ALPHA 0.90f
#define NOISE_UP_ALPHA 0.995f
#define COVARIANCE_ALPHA 0.94f
#define MASK_THRESHOLD 1.5f
#define DIAGONAL_LOAD 0.05f
#define MAX_WEIGHT_POWER 16.0f
/* -80 dBFS: the previous -54 dBFS cutoff suppressed nearly all Whisper hops
 * in the physical low-noise pilot, so keep only the near-digital-silence guard. */
#define MVDR_MIN_ACTIVE_RMS 0.0001f

enum {
    MVDR_FALLBACK_OK = 0,
    MVDR_FALLBACK_LOW_ENERGY = 1,
    MVDR_FALLBACK_CLIPPED = 3,
    MVDR_FALLBACK_CHANNEL_FAULT = 4,
    MVDR_FALLBACK_WARMUP = 6,
    MVDR_FALLBACK_NUMERIC = 7,
};

typedef struct { float re, im; } complex_f_t;

static float clampf(float value, float low, float high)
{
    return value < low ? low : value > high ? high : value;
}

static bool finite_complex(complex_f_t value)
{
    return isfinite(value.re) && isfinite(value.im);
}

static complex_f_t multiply(complex_f_t a, complex_f_t b)
{
    return (complex_f_t){a.re * b.re - a.im * b.im,
                         a.re * b.im + a.im * b.re};
}

static complex_f_t conjugate(complex_f_t value)
{
    return (complex_f_t){value.re, -value.im};
}

static complex_f_t divide_real(complex_f_t value, float divisor)
{
    return (complex_f_t){value.re / divisor, value.im / divisor};
}

static void fft(float *data, bool inverse)
{
    const size_t n = DUAL_END_MVDR_FFT_SIZE;
    for (size_t i = 1U, j = 0U; i < n; ++i) {
        size_t bit = n >> 1U;
        for (; j & bit; bit >>= 1U) j ^= bit;
        j ^= bit;
        if (i < j) {
            const float re = data[2U * i], im = data[2U * i + 1U];
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
        for (size_t base = 0U; base < n; base += length) {
            float twiddle_re = 1.0f, twiddle_im = 0.0f;
            for (size_t offset = 0U; offset < half; ++offset) {
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
        for (size_t i = 0U; i < 2U * n; ++i) data[i] *= scale;
    }
}

static bool output_push(dual_end_mvdr_t *state, int16_t sample)
{
    if (state->output_count >= DUAL_END_MVDR_OUTPUT_CAPACITY) {
        if (state->queue_overflows != UINT64_MAX) ++state->queue_overflows;
        return false;
    }
    const size_t index = (state->output_head + state->output_count) %
                         DUAL_END_MVDR_OUTPUT_CAPACITY;
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

static void covariance_update(float *xx, float *yy, float *xy_re, float *xy_im,
                              float *weight, float mask,
                              float x_power, float y_power, complex_f_t xy)
{
    const float next_weight = COVARIANCE_ALPHA * *weight +
        (1.0f - COVARIANCE_ALPHA) * mask;
    if (next_weight <= POWER_FLOOR) {
        *weight = next_weight;
        return;
    }
    const float previous = COVARIANCE_ALPHA * *weight;
    const float current = (1.0f - COVARIANCE_ALPHA) * mask;
    *xx = (previous * *xx + current * x_power) / next_weight;
    *yy = (previous * *yy + current * y_power) / next_weight;
    *xy_re = (previous * *xy_re + current * xy.re) / next_weight;
    *xy_im = (previous * *xy_im + current * xy.im) / next_weight;
    *weight = next_weight;
}

static bool mvdr_bin(dual_end_mvdr_t *state, size_t bin,
                     complex_f_t x0, complex_f_t x1, complex_f_t *output)
{
    if (state->speech_weight[bin] < 0.03f)
        return false;

    const float a = fmaxf(state->speech_xx[bin], 0.0f);
    const float b = fmaxf(state->speech_yy[bin], 0.0f);
    const complex_f_t c = {state->speech_xy_re[bin], state->speech_xy_im[bin]};
    const float c_power = c.re * c.re + c.im * c.im;
    const float discriminant = sqrtf(fmaxf((a - b) * (a - b) + 4.0f * c_power, 0.0f));
    const float eigenvalue = 0.5f * (a + b + discriminant);

    complex_f_t d0, d1;
    const float c_magnitude = sqrtf(c_power);
    if (c_magnitude > COVARIANCE_FLOOR) {
        d0 = c;
        d1 = (complex_f_t){eigenvalue - a, 0.0f};
    } else if (a >= b) {
        d0 = (complex_f_t){1.0f, 0.0f};
        d1 = (complex_f_t){0.0f, 0.0f};
    } else {
        d0 = (complex_f_t){0.0f, 0.0f};
        d1 = (complex_f_t){1.0f, 0.0f};
    }
    const float steering_norm = sqrtf(d0.re * d0.re + d0.im * d0.im +
                                      d1.re * d1.re + d1.im * d1.im);
    if (!isfinite(steering_norm) || steering_norm <= COVARIANCE_FLOOR) return false;
    d0 = divide_real(d0, steering_norm);
    d1 = divide_real(d1, steering_norm);

    const float trace = fmaxf(state->noise_xx[bin], 0.0f) +
                        fmaxf(state->noise_yy[bin], 0.0f);
    const float load = fmaxf(trace * DIAGONAL_LOAD, COVARIANCE_FLOOR);
    const float n0 = fmaxf(state->noise_xx[bin], 0.0f) + load;
    const float n1 = fmaxf(state->noise_yy[bin], 0.0f) + load;
    const complex_f_t nc = {state->noise_xy_re[bin], state->noise_xy_im[bin]};
    const float determinant = n0 * n1 - nc.re * nc.re - nc.im * nc.im;
    if (!isfinite(determinant) || determinant <= COVARIANCE_FLOOR * COVARIANCE_FLOOR)
        return false;

    const complex_f_t c_d1 = multiply(nc, d1);
    const complex_f_t cc_d0 = multiply(conjugate(nc), d0);
    const complex_f_t u0 = divide_real((complex_f_t){n1 * d0.re - c_d1.re,
                                                       n1 * d0.im - c_d1.im}, determinant);
    const complex_f_t u1 = divide_real((complex_f_t){n0 * d1.re - cc_d0.re,
                                                       n0 * d1.im - cc_d0.im}, determinant);
    const complex_f_t d0_u0 = multiply(conjugate(d0), u0);
    const complex_f_t d1_u1 = multiply(conjugate(d1), u1);
    const float denominator = d0_u0.re + d1_u1.re;
    if (!finite_complex(u0) || !finite_complex(u1) || !isfinite(denominator) ||
        denominator <= COVARIANCE_FLOOR) return false;

    const complex_f_t w0 = divide_real(u0, denominator);
    const complex_f_t w1 = divide_real(u1, denominator);
    const float weight_power = w0.re * w0.re + w0.im * w0.im +
                               w1.re * w1.re + w1.im * w1.im;
    if (!finite_complex(w0) || !finite_complex(w1) || !isfinite(weight_power) ||
        weight_power > MAX_WEIGHT_POWER) return false;

    const complex_f_t y0 = multiply(conjugate(w0), x0);
    const complex_f_t y1 = multiply(conjugate(w1), x1);
    *output = (complex_f_t){y0.re + y1.re, y0.im + y1.im};
    return finite_complex(*output);
}

static bool process_hop(dual_end_mvdr_t *state)
{
    float rms[2] = {0.0f, 0.0f};
    int peak[2] = {0, 0};
    bool clipped = false;
    for (size_t channel = 0U; channel < 2U; ++channel) {
        for (size_t i = 0U; i < DUAL_END_MVDR_FFT_SIZE; ++i) {
            const float sample = state->frame[channel][i];
            const float pcm = sample * 32768.0f;
            const int magnitude = (int)fabsf(pcm);
            if (magnitude > peak[channel]) peak[channel] = magnitude;
            if (magnitude >= 32700) clipped = true;
            rms[channel] += sample * sample;
            state->fft[channel][2U * i] = sample * state->window[i];
            state->fft[channel][2U * i + 1U] = 0.0f;
        }
        rms[channel] = sqrtf(rms[channel] / (float)DUAL_END_MVDR_FFT_SIZE);
        fft(state->fft[channel], false);
    }

    const bool left_dead = peak[0] <= 2 && rms[1] >= MVDR_MIN_ACTIVE_RMS;
    const bool right_dead = peak[1] <= 2 && rms[0] >= MVDR_MIN_ACTIVE_RMS;
    const bool low_energy = rms[0] < MVDR_MIN_ACTIVE_RMS &&
                            rms[1] < MVDR_MIN_ACTIVE_RMS;
    uint64_t valid_this_hop = 0U;
    double mask_sum = 0.0;

    for (size_t bin = 0U; bin < DUAL_END_MVDR_BINS; ++bin) {
        float *fft0 = state->fft[0], *fft1 = state->fft[1];
        const complex_f_t x0 = {fft0[2U * bin], fft0[2U * bin + 1U]};
        const complex_f_t x1 = {fft1[2U * bin], fft1[2U * bin + 1U]};
        const float p0 = x0.re * x0.re + x0.im * x0.im;
        const float p1 = x1.re * x1.re + x1.im * x1.im;
        const float power = 0.5f * (p0 + p1);
        const complex_f_t xy = multiply(x0, conjugate(x1));
        if (!isfinite(power) || !finite_complex(xy)) {
            state->fft[0][2U * bin] = state->fft[1][2U * bin];
            state->fft[0][2U * bin + 1U] = state->fft[1][2U * bin + 1U];
            if (state->fallback_bins != UINT64_MAX) ++state->fallback_bins;
            continue;
        }

        float speech_mask = 0.0f;
        if (state->noise_valid) {
            const float floor = fmaxf(state->noise_power[bin], POWER_FLOOR);
            if (power > floor * MASK_THRESHOLD)
                speech_mask = clampf(1.0f - floor * MASK_THRESHOLD / power, 0.0f, 1.0f);
        }
        const float noise_mask = 1.0f - speech_mask;
        mask_sum += speech_mask;

        if (!state->noise_valid) {
            state->noise_power[bin] = fmaxf(power, POWER_FLOOR);
            state->noise_xx[bin] = p0;
            state->noise_yy[bin] = p1;
            state->noise_xy_re[bin] = xy.re;
            state->noise_xy_im[bin] = xy.im;
            state->noise_weight[bin] = 1.0f;
        } else {
            const float floor = fmaxf(state->noise_power[bin], POWER_FLOOR);
            const float alpha = power < floor * MASK_THRESHOLD ?
                NOISE_DOWN_ALPHA : NOISE_UP_ALPHA;
            state->noise_power[bin] = fmaxf(alpha * floor + (1.0f - alpha) * power,
                                              POWER_FLOOR);
        }
        covariance_update(&state->speech_xx[bin], &state->speech_yy[bin],
            &state->speech_xy_re[bin], &state->speech_xy_im[bin],
            &state->speech_weight[bin], speech_mask, p0, p1, xy);
        covariance_update(&state->noise_xx[bin], &state->noise_yy[bin],
            &state->noise_xy_re[bin], &state->noise_xy_im[bin],
            &state->noise_weight[bin], noise_mask, p0, p1, xy);

        complex_f_t y = {0.0f, 0.0f};
        bool valid = !clipped && !low_energy && !left_dead && !right_dead &&
            mvdr_bin(state, bin, x0, x1, &y);
        if (!valid) {
            if (left_dead) y = x1;
            else if (right_dead) y = x0;
            else y = (complex_f_t){0.5f * (x0.re + x1.re), 0.5f * (x0.im + x1.im)};
            if (state->fallback_bins != UINT64_MAX) ++state->fallback_bins;
        } else {
            if (state->valid_bins != UINT64_MAX) ++state->valid_bins;
            ++valid_this_hop;
        }
        state->fft[0][2U * bin] = y.re;
        state->fft[0][2U * bin + 1U] = y.im;
    }
    state->noise_valid = true;
    state->speech_mask_q15 = (uint16_t)clampf(
        (float)(mask_sum * 32767.0 / (double)DUAL_END_MVDR_BINS), 0.0f, 32767.0f);

    for (size_t bin = 1U; bin < DUAL_END_MVDR_FFT_SIZE / 2U; ++bin) {
        const size_t mirror = DUAL_END_MVDR_FFT_SIZE - bin;
        state->fft[0][2U * mirror] = state->fft[0][2U * bin];
        state->fft[0][2U * mirror + 1U] = -state->fft[0][2U * bin + 1U];
    }
    state->fft[0][1U] = 0.0f;
    state->fft[0][DUAL_END_MVDR_FFT_SIZE + 1U] = 0.0f;
    fft(state->fft[0], true);

    for (size_t i = 0U; i < DUAL_END_MVDR_FFT_SIZE; ++i)
        state->synthesis[i] = state->fft[0][2U * i] * state->window[i];

    if (state->processed_hops != UINT64_MAX) ++state->processed_hops;
    if (valid_this_hop == 0U && state->fallback_hops != UINT64_MAX)
        ++state->fallback_hops;

    if (state->first_frame) {
        memcpy(state->overlap, state->synthesis + DUAL_END_MVDR_HOP_SIZE,
               sizeof(state->overlap));
        state->first_frame = false;
        state->fallback = MVDR_FALLBACK_WARMUP;
        return true;
    }
    for (size_t i = 0U; i < DUAL_END_MVDR_HOP_SIZE; ++i) {
        const float norm = fmaxf(state->overlap_norm[i], POWER_FLOOR);
        const float sample = (state->overlap[i] + state->synthesis[i]) / norm * 32768.0f;
        if (!output_push(state, pcm16(sample))) return false;
        state->overlap[i] = state->synthesis[DUAL_END_MVDR_HOP_SIZE + i];
    }
    state->fallback = clipped ? MVDR_FALLBACK_CLIPPED :
        (left_dead || right_dead) ? MVDR_FALLBACK_CHANNEL_FAULT :
        low_energy ? MVDR_FALLBACK_LOW_ENERGY :
        valid_this_hop ? MVDR_FALLBACK_OK : MVDR_FALLBACK_NUMERIC;
    return true;
}

void dual_end_mvdr_reset(dual_end_mvdr_t *state)
{
    if (!state) return;
    memset(state, 0, sizeof(*state));
    for (size_t i = 0U; i < DUAL_END_MVDR_FFT_SIZE; ++i)
        state->window[i] = 0.5f - 0.5f * cosf(2.0f * PI_F * (float)i /
                                             (float)DUAL_END_MVDR_FFT_SIZE);
    for (size_t i = 0U; i < DUAL_END_MVDR_HOP_SIZE; ++i) {
        const float first = state->window[i];
        const float second = state->window[i + DUAL_END_MVDR_HOP_SIZE];
        state->overlap_norm[i] = first * first + second * second;
    }
    state->output_count = 2U * DUAL_END_MVDR_HOP_SIZE;
    state->first_frame = true;
    state->fallback = MVDR_FALLBACK_WARMUP;
}

bool dual_end_mvdr_process(dual_end_mvdr_t *state, int16_t *interleaved,
                           size_t frames)
{
    if (!state || !interleaved || frames != 320U) return false;
    for (size_t i = 0U; i < frames; ++i) {
        state->pending[0][state->pending_count] = (float)interleaved[2U * i] / 32768.0f;
        state->pending[1][state->pending_count] = (float)interleaved[2U * i + 1U] / 32768.0f;
        if (++state->pending_count != DUAL_END_MVDR_HOP_SIZE) continue;

        for (size_t channel = 0U; channel < 2U; ++channel) {
            memmove(state->frame[channel], state->frame[channel] + DUAL_END_MVDR_HOP_SIZE,
                    DUAL_END_MVDR_HOP_SIZE * sizeof(state->frame[channel][0]));
            memcpy(state->frame[channel] + DUAL_END_MVDR_HOP_SIZE,
                   state->pending[channel], sizeof(state->pending[channel]));
        }
        state->pending_count = 0U;
        if (!process_hop(state)) return false;
    }

    for (size_t i = 0U; i < frames; ++i) {
        int16_t sample = 0;
        if (state->output_count) {
            sample = state->output[state->output_head];
            state->output_head = (uint16_t)((state->output_head + 1U) %
                                             DUAL_END_MVDR_OUTPUT_CAPACITY);
            --state->output_count;
        } else if (state->queue_underflows != UINT64_MAX) {
            ++state->queue_underflows;
        }
        interleaved[2U * i] = sample;
        interleaved[2U * i + 1U] = sample;
    }
    return true;
}
