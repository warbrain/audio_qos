/*
 * win_spatial_wav.c â€” run the production Scepter spatial/Wiener code on Windows.
 *
 * Reads a 16 kHz 16-bit PCM WAV, feeds it through the unmodified firmware
 * sources (dual_end_spatial.c + dual_end_wiener.c) in 20 ms blocks, and writes
 * a WAV back. Used to A/B speech enhancement before it reaches an ASR model.
 *
 * Usage:
 *   win_spatial_wav.exe in.wav out.wav --mode N [--gain F] [--out-ch mono|stereo]
 *
 * Modes (from dual_end_spatial.h):
 *   0 bypass, 1 DAS, 2 NLMS, 3 NLMS stereo, 4 DAS+Wiener, 5 stereo Wiener
 */
#include "dual_end_spatial.h"

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_SAMPLES 320U /* 20 ms at 16 kHz */

static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint16_t rd16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static void wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static void wr16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}

/* Minimal RIFF reader. Returns 0 on success; caller owns *out. */
static int read_wav(const char *path, int16_t **out, size_t *frames,
                    unsigned *channels, unsigned *rate)
{
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); return 1; }
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (size < 44) { fclose(f); return 2; }
    uint8_t *buf = (uint8_t *)malloc((size_t)size);
    if (!buf || fread(buf, 1, (size_t)size, f) != (size_t)size) { fclose(f); free(buf); return 2; }
    fclose(f);

    if (memcmp(buf, "RIFF", 4) != 0 || memcmp(buf + 8, "WAVE", 4) != 0) { free(buf); return 3; }

    unsigned fmt_tag = 0, ch = 0, sr = 0, bits = 0;
    const uint8_t *data = NULL;
    size_t data_bytes = 0;

    size_t pos = 12;
    while (pos + 8 <= (size_t)size) {
        const uint8_t *id = buf + pos;
        uint32_t len = rd32(buf + pos + 4);
        const uint8_t *body = buf + pos + 8;
        if (pos + 8 + len > (size_t)size) break;
        if (memcmp(id, "fmt ", 4) == 0 && len >= 16) {
            fmt_tag = rd16(body);
            ch = rd16(body + 2);
            sr = rd32(body + 4);
            bits = rd16(body + 14);
        } else if (memcmp(id, "data", 4) == 0) {
            data = body;
            data_bytes = len;
        }
        pos += 8 + len + (len & 1U);
    }

    if (fmt_tag != 1 || bits != 16 || (ch != 1 && ch != 2) || sr != 16000 || !data) {
        fprintf(stderr, "unsupported WAV: tag=%u ch=%u rate=%u bits=%u\n", fmt_tag, ch, sr, bits);
        free(buf);
        return 4;
    }

    const size_t n = data_bytes / 2 / ch;
    int16_t *pcm = (int16_t *)malloc(n * sizeof(int16_t) * 2);
    if (!pcm) { free(buf); return 5; }
    for (size_t i = 0; i < n; ++i) {
        const int16_t l = (int16_t)rd16(data + i * 2 * ch);
        const int16_t r = (ch == 2) ? (int16_t)rd16(data + i * 2 * ch + 2) : l;
        pcm[2 * i] = l;
        pcm[2 * i + 1] = r;
    }
    free(buf);
    *out = pcm;
    *frames = n;
    *channels = ch;
    *rate = sr;
    return 0;
}

static int write_wav(const char *path, const int16_t *pcm, size_t frames, unsigned channels)
{
    const uint32_t data_bytes = (uint32_t)(frames * channels * 2);
    uint8_t hdr[44];
    memcpy(hdr, "RIFF", 4);
    wr32(hdr + 4, 36 + data_bytes);
    memcpy(hdr + 8, "WAVEfmt ", 8);
    wr32(hdr + 16, 16);
    wr16(hdr + 20, 1);
    wr16(hdr + 22, (uint16_t)channels);
    wr32(hdr + 24, 16000);
    wr32(hdr + 28, 16000 * channels * 2);
    wr16(hdr + 32, (uint16_t)(channels * 2));
    wr16(hdr + 34, 16);
    memcpy(hdr + 36, "data", 4);
    wr32(hdr + 40, data_bytes);

    FILE *f = fopen(path, "wb");
    if (!f) { fprintf(stderr, "cannot write %s (errno=%d)\n", path, errno); return 1; }
    fwrite(hdr, 1, sizeof(hdr), f);
    fwrite(pcm, 2, frames * channels, f);
    fclose(f);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr,
                "usage: %s in.wav out.wav --mode N [--gain F] [--out-ch mono|stereo]\n"
                "  modes: 0 bypass, 1 DAS, 2 NLMS, 3 NLMS-stereo, 4 DAS+Wiener, 5 stereo Wiener\n",
                argv[0]);
        return 64;
    }
    const char *in_path = argv[1];
    const char *out_path = argv[2];
    long mode = 4;
    double gain = 1.0;
    int out_stereo = 0;

    for (int i = 3; i < argc; ++i) {
        if (strcmp(argv[i], "--mode") == 0 && i + 1 < argc) mode = strtol(argv[++i], NULL, 10);
        else if (strcmp(argv[i], "--gain") == 0 && i + 1 < argc) gain = strtod(argv[++i], NULL);
        else if (strcmp(argv[i], "--out-ch") == 0 && i + 1 < argc) out_stereo = strcmp(argv[++i], "stereo") == 0;
        else { fprintf(stderr, "unknown option %s\n", argv[i]); return 64; }
    }
    if (mode < 0 || mode > 5) { fprintf(stderr, "mode must be 0..5\n"); return 64; }
    if (gain <= 0.0 || gain > 64.0) { fprintf(stderr, "gain must be (0, 64]\n"); return 64; }

    int16_t *in = NULL;
    size_t frames = 0;
    unsigned channels = 0, rate = 0;
    if (read_wav(in_path, &in, &frames, &channels, &rate) != 0) return 2;

    if (gain != 1.0) {
        for (size_t i = 0; i < frames * 2; ++i) {
            double v = (double)in[i] * gain;
            if (v > 32767.0) v = 32767.0;
            if (v < -32768.0) v = -32768.0;
            in[i] = (int16_t)lrint(v);
        }
    }

    dual_end_spatial_t state;
    if (!dual_end_spatial_reset(&state, (uint16_t)mode, 1U)) {
        fprintf(stderr, "spatial reset failed for mode %ld\n", mode);
        free(in);
        return 3;
    }

    const size_t blocks = (frames + FRAME_SAMPLES - 1) / FRAME_SAMPLES;
    int16_t *out = (int16_t *)calloc(blocks * FRAME_SAMPLES * 2, sizeof(int16_t));
    if (!out) { free(in); return 5; }

    for (size_t b = 0; b < blocks; ++b) {
        int16_t block[FRAME_SAMPLES * 2];
        const size_t base = b * FRAME_SAMPLES;
        for (size_t i = 0; i < FRAME_SAMPLES; ++i) {
            if (base + i < frames) {
                block[2 * i] = in[2 * (base + i)];
                block[2 * i + 1] = in[2 * (base + i) + 1];
            } else {
                block[2 * i] = block[2 * i + 1] = 0;
            }
        }
        if (!dual_end_spatial_process(&state, block, FRAME_SAMPLES)) {
            fprintf(stderr, "spatial process failed at block %zu\n", b);
            free(in); free(out);
            return 6;
        }
        memcpy(out + base * 2, block, sizeof(block));
    }

    /* Wiener modes add a 512-sample (32 ms) startup queue. Compensate so the
     * output stays time-aligned with the input for ASR evaluation. */
    const size_t delay = (mode == 4 || mode == 5) ? 512U : 0U;
    const size_t out_ch = out_stereo ? 2U : 1U;
    int16_t *final = (int16_t *)calloc(blocks * FRAME_SAMPLES * out_ch, sizeof(int16_t));
    if (!final) { free(in); free(out); return 5; }
    for (size_t i = 0; i < frames; ++i) {
        const size_t src = i + delay;
        if (src >= blocks * FRAME_SAMPLES) break;
        const int16_t l = out[2 * src];
        const int16_t r = out[2 * src + 1];
        if (out_ch == 2) {
            final[2 * i] = l;
            final[2 * i + 1] = r;
        } else {
            final[i] = (int16_t)(((int32_t)l + (int32_t)r) / 2);
        }
    }

    const int rc = write_wav(out_path, final, frames, (unsigned)out_ch);

    fprintf(stderr,
            "mode=%ld gain=%.3f in_frames=%zu out_ch=%u delay=%zu frames=%llu "
            "fallback=%u corr=%.3f tdoa=%.3f wiener_hops=%llu\n",
            mode, gain, frames, (unsigned)out_ch, delay,
            (unsigned long long)state.stats.frames, state.stats.fallback,
            state.stats.correlation, state.stats.delay_samples,
            (unsigned long long)state.wiener.processed_hops);

    free(in);
    free(out);
    free(final);
    return rc;
}
