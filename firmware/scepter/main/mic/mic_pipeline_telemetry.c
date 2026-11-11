#include "mic_pipeline_telemetry.h"
#include <inttypes.h>
#include <stdio.h>

size_t mic_pipeline_telemetry_format(const mic_pipeline_telemetry_t *r,
    unsigned part, char *out, size_t capacity)
{
    if (out && capacity) out[0] = '\0';
    if (!r || !out || !capacity || !r->boot_id || part >= MIC_PIPELINE_TELEMETRY_PARTS ||
        r->mode > MIC_PIPELINE_SPATIAL_MAX_MODE ||
        (r->codec_known && r->codec != 3 && r->codec != 4 && r->codec != 5)) return 0;
    char codec[8];
    if (r->codec_known) snprintf(codec, sizeof(codec), "%u", (unsigned)r->codec);
    else snprintf(codec, sizeof(codec), "null");
    int prefix = snprintf(out, capacity,
        "{\"event\":\"audio.pipeline\",\"schema\":1,\"boot_id\":\"%08" PRIx32
        "\",\"role\":0,\"session_id\":%" PRIu32 ",\"seq\":%" PRIu32
        ",\"part\":%u,\"c\":%s,\"m\":%u,\"g\":%u",
        r->boot_id, r->session_id, r->sequence, part, codec,
        (unsigned)r->mode, (unsigned)r->generation);
    if (prefix < 0 || (size_t)prefix >= capacity) { out[0] = '\0'; return 0; }
    char *tail = out + prefix; size_t left = capacity - (size_t)prefix; int n = -1;
    if (part >= 4 && part <= 18) {
        const mic_pipeline_metric_t *metrics[] = { &r->q, &r->a, &r->d, &r->s, &r->w };
        const char keys[] = {'q','a','d','s','w'};
        unsigned index = (part - 4) / 3, field = (part - 4) % 3;
        const mic_pipeline_metric_t *m = metrics[index]; char key = keys[index];
        if (field == 0) n = snprintf(tail,left,",\"%c_us\":\"%" PRIu64 "\"}",key,m->us);
        else if (field == 1 && m->n) n = snprintf(tail,left,",\"%c_n\":%" PRIu32 ",\"%c_lo\":%" PRIu32 "}",key,m->n,key,m->lo);
        else if (field == 1) n = snprintf(tail,left,",\"%c_n\":0,\"%c_lo\":null}",key,key);
        else if (m->n) n = snprintf(tail,left,",\"%c_hi\":%" PRIu32 "}",key,m->hi);
        else n = snprintf(tail,left,",\"%c_hi\":null}",key);
    } else switch (part) {
    case 0: n=snprintf(tail,left,",\"part_count\":32}");break;
    case 1: n=snprintf(tail,left,",\"us\":\"%" PRIu64 "\"}",r->sampled_us);break;
    case 2: n=snprintf(tail,left,",\"t0\":\"%" PRIu64 "\"}",r->start_us);break;
    case 3: n=snprintf(tail,left,",\"cg\":%" PRIu32 ",\"win\":%" PRIu32 "}",r->capture_generation,r->window);break;
    case 19: n=snprintf(tail,left,",\"diag_q\":%" PRIu32 "}",r->diag_q);break;
    case 20: n=snprintf(tail,left,",\"diag_b\":%" PRIu32 "}",r->diag_b);break;
    case 21: n=snprintf(tail,left,",\"diag_o\":%" PRIu32 "}",r->diag_o);break;
    case 22: n=snprintf(tail,left,",\"diag_s\":%" PRIu32 "}",r->diag_s);break;
    case 23: if(r->fft_us)n=snprintf(tail,left,",\"fft_us\":\"%" PRIu64 "\"}",r->fft_us);else n=snprintf(tail,left,",\"fft_us\":null}");break;
    case 24: if(r->fft_us && r->sampled_us>=r->fft_us)n=snprintf(tail,left,",\"age\":\"%" PRIu64 "\"}",r->sampled_us-r->fft_us);else n=snprintf(tail,left,",\"age\":null}");break;
    case 25: n=snprintf(tail,left,",\"fft_n\":\"%" PRIu64 "\"}",r->fft_n);break;
    case 26: n=snprintf(tail,left,",\"fft_u\":\"%" PRIu64 "\"}",r->fft_u);break;
    case 27: n=snprintf(tail,left,",\"fft_hi\":%" PRIu32 "}",r->fft_hi);break;
    case 28: n=snprintf(tail,left,",\"win_lost\":%" PRIu32 "}",r->win_lost);break;
    case 29: n=snprintf(tail,left,",\"late\":%" PRIu32 "}",r->late);break;
    case 30: n=snprintf(tail,left,",\"raw_seq\":%" PRIu32 "}",r->raw_seq);break;
    case 31: n=snprintf(tail,left,",\"q_hi_all\":%" PRIu32 "}",r->q_hi_all);break;
    }
    if (n < 0 || (size_t)n >= left || (size_t)prefix + (size_t)n >= MIC_PIPELINE_TELEMETRY_BYTES) {
        out[0]='\0'; return 0; /* Never emit a truncated JSON as success. */
    }
    return (size_t)prefix + (size_t)n;
}
