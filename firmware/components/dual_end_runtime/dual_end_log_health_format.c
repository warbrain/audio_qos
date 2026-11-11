#include "dual_end_log_health_format.h"
#include "dual_end_perf_format.h"
#include <inttypes.h>
#include <stdio.h>
size_t dual_end_log_health_format(const dual_end_log_transport_stats_t *s,
    uint32_t boot, unsigned role, uint32_t seq, uint64_t us,
    unsigned part, char *buf, size_t cap)
{
    if (buf && cap) buf[0]='\0';
    if (!s || !buf || !cap || role>1 || part>=DUAL_END_LOG_HEALTH_PARTS) return 0;
    int p=snprintf(buf,cap,"{\"event\":\"audio.log_health\",\"schema\":1,\"part\":%u,"
        "\"boot_id\":\"%08" PRIx32 "\",\"role\":%u,\"seq\":%" PRIu32,
        part,boot,role,seq);
    if (p<0 || (size_t)p>=cap) {buf[0]='\0';return 0;}
    char *out=buf+p; size_t left=cap-(size_t)p; int n=-1;
    switch (part) {
    case 0: n=snprintf(out,left,",\"parts\":5,\"enabled\":%s,\"queue_capacity\":%u}",s->enabled?"true":"false",(unsigned)DUAL_END_LOG_TRANSPORT_QUEUE_DEPTH); break;
    case 1: n=snprintf(out,left,",\"sampled_us\":\"%" PRIu64 "\"}",us); break;
    case 2: n=snprintf(out,left,",\"queue_drop\":%" PRIu32 ",\"format_drop\":%" PRIu32 "}",s->queue_drop,s->format_drop); break;
    case 3: n=snprintf(out,left,",\"short_write\":%" PRIu32 ",\"backend_error\":%" PRIu32 ",\"queue_highwater\":%" PRIu32 "}",s->short_write,s->backend_error,s->queue_highwater); break;
    case 4: n=snprintf(out,left,",\"format_long\":%" PRIu32 ",\"format_chars\":%" PRIu32 "}",s->format_drop_long,s->format_drop_chars); break;
    }
    if(n<0 || (size_t)n>=left || (size_t)p+(size_t)n>DUAL_END_PERF_JSON_MAX_BYTES) {buf[0]='\0';return 0;}
    return (size_t)p+(size_t)n;
}
