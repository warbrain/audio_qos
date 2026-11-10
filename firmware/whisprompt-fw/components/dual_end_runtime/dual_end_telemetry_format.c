#include "dual_end_telemetry_format.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>

static int telemetry_prefix(char *out,size_t cap,const char *event,unsigned schema,
    uint32_t boot,unsigned role,uint32_t session,uint32_t sequence,unsigned part)
{
    return snprintf(out,cap,"{\"event\":\"%s\",\"schema\":%u,\"boot_id\":\"%08" PRIx32
        "\",\"role\":%u,\"session_id\":%" PRIu32 ",\"seq\":%" PRIu32 ",\"part\":%u",
        event,schema,boot,role,session,sequence,part);
}
static size_t telemetry_finish(char *buf,size_t cap,int prefix,int fields)
{
    if(prefix<0 || (size_t)prefix>=cap || fields<0 || (size_t)fields>=cap-(size_t)prefix ||
       (size_t)prefix+(size_t)fields>=DUAL_END_TELEMETRY_JSON_BYTES) {
        if(buf&&cap)buf[0]='\0';
        return 0;
    }
    return (size_t)prefix+(size_t)fields;
}
size_t dual_end_spatial_telemetry_format(const dual_end_spatial_telemetry_t *r,
    unsigned part,char *buf,size_t cap)
{
    if(buf&&cap)buf[0]='\0';
    if(!r||!buf||!cap||!r->boot_id||r->role!=0||part>=DUAL_END_SPATIAL_TELEMETRY_PARTS||
       !isfinite(r->delay_samples)||!isfinite(r->correlation)||!isfinite(r->current_mix)||
       (r->codec_known&&r->codec!=3&&r->codec!=4&&r->codec!=5))return 0;
    int prefix=telemetry_prefix(buf,cap,"audio.spatial",2,r->boot_id,r->role,
        r->session_id,r->sequence,part);
    if(prefix<0||(size_t)prefix>=cap){buf[0]='\0';return 0;}
    char *out=buf+prefix;size_t left=cap-(size_t)prefix;int n=-1;
    switch(part){
    case 0:n=snprintf(out,left,",\"part_count\":15,\"sampled_us\":\"%" PRIu64 "\"}",r->sampled_us);break;
    case 1:{char codec[8];if(r->codec_known)snprintf(codec,sizeof(codec),"%u",(unsigned)r->codec);else snprintf(codec,sizeof(codec),"null");n=snprintf(out,left,",\"codec\":%s,\"generation\":%u}",codec,(unsigned)r->generation);break;}
    case 2:n=snprintf(out,left,",\"requested\":%u,\"applied\":%u,\"layout\":%u}",(unsigned)r->requested,(unsigned)r->applied,(unsigned)r->layout);break;
    case 3:n=snprintf(out,left,",\"fallback\":%u,\"polarity_inverted\":%s}",(unsigned)r->fallback,r->polarity_inverted?"true":"false");break;
    case 4:n=snprintf(out,left,",\"aligned_delay_samples\":%.6f,\"correlation\":%.6f}",(double)r->delay_samples,(double)r->correlation);break;
    case 5:if(r->frames)n=snprintf(out,left,",\"current_mix\":%.6f,\"mean_mix\":%.6f}",(double)r->current_mix,(double)r->mix_q16_sum/(double)r->frames/65535.0);else n=snprintf(out,left,",\"current_mix\":%.6f,\"mean_mix\":null}",(double)r->current_mix);break;
    case 6:n=snprintf(out,left,",\"frames\":\"%" PRIu64 "\"}",r->frames);break;
    case 7:n=snprintf(out,left,",\"learned_frames\":\"%" PRIu64 "\"}",r->learned_frames);break;
    case 8:n=snprintf(out,left,",\"frozen_frames\":\"%" PRIu64 "\"}",r->frozen_frames);break;
    case 9:n=snprintf(out,left,",\"mixed_frames\":\"%" PRIu64 "\"}",r->mixed_frames);break;
    case 10:n=snprintf(out,left,",\"nlms_fallback_frames\":\"%" PRIu64 "\"}",r->nlms_fallback_frames);break;
    case 11:n=snprintf(out,left,",\"processing_us\":\"%" PRIu64 "\"}",r->processing_us);break;
    case 12:n=snprintf(out,left,",\"processing_frames\":%" PRIu32 ",\"bounded_updates\":%" PRIu32 "}",r->processing_frames,r->bounded_updates);break;
    case 13:n=snprintf(out,left,",\"processing_max_us\":%" PRIu32 ",\"resets\":%" PRIu32 "}",r->processing_max_us,r->resets);break;
    case 14:n=snprintf(out,left,",\"mix_q16_sum\":\"%" PRIu64 "\"}",r->mix_q16_sum);break;
    }
    return telemetry_finish(buf,cap,prefix,n);
}
size_t dual_end_rx_telemetry_format(const dual_end_rx_telemetry_t *r,
    unsigned part,char *buf,size_t cap)
{
    if(buf&&cap)buf[0]='\0';
    if(!r||!buf||!cap||!r->boot_id||r->role!=1||part>=DUAL_END_RX_TELEMETRY_PARTS||
       (r->codec_known&&r->codec!=3&&r->codec!=4&&r->codec!=5))return 0;
    int prefix=telemetry_prefix(buf,cap,"audio.rx_telemetry",1,r->boot_id,r->role,
        r->session_id,r->sequence,part);
    if(prefix<0||(size_t)prefix>=cap){buf[0]='\0';return 0;}
    char *out=buf+prefix;size_t left=cap-(size_t)prefix;int n=-1;
    switch(part){
    case 0:n=snprintf(out,left,",\"part_count\":6,\"sampled_us\":\"%" PRIu64 "\"}",r->sampled_us);break;
    case 1:{char codec[8];if(r->codec_known)snprintf(codec,sizeof(codec),"%u",(unsigned)r->codec);else snprintf(codec,sizeof(codec),"null");n=snprintf(out,left,",\"codec\":%s,\"active\":%s}",codec,r->active?"true":"false");break;}
    case 2:n=snprintf(out,left,",\"snapshot_dropped\":%" PRIu32 ",\"event_dropped\":%" PRIu32 "}",r->snapshot_dropped,r->event_dropped);break;
    case 3:n=snprintf(out,left,",\"snapshot_age_us\":\"%" PRIu64 "\"}",r->snapshot_age_us);break;
    case 4:n=snprintf(out,left,",\"rx_stack_min_bytes\":%" PRIu32 "}",r->rx_stack_min_bytes);break;
    case 5:n=snprintf(out,left,",\"telemetry_stack_min_bytes\":%" PRIu32 "}",r->telemetry_stack_min_bytes);break;
    }
    return telemetry_finish(buf,cap,prefix,n);
}
