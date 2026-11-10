#include "dual_end_resource_format.h"
#include <stdio.h>
#include <inttypes.h>
int32_t dual_end_resource_cpu(uint32_t before,uint32_t after,uint64_t window_us)
{
    const uint32_t idle_delta=after-before;
    if(!window_us||window_us>=UINT64_C(4294967296)||idle_delta>window_us)return -1;
    return (int32_t)(1000-((uint64_t)idle_delta*1000/window_us));
}
size_t dual_end_resource_format(const dual_end_resource_snapshot_t *r,unsigned part,char *buf,size_t cap)
{
    if(!r||!buf||!cap||part>=DUAL_END_RESOURCE_PARTS)return 0;
    buf[0]='\0';
    int prefix=snprintf(buf,cap,"{\"event\":\"audio.device_resources\",\"schema\":1,\"boot_id\":\"%08" PRIx32 "\",\"role\":%u,\"seq\":%" PRIu32 ",\"part\":%u",r->boot_id,(unsigned)r->role,r->sequence,part);
    if(prefix<0||(size_t)prefix>=cap){buf[0]='\0';return 0;}
    char *out=buf+prefix;size_t remaining=cap-(size_t)prefix;int n=-1;
    switch(part){
    case 0:n=snprintf(out,remaining,",\"parts\":10,\"sampled_us\":%" PRIu64 "}",r->sampled_us);break;
    case 1:n=snprintf(out,remaining,",\"internal_free\":%" PRIu32 ",\"internal_min\":%" PRIu32 "}",r->internal_free,r->internal_min);break;
    case 2:n=snprintf(out,remaining,",\"internal_largest\":%" PRIu32 ",\"psram_free\":%" PRIu32 "}",r->internal_largest,r->psram_free);break;
    case 3:n=snprintf(out,remaining,",\"psram_min\":%" PRIu32 ",\"psram_largest\":%" PRIu32 "}",r->psram_min,r->psram_largest);break;
    case 4:n=snprintf(out,remaining,",\"logger_stack\":%" PRIu32 ",\"telemetry_drop\":%" PRIu32 "}",r->logger_stack,r->telemetry_drop);break;
    case 5:{char cpu[2][16];for(unsigned i=0;i<2;++i){if(r->cpu_busy_permille[i]<0)snprintf(cpu[i],sizeof(cpu[i]),"null");else snprintf(cpu[i],sizeof(cpu[i]),"%" PRId32,r->cpu_busy_permille[i]);}n=snprintf(out,remaining,",\"cpu0_permille\":%s,\"cpu1_permille\":%s}",cpu[0],cpu[1]);break;}
    case 6:n=snprintf(out,remaining,",\"interval_us\":%" PRIu64 ",\"io_available\":%s}",r->interval_us,r->io_available?"true":"false");break;
    case 7:n=snprintf(out,remaining,",\"io0\":%" PRIu32 ",\"io1\":%" PRIu32 ",\"io2\":%" PRIu32 "}",r->io[0],r->io[1],r->io[2]);break;
    case 8:n=snprintf(out,remaining,",\"io3\":%" PRIu32 ",\"io4\":%" PRIu32 ",\"io5\":%" PRIu32 "}",r->io[3],r->io[4],r->io[5]);break;
    case 9:n=snprintf(out,remaining,",\"io6\":%" PRIu32 ",\"io7\":%" PRIu32 "}",r->io[6],r->io[7]);break;
    default:break;
    }
    if(n<0||(size_t)n>=remaining){buf[0]='\0';return 0;}
    return (size_t)prefix+(size_t)n;
}
