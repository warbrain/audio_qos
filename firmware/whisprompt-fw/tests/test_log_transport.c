#include "transport_test_stubs.h"
#include <stdatomic.h>
#include <assert.h>
#include <setjmp.h>
#include <stdlib.h>
#include <string.h>
#include "sdkconfig.h"
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG && CONFIG_LOG_VERSION == 1
static bool force_format_failure;
static int transport_test_vsnprintf(char *out, size_t cap, const char *fmt, va_list args)
{
    return force_format_failure ? -1 : vsnprintf(out, cap, fmt, args);
}
#define vsnprintf transport_test_vsnprintf
#endif
#include "dual_end_log_transport.c"
#undef vsnprintf

static unsigned allocations,frees,queue_creates,queue_deletes,task_creates,task_deletes;
static unsigned installs,vfs_switches,hook_changes,notifications,writes,old_logs,locks;
static bool allocation_fail,queue_fail,task_fail,install_fail,driver_installed=true,worker_context;
static unsigned allocated_caps,task_caps;
static int task_core;
static unsigned task_priority,task_stack;
static TaskFunction_t worker_function;
static char output[32768];
static size_t output_bytes;
static int plan[32];
static unsigned plan_count,plan_cursor;
static jmp_buf worker_end;
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG && CONFIG_LOG_VERSION == 1
static void native_write_all(const char *);
#endif
static int old_sink(const char *format,va_list args) {
    char buffer[1024];++old_logs;return vsnprintf(buffer,sizeof(buffer),format,args);
}
static vprintf_like_t log_sink=old_sink;
void *heap_caps_malloc(size_t bytes,uint32_t caps) {
    ++allocations;allocated_caps=caps;return allocation_fail?NULL:malloc(bytes);
}
void heap_caps_free(void *data) {assert(data);++frees;free(data);}
QueueHandle_t xQueueCreateStatic(UBaseType_t length,UBaseType_t bytes,uint8_t *storage,StaticQueue_t *control) {
    ++queue_creates;
    if(queue_fail)return NULL;
    *control=(StaticQueue_t){.storage=storage,.length=length,.item_bytes=bytes};return control;
}
BaseType_t xQueueSend(QueueHandle_t queue,const void *record,TickType_t wait) {
    assert(!worker_context && wait==0 && queue);
    if(queue->used==queue->length)return pdFALSE;
    memcpy(queue->storage+queue->write*queue->item_bytes,record,queue->item_bytes);
    queue->write=(queue->write+1U)%queue->length;++queue->used;return pdTRUE;
}
BaseType_t xQueueReceive(QueueHandle_t queue,void *record,TickType_t wait) {
    assert(worker_context && wait==portMAX_DELAY && queue);
    if(!queue->used)longjmp(worker_end,1);
    memcpy(record,queue->storage+queue->read*queue->item_bytes,queue->item_bytes);
    queue->read=(queue->read+1U)%queue->length;--queue->used;return pdTRUE;
}
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t queue) {return queue->used;}
void vQueueDelete(QueueHandle_t queue) {assert(queue);++queue_deletes;}
BaseType_t xTaskCreatePinnedToCoreWithCaps(TaskFunction_t function,const char *name,uint32_t stack,void *arg,
    UBaseType_t priority,TaskHandle_t *task,BaseType_t core,UBaseType_t caps) {
    assert(strcmp(name,"usb_log_tx")==0 && !arg);
    ++task_creates;worker_function=function;task_core=core;task_priority=priority;task_stack=stack;task_caps=caps;
    if(task_fail)return pdFALSE;
    *task=(void *)1;return pdPASS;
}
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t function,const char *name,uint32_t stack,void *arg,
    UBaseType_t priority,TaskHandle_t *task,BaseType_t core) {
    return xTaskCreatePinnedToCoreWithCaps(function,name,stack,arg,priority,task,core,MALLOC_CAP_INTERNAL);
}
void vTaskDeleteWithCaps(TaskHandle_t task) {assert(task==(void *)1);++task_deletes;}
void vTaskDelete(TaskHandle_t task) {vTaskDeleteWithCaps(task);}
uint32_t ulTaskNotifyTake(BaseType_t clear,TickType_t wait) {
    assert(worker_context && clear==pdTRUE && wait==portMAX_DELAY && notifications);return 1;
}
void xTaskNotifyGive(TaskHandle_t task) {assert(task==(void *)1 && log_sink!=old_sink);++notifications;}
vprintf_like_t esp_log_set_vprintf(vprintf_like_t sink) {
    vprintf_like_t previous=log_sink;log_sink=sink;++hook_changes;return previous;
}
bool usb_serial_jtag_is_driver_installed(void) {return driver_installed;}
esp_err_t usb_serial_jtag_driver_install(usb_serial_jtag_driver_config_t *config) {
    ++installs;assert(config->tx_buffer_size==2048 && config->rx_buffer_size==256);
    if(install_fail)return ESP_FAIL;
    driver_installed=true;return ESP_OK;
}
void usb_serial_jtag_vfs_use_driver(void) {assert(driver_installed && log_sink==old_sink);++vfs_switches;}
void transport_test_flockfile(FILE *file) {assert(file==stdout && !locks);++locks;}
void transport_test_funlockfile(FILE *file) {assert(file==stdout && locks==1);--locks;}
int usb_serial_jtag_write_bytes(const void *data,size_t bytes,TickType_t wait) {
    assert(locks==1 && driver_installed && bytes && wait);
    ++writes;
    int result=plan_cursor<plan_count?plan[plan_cursor++]:(int)bytes;
    if(result>0 && (size_t)result<=bytes) {
        assert(output_bytes+(size_t)result<sizeof(output));
        memcpy(output+output_bytes,data,(size_t)result);output_bytes+=(size_t)result;output[output_bytes]='\0';
    }
    return result;
}
static void emit(const char *format,...) {
    unsigned previous_allocations=allocations,previous_writes=writes,previous_locks=locks;
    va_list args;va_start(args,format);(void)log_sink(format,args);va_end(args);
    assert(allocations==previous_allocations && writes==previous_writes && locks==previous_locks);
}
static dual_end_log_transport_stats_t snapshot(void) {
    dual_end_log_transport_stats_t stats;
    unsigned previous_writes=writes,previous_allocations=allocations;
    dual_end_log_transport_snapshot(&stats);dual_end_log_transport_snapshot(NULL);
    assert(writes==previous_writes && allocations==previous_allocations);return stats;
}
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG && CONFIG_LOG_VERSION == 1
static void drain(void) {
    assert(worker_function);worker_context=true;
    if(!setjmp(worker_end))worker_function(NULL);
    worker_context=false;assert(!locks);
}
static void start(void) {
    assert(dual_end_log_transport_start(SCEPTER_ROLE_SCEPTER)==ESP_OK);
    assert(snapshot().enabled && hook_changes==1 && vfs_switches==1 && notifications==1);
    assert(task_core==0 && task_priority==1 && task_stack==4096);
#if CONFIG_SPIRAM
    assert((allocated_caps&MALLOC_CAP_SPIRAM)!=0 && (allocated_caps&MALLOC_CAP_INTERNAL)==0);
    assert(task_caps==(MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
#else
    assert((allocated_caps&MALLOC_CAP_INTERNAL)!=0 && (allocated_caps&MALLOC_CAP_SPIRAM)==0);
    assert(task_caps==MALLOC_CAP_INTERNAL);
#endif
    assert(dual_end_log_transport_start(SCEPTER_ROLE_SCEPTER)==ESP_OK);
    assert(allocations==1 && task_creates==1 && hook_changes==1);
}
static void whole(void) {
    start();assert(!installs); /* Keep existing pairing RX/TX driver intact. */
    for(unsigned i=0;i<DUAL_END_LOG_TRANSPORT_QUEUE_DEPTH+1U;++i)
        emit("I (123) runtime: {\"event\":\"test\",\"part\":%u}\n",i);
    dual_end_log_transport_stats_t stats=snapshot();
    assert(stats.queue_drop==1 && stats.queue_highwater==DUAL_END_LOG_TRANSPORT_QUEUE_DEPTH && !writes);
    /* Existing actual command helper shares stdout's lock without redirecting
     * input or putting token/OTA replies into the optional diagnostic queue. */
    native_write_all("OK TELEMETRY token=77 {\"event\":\"reply\"}\n");
    assert(writes==1);
    drain();assert(writes==DUAL_END_LOG_TRANSPORT_QUEUE_DEPTH+1U);
    assert(strstr(output,"OK TELEMETRY token=77") == output);
    for(unsigned i=0;i<DUAL_END_LOG_TRANSPORT_QUEUE_DEPTH;++i) {
        char line[128];snprintf(line,sizeof(line),"I (123) runtime: {\"event\":\"test\",\"part\":%u}\n",i);
        assert(strstr(output,line));
    }
    char dropped[32];
    snprintf(dropped,sizeof(dropped),"\"part\":%u}",(unsigned)DUAL_END_LOG_TRANSPORT_QUEUE_DEPTH);
    assert(!strstr(output,dropped));
    stats=snapshot();assert(!stats.short_write && !stats.backend_error && stats.queue_drop==1);
}
static void format(void) {
    start();emit("one");emit("two\n");
    char maxline[DUAL_END_LOG_TRANSPORT_LINE_BYTES];
    memset(maxline,'x',sizeof(maxline)-2U);maxline[sizeof(maxline)-2U]='\n';
    maxline[sizeof(maxline)-1U]='\0';emit("%s",maxline);
    maxline[sizeof(maxline)-2U]='x';emit("%s",maxline); /* No space for final LF. */
    char too_long[DUAL_END_LOG_TRANSPORT_LINE_BYTES+44U];
    memset(too_long,'x',sizeof(too_long)-1U);too_long[sizeof(too_long)-1U]='\0';emit("%s",too_long);
    emit("foo\nbar\n");emit("%c",0);emit("");emit(NULL);
    force_format_failure=true;emit("encoding failure");force_format_failure=false;
    dual_end_log_transport_stats_t stats=snapshot();
    assert(stats.format_drop==7 && stats.format_drop_long==2 && stats.format_drop_chars==5);
    assert(stats.format_drop==stats.format_drop_long+stats.format_drop_chars);
    drain();assert(writes==3 && strstr(output,"one\ntwo\n")==output &&
        output_bytes==8U+DUAL_END_LOG_TRANSPORT_LINE_BYTES-1U);
}
static void partial(void) {
    start();plan[0]=3;plan[1]=2;plan_count=2;emit("abcdef\n");drain();
    assert(strcmp(output,"abcdef\n")==0 && writes==3);
    assert(snapshot().short_write==2 && !snapshot().backend_error);
    plan[0]=plan[1]=plan[2]=0;plan_count=3;plan_cursor=0;emit("dropped\n");drain();
    assert(snapshot().short_write==5 && snapshot().backend_error==1);
    plan_count=plan_cursor=0;emit("next\n");drain();
    assert(strcmp(output,"abcdef\nnext\n")==0);
}
static void boundary(void) {
    start();plan[0]=3;plan[1]=plan[2]=plan[3]=0;plan_count=4;
    emit("abcdefgh\n");drain();assert(strcmp(output,"abc")==0);
    assert(s_needs_boundary && snapshot().short_write==4 && snapshot().backend_error==1);
    plan[0]=0;plan_count=1;plan_cursor=0;emit("must_drop\n");drain();
    assert(strcmp(output,"abc")==0 && s_needs_boundary && snapshot().backend_error==2);
    plan_count=plan_cursor=0;emit("final\n");drain();
    assert(strcmp(output,"abc\nfinal\n")==0 && !s_needs_boundary);
    assert(snapshot().short_write==5 && snapshot().backend_error==2 && !snapshot().queue_drop);
}
static void backend(void) {
    start();emit("missing\n");driver_installed=false;drain();
    assert(snapshot().backend_error==1 && !snapshot().short_write && !writes);
    driver_installed=true;plan[0]=-1;plan_count=1;emit("negative\n");drain();
    assert(snapshot().backend_error==2 && snapshot().short_write==1);
    plan[0]=1000;plan_count=1;plan_cursor=0;emit("invalid_length\n");drain();
    assert(snapshot().backend_error==3 && snapshot().short_write==2);
    plan_count=plan_cursor=0;emit("ok\n");drain();assert(strcmp(output,"ok\n")==0);
}
static void saturate(void) {
    start();atomic_store(&s_queue_drop,UINT32_MAX);atomic_store(&s_format_drop,UINT32_MAX);
    atomic_store(&s_short_write,UINT32_MAX);atomic_store(&s_backend_error,UINT32_MAX);
    atomic_store(&s_format_drop_long,UINT32_MAX);atomic_store(&s_format_drop_chars,UINT32_MAX);
    for(unsigned i=0;i<DUAL_END_LOG_TRANSPORT_QUEUE_DEPTH+1U;++i)emit("test\n");
    emit("");emit("%0*u",(int)DUAL_END_LOG_TRANSPORT_LINE_BYTES,1U);
    driver_installed=false;drain();
    dual_end_log_transport_stats_t stats=snapshot();
    assert(stats.queue_drop==UINT32_MAX && stats.format_drop==UINT32_MAX &&
        stats.short_write==UINT32_MAX && stats.backend_error==UINT32_MAX &&
        stats.queue_highwater==DUAL_END_LOG_TRANSPORT_QUEUE_DEPTH &&
        stats.format_drop_long==UINT32_MAX && stats.format_drop_chars==UINT32_MAX);
    driver_installed=true;plan[0]=plan[1]=plan[2]=0;plan_count=3;emit("timeout\n");drain();
    stats=snapshot();assert(stats.short_write==UINT32_MAX && stats.backend_error==UINT32_MAX);
}
static void failed(const char *scenario) {
    if(strcmp(scenario,"alloc_fail")==0)allocation_fail=true;
    if(strcmp(scenario,"queue_fail")==0)queue_fail=true;
    if(strcmp(scenario,"task_fail")==0)task_fail=true;
    if(strcmp(scenario,"driver_fail")==0){driver_installed=false;install_fail=true;}
    esp_err_t result=dual_end_log_transport_start(SCEPTER_ROLE_SCEPTER);
    assert(result==(install_fail?ESP_FAIL:ESP_ERR_NO_MEM));
    assert(!snapshot().enabled && !hook_changes && !vfs_switches && !notifications && log_sink==old_sink);
    emit("old remains\n");assert(old_logs==1);
    assert(dual_end_log_transport_start(SCEPTER_ROLE_SCEPTER)==ESP_ERR_INVALID_STATE);
    if(allocation_fail)assert(!frees && !queue_creates && !task_creates);
    else assert(frees==1);
    if(queue_fail)assert(!queue_deletes && !task_creates);
    if(task_fail)assert(queue_deletes==1 && !task_deletes && !installs);
    if(install_fail)assert(queue_deletes==1 && task_deletes==1 && installs==1 && snapshot().backend_error==1);
    else assert(!snapshot().backend_error && !installs);
}
#endif
static void noop(void) {
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG && CONFIG_LOG_VERSION == 1
    assert(dual_end_log_transport_start(SCEPTER_ROLE_DONGLE)==ESP_OK);
#else
    assert(dual_end_log_transport_start(SCEPTER_ROLE_SCEPTER)==ESP_OK);
    assert(dual_end_log_transport_start(SCEPTER_ROLE_DONGLE)==ESP_OK);
#endif
    dual_end_log_transport_stats_t stats=snapshot();
    assert(!stats.enabled && !stats.queue_drop && !stats.format_drop && !stats.short_write &&
        !stats.backend_error && !stats.queue_highwater);
    assert(!allocations && !task_creates && !hook_changes && !vfs_switches && !installs);
    emit("original UART/Dongle mirror retained\n");assert(log_sink==old_sink && old_logs==1);
}
int main(int argc,char **argv) {
    assert(argc==2);
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG && CONFIG_LOG_VERSION == 1
    if(strcmp(argv[1],"whole")==0)whole();
    else if(strcmp(argv[1],"format")==0)format();
    else if(strcmp(argv[1],"partial")==0)partial();
    else if(strcmp(argv[1],"boundary")==0)boundary();
    else if(strcmp(argv[1],"backend")==0)backend();
    else if(strcmp(argv[1],"saturate")==0)saturate();
    else if(strstr(argv[1],"_fail"))failed(argv[1]);
    else if(strcmp(argv[1],"new_driver")==0){driver_installed=false;start();assert(installs==1);emit("new\n");drain();}
    else if(strcmp(argv[1],"noop")==0)noop();
    else assert(0);
#else
    noop();
#endif
    printf("PASS: %s USB=%u LOG_VERSION=%u PSRAM=%u writes=%u queue_drop=%u format_drop=%u short=%u backend=%u\n",
        argv[1],(unsigned)CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG,(unsigned)CONFIG_LOG_VERSION,
        (unsigned)CONFIG_SPIRAM,writes,(unsigned)snapshot().queue_drop,(unsigned)snapshot().format_drop,
        (unsigned)snapshot().short_write,(unsigned)snapshot().backend_error);
    return 0;
}
