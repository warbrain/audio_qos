"""Exercise actual shared runtime ID/drop helpers and publication with two owners.

Host threads model separate per-stage producers; no ESP device or scheduler is
used. A CAS hook also makes the formerly unsafe load/update interleaving exact.
"""
from pathlib import Path
import re

import pytest

from test_audio_recovery_042 import ROOT as REFERENCE_ROOT, _build_host_executable, _run
from test_runtime_codec import function


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / "components/dual_end_runtime"


@pytest.fixture(scope="module")
def runtime_ids(tmp_path_factory):
    build_dir = tmp_path_factory.mktemp("runtime-two-producers")
    source = (RUNTIME / "dual_end_runtime.c").read_text(encoding="utf-8")
    header = (RUNTIME / "include/dual_end_runtime.h").read_text(encoding="utf-8")
    enum_start = header.index("typedef enum {")
    enum_end = header.index("} dual_end_perf_stage_t;", enum_start) + len("} dual_end_perf_stage_t;")
    types_start = source.index("typedef struct {\n    dual_end_perf_record_t record;")
    types_end = source.index("static QueueHandle_t s_snapshots;", types_start)
    counter_declarations = []
    for name in ("s_window_seq", "s_telemetry_drop"):
        declaration = re.search(r"^static atomic_uint " + name + r";\s*$", source, re.MULTILINE)
        assert declaration is not None, f"{name} must remain an atomic shared counter"
        counter_declarations.append(declaration.group(0))
    code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include "dual_end_perf_format.h"
#ifdef _WIN32
#include <windows.h>
#else
#include <pthread.h>
#include <sched.h>
#endif
#define pdTRUE 1
#define ITERATIONS 40000U
#define RECORDS (2U*ITERATIONS)
typedef void *TaskHandle_t;
typedef void *QueueHandle_t;
''' + header[enum_start:enum_end] + "\n" + source[types_start:types_end] + "\n" + "\n".join(counter_declarations) + r'''
static QueueHandle_t s_snapshots=(void *)1;
static unsigned s_role;
static uint32_t s_boot_id=7;
static atomic_uint captured,ready;
static atomic_bool start_gate;
static uint32_t captured_ids[RECORDS],captured_drops[RECORDS];
static uint8_t captured_stages[RECORDS];
static bool queue_fails,inject_window,inject_drop;
static uint32_t nested_id;
static uint32_t runtime_next_window_id(void);
static void runtime_note_telemetry_drop(void);
static bool compare_exchange_with_interleave(atomic_uint *counter,unsigned *expected,
    unsigned desired,memory_order success,memory_order failure){
    assert(success==memory_order_relaxed && failure==memory_order_relaxed);
    if(counter==&s_window_seq && inject_window){
        inject_window=false;
        nested_id=runtime_next_window_id();
    }
    if(counter==&s_telemetry_drop && inject_drop){
        inject_drop=false;
        runtime_note_telemetry_drop();
    }
    return atomic_compare_exchange_weak_explicit(counter,expected,desired,success,failure);
}
/* Only the real production helper CAS calls are intercepted. The hook above
 * retains the real atomic primitive and injects one second producer before it. */
#undef atomic_compare_exchange_weak_explicit
#define atomic_compare_exchange_weak_explicit(c,e,d,s,f) compare_exchange_with_interleave((c),(e),(d),(s),(f))
''' + function(source, "runtime_next_window_id") + "\n" + function(source, "runtime_note_telemetry_drop") + r'''
#undef atomic_compare_exchange_weak_explicit
static int xQueueSend(QueueHandle_t queue,const void *value,unsigned wait){
    assert(queue==s_snapshots && wait==0U);
    const runtime_snapshot_t *snapshot=value;
    assert(snapshot->record.boot_id==7U && snapshot->record.role==0U);
    assert(snapshot->record.stage<=DUAL_END_STAGE_MIC_PIPELINE);
    assert(snapshot->record.stats.samples==1U && snapshot->record.stats.session_id==9U);
    const unsigned index=atomic_fetch_add_explicit(&captured,1U,memory_order_relaxed);
    assert(index<RECORDS);
    captured_ids[index]=snapshot->record.window_seq;
    captured_drops[index]=snapshot->record.telemetry_drop;
    captured_stages[index]=snapshot->record.stage;
    return queue_fails?0:pdTRUE;
}
''' + function(source, "runtime_publish") + r'''
static runtime_window_t window(void){
    runtime_window_t w={.active=true,.started_us=10,.ended_us=20,
        .stats={.codec=4,.session_id=9,.samples=1}};
    return w;
}
static void deterministic(void){
    atomic_store(&s_window_seq,0U);inject_window=true;
    const uint32_t outer=runtime_next_window_id();
    assert(!inject_window && nested_id==1U && outer==2U && atomic_load(&s_window_seq)==2U);
    atomic_store(&s_window_seq,UINT32_MAX-1U);
    assert(runtime_next_window_id()==UINT32_MAX);
    assert(runtime_next_window_id()==1U && runtime_next_window_id()==2U);
    atomic_store(&s_telemetry_drop,UINT32_MAX-1U);inject_drop=true;
    runtime_note_telemetry_drop();
    assert(!inject_drop && atomic_load(&s_telemetry_drop)==UINT32_MAX);
    runtime_note_telemetry_drop();assert(atomic_load(&s_telemetry_drop)==UINT32_MAX);
    /* Capture is made before this publication's rejected queue admission. */
    atomic_store(&s_telemetry_drop,UINT32_MAX-1U);queue_fails=true;
    runtime_window_t w=window();runtime_publish(&w,DUAL_END_STAGE_ENCODE);
    assert(atomic_load(&captured)==1U && captured_drops[0]==UINT32_MAX-1U);
    assert(atomic_load(&s_telemetry_drop)==UINT32_MAX && captured_ids[0]!=0U);
    puts("PASS deterministic competing CAS, nonzero wrap and saturated admission");
}
typedef struct {dual_end_perf_stage_t stage;} producer_arg_t;
static void yield_thread(void){
#ifdef _WIN32
    (void)SwitchToThread();
#else
    (void)sched_yield();
#endif
}
static void produce(const producer_arg_t *arg){
    runtime_window_t own=window();
    atomic_fetch_add_explicit(&ready,1U,memory_order_release);
    while(!atomic_load_explicit(&start_gate,memory_order_acquire))yield_thread();
    for(unsigned i=0;i<ITERATIONS;++i)runtime_publish(&own,arg->stage);
}
#ifdef _WIN32
static DWORD WINAPI producer(LPVOID arg){produce(arg);return 0;}
#else
static void *producer(void *arg){produce(arg);return NULL;}
#endif
static int compare_ids(const void *a,const void *b){
    const uint32_t left=*(const uint32_t *)a,right=*(const uint32_t *)b;
    return left<right?-1:(left>right?1:0);
}
static void stress(bool saturated){
    const uint32_t seed=saturated?UINT32_MAX-ITERATIONS+1U:0U;
    atomic_store(&s_window_seq,seed);
    atomic_store(&s_telemetry_drop,saturated?UINT32_MAX-1U:0U);
    queue_fails=saturated;
    producer_arg_t args[2]={{DUAL_END_STAGE_ENCODE},{DUAL_END_STAGE_MIC_PIPELINE}};
#ifdef _WIN32
    HANDLE threads[2];
    for(unsigned i=0;i<2U;++i){threads[i]=CreateThread(NULL,0,producer,&args[i],0,NULL);assert(threads[i]!=NULL);}
#else
    pthread_t threads[2];
    for(unsigned i=0;i<2U;++i)assert(pthread_create(&threads[i],NULL,producer,&args[i])==0);
#endif
    while(atomic_load_explicit(&ready,memory_order_acquire)!=2U)yield_thread();
    atomic_store_explicit(&start_gate,true,memory_order_release);
#ifdef _WIN32
    assert(WaitForMultipleObjects(2,threads,TRUE,30000)==WAIT_OBJECT_0);
    for(unsigned i=0;i<2U;++i)assert(CloseHandle(threads[i]));
#else
    for(unsigned i=0;i<2U;++i)assert(pthread_join(threads[i],NULL)==0);
#endif
    assert(atomic_load(&captured)==RECORDS);
    unsigned stages[2]={0,0};
    for(unsigned i=0;i<RECORDS;++i){
        assert(captured_ids[i]!=0U && captured_stages[i]<2U);
        ++stages[captured_stages[i]];
        if(saturated)assert(captured_drops[i]>=UINT32_MAX-1U);
        else assert(captured_drops[i]==0U);
    }
    assert(stages[0]==ITERATIONS && stages[1]==ITERATIONS);
    qsort(captured_ids,RECORDS,sizeof(captured_ids[0]),compare_ids);
    for(unsigned i=1;i<RECORDS;++i)assert(captured_ids[i]!=captured_ids[i-1U]);
    if(saturated){
        assert(atomic_load(&s_telemetry_drop)==UINT32_MAX);
        assert(atomic_load(&s_window_seq)==ITERATIONS+1U);
    }else{
        assert(atomic_load(&s_telemetry_drop)==0U);
        assert(atomic_load(&s_window_seq)==RECORDS);
    }
    puts(saturated?"PASS two real producers, unique wrapped IDs, saturated drops":"PASS two real producers, unique accepted IDs");
}
int main(int argc,char **argv){
    assert(argc==2);
    if(strcmp(argv[1],"deterministic")==0)deterministic();
    else if(strcmp(argv[1],"accepted")==0)stress(false);
    else if(strcmp(argv[1],"saturated")==0)stress(true);
    else assert(false);
    return 0;
}
'''
    harness = build_dir / "runtime_ids.c"
    harness.write_text(code, encoding="utf-8")
    executable, env = _build_host_executable(
        [harness], build_dir,
        extra_includes=(REFERENCE_ROOT / "components/dual_end_runtime/include",),
    )
    return executable, env, build_dir


@pytest.mark.parametrize("scenario", ["deterministic", "accepted", "saturated"])
def test_actual_runtime_two_producer_identity_and_drop_saturation(runtime_ids, scenario):
    executable, env, build_dir = runtime_ids
    assert "PASS" in _run([str(executable), scenario], cwd=build_dir, env=env).stdout
