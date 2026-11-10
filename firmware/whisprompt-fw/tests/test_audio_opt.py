"""0.5.8 actual C behavior; synthetic PCM/SDK mocks, no devices or timing claim."""
from pathlib import Path
from test_runtime_codec import function
from test_audio_recovery_042 import _build_host_executable, _run

ROOT=Path(__file__).resolve().parents[1]
MIC=ROOT/'scepter/main/mic'
CODEC=ROOT/'components/dual_end_audio_codec'

def run_c(tmp_path,code,includes=()):
    harness=tmp_path/'audio_opt.c';harness.write_text(code,encoding='utf-8')
    exe,env=_build_host_executable([harness],tmp_path,extra_includes=includes)
    return _run([str(exe)],cwd=tmp_path,env=env).stdout

def test_actual_sample_scheduler_full_rate_wrap_and_restart(tmp_path):
    source=(MIC/'mic_audio.c').read_text(encoding='utf-8')
    code=r'''
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <stdio.h>
#define MIC_DIAGNOSTIC_SAMPLE_FRAMES 10U
'''+function(source,'mic_diagnostic_sample_due')+r'''
int main(void){
 uint32_t previous=0;bool started=false;unsigned samples=0;
 for(uint32_t seq=1;seq<=100;++seq) {
   bool due=mic_diagnostic_sample_due(seq,&previous,&started);
   assert(due==((seq-1)%10==0));samples+=due;
 }
 assert(samples==10);
 previous=0;started=false;samples=0;
 for(uint32_t i=0;i<100;++i) samples+=mic_diagnostic_sample_due(UINT32_MAX-4+i,&previous,&started);
 assert(samples==10);
 /* New capture generation resets only scheduler state, not audio sequencing. */
 started=false;assert(mic_diagnostic_sample_due(999,&previous,&started));
 assert(!mic_diagnostic_sample_due(1008,&previous,&started));
 assert(mic_diagnostic_sample_due(1009,&previous,&started));
 puts("PASS: 50 Hz audio schedules 5 Hz diagnostic attempts, wrap/restart preserved");return 0;
}
'''
    assert 'PASS' in run_c(tmp_path,code)

def test_actual_optional_workspace_failure_alignment_lifecycle_and_ema_time(tmp_path):
    stubs=tmp_path/'stubs';stubs.mkdir()
    (stubs/'esp_err.h').write_text('#pragma once\ntypedef int esp_err_t;\n#define ESP_OK 0\n#define ESP_ERR_NO_MEM -3\n',encoding='utf-8')
    (stubs/'esp_check.h').write_text('#pragma once\n',encoding='utf-8')
    (stubs/'esp_timer.h').write_text('#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n',encoding='utf-8')
    (stubs/'esp_heap_caps.h').write_text('#pragma once\n#include <stddef.h>\n#define MALLOC_CAP_SPIRAM 4\n#define MALLOC_CAP_8BIT 2\nvoid *heap_caps_aligned_alloc(size_t,size_t,unsigned);\n',encoding='utf-8')
    code=r'''
#include <stdbool.h>
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
_Alignas(16) static unsigned char storage[32768];
static bool fail=true,live=false;static unsigned allocations,frees;
void *heap_caps_aligned_alloc(size_t alignment,size_t bytes,unsigned caps){
 ++allocations;assert(alignment==16 && bytes==28480 && caps==6 && !live);
 if(fail)return NULL;live=true;return storage;
}
static void test_free(void *ptr){if(ptr){assert(ptr==storage && live);live=false;++frees;}}
int64_t esp_timer_get_time(void){return 1234567;}
#define free test_free
'''+ '#include "'+(MIC/'mic_analysis.c').as_posix()+'"\n'+r'''
int main(void){
 int16_t pcm[1920]={0};dual_end_audio_analysis_metadata_t metadata;
 assert(mic_analysis_init()==ESP_ERR_NO_MEM && !s_workspace && allocations==1);
 mic_analysis_reset();
 assert(mic_analysis_process_frame(pcm,960,false,&metadata)==0 && metadata.version==0);
 fail=false;assert(mic_analysis_init()==ESP_OK && allocations==2);
 assert(mic_analysis_init()==ESP_OK && allocations==2); /* no second allocation */
 assert((uintptr_t)s_workspace%16==0 && (uintptr_t)s_fft_left%16==0 && (uintptr_t)s_fft_right%16==0);
 assert((uintptr_t)s_hann%16==0 && (uintptr_t)s_cross_ema_re%16==0 && (uintptr_t)s_right_power_ema%16==0);
 mic_analysis_set_frame_gap(10);assert(fabsf(s_spectral_ema_alpha-powf(.8f,10))<1e-7f);
 mic_analysis_process_frame(pcm,960,false,&metadata);assert(metadata.version==DUAL_END_AUDIO_ANALYSIS_VERSION);
 mic_analysis_set_frame_gap(20);assert(fabsf(s_spectral_ema_alpha-powf(.8f,20))<1e-7f);
 mic_analysis_reset();assert(!s_spectral_ema_valid && s_spectral_ema_alpha==.8f);
 mic_analysis_release();assert(!s_workspace && frees==1 && !live);
 mic_analysis_release();assert(frees==1);
 assert(mic_analysis_process_frame(pcm,960,false,&metadata)==0 && metadata.version==0);
 puts("PASS: PSRAM-only 28480 bytes, aligned members, recoverable allocation failure, one owner, gap-aware EMA");return 0;
}
'''
    assert 'PASS' in run_c(tmp_path,code,(stubs,MIC))

def test_actual_private_pcm_helpers_boundary_roundtrip_and_non16_input(tmp_path):
    code=r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "dual_end_pcm_layout.h"
_Alignas(16) static int16_t channels[2][320];
_Alignas(16) static int16_t raw[642],output[642];
int main(void){
 _Static_assert(sizeof(channels[0])%16==0,"channel stride");
 assert((uintptr_t)channels[0]%16==0 && (uintptr_t)channels[1]%16==0);
 for(unsigned i=0;i<640;++i)raw[i+1]=(int16_t)(i*65537U+32768U);
 output[0]=123;output[641]=456;
 dual_end_pcm_split(raw+1,channels[0],channels[1],320);
 for(unsigned i=0;i<320;++i){assert(channels[0][i]==raw[2*i+1]);assert(channels[1][i]==raw[2*i+2]);}
 dual_end_pcm_interleave(channels[0],channels[1],output+1,320);
 assert(memcmp(raw+1,output+1,1280)==0 && output[0]==123 && output[641]==456);
 dual_end_pcm_split(NULL,NULL,NULL,0);dual_end_pcm_interleave(NULL,NULL,NULL,0);
 puts("PASS: full-range stereo exact roundtrip, 16-byte rows, no public input alignment assumption");return 0;
}
'''
    assert 'PASS' in run_c(tmp_path,code,(CODEC/'include',))

def test_fir_real_storage_stride_preserves_logical_cycle_and_no_pad_access(tmp_path):
    source=(MIC/'mic_audio.c').read_text(encoding='utf-8')
    code=r'''
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#define MIC_CHANNEL_COUNT 2
#define MIC_DECIMATOR_FIR_TAPS 95
#define MIC_DECIMATION_FACTOR 3
static uint16_t s_config[13]={1,0,1,1,0,80,1,7000,0,20000,26000,48,300};
static float s_runtime_hpf=.9895826647f;
static float s_hpf_previous_input[2],s_hpf_previous_output[2];
_Alignas(16) static float s_fir_history[2][96],s_runtime_fir[96];
static size_t s_fir_write_index;static uint8_t s_decimation_phase;
static uint32_t s_soft_limited_frames;static float s_soft_limit_min_gain=1;
'''+function(source,'clamp_i16_from_float')+'\n'+function(source,'mic_shared_soft_limit_gain')+'\n'+function(source,'mic_filter_and_decimate')+r'''
int main(void){
 for(unsigned i=0;i<95;++i)s_runtime_fir[i]=1.0f/95;
 s_fir_history[0][95]=NAN;s_fir_history[1][95]=NAN;s_runtime_fir[95]=NAN;
 int16_t l=0,r=0;
 for(unsigned i=0;i<9500;++i){
   bool output=mic_filter_and_decimate(950,-950,&l,&r);
   assert(output==((i+1)%3==0));assert(s_fir_write_index==(i+1)%95);
   if(output && i>100){assert(l==950 && r==-950);}
 }
 assert(isnan(s_fir_history[0][95]) && isnan(s_fir_history[1][95]) && isnan(s_runtime_fir[95]));
 assert(sizeof(s_fir_history[0])==384 && (uintptr_t)s_fir_history[1]%16==0);
 puts("PASS: 95 logical taps, 96 float stride, pad never participates, exact decimation cadence");return 0;
}
'''
    assert 'PASS' in run_c(tmp_path,code)

def test_unchanged_wire_layout_and_direct_payload_copy_ownership(tmp_path):
    recovery=ROOT/'components/dual_end_audio_recovery'
    code=r'''
#include <assert.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "audio_recovery_tx.h"
#include "audio_recovery_rx.h"
static audio_recovery_tx_t tx;
int main(void){
 dual_end_audio_hardware_payload_t payload={0};uint8_t legacy[320];
 for(unsigned i=0;i<320;++i)payload.audio[i]=(uint8_t)(i*37);
 dual_end_audio_recovery_pack(legacy,&payload);assert(memcmp(legacy,payload.audio,320)==0);
 audio_recovery_tx_reset(&tx,7);
 dual_end_audio_frame_header_t header={0};
 audio_recovery_tx_store(&tx,11,20000,&header,payload.audio);
 memset(payload.audio,0,320);
 const audio_recovery_tx_entry_t *entry=NULL;
 assert(audio_recovery_tx_get(&tx,7,11,21000,&entry)==AUDIO_RECOVERY_TX_HIT);
 assert(memcmp(entry->encoded,legacy,320)==0);
 _Static_assert(DUAL_END_AUDIO_PACKET_BYTES==400,"unchanged normal wire");
 _Static_assert(sizeof(dual_end_audio_repair_packet_t)==350,"unchanged repair wire");
 puts("PASS: direct payload matches pack and cache owns its copy, wire 400/350 unchanged");return 0;
}
'''
    harness=tmp_path/'recovery_copy.c';harness.write_text(code,encoding='utf-8')
    exe,env=_build_host_executable([harness,recovery/'audio_recovery_tx.c'],tmp_path)
    assert 'PASS' in _run([str(exe)],cwd=tmp_path,env=env).stdout
