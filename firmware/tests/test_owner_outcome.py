"""Actual C counters/formatters. No ESP driver, device or audio action."""
from pathlib import Path
import json
import re
from test_audio_task_affinity import compile_run, run_perf, RUNTIME


def test_actual_outcomes_are_exclusive_saturating_and_legacy_compatible(tmp_path):
    result = run_perf(tmp_path, r'''
    dual_end_perf_stats_t s;
    dual_end_perf_reset(&s,3,99,1);
    for(unsigned i=0;i<DUAL_END_OUTCOME_COUNT;++i)
        assert(dual_end_perf_record_outcome(&s,123,1,(dual_end_owner_outcome_t)i));
    assert(s.samples==8 && s.failures==7 && s.sum_us==984);
    for(unsigned i=0;i<DUAL_END_OUTCOME_COUNT;++i)assert(s.outcomes[i]==1);
    assert(dual_end_perf_record_outcome(&s,1,1,(dual_end_owner_outcome_t)-1));
    assert(s.outcomes[DUAL_END_OUTCOME_UNCLASSIFIED]==2);
    assert(!dual_end_perf_record_outcome(&s,-1,1,DUAL_END_OUTCOME_OK));
    assert(s.samples==9 && s.invalid_duration==1 && s.outcomes[0]==1);
    assert(dual_end_perf_record(&s,1,1,true));
    assert(dual_end_perf_record(&s,1,1,false));
    assert(s.outcomes[0]==2 && s.outcomes[1]==2 && s.samples==11 && s.failures==9);
    s.outcomes[DUAL_END_OUTCOME_WAIT_ACK]=UINT32_MAX;
    assert(dual_end_perf_record_outcome(&s,1,1,DUAL_END_OUTCOME_WAIT_ACK));
    assert(s.outcomes[DUAL_END_OUTCOME_WAIT_ACK]==UINT32_MAX && s.overflow==1);
    unknown(&s);
    s.samples=UINT32_MAX;
    assert(!dual_end_perf_record_outcome(&s,1,1,DUAL_END_OUTCOME_OK));
    assert(s.samples==UINT32_MAX && s.outcomes[0]==2 && s.overflow==2);
    dual_end_perf_reset(&s,4,100,1);
    for(unsigned i=0;i<DUAL_END_OUTCOME_COUNT;++i)assert(s.outcomes[i]==0);
    puts("outcomes exclusive/legacy/saturation PASS");
    ''')
    assert "outcomes exclusive/legacy/saturation PASS" in result


def test_real_additive_formatters_bounds_identity_and_decimal_u64(tmp_path):
    stubs=tmp_path/'stubs'
    stubs.mkdir()
    (stubs/'esp_err.h').write_text('#pragma once\ntypedef int esp_err_t;\n')
    code=r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "dual_end_perf_format.h"
#include "dual_end_log_health_format.h"
int main(void) {
    dual_end_perf_record_t r={.role=0,.stage=1,.boot_id=UINT32_MAX,
        .window_seq=UINT32_MAX,.start_us=UINT64_MAX-1,.end_us=UINT64_MAX};
    dual_end_perf_reset(&r.stats,3,UINT32_MAX,1);
    r.stats.samples=UINT32_MAX;r.stats.overflow=UINT32_MAX;
    for(unsigned i=0;i<DUAL_END_OUTCOME_COUNT;++i)r.stats.outcomes[i]=UINT32_MAX;
    char buf[181];
    for(unsigned p=0;p<7;++p) {
        size_t n=dual_end_owner_outcome_format_part(&r,p,buf,sizeof(buf));
        assert(n>0 && n<=180 && strlen(buf)==n);puts(buf);
        assert(dual_end_owner_outcome_format_part(&r,p,buf,n)==0 && !buf[0]);
    }
    r.role=1;assert(!dual_end_owner_outcome_format_part(&r,0,buf,sizeof(buf)));
    r.role=0;r.stage=0;assert(!dual_end_owner_outcome_format_part(&r,0,buf,sizeof(buf)));
    dual_end_log_transport_stats_t s={.enabled=true,.queue_drop=UINT32_MAX,
        .format_drop=UINT32_MAX,.short_write=UINT32_MAX,.backend_error=UINT32_MAX,
        .queue_highwater=DUAL_END_LOG_TRANSPORT_QUEUE_DEPTH,.format_drop_long=UINT32_MAX,.format_drop_chars=UINT32_MAX};
    for(unsigned p=0;p<DUAL_END_LOG_HEALTH_PARTS;++p) {
        size_t n=dual_end_log_health_format(&s,UINT32_MAX,0,UINT32_MAX,UINT64_MAX,p,buf,sizeof(buf));
        assert(n>0 && n<=180 && strlen(buf)==n);puts(buf);
        assert(!dual_end_log_health_format(&s,UINT32_MAX,0,UINT32_MAX,UINT64_MAX,p,buf,n) && !buf[0]);
    }
    assert(!dual_end_log_health_format(&s,1,2,1,1,0,buf,sizeof(buf)));
    assert(!dual_end_log_health_format(&s,1,0,1,1,DUAL_END_LOG_HEALTH_PARTS,buf,sizeof(buf)));
    assert(!dual_end_log_health_format(NULL,1,0,1,1,0,buf,sizeof(buf)));
    return 0;
}
'''
    out=compile_run(tmp_path,code,sources=(RUNTIME/'dual_end_perf_stats.c',
        RUNTIME/'dual_end_perf_format.c',RUNTIME/'dual_end_log_health_format.c'),includes=(stubs,))
    rows=[json.loads(line) for line in out.splitlines() if line.startswith('{')]
    assert len(rows)==12
    header=(RUNTIME/'include/dual_end_log_transport.h').read_text(encoding='utf-8')
    queue_depth=int(re.search(r'DUAL_END_LOG_TRANSPORT_QUEUE_DEPTH (\d+)U',header).group(1))
    assert all(r['boot_id']=='ffffffff' and r['role']==0 for r in rows)
    assert rows[1]['end_us']==2**64-1
    assert rows[8]['sampled_us']==str(2**64-1)
    assert rows[7]['queue_capacity']==queue_depth
    assert rows[10]['queue_highwater']==queue_depth

    assert rows[7]["parts"]==5
    assert rows[11]["format_long"]==2**32-1
    assert rows[11]["format_chars"]==2**32-1
