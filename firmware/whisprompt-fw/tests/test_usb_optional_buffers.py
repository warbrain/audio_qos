"""Compile actual USB OUT declarations for zero/positive per-function sizes."""
from pathlib import Path
import pytest
from test_audio_recovery_042 import _build_host_executable,_run
ROOT=Path(__file__).resolve().parents[1]

@pytest.mark.parametrize('enabled,functions,sizes',[
    (1,1,(0,0,0)),(0,1,(32,0,0)),(1,1,(32,0,0)),
    (1,2,(0,32,0)),(1,3,(0,0,32)),
])
def test_actual_out_buffer_declarations(tmp_path,enabled,functions,sizes):
    s=(ROOT/'dongle/components/dongle_usb_device/tinyusb/audio_device.c').read_text(encoding='utf-8')
    begin=s.index('// EP OUT software buffers');end=s.index('// Control buffers',begin)
    declarations=s[begin:end]
    begin=s.index('// Aligned buffer for feedback EP');end=s.index('// Aligned buffer for interrupt EP',begin)
    declarations+=s[begin:end]
    code='#include <assert.h>\n#include <stdint.h>\n#include <stdio.h>\n'
    code+=f'#define CFG_TUD_AUDIO_ENABLE_EP_OUT {enabled}\n#define CFG_TUD_AUDIO {functions}\n'
    for i,size in enumerate(sizes,1):
        code+=f'#define CFG_TUD_AUDIO_FUNC_{i}_EP_OUT_SW_BUF_SZ {size}\n#define CFG_TUD_AUDIO_FUNC_{i}_EP_OUT_SZ_MAX {size}\n'
    code+='''#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP 1
#define USE_LINEAR_BUFFER 1
#define tu_static static
#define OUT_SW_BUF_MEM_ATTR
#define CFG_TUD_MEM_SECTION
#define TUD_EPBUF_DEF(name,size) uint8_t name[size]
#define TUD_EPBUF_TYPE_DEF(type,name) type name
'''+declarations+'\nint main(void){\n'
    if enabled:
        for i,size in enumerate(sizes[:functions],1):
            if size:
                code+=f'ep_out_sw_buf.buf_{i}[0]=1;lin_buf_out.buf_{i}[0]=2;fb_ep_buf.buf_{i}=3;\n'
                code+=f'assert(ep_out_sw_buf.buf_{i}[0]==1 && lin_buf_out.buf_{i}[0]==2 && fb_ep_buf.buf_{i}==3);\n'
    code+='puts("actual optional USB buffer declarations PASS");return 0;}\n'
    p=tmp_path/'usb_optional_buffers.c';p.write_text(code,encoding='utf-8')
    exe,env=_build_host_executable([p],tmp_path)
    assert 'PASS' in _run([str(exe)],cwd=tmp_path,env=env).stdout
