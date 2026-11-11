# box-sim — 1 m³ 封闭箱体声学仿真与实测工具

用于在**不依赖物理箱体**的情况下，批量生成"双麦 + 人工嘴 + 两个噪声音箱"的测试音频，
并与设备端空间处理、ASR 评测链路对接。

物理箱体（1 m³）与人工嘴（距双麦 5 cm）已经固定，仿真用于：

- 覆盖物理环境难以稳定复现的条件（T60、SNR、相干/非相干噪声）；
- 在改动固件之前先做模式对比（直通 / DAS / NLMS / Wiener）；
- 生成可重复的数据集，供 CER 评测与回归测试。

## 目录

| 文件 | 作用 |
|---|---|
| `box_sim.py` | pyroomacoustics 仿真：1 m³ 箱体、20 mm 双麦、5 cm 人工嘴、两只噪声音箱 |
| `box_test.py` | 物理箱体实测：US-4x4HR 声卡播放 + Dongle UAC 录音 |
| `make_manifest.py` | 由参考文本和 wav 目录生成 CER 评测清单（TSV） |
| `dsp_param.py` | 通过 Dongle HID 切换设备端 codec / 空间模式（配合实测） |
| `run_sim.ps1` | 一键跑仿真（含依赖路径设置） |
| `run_cer.ps1` | 一键对某个数据目录跑 CER（可指定空间模式） |
| `requirements.txt` | Python 依赖 |

## 环境（uv 管理）

项目自带 `.venv`，由 uv 创建并锁定版本，不依赖系统 Python：

```powershell
cd simulator/box-sim
uv venv .venv --python 3.12
uv pip install --python .venv\Scripts\python.exe -r requirements.txt
```

`run_sim.ps1` / `run_cer.ps1` 会自动优先使用 `.venv\Scripts\python.exe`，
也可以用 `-Python <path>` 指定其它解释器。

已验证版本：Python 3.12.16、numpy 2.5.4、scipy 1.18.1、pyroomacoustics 0.10.1。
## 仿真用法

```powershell
# 几何与 RIR 自检
.\run_sim.ps1 -Report

# 生成 10 句、SNR 10 dB、T60 0.3 s、非相干粉噪
.\run_sim.ps1 -SpeechDir D:\speech\clean -Out D:\data\sim10 -Snr 10 -T60 0.3

# 相干噪声（两只音箱放同一信号）
.\run_sim.ps1 -SpeechDir D:\speech\clean -Out D:\data\sim10c -Snr 10 -Coherent
```

参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `--t60` | 0.3 | 混响时间（反推吸声系数，Image Source 阶数随之确定） |
| `--snr` | 10 | 目标信噪比（dB） |
| `--noise` | pink | `pink` / `white` |
| `--coherent` | 关 | 两只音箱播放同一噪声（相干场） |
| 几何 | 固定 | 房间 1×1×1 m，双麦 20 mm，嘴距 5 cm，音箱 (0.15,0.75) / (0.85,0.75) |

输出：16 kHz 双声道 WAV（每通道一只麦克风）+ `manifest.csv`。

## 物理实测用法

声卡通道映射（实测确认）：

| 声卡输出 | WAV 通道 | 接什么 |
|---|---|---|
| Out 1 | **ch2（索引 1）** | 人工嘴 |
| Out 3 | ch3（索引 2） | 左音箱 |
| Out 4 | ch4（索引 3） | 右音箱 |
| — | ch1（索引 0） | 空 |

```powershell
# 先校准（播放语音-only / 噪声-only，打印电平）
python box_test.py --speech-dir D:\speech\clean --calibrate-only

# 正式采集
python box_test.py --speech-dir D:\speech\clean --out D:\data\box --snr 10
```

注意：US-4x4HR 在 WASAPI 下只接受 **48 kHz / 4 声道**，脚本按 4 通道播放、按 48 kHz 录音后重采样到 16 kHz。

## 生成 CER 清单并评测

```powershell
python make_manifest.py --refs refs.tsv --wav-dir D:\data\sim10 --out D:\data\sim10\manifest.tsv
.\run_cer.ps1 -Manifest D:\data\sim10\manifest.tsv -Out D:\data\sim10\cer -Modes 0,1,4,5
```

`refs.tsv` 格式：`id<TAB>ref`（参考文本）。评测工具位于固件树的
`tools/asr_eval/asr_cer_eval.py`，`run_cer.ps1` 通过 `-EvalTool` 指定。

## 已知限制

1. RIR 有约 1 ms 的固定前置延迟（pyroomacoustics 数值实现），对所有声源一致，不影响声学特性；
2. 人工嘴在阵列轴线上时 ILD 只有约 −0.8 dB；需要更强近场线索时，把声源横向偏移 2–3 cm；
3. 脚本内的 DRR 估计较粗糙（峰值窗口法），只用于快速自检；
4. 仿真不含设备端的 HPF/LPF、增益、限幅和 G.722 编码；这些在评测链路里另行模拟。

## 与固件的配合

`dsp_param.py` 可通过 Dongle 的 HID Feature Report 读写 DSP 参数：

```powershell
python dsp_param.py --all           # 读 codec / 空间能力 / 当前模式
python dsp_param.py --mode 4        # 切空间模式（0-5）
python dsp_param.py --codec 3       # 切 codec（3=Opus32, 4=G722, 5=Opus62）
```

实测时用它切换模式，仿真时用 `run_cer.ps1 -Modes` 复现同样的处理。
