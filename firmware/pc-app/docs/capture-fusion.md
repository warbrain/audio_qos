# 采集融合（stereo → mono）

USB dongle 通过 UAC 送的是**双声道**：左、右各一路麦克风。ASR 需要单声道，
所以融合必须在某处发生。以前这件事由 Windows 的 WASAPI 音频引擎替我们做
（`AUTOCONVERTPCM` + 请求 1 声道），效果等价于**直接平均**，而且应用层看不到
两路信号，也就无法挑选更好的一路。

现在采集层自己完成这件事。

## 行为

- 端点原生格式 ≥2 声道时，以 **2 声道 / 16 kHz / float** 打开；打开失败自动回退 1 声道
  （例如只支持单声道的麦克风或虚拟设备）。
- 融合策略由 `HMI_CAPTURE_FUSION` 环境变量决定：

| 值 | 行为 |
|---|---|
| `mean`（默认） | 两路平均，与旧行为一致 |
| `best` | 跟随能量更高的一路；+1.5 dB 迟滞（每 10 ms 块判定一次），避免在两句之间来回切换 |
| `left` / `right` | 固定取单路 |

- 送进 ASR 的仍然是 **16 kHz 单声道 float**，sink 契约不变。

## 为什么不默认 `best`

20 mm 间距在 4 kHz 以下几乎全向，两路能量差主要来自指向性噪声和遮挡，不是
语音本身。`best` 在强指向性噪声下应该更好，但也可能在语音/噪声交替时产生切换
伪影。**先用 `mean` 作为基线，配对 A/B 之后再决定是否默认 `best`**：

```powershell
# 基线
$env:HMI_CAPTURE_FUSION = "mean"; .\build\bin\Release\HMI.exe
# 候选
$env:HMI_CAPTURE_FUSION = "best"; .\build\bin\Release\HMI.exe
```

## 验证状态

- 编译：`cmake --build build --config Release --target HMI` 通过。
- 运行时：需要真实 dongle + 双麦录音；离线等价实验见
  `scepterfirmware_2/tools/asr_eval/asr_cer_eval.py --downmix mean|best`。
- `best` 的迟滞参数（1.41 ≈ +1.5 dB、每 10 ms 块一次）是保守初值，若观察到切换
  频繁可调大比例或延长判定窗口。
