# 把 PC 端改动并进既有工程

`pc-app/` 带的是全量 `src/`，最省事的做法是整目录替换；若目标仓库已有其他改动，
只合并下面 3 个文件即可。

## 需要覆盖的文件

| 文件 | 变更要点 |
|---|---|
| `src/capture/audio_capture.hpp` | 新增 `Fusion` 枚举、`start(sink, error, Fusion)` 重载、`fusion()`、`stream_channels()` |
| `src/capture/audio_capture.cpp` | 采集改为"端点支持就开 2 声道，失败回退 1 声道"；在采集线程里逐块融合 |
| `src/app/voice_runtime.cpp` | 启动采集前读 `HMI_CAPTURE_FUSION`（mean/best/left/right），传给 `start()` |

其余同名文件与原始工程一致，可直接覆盖或忽略。

## 三个文件的语义差异（便于手工合并）

1. **`CaptureSession::open`**：原来固定 `stream.nChannels = 1`，由 WASAPI `AUTOCONVERTPCM` 做降混。
   现在先看 `native.channels >= 2`，请求 2 声道；`Initialize` 失败则回退 1 声道。
2. **采集循环**：原来直接 `samples[index]`；现在按 `session.stream_channels` 取帧内样本，再按策略融合。
   `best` 每 10 ms 块比较左右能量，+1.5 dB 迟滞才切换。
3. **`voice_runtime`**：`capture.start(..., &error)` 改为 `capture.start(..., &error, fusion)`；
   `fusion` 由环境变量解析，默认 `Mean`（用 `_dupenv_s`，避免 MSVC 的 getenv 弃用告警）。

## 验证

```powershell
cmake --build build --config Release --target HMI
```

本机已验证通过。运行时需要真实双麦设备：

```powershell
$env:HMI_CAPTURE_FUSION = "mean"   # 基线
$env:HMI_CAPTURE_FUSION = "best"   # 候选
```

默认值保持 `mean`，在拿到配对 CER 数据前不要改默认。

## 注意

- `src/capture/` 与 `src/engine/` 等层按 `AGENTS.md` 的契约**禁止 include imgui.h**，本次改动遵守了这一点。
- 采集线程里不做分配、不加锁（融合只用到几个栈上变量）。