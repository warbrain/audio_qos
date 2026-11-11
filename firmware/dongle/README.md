# CytoScepter Dongle

本工程是双麦系统的 USB 接收端，版本为 0.5.21 / ESP-IDF 5.5.6，目标芯片为 ESP32-S3-N16R8。本版只同步发布版本号；Dongle 协议、遥测与音频处理均未改变。DSP 请求转发失败原因日志沿用 0.5.20 实现。VAD 和实际增益处理位于 Scepter。

**本次没有修改 GPIO 或硬件外设配置。** NewPCB 单麦板和当前双麦板电路不同，不能交叉刷写镜像。

0.5.18（历史版本）曾与 Scepter 保持同一公开版本；当前源码版本同步为 0.5.21。版本号同步不代表 Dongle 音频处理逻辑发生变化。

0.5.15 在既有 V2 诊断快照中附加 CDC 状态与累计计数：DTR、完整命令行、接收队列溢出、响应队列丢弃、USB 发送队列接受字节及未完整响应数。字段加入现有短分片，不改变分片数；发送队列接受量不代表主机已收到。

0.5.16 修复 USB 总线挂起/恢复时的 UAC 麦克风流状态：挂起前记录接口是否已打开，恢复且仍挂载时重建预填充并恢复流；卸载会清除待恢复标志。物理断开后重新枚举仍由主机重新打开 UAC alternate setting，固件不会在 USB attach 时盲目启动麦克风。

## 主要职责

- 从 NVS 加载 Scepter MAC，建立 CCMP ESP-NOW 单播链路。
- 接收 400-byte 双路 G.722/HID 正常音频包。
- 使用 32 槽（640 ms）接收播放时间线；目标播放延迟为 232 ms，缺口检测/恢复窗口为 16 帧，并据此发送 NACK。
- 接收 350-byte 双路 repair 包，并在截止时间前补回时间线。
- 解码固定 L/R 两路 G.722，恢复 16 kHz / PCM16 / stereo。
- 通过 USB UAC 把音频送入 PC。
- 把 Scepter 的按键、触摸和滚轮转换为 USB HID。
- 通过 Raw HID 把双麦固定增益命令转发到 Scepter；范围 1×～32×，无 NVS 覆盖时 Scepter 构建默认 8×，实际值从 Scepter telemetry 读取。
- 通过 USB CDC 提供状态、配对、可靠 OTA、UI 控制和重启命令。
- 管理自身 `ota_0` / `ota_1`，并转发 Scepter OTA。

## 通信配置

| 项目 | 当前值 |
| --- | --- |
| 基础 ESP-NOW 应用协议 | v1 |
| 音频恢复子协议 | v5 |
| 配对认证协议 | v2 |
| 无线 OTA 协议 | v2 |
| ESP-NOW 速率 | 5.5 Mbps |
| 信道 | 自动选择 1/6/11，失联会合信道 6 |
| 接收播放时间线 / 目标延迟 / 恢复窗口 | 32 槽（640 ms） / 232 ms / 16 帧 |
| NACK | 最多 4 次，间隔 60 ms |
| 音频 | 16 kHz / 16 bit / stereo / 20 ms |
| 正常包 | 400 bytes |
| repair 包 | 350 bytes |
| CPU | 240 MHz |

恢复子协议 v5 独立于基础应用协议 v1。两端必须成对使用当前双麦版本。

## 按 0.4.2 移植的恢复机制

Dongle 在 16 帧恢复窗口内发现缺口后，最多发出 4 次 NACK，间隔 60 ms。Scepter 使用 32 槽缓存，缓存项在创建 400 ms 后过期；音频帧为 20 ms，因此连续流中按时效最多可恢复最近 20 帧。每帧最多发送 2 次 repair，并限制任意 100 ms 最多 4 个 repair 包。

正常音频包为 400 bytes。repair 包为 350 bytes，单包包含左右两路各 160 bytes G.722，因此一次 repair 可恢复完整双声道 20 ms 帧。

控制包与 repair 共用 priority FIFO，在 Scepter 下一次发送选择中先于 paced original；配额、截止时间、会话检查和最后槽保留用于限制恢复占用。Dongle 对过期 repair 只统计为 late，不回写已经越过的时间线位置。

新会话清除旧窗口。断连、OTA、扫描和信道切换期间使用暂停门控，暂停区间的 ACK、NACK 和 RESYNC 不会进入恢复后的新会话。

## 音频接收与 USB

正常音频路径：

```text
ESP-NOW callback
  -> RX queue
  -> header / CRC
  -> 恢复时间线
  -> G.722 L/R decode
  -> stereo PCM16
  -> UAC ring buffer
  -> USB microphone
```

UAC 暴露 16 kHz / 16 bit / stereo 麦克风。环形缓冲按完整 L/R 采样帧计数；补样和丢样使用相同双声道位置，避免左右错位。新 stream、断连、OTA 和接收超时会清除旧音频。

USB 复合设备同时提供：

- **UAC microphone**：双声道音频。
- **HID keyboard/mouse**：按键、触摸、滚轮。
- **CDC ACM**：状态、配对、UI、OTA 和诊断。

主机未持续读取 UAC 时，缓冲指标可能变化，这不等同于无线永久丢包。判断恢复效果应查看 `accepted`、`output_frames`、`permanent_loss`、`recovered`、`repair_ontime`、`repair_late`、`duplicates` 和 `rejected`。

## 配对与可靠 OTA

配对协议 v2 使用 prepare/commit/confirm/rollback 事务，NVS 保存 active/pending，ESP-NOW peer 启用 CCMP。使用交接包 `启动配对工具.cmd` 完成双端配对。

Dongle OTA 写入自身 OTA 分区；Scepter OTA 经 USB CDC 和 ESP-NOW 转发。无线 OTA v2 对 BEGIN/DATA/END 逐包确认并有限重试，校验目标、会话、偏移、长度、CRC、项目名和 app 标识。

USB CDC 主机侧仅接受 OTA2（OTA1 已禁用）。`OTA2 + CHUNK` 对每个 16～4096 bytes 数据块校验 CRC，返回滚动 CRC，并能幂等确认丢失 ACK 后重发的上一块。图形化非加密工具可读取 `FIRMWARE_MANIFEST.json`，升级 Dongle 自身或经 ESP-NOW 转发升级 Scepter；串口打开时允许 DTR/RTS 保持低电平。也可使用 `启动OTA工具.cmd` 或：

```powershell
python OTA升级上位机\ota.py --port COMx status
python OTA升级上位机\ota.py --port COMx scepter <scepter.bin>
python OTA升级上位机\ota.py --port COMx dongle <dongle.bin>
```

## 录音

运行交接包 `查看录音设备.cmd` 确认 Audio QoS 输入，随后运行 `启动录音工具.cmd` 或 `录音60秒.cmd`。输出为 16 kHz / PCM16 / stereo WAV。

FFmpeg 也可录制：

```powershell
ffmpeg -f dshow -i 'audio=麦克风 (Audio QoS Microphone)' -t 15 -ac 2 -ar 16000 -c:a pcm_s16le recording.wav
```

L/R 是 USB 声道顺序，实体麦克风对应关系需分别近讲或轻敲确认。

## 分区

| 分区 | 偏移 | 大小 | 用途 |
| --- | ---: | ---: | --- |
| nvs | 0x9000 | 64 KB | 配对与配置 |
| factory | 0x80000 | 0x180000 | 工厂应用 |
| ota_0 | 0x200000 | 2 MB | OTA A |
| ota_1 | 0x400000 | 2 MB | OTA B |
| storage | 0x600000 | 10 MB | 预留 |

## 构建

目标 SDK 为 `D:\Espressif\frameworks\esp-idf-v5.5.6`。在源码根目录执行：

```powershell
.\build_all.ps1
```

`build_all.ps1` 会使用当前本机 ESP-IDF 5.5.6 设置，在隔离目录构建双端固件；不传刷写开关时不会刷写设备。USB UAC 本地组件随源码交付，不依赖修改下载缓存。

## 日志检查

- 启动：`fw=0.5.16`、`rate=5.5 Mbps`、`target=232ms`、`window=16`、`audio receiver started: stereo G722`。
- 正常接收：约 49–50 fps，查看 accepted/output/permanent_loss。
- 恢复：查看 recovered、repair_ontime、repair_late、duplicates、rejected。
- USB：查看 status events、overflow、underflow、stall。
- OTA：用 `tools\ota\ota.py --port COMx status` 查询。
- 若 CDC/HID/UAC 均消失，先检查 USB 是否被其他虚拟机或驱动占用。

0.5.15 保持既有恢复、配对和 OTA 协议兼容；生产路径移除了 32 KiB 旧 UAC ring，使用时间线直连输出，并提供上下文关联的 Dongle telemetry 与 CDC 传输诊断。UAC 静音填充属于欠载保护，不是 VAD，仍保留。
