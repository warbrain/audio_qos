# CytoScepter Scepter

本工程是双麦系统的手持发送端，当前配对版本为 0.5.26 / ESP-IDF 5.5.6，目标芯片为 ESP32-S3-N16R8。本版新增 `SCEPTER_TELEMETRY <token> 6 <part>` 查询，共 17 段返回 `audio.dispatch` 的队列等待、帧龄、队列满、槽位失效/不可用、事件丢失、高水位和迟到事件计数。part 15 的 `le` 是旧 dispatch epoch 事件数，part 16 的 `la` 是无法归属当前遥测窗口/会话/配置的时序完成数；part 11 的 `li`、part 12 的 `lx`、part 13 的 `lb` 分别表示无活动窗口、窗口身份不匹配、入队时间早于当前窗口。所有原因均为 boot 累计，并共同归入 `late_events` 汇总；它们用于定位遥测归属问题，不等同于 PCM 丢帧或音频截止期错过。版本 0.5.22 起使用预分配 PCM 缓冲池和 SPSC 槽位编号环形队列，保留两帧排队上限，避免通过 FreeRTOS 队列复制整块 PCM；本版不改变编解码器、DSP、无线帧或 NVS 设置。0.5.21 增加 Scepter Opus 编码遥测；0.5.19 起每个五秒 capture-health 统计窗附加左右实际固定增益标签；空间模式 5（左右声道各自独立 Wiener）和其 Wiener 计时沿用 0.5.17 实现。保留双麦板自己的 PDM、LCD、按键、触摸、滚轮、IMU、电池、马达和 RGB 实现。

**本次没有修改 GPIO 或硬件外设配置。** NewPCB 单麦板和当前双麦板电路不同，不能交叉刷写镜像。

模式 4 在现有 DAS 融合单声道输出后执行 16 kHz、512 点、256 点 hop 的固定内存 Wiener 后滤波。该单通道功率谱噪声跟踪器不依赖 VAD；切换时随原 PCM 边界事务复位，启动缓冲约 32 ms。合成主机测试只能验证数值/流连续性，是否改善隔离箱内识别率仍需真实扬声器噪声与配对 ASR 测试。

模式 5 对输入 L/R 分别运行独立的 16 kHz、512 点、256 点 hop Wiener 状态，分别跟踪噪声功率并保留立体声输出；不执行 DAS、对齐、左右平均或健康声道复制。每次模式切换时两套状态随现有 PCM 边界事务复位。该模式要求空间能力位 4（值 16），布局为 stereo（0）；HMI 只有读到该能力后才提供选择。独立路径处理量和状态内存高于单通道模式 4，ESP32 实时余量仍需设备遥测验证；合成测试不能证明隔离箱中的 ASR 改善。

0.5.17 的 owner-timing telemetry 增加 Wiener 调用耗时，不改变音频样本或算法状态：模式 4 报告 `wiener_us_n/p50/p95/max`；模式 5 分别报告 `wiener_l_us_n/p50/p95/max` 和 `wiener_r_us_n/p50/p95/max`。当前一轮无有效计时样本时，统计值为 `null`。这些字段是源码能力说明；只有同一设备运行日志才能作为板上时序证据。

0.5.18 的 `audio.capture_health` 使用 18 个有序 part。part 0–16 保留既有五秒窗口统计；part 17 新增 `gain_l_x` / `gain_r_x`，表示这个不可混合不同增益的窗口所使用的左右固定数字增益。若读数无效则输出 `null`，不伪报 0×。`rms_dbfs_l/r`、`peak_dbfs_l/r` 与 `clip_pct` 来自固定增益前的 48 kHz PCM 数字采样；它们不是扬声器 SPL 或校准模拟增益。窗口在 codec、空间模式、采集代次或左右固定增益变化时关闭，因此 ASR/音频分析可把统计窗与真实 gain setting 对齐。HMI 可读取 18-part 新格式及 17-/14-part 历史格式；历史记录没有窗口级 gain 时保持 unavailable。合格的 arm 统计在报告中按 run/arm/context、boot/session、codec/mode、声道和实测增益分组，并给出逐五秒窗 RMS dBFS p50/p95、peak 最大值与按样本总数汇总的削波率；不跨 arm 混合，未知增益独立保留。

## 双麦音频路径

两只 PDM 麦克风共用 I2S0 PDM RX 的时钟和数据线，使用硬件 L/R slot 区分通道：

```text
PDM L/R 48 kHz stereo
  -> 每路 80 Hz HPF
  -> 95-tap / 7 kHz FIR
  -> 3:1 降采样
  -> 16 kHz / PCM16 / stereo
  -> 固定增益（无 NVS 覆盖时构建默认 8×，可配置 1×～32×）+ 双路共享限幅
  -> VAD 配置默认关闭；本次运行状态须由设备读回/遥测确认，关闭时静音和低声 PCM 持续送入编码器
  -> G.722-L 160 bytes + G.722-R 160 bytes
  -> 400-byte 正常音频包
```

每 20 ms 产生一帧。左右通道在采集、滤波、编码和传输过程中保持固定映射，不按响度交换。0.5.14 不再根据语音活动度修改 PCM 增益；固定增益在一次设置后保持不变，且左右声道共用同一值。共享软限幅仅在过载采样出现时工作。

无 NVS 覆盖时，构建默认固定增益为 **8×**（约 +18.06 dB；见 `sdkconfig.defaults` 中的 `CONFIG_MIC_OUTPUT_GAIN_X`）。HID 工具可在 **1×～32×** 之间设置；持久值存于 NVS `mic_cfg/fixed_gain_x`，擦除 NVS 后恢复 8×。运行时实际值以 `audio.capture_health` 窗口的 `gain_l_x` / `gain_r_x` 为准；它是固定数字增益，不是 AGC。

| 项目 | 数值 |
| --- | ---: |
| PDM 输入 | 48 kHz / 16 bit / stereo |
| 发送前 PCM | 16 kHz / 16 bit / stereo / 20 ms |
| 单路 PCM | 320 samples / 640 bytes |
| 单路 G.722 | 160 bytes / 64 kbps |
| 双路 G.722 | 320 bytes |
| HID/input | 24 bytes |
| 正常音频包 | 400 bytes，ESP-NOW v2 |

400 bytes 正常包由 10-byte 通用头、16-byte 音频头、左右各 160-byte G.722、24-byte HID/input 和 30-byte padding 组成。

## 通信协议

公共协议位于 `components/dual_end_common/include/dual_end_link_config.h`。

| 项目 | 当前值 |
| --- | --- |
| 基础 ESP-NOW 应用协议 | v1 |
| 音频恢复子协议 | v5 |
| 配对认证协议 | v2 |
| 无线 OTA 协议 | v2 |
| ESP-NOW 速率 | 5.5 Mbps |
| 接收时间线 / 恢复窗口 | 232 ms / 16 帧 |
| 信道 | 自动选择 1/6/11，失联会合信道 6 |
| CPU | 240 MHz |
| 对端记录 | NVS `pairing/peer` |
| 双麦固定增益 | 无 NVS 覆盖时构建默认 8×；运行值读 `gain_l_x` / `gain_r_x`；NVS `mic_cfg/fixed_gain_x`；范围 1×～32× |

恢复子协议 v5 独立于基础应用协议 v1。两端必须成对使用当前双麦版本。

## 按 0.4.2 移植的恢复机制

Scepter 在 PSRAM 中使用 32 槽的左右双路 G.722 恢复缓存，缓存项在创建 400 ms 后过期；每帧 20 ms，因此连续流中的时效上限是最近 20 帧。Dongle 检测到 16 帧窗口内的缺口后，最多发送 4 次 NACK，间隔 60 ms。

- repair 包为 **350 bytes**，单包含左右各 **160 bytes G.722**。
- 每个原始帧最多发送 **2 次 repair**。
- 任意 **100 ms 最多 4 个 repair 包**。
- cache miss、cache expired、过期请求和失效会话不会进入无线发送。
- 会话切换会清除旧队列，并使旧恢复缓存失效。

### 发送调度

控制包和 `AUDIO_REPAIR` 共用 priority FIFO。发送任务选择下一请求时，先取 priority FIFO，再处理到发送时间的 paced original 音频。这一顺序与单麦 0.4.2 一致。

repair 仍受配额、帧截止时间、会话有效性和最后槽保留限制。slot 入队和占用由同一提交锁保护；最后一个可用槽为控制或正常音频保留，避免恢复流量耗尽全部提交能力。

### 暂停与会话隔离

连接断开、OTA、信道扫描或切换会关闭音频暂停门控。暂停边沿只使会话失效一次；暂停区间积压的 ACK、NACK 和 RESYNC 会被拒绝。恢复边沿只启动一个新会话，旧控制包不能污染新会话。

## 配对与 OTA

配对协议 v2 使用 prepare/commit/confirm/rollback 事务，NVS 保存 active/pending 记录，ESP-NOW peer 启用 CCMP。交接包使用 `启动配对工具.cmd` 完成双端配对。

可靠 OTA 路径为：

```text
PC -> Dongle USB CDC -> ESP-NOW -> Scepter OTA 分区
```

无线 OTA v2 对 BEGIN/DATA/END 逐包确认并有限重试，校验目标、会话、偏移、长度、CRC、项目名和 app 标识。OTA 期间暂停音频、HID 和信道切换，保留 HELLO。

USB 主机侧仅接受 OTA2（OTA1 已禁用）。`OTA2 + CHUNK` 对每个 16～4096 bytes 数据块校验 CRC，返回累计字节数和滚动 CRC，并能对丢失 ACK 后重发的上一块作幂等确认。图形化非加密 OTA 工具既可经 Dongle 无线转发升级 Scepter，也可通过 Scepter Type-C 端口直连升级；两种方式都允许 DTR/RTS 保持低电平。

## 硬件资源

下表是原双麦硬件配置，本次通信与重传修改未改动这些定义。

| 模块 | 信号 | GPIO | 说明 |
| --- | --- | ---: | --- |
| LCD ST7789 | SCLK / MOSI | 1 / 2 | SPI2 |
| LCD ST7789 | CS / DC / RST / BL | 39 / 40 / 41 / 38 | 背光使用 LEDC |
| 双麦克风 | PDM CLK | 14 | 3.072 MHz |
| 双麦克风 | PDM DATA | 47 | 两只麦克风共享，硬件 L/R slot |
| 触摸板 | SDA / SCL / INT | 8 / 18 / 17 | I2C |
| IMU | SDA / SCL / INT1 / INT2 | 6 / 5 / 7 / 42 | I2C |
| 用户按键 | KEY1 / KEY2 / KEY3 | 9 / 10 / 0 | 低电平按下 |
| 编码滚轮 | A / B / Button | 12 / 13 / 15 | 上拉输入 |
| 电池 / 充电 | ADC / CHRG | 4 / 11 | TP4054 |
| 马达 | PWM | 44 | LEDC 1 kHz |
| RGB | R / G / B | 46 / 45 / 43 | 高电平点亮 |

PDM 模式不使用 BCLK、LRCLK/WS，硬件没有独立麦克风 RST/EN。

## 分区

| 分区 | 偏移 | 大小 | 用途 |
| --- | ---: | ---: | --- |
| nvs | 0x9000 | 64 KB | 配对与配置 |
| factory | 0x80000 | 0x180000 | 工厂应用 |
| ota_0 | 0x200000 | 2 MB | OTA A |
| ota_1 | 0x400000 | 2 MB | OTA B |
| animation | 0x600000 | 10 MB | LCD 动画 |

仅 OTA 应用镜像不会更新 `animation.bin`；动画变化需要完整 ROM 烧录。

## 构建

目标 SDK 为 `D:\Espressif\frameworks\esp-idf-v5.5.6`。在源码根目录执行：

```powershell
.\build_all.ps1
```

`build_all.ps1` 会使用当前本机 ESP-IDF 5.5.6 设置，在隔离目录构建双端固件；不传刷写开关时不会刷写设备。

## 日志检查

- 启动：fw=0.5.23、codec=G722 stereo、air packet=400B、rate=5.5 Mbps、target=232ms、window=16。
- 音频分析日志应显示 `vad=disabled`，且发送速率在静音时仍约为 50 fps。
- 正常发送：约 50 fps，观察 normal MAC ok/fail、deadline/queue drop。
- 恢复发送：观察 NACK、repair queued/submitted/ok/fail/cancelled、budget_drop、cache miss/expired。
- OTA：观察 `OTA transfer started/stopped`。
- UI/马达/增益：观察 `control execute`；增益写入后应看到 `fixed gain set to N× ... and saved`。

0.5.15 延续无编码前 VAD 衰减与可持久双麦固定增益，并保留采集健康度、owner 时序和 NLMS 诊断 telemetry。

## 2026-11-10 本机验证记录

- 使用 ESP-IDF 5.5.6 构建 `build-rsi/cytoscepter_scepter.bin`，镜像 SHA-256：`88030e2c8a860591aa644c1ecca442e44fc98d2f05cb4a1868733abb07f3880d`。
- 按受控应用分区刷写后核对读回镜像，SHA 与构建产物一致；NVS 刷写前后哈希未变化。刷写前完整设备读回备份和刷写后启动日志保存在本机 RSI 证据目录。此项是应用分区刷写验证，不称为 OTA 测试。
- 启动日志中的运行配置为 G.722 codec 4、空间模式 0、左右运行增益 1×；采样窗口内 G.722 编码错误/截止计数为 0。构建默认增益仍为 8×，运行时 1× 与 build default 的来源没有在本轮改写或归因。
- 保留的 Opus32 / Opus62 物理遥测 pilot 分别记录最大编码时间 15.067 ms / 15.955 ms，低于 20 ms 帧周期，未观察到编码失败。Opus FEC 当前为关闭；未验证恢复行为。
- 上述属于遥测/实时编码验证，不构成 400 条 ASR 精度结论。400 条 G.722/Opus62 物理录音的固件镜像哈希不全；Qwen 首跑在 383/800 调用处出现第二个 `bad allocation` 后停止，不能从这轮提升或默认切换 codec。

## 空间模式 6：soft-mask MVDR（0.5.25，实验候选）

模式 6 已加入固件空间模式控制、能力协商和主机 DSP 评测入口。能力位 5（值 32）表示双方协议/输出布局支持，item 18 选择模式；默认仍为模式 0，模式 0–5 的编号、布局和处理逻辑保留。算法在 16 kHz 双声道上使用 512 点 Hann STFT / 256 点 hop，以信号噪声底得到软掩码，跟踪双通道语音/噪声协方差，用对角加载的 2×2 MVDR 权重生成双声道中的单声道增强信号。它不是神经掩码，不依赖 VAD，也没有修改模型权重。STFT 启动延时约 32 ms；处理状态固定内存存放于 PSRAM 空间状态中。

新增 `SCEPTER_TELEMETRY <token> 7 <part>`（kind 7）8 段 `audio.mvdr`，记录 noise 状态、回退原因、hop、有效/回退频点及输出队列计数；既有 `audio.pipeline` 的 `s` 项记录空间处理耗时。PC 评测工具现支持 mode 6，要求输入为双声道 16 kHz WAV，并保留原始左右通道进入固件 DSP；Qwen 输入只在增强后下混为单声道。主机合成测试已覆盖流连续、双声道输入/单声道输出、数值保护、静音/削波/单通道故障回退、重置和能力位。**尚未做 ESP32 上的 20 ms 实时余量测试、物理声场测试或 Qwen 配对评分，因此 mode 6 目前仅是 NO_PROMOTION 候选，不代表 ASR 精度已提升。**
