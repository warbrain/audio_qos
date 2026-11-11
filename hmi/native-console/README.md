# HMI Native Console

Windows 原生 C++23 语音控制台：CMake、Vulkan、GLFW、Dear ImGui。

## 项目目录

所有项目代码、依赖源码、模型和生成文件都位于本项目根目录。构建不读取其他工作副本，不下载源码；编译器、Windows/Vulkan/CUDA SDK、可选 ccache 和系统字体使用本机安装。

| 目录 | 内容 |
| --- | --- |
| `src/` | 应用、界面、采集、语音 SDK 适配、规则和动作 |
| `3rd_party/` | 已提供的依赖与 `native/source` 源码子集 |
| `models/` | ASR/TTS、静音检测、双通道增强权重及 SHA-256 清单 |
| `config/`、`rules/` | 配置模板与默认规则 |
| `cmake/` | 项目内 SDK 构建、部署和路径检查 |
| `tools/` | 统一构建入口及可选诊断程序 |
| `tests/` | 基础回归、窗口集成检查及测试夹具 |
| `docs/plan/` | 当前计划、实现进度和使用说明 |
| `docs/history/` | 历史报告，保留当时的证据和限制 |
| `build/` | 唯一默认构建目录，包含程序、测试、SDK 和诊断证据 |

默认 Release 程序为 `build/bin/Release/HMI.exe`。测试程序和夹具 DLL 在 `build/tests/bin/Release/`；构建/识别诊断记录集中在 `build/diagnostics/`。所有DLL/LIB、对象文件、PDB、SDK交付包和编译缓存只输出到 `build/` 或其子目录，构建配置会拒绝源码目录及其他项目内目录。不要提交构建产物或模型权重。

## 构建与验证

在本项目根目录执行：

```powershell
.\tools\build.ps1 -Test
```

脚本自动准备 Visual Studio 环境并复用 `build/`。也可直接使用：

```powershell
cmake -S . -B build -G "Visual Studio 18 2026"
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

已有 `build/CMakeCache.txt` 时沿用原生成器及平台。自定义BuildDirectory必须为 `build/` 或其子目录；Ninja可选入口为 `tools/build.ps1 -Generator Ninja -Test`，输出在 `build/ninja/`。统一脚本还将当前进程TEMP/TMP设置为构建目录内的 `tmp/`；直接手工调用CMake/MSBuild时，编译器的短期系统临时文件仍由调用者的TEMP/TMP控制。ccache缺失时可正常构建。启用cuDNN可加 `-Cudnn`；本机CUDA架构可用 `-CudaArchitectures 89-real` 指定。CPU/GPU/OpenMP参数沿用既有配置。

普通构建从 `3rd_party/native/source/` 生成独立 SDK DLL，再将完整配套集合部署到程序旁 `native/`。SDK 源码路径必须位于本项目内，头文件、导入库、后端 DLL 和模型规格按同一次构建的清单校验。详见 [SDK 整合说明](docs/plan/audio-source-integration.md)。

native 保留当前 Qwen3-ASR/TTS、Silero 静音检测和 H-GTCRN 所需的构建依赖，仅支持 Windows x64 CPU、CUDA、Vulkan；未引入完整上游仓库。保留模型规格、构建输入和原许可证/作者说明。`3rd_party/native/` 只放 `source/`，完整生成包统一位于 `build/audio-sdk/package/<配置>/`，不再放重复预构建快照。来源和裁剪范围见 [更新记录](docs/plan/native-update-2026-10-30.md)。

2026-11-04 按用户提供的本地 audio.cpp `4a6b20c3` 更新必需源码子集，保留本地 TTS 截断保护、聚合诊断和 Windows 构建适配。Qwen3-ASR 新增的 `cuda_perf` 性能选项默认仍关闭。本地 SDK ABI 为 0.5，包含 `audiocpp_diagnostics_get_snapshot` 和独立的录音前处理、H-GTCRN 接口。调用者设置 `struct_size`，读取操作成功/失败次数及加载、建会话、推理和流式处理的累计时间；短操作即使省略日志行也计入统计，查询不清零，原 112 字节诊断结构保持不变。时间是宿主操作范围的累计墙钟，存在嵌套或并发重叠，不等于 GPU kernel 耗时。应用继续要求配套 ABI，必须一起更新头文件、SDK 和主程序。

## 模型与配置

两份权重位于 `models/asr.gguf`、`models/tts.gguf`，共 7,267,640,768 字节，校验值在 `models/manifest.json`。默认先检查程序旁 `models/`，再检查识别到的项目根 `models/`，不继续搜索项目外的父级模型目录。独立部署时把模型放在程序旁 `models/`。保留 `HMI_MODEL_DIR` 和单模型显式路径覆盖，实际加载前会拒绝项目外路径。

静音检测和 H-GTCRN 权重位于 `models/frontend/`，共 1,455,784 字节。来源、转换格式和许可证见 [模型说明](models/README.md)。程序和构建均不需要 Python；官方输出对照环境只保留在 `build/diagnostics/`。

应用配置为程序旁 `config/app.yaml`，示例为 `config/app.example.yaml`。相对配置路径以程序目录为准；`HMI_CONFIG` 可显式覆盖项目内配置文件。配置、规则、日志、模型、参考音频和 SDK 路径都会规范化并检查项目边界；不再回退到用户目录创建引擎配置。构建仅补充缺失模板，保留用户已有配置、规则和日志。默认日志为程序旁 `logs/hmi.log`。构建脚本和测试分别使用 `build/tmp`、`build/tests/tmp`，只修改当前进程的临时目录。配置和日志细节见 [基础库说明](docs/plan/foundation-libraries.md)。

## 设备日志与遥测

HMI 启动后会在独立后台线程发现 USB 串口，验证 Scepter 与 Dongle 身份并接收设备日志。
设备插拔或串口换号后自动重连；端口被刷机或评测程序占用时退避重试。
设备收集不依赖语音模型加载结果，不在界面或录音回调中读串口、解析或写文件。

默认保存到程序旁 `logs/devices/`，自定义主日志位置时使用同一日志目录下的 `devices/`：

- `<设备标识>.log`：设备文本日志，含收到的结构化原文；警告和错误保留。
- `<设备标识>.telemetry.jsonl`：合法遥测，每行包含主机接收时间、设备标识、串口、主机运行编号、连接编号及原始 payload。

精确相同的遥测分段去重；不同启动、窗口、part 或内容冲突的记录继续保存。
主机不把多个 part 强行合并，也不把缺失数据补成零。JSON payload 保留原始整数文本。
文件大小和备份数沿用 `logging.max_size_mb` 与 `logging.max_files`，每两秒刷新，退出时关闭并刷新。
设备连接、查询异常、去重及存储失败统计记录在 `hmi.log`。

可在 `config/app.yaml` 停用串口收集，重启后生效；刷机前关闭 HMI 也会释放端口。
旧配置缺少此段时自动采用以下默认值，升级不覆盖原配置：

```yaml
devices:
  logging_enabled: true
  poll_interval_seconds: 10
```

Scepter 的 DTR/RTS 保持低；确认 Dongle 身份后开启其 CDC 日志镜像，RTS 保持低。
只读查询不改变增益、滤波、VAD、编码或空间模式。HMI 只能保存设备实际发出的数据，
不能恢复固件在发送前已经丢弃或截断的日志。
资源与 Dongle 分段查询会核对整组身份；跨窗口时最多追加两次整组重读，保留原始分段并记录混窗、重试和最终失败。
Scepter 还会查询 `SCEPTER_TELEMETRY` kind 6 的 17 段 `audio.dispatch` 快照，覆盖 PCM 派发队列等待/帧龄及队列满、失效、不可用、丢事件、高水位和迟到事件计数；各段按启动、会话、序号、codec 与空间模式校验，完整组才记为 `snapshot_complete`。旧固件不支持时记录一次 `query_unavailable`，不会伪造缺失字段或零值。
写入或刷新失败后清除尚不可靠的去重历史，允许设备重发缓存再次保存；正常重连继续保留去重历史。
Windows 串口接收已实现，其他平台返回明确的不可用状态。
可单独构建 `hmi_device_telemetry_probe`，验证接收与落盘而不加载语音模型。
诊断程序的 `--require 2` 要求两种设备均保存有效遥测、存在完整 Dongle 快照，且接收线程、查询和保存均通过检查。

## 语音行为

- 启动先预热识别，准备完成前麦克风保持关闭。
- 按住 `Ctrl+Alt` 打开麦克风；10 ms 采集块进入预分配环形缓冲，识别线程按最多 250 ms 的步长读取，SDK 传输保留自身偏好块。
- 默认约每秒生成可替换的累计草稿；慢后端按实际推理耗时降低预览频率。Overview 和独立悬浮采集条显示真实音量、时长和本次文字预览；悬浮条不抢焦点，主窗口最小化仍可更新。
- 默认启用静音检测，按 512 帧分析完整时间线，仅在有人声时请求新的预览；不裁掉静音，也不自动结束快捷键录音。短句和不确定的检测继续走原始识别。
- H-GTCRN 默认启用，在采集线程保留同步双通道，录音结束后由 native 执行整段去混响、分离和增强，再生成终稿；实时草稿仍使用原始音频。这不是流式 H-GTCRN。单声道、通道退化、短于 288 毫秒和增强失败均保留原始识别，失败不会复用上一次结果。可在配置中设置 `voice.hgtcrn_enabled: false`，下次启动生效。
- 松键关闭麦克风，排空尾块并经同一 SDK 会话修订全句。只有终稿会输入或执行命令。
- 达到单次录音上限时自动关麦，保留上限内的音频并完成终稿；实际松开按住说话的快捷键或结束切换监听后才绑定目标并输入。默认保留前30秒，较长内容请分次录音。
- 普通语句输入松键时的前台编辑框，不自动发送；保持目标窗口和输入框焦点。命令规则优先，规则文件每 2 秒检查更新。

微信使用整段 Unicode 粘贴兼容输入，以避开逐字输入时首字变问号的问题。确认微信读取后恢复原剪贴板；结果无法确认时会提示失败并停止重试。若剪贴板内容无法安全暂存，则本次输入保留原剪贴板并停止。

语音识别和合成均经 native SDK C ABI 调用 Qwen3-ASR/TTS，模型实现不加入 HMI 目标。累计前缀预览会重复计算部分音频，额外成本计入 RTF；目前不复用跨前缀的 KV 缓存，也不是逐 token 输出。预览最小间隔、最终修订和最长录音默认分别为 1 秒、开启、30 秒。松键后不再启动新的临时预览，终稿仍覆盖完整音频。已有本次有效预览的轻声录音继续完成终稿；纯静音、过短录音和没有有效预览的低音量录音仍会过滤。默认关闭 TTS 开关和预热。

TTS 朗读优先在合适的逗号等自然停顿处出声，首段通常最多 12 个 Unicode 码点；随后按上一段发音字符数逐步增加段长，上限 64，并优先保留完整分句，减少跨请求断词。整数、时间、千分位和小数词元保持完整（最多保护 64 码点；超长编号继续有界分段）。生成与播放并行，待播放 PCM 队列最多两段；同一轮朗读只打开一次 WASAPI client，首块预填后启动，最后一块排空后才报告完成。新朗读会替换旧内容，停止会丢弃旧队列；当前 SDK 调用仍需等待正在生成的短段返回。

TTS 默认闲置 60 秒释放模型、SDK 会话及参考音频，保留最多 4 条、合计 2 MiB 的短句 PCM 缓存（单条最多 512 KiB）。重复短句可直接重播，新的内容会自动加载模型。缓存只存于内存，停止或重新打开会话时清空；运行中更换模型或参考音频文件后应重启应用。诊断时可用 `HMI_TTS_IDLE_SECONDS=0..3600` 调整闲置时间（0 关闭释放），用 `HMI_TTS_PCM_CACHE=0|1` 控制缓存。测试新文本推理性能时应关闭缓存，重复句子命中数据单独比较。

TTS 默认通过 SDK 关闭主音码流和 subtalker 的随机采样，并按文本长度设置有限生成预算，减少短段偶发异常长输出。`HMI_TTS_DO_SAMPLE=1` 可显式恢复随机采样，在打开会话时生效。生成预算是每段 `32..2048` 个 codec frame，并非剪裁已生成 PCM；验收同时检查完整分段音频的全文回转写与末尾内容。

native 通过现有 artifact 接口报告每段的 EOS/预算封顶原因。封顶音频不播放、不缓存，失败段先尝试受限的备用采样，仍失败则有界拆分；无法恢复时明确报错。历史保留实际朗读内容，支持从头重播。新录音或关闭开关导致的中断单独记为 cancelled，不能算作完整播放。

开始录音时，已加载的 TTS 模型获得最多 45 秒的临时保活；终稿发布后给予 2 秒排队宽限，空录音、失败或关闭开关清除保活。保活不加载模型，不改变真正闲置时的释放策略。

开启自动朗读后，录音中的第一段有效草稿会请求提前准备 TTS 模型和会话，每次录音最多请求一次。准备复用现有 TTS 工作线程，不合成示例文字、不生成 PCM，也不覆盖上一条朗读统计；松键、静音空结果、采集失败或关闭开关不会新建准备任务。实际朗读仍只使用最终文本。加载和剩余录音重叠，减少定稿后的首包等待；加载期间会更早占用内存和部分 CPU/GPU，不能理解为减少推理量。准备后的模型仍按闲置 60 秒释放。

Overview 的自动朗读滑动开关默认关闭，点击或拖动后会保存 `voice.tts_enabled`，下次启动沿用。开启后，普通转写先输入目标输入框，再独立排队朗读同一份最终文本；草稿不朗读，命令规则保留原动作，已有朗读动作不重复播报。按住 Ctrl+Alt 说“朗读”或点击“试听”可触发默认问候语。关闭会停止播放并清掉待朗读内容和准备任务，保留文字输入及同一规则里的其它动作；再次开始录音会取消旧朗读和旧回读队列。正在执行的 SDK 调用不能硬中断，但返回后的旧 PCM 不再播放，界面不等待它结束。记录页分别显示识别和朗读耗时，自动回读日志标记 `source=asr_final`。启动 TTS 预热与开关分别控制；默认不做启动预热，录音中的提前准备只在开启朗读且有有效草稿时发生。

SDK 初始化日志逐项记录旧实例释放、路径解析、依赖预检、依赖预加载、DLL 加载、函数绑定、ABI 检查、配置和注册表/后端发现的开始、结束及耗时。`hmi_app_startup_probe --sdk-only` 可单独测量这些阶段，不加载模型或打开音频设备。构建本机 CUDA 版本可用 `tools/build.ps1 -Test -CudaArchitectures 89-real`；该参数用于 SDK 和可选算子诊断。CUDA/OpenMP 推理由 native SDK 提供，主程序不再编译无人调用的演示内核或 OpenMP 基准。

当前 SDK 公共 C ABI 会在整段请求结束后返回 PCM，因此采用宿主文本分段，不把 SDK 内部 codec 流式能力当作已暴露的实时接口。同步 `synthesize()` 保留完整 PCM 诊断用途，实际朗读 `speak()` 使用分段流水线。`hmi_voice_probe --tts-segments` 可无声测量生成与模拟播放；`--tts-muted-playback` 使用全零 PCM 验证真实连续播放设备，均不访问麦克风。

`hmi_voice_probe --stream --realtime --pcm FILE --readback-probe` 与同参数的 `--prepare-tts` 可无声对比定稿到首段 PCM；两者使用独立的 250 ms 有界供给线程，后者在首个有效预览后提前加载。计时包含未完成准备的等待，不代表实际可听首声。`hmi_tts_recovery_probe --prepare-wait-ms N` 单独测量提前准备与实际分段工作线程，准备时间和朗读首包分开报告。应用的首包和总耗时从接受朗读请求起算，`tts.queue_ms` 单列工作线程排队等待，推理耗时不重复累加提前准备时间。`playback.client_started_ms` 仍从播放函数调用起算，与首包比较时应加上工作线程排队时间；日志显式标记两者起点。准备日志使用 `prepare_requested`、`prepare_started`、`prepare_finished`，每个准备任务记录一次起止。

实际后端、线程配置、SDK 编译 OpenMP、进程 CPU/GPU、内存和缓冲状态由日志与诊断工具提供，按窗口汇总性能；草稿、终稿耗时及 RTF 单独记录。GPU 为本进程最忙引擎，包含界面渲染与推理。可选算子诊断见 [ggml 说明](docs/plan/ggml-kernel-probe.md)。

## 架构与进度

`main.cpp → platform::Desktop → app::App / Services → ui`；采集、SDK 适配、规则和动作层独立于 ImGui，通过加锁快照传递跨线程状态。摄像头目前只枚举设备，麦克风选择器尚未绑定采集端点，实际采集使用系统默认输入设备。

界面文件按实际职责保留：`ui/shell.*` 负责标题栏、导航和设置说明，`ui/theme.*` 负责主题与字体，`ui/device_controls.*` 负责设备页和麦克风选择器，`ui/pages/voice_page.*` 负责 Overview，`ui/pages/history_page.*` 负责记录和重播。左侧导航依次为 Overview、记录、设备、设置；顶部不再重复显示“使用 / 记录”切换。Ctrl+E 仍可切换记录与原使用页面。

2026-11-03 清理移除了无人调用的 `audio_bridge`、宿主 OpenMP/CUDA 演示代码、旧页面分支与无消费者的规则预览轮询。规则执行与文件热加载继续由 `VoiceRuntime` 负责，SDK 只由语音引擎加载路径持有。`tools/` 中的程序均为显式目标，部分诊断按需构建；它们不属于未使用文件。

以 [当前实现与验证记录](docs/plan/V1-report.md) 为准；节点任务卡和机器计划保留在 [V1-plan](docs/plan/V1-plan.md) 与 [V1-dag](docs/plan/V1-dag.json)。
