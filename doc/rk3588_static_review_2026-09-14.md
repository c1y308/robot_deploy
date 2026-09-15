# RK3588 人形机器人部署框架务实静态审阅（2026-09-14）

## 1. 审阅结论

审阅基线为 `cb35f06bc47cf0b435b612ef00cc3eea2523ed65`（2026-09-13）。本报告复核
[2026-09-07 旧报告](rk3588_static_review_2026-09-07.md)，并按 [rules.md](rules.md)
只讨论标准配置、正常启停／模式切换以及通信抖动、策略超时、传感器单帧丢失、
单轴故障这类现实条件。

**总体评分：5.9 / 10。** 相比旧报告，策略目标期限、控制命令期限、STOP 独立通道、
全身故障锁存、shutdown 顺序以及 RK3588 线程／主机配置已经明显加强。当前仍不建议
直接放行无保护行走实机，原因不是旧 B2/B3，而是传感器和 PDO 的“有效数据时间”
尚未闭环：不完整 WKC 周期会被赋予新时间，1 kHz 命令线程也会用未检查年龄的缓存反馈。
此外，当前板端没有通过新主机预检，`policy_test` 会在硬件初始化前正确拒绝启动。

| 维度 | 分数 | 结论 |
|---|---:|---|
| 运动安全与故障收敛 | 7.5 | STOP、命令过期和单轴故障整机锁存已形成可测试的失效保护链 |
| 实时确定性 | 5.5 | CPU6/7、FIFO70/80 与 Torch 线程数已固定；EtherCAT 周期迟到处理和板端时延证据仍不足 |
| 数据新鲜度与同步 | 4.0 | 策略入口有年龄守卫，但 WKC 无效数据、控制反馈年龄、IMU 分帧一致性仍有缺口 |
| 配置与模型一致性 | 5.5 | 当前 `policy.pt` 的 705→12 形状匹配；速度范围和 gait 语义没有由配置真正约束 |
| 部署与可追溯性 | 7.0 | 主机脚本、预检、fixture 和日志序号均有改善；当前主机状态及部分日志语义仍未闭环 |

### 部署判定

- **代码级无硬件回归：通过。** 五个独立 Debug 构建均成功，CTest 分别为 3/3、3/3、
  4/4、14/14、1/1。
- **当前 RK3588 主机准备状态：不通过。** `robot-rt-setup.sh check` 和 `policy_test`
  均首先报告 cpufreq governor 为 `ondemand`；进一步只读检查还发现 EtherCAT 主站 MAC、
  IRQ、NetworkManager guard、ready marker 和 `/dev/EtherCAT0` 未达到当前 profile 要求。
- **实机放行：暂缓。** 先完成本文 P1 整改／证据门槛和板端预检，再在悬挂或支撑台架上
  完成 STOP、单轴故障、WKC 丢帧和正常负载时延验收。

## 2. 本次审阅边界

覆盖链路为：Xsens/CAN IMU、电机反馈、策略观测、TorchScript 推理、策略目标期限、
1 kHz 命令生成、EtherCAT PDO、STOP、故障锁存和 shutdown；同时检查 CPU6–7 隔离、
线程亲和性、FIFO70/80、Torch/OpenBLAS 线程数、主机预检、IRQ 和 NetworkManager 防护。

未把以下内容列为问题：蓄意灌满队列、多故障同时发生、无限磁盘阻塞、超长时间戳回绕、
恶意内部输入、运行时替换异常模型、理论上的罕见竞态及不受支持配置。本次也不展开 NPU
迁移设计，没有执行安装、系统调参、真实 EtherCAT/CAN 写入或电机程序。

## 3. 旧报告 B1–B3、R1–R15 复核

| 编号 | 当前状态 | 复核结论 |
|---|---|---|
| B1 | **部分修复** | 策略结果与控制命令都有不可续租期限，过期会锁存并 STOP；但命令工作线程仍可使用未校验年龄的缓存电机反馈，见 P1-2 |
| B2 | **已修复** | STOP 使用独立 mailbox，不受普通命令队列容量影响；RT 周期优先处理且按有效 PDO 回读确认，shutdown 先停机再清理 |
| B3 | **已修复** | 单轴 offline、驱动故障、意外失能或模式漂移会锁存全身故障并对所有轴 quick-stop；故障后 RESTART 和新 setpoint 被拒绝 |
| R1 | **部分修复** | `policy_cmd` 已固定 CPU6/FIFO70，`ecat_rt` 已固定 CPU7/FIFO80；独立周期使用缓存反馈的问题仍在，见 P1-2 |
| R2 | **仍存在** | EtherCAT 1 kHz 循环仍忽略绝对睡眠结果及迟到周期，见 P1-3 |
| R3 | **仍存在** | SocketCAN 仍在用户态 `read` 完成后取主机单调时间，排队帧会显得更新，见 P1-4 |
| R4 | **部分修复** | 每次发布后会清除 quaternion/rate fresh 标志，但字段仍没有共同样本标识或配对跨度，见 P1-4 |
| R5 | **仍存在** | 四元数只检查计算结果有限，没有范数约束；并入 P1-4 处理 |
| R6 | **仍存在** | WKC 完整性参与看门狗，却没有参与本周期反馈有效性和时间戳，见 P1-1 |
| R7 | **已修复** | shutdown 和策略失败入口都先请求 STOP，再 join、卸载模型和关闭记录器 |
| R8 | **未证实为缺陷** | 踝 FK/IK 循环有界，但目标板正常工作域 WCET 尚未测量；保留为实机验收项，不凭静态上界判定超时 |
| R9 | **已改善，无当前高置信缺陷** | recorder 已在后台线程，Torch/OpenBLAS 线程数和 OpenMP dynamic 已固定并有读回测试；正常负载尾延迟仍需实测 |
| R10 | **部分修复** | 日志已随 `policy_seq` 对齐第一条命令，但 `command_applied=true` 实际只证明发布到 latest 通道，见 P2-1 |
| R11 | **仍存在** | 705 分支 gait 参数仍由代码默认值提供，YAML 无法表达，见 P1-5 |
| R12 | **本次排除** | 当前固定模型启动时已做形状检查，实际模型输入输出也已验证；动态替换数据相关异常模型不属于本次现实场景 |
| R13 | **仍待版本证据** | 代码与仓库 ESI 的 padding 对象仍不一致，见 P1-6；不据此推断主字段已错位 |
| R14 | **仍存在** | IgH adapter 仍忽略 `ifname` 并固定请求 master 0，见 P2-2 |
| R15 | **仍存在** | reset 启动 FK 不可达时仍静默使用目标默认姿态作为插值起点，见 P2-3 |

已确认的关键改进包括：

- [robot_interface.cpp#L685](../src/inference/src/robot/robot_interface.cpp#L685) 从最老观测时间计算
  策略截止期，并在推理后再次检查；[robot_interface.cpp#L495](../src/inference/src/robot/robot_interface.cpp#L495)
  将控制命令有效期限制为策略期限与独立 10 ms 期限的较小值。
- [motor_controller_base.cpp#L708](../src/motors/src/motor_base/motor_controller_base.cpp#L708)
  在 RT 消费端验证并锁存命令过期；[motor_controller_base.cpp#L319](../src/motors/src/motor_base/motor_controller_base.cpp#L319)
  的 STOP 独立于普通队列。
- [myact_motor_controller.cpp#L344](../src/motors/src/drivers/myact/myact_motor_controller.cpp#L344)
  将单轴现实故障收敛为整机锁存，[myact_motor_controller.cpp#L434](../src/motors/src/drivers/myact/myact_motor_controller.cpp#L434)
  对全轴应用 quick-stop。
- [rk3588_runtime_profile.hpp#L32](../src/inference/include/robot/rk3588_runtime_profile.hpp#L32)
  固定线程布局和 Torch/OpenBLAS 线程数；[thread_runtime.hpp#L53](../src/base/include/tool/thread_runtime.hpp#L53)
  对名称、亲和性和调度参数设置后读回验证，失败时业务线程不会进入主体。
- [policy_test.cpp#L82](../src/inference/examples/policy_test.cpp#L82) 在打开 Xbox、模型和机器人硬件前
  强制执行主机预检，当前配置错误会快速失败。

## 4. 当前高置信问题

### P1-1：WKC 不完整周期仍发布带新时间戳的旧／不完整电机反馈

**源码证据。** [myact_motor_controller.cpp#L286](../src/motors/src/drivers/myact/myact_motor_controller.cpp#L286)
每周期接收后无条件生成 `current_cycle_host_timestamp_ns_`；
[myact_motor_controller.cpp#L311](../src/motors/src/drivers/myact/myact_motor_controller.cpp#L311)
计算 `process_data_ok`，但 [L317](../src/motors/src/drivers/myact/myact_motor_controller.cpp#L317)
仍只凭单轴 online/operational 读取域数据。随后 [L931](../src/motors/src/drivers/myact/myact_motor_controller.cpp#L931)
和 [L973](../src/motors/src/drivers/myact/myact_motor_controller.cpp#L973) 都把本周期新时间写入反馈，
`comm_ok` 也仍来自单轴配置状态。通信看门狗到 10 个连续坏周期才锁存，并在一个完整周期后清零
（[L391](../src/motors/src/drivers/myact/myact_motor_controller.cpp#L391)）。现有测试明确覆盖 9 个坏周期
不锁存及 10 个坏周期锁存（[motor_realtime_channel_test.cpp#L1509](../src/motors/tests/motor_realtime_channel_test.cpp#L1509)），
但没有检查坏周期的反馈有效性和时间戳。

**现实触发与影响。** EtherCAT 短暂通信抖动造成少于看门狗阈值的 incomplete WKC 时，域缓冲可能是
上一周期或不完整数据，却会被策略年龄守卫当作刚收到的样本。当前踝控制会据此计算 FK、速度和力矩。

**最小修复。** 保留现有看门狗容忍窗口，但将“故障锁存”和“本周期数据有效”分开：只有完整 WKC
才更新 observed 值及 last-valid 时间；对外快照携带本周期有效标志或最后有效时间。策略观测和 1 kHz
控制必须按 last-valid 时间判断，而不是按循环时间判断。

**验收。** FakeAdapter 注入 1–9 个 incomplete WKC：不得给反馈续新时间，策略／控制在既定年龄上限后
停止使用该数据；恢复完整 WKC 后只有真实新 PDO 才更新时间。10 个连续坏周期仍应锁存整机保护。

### P1-2：1 kHz 命令线程会复用未校验年龄的缓存电机反馈

**源码证据。** [robot_interface.cpp#L474](../src/inference/src/robot/robot_interface.cpp#L474) 仅在有新快照时
替换 `motor_feedback`，首次取得后可无限复用；[action_processor.cpp#L267](../src/inference/src/robot/action_processor.cpp#L267)
检查 `comm_ok/enabled/faulted/control_ready` 和关节范围，但不检查 `host_timestamp_ns`。
[robot_interface.cpp#L495](../src/inference/src/robot/robot_interface.cpp#L495) 随后用当前时间生成一条“新”命令期限。
同时，电机 RT 主循环在本周期接收 PDO 前先消费 setpoint
（[motor_controller_base.cpp#L223](../src/motors/src/motor_base/motor_controller_base.cpp#L223)）。

**现实触发与影响。** EtherCAT 线程因一次调度延迟停顿数毫秒，而 CPU6 上的命令线程继续运行时，
它会反复用最后一次反馈生成仍在有效期内的新命令。EtherCAT 线程恢复后可能先消费该命令，再接收新 PDO；
并联踝的纯力矩项因此与真实位置／速度脱节。策略入口的 50 Hz 传感器年龄守卫不能保护这条独立 1 kHz 路径。

**最小修复。** 在命令线程使用 `MotorStatusSnapshot::host_timestamp_ns`，按明确的控制反馈年龄上限快速失败；
P1-1 修复后必须使用 last-valid PDO 时间。若不调整循环次序，至少保证恢复后的旧反馈命令在 RT 消费前已经过期。

**验收。** 暂停 FakeAdapter/RT 周期但保持 `policy_cmd` 运行，确认反馈超过上限后请求 STOP，且不会通过刷新
`produced_at_ns` 延长旧反馈命令寿命；恢复时第一条动作必须基于恢复后的有效 PDO。

### P1-3：EtherCAT 绝对周期睡眠忽略迟到，可能连续追赶历史周期

**源码证据。** [motor_controller_base.cpp#L218](../src/motors/src/motor_base/motor_controller_base.cpp#L218)
每轮把 `next_period` 固定加 1 ms，再调用 `clock_nanosleep`；返回值未处理，也没有将已经过去的期限跳到
未来。相比之下，策略命令线程在迟到超过一个周期时会重置 `next_wake`
（[robot_interface.cpp#L536](../src/inference/src/robot/robot_interface.cpp#L536)）。

**现实触发与影响。** 正常负载下的一次调度延迟或周期执行超时即可使绝对截止点落在过去；后续循环会无睡眠
连续追赶，1 ms 滤波／状态机步长假设被破坏，并暂时增加 CPU7 占用。当前没有 deadline-miss 计数能从日志定位。

**最小修复。** 对同一绝对期限正确处理 `EINTR`；唤醒或周期结束后比较单调时钟，错过整周期时跳过历史期限
并记录迟到量／miss 计数。是否因单次 miss 停机应由实机预算决定，不在未测量前增加复杂策略。

**验收。** 无硬件测试注入一次 3–5 ms 调度停顿，恢复后不得出现多个背靠背回调；miss 计数和最大迟到量
应可观测，稳态周期恢复到未来绝对时间点。

### P1-4：Xsens 分帧数据仍可能跨样本组合，主机时间也不能表示真实到达时间

**源码证据。** [can_parser.cpp#L109](../src/imu/src/protocol/xsens_mti/can_parser.cpp#L109) 和
[L126](../src/imu/src/protocol/xsens_mti/can_parser.cpp#L126) 分别覆盖同一 `AHRSData`，任一帧都会覆盖统一的
`receive_timestamp_ns`；[L170](../src/imu/src/protocol/xsens_mti/can_parser.cpp#L170) 只要求两个 fresh 布尔值，
`SampleTime` 没有参与配对。测试还明确允许复用旧 SampleTime，甚至没有 SampleTime 也发布
（[xsens_mti_can_parser_test.cpp#L88](../src/imu/tests/xsens_mti_can_parser_test.cpp#L88)）。
[socket_can_port.cpp#L129](../src/imu/src/drivers/socket_can_port.cpp#L129) 使用 `read` 而非 `recvmsg`，并在读取完成后
才调用 `monotonic_now_ns()`。此外，[can_parser.cpp#L140](../src/imu/src/protocol/xsens_mti/can_parser.cpp#L140)
没有检查四元数范数，全零四元数仍能产生有限的重力分量并被标为 valid。

**现实触发与影响。** 一次 quaternion 或 rate 帧丢失、或者读取线程短时未被调度，就可能把两个采样周期的
字段组成一帧，并给排队旧帧赋予较新的主机时间。策略侧的 50 ms 年龄和 30 ms IMU/电机 skew 检查因此可能放行
不一致姿态；设备刚启动时的零／未稳定四元数也可能伪装成直立。

**最小修复。** 为 quaternion、rate 和 SampleTime 分别保存样本标识与接收时间，只发布同一采样或配对跨度在
明确上限内的组合；SocketCAN 使用内核接收时间并记录接收溢出；在外部传感器边界检查四元数范数，允许小误差
归一化，零值或明显异常直接拒绝。

**验收。** 解析器测试只丢一个字段并按下一正常样本继续喂帧，旧字段不得与新字段组合；排队帧不能获得出队时刻
的新鲜度；零四元数拒绝，近单位四元数归一化后通过。

### P1-5：部署配置声明的速度范围未生效，705 gait 语义也无法由 YAML 表达

**源码证据。** 当前 [deploy.yaml#L7](../src/inference/config/deploy.yaml#L7) 把 `lin_vel_x/y` 和 `ang_vel_z`
都声明为 `[0.0, 0.0]`；加载器却将三组数读入 `ignored_range`
（[deploy_config.cpp#L481](../src/inference/src/config/deploy_config.cpp#L481)）。Xbox 正常映射可产生
`vx=±0.3 m/s`（[xbox_controller.cpp#L288](../src/xbox_control/src/xbox_controller.cpp#L288)），`policy_test`
直接送入策略（[policy_test.cpp#L147](../src/inference/examples/policy_test.cpp#L147)）。

默认编译启用 705 维观测（[CMakeLists.txt#L9](../src/inference/CMakeLists.txt#L9)），实际模型也只接受 705 维；
运行时会插入 gait phase（[observation_builder.cpp#L272](../src/inference/src/robot/observation_builder.cpp#L272)），
但加载器的 observation 白名单没有 `gait_phase`
（[deploy_config.cpp#L643](../src/inference/src/config/deploy_config.cpp#L643)）。代码默认 period 为 0.74 s
（[robot_config.hpp#L73](../src/inference/include/robot/robot_config.hpp#L73)），YAML 注释示例却是 0.6 s。

**现实触发与影响。** 用户正常推动摇杆即可让观测命令超出部署文件声明范围。仓库无法单独证明当前模型究竟只训练
站立还是支持 ±0.3 m/s，也无法证明 gait period 应为 0.74 还是 0.6；因此问题是部署契约未闭合，而不是本报告
武断判定某个数值错误。错误语义会直接改变 15 帧历史观测和策略输出。

**最小修复。** 以训练导出配置／黄金观测为唯一依据：若当前模型只支持站立，禁止 Xbox 非零速度；若支持行走，
把真实范围存入运行配置并在用户输入边界限幅。705 分支应加载并验证 gait 参数，675 分支明确拒绝；同时删除与实际
默认值冲突的注释示例。

**验收。** 用一个非零摇杆输入验证范围生效；从同一训练样本生成完整 705 维黄金观测，逐字段比较顺序、缩放、
15 帧历史和 gait 值，再比较 12 维动作。

### P1-6：代码与仓库 ESI 的 PDO padding 对象版本仍不一致

**源码证据。** [ethercat_adapter_igh.cpp#L59](../src/motors/src/protocol/ethercat/ethercat_adapter_igh.cpp#L59)
注册 `0x2ffd/0x2ffe`，而仓库 `MT-Device 250702.xml` 的同一 0x1601/0x1A00 映射使用
[0x5FF1](<../src/motors/datasheet/myact/EtherCAT%20ESI/MT-Device%20250702.xml#L1867>) 和
[0x5FF2](<../src/motors/datasheet/myact/EtherCAT%20ESI/MT-Device%20250702.xml#L1918>)。

**现实触发与影响。** 使用与仓库 ESI 对应的驱动器固件部署时，主站 PDO 配置可能直接失败；若现场固件是另一版，
则代码可能正确。仅凭静态差异不能断言主字段错位或电机会异常运动，但在上电前必须固定版本证据。

**最小修复。** 回读实际驱动器固件及 0x1C12/0x1C13、0x1601/0x1A00 assignment/mapping，选定唯一受支持版本，
使代码、ESI 和部署记录一致。

**验收。** 所有 12 轴身份／固件一致，主站激活无 PDO registration 错误，实际 assignment/mapping 与代码逐项相同，
并验证完整 WKC。不要用结构体 `sizeof` 代替对象字典核对。

## 5. P2 可追溯性与启动问题

### P2-1：`command_applied` 只表示命令已入 latest 通道

[robot_interface.cpp#L509](../src/inference/src/robot/robot_interface.cpp#L509) 在
`apply_impedance_setpoints_realtime` 返回后立即写 `command_applied=true`；底层成功条件只是
[motor_controller_base.cpp#L411](../src/motors/src/motor_base/motor_controller_base.cpp#L411) 发布到 SPSC latest 通道，
并不证明 RT 已消费、PDO 已发送或驱动器已响应。最小改法是把字段改成准确的 `command_enqueued`；若定位链路确实需要，
再由 RT 快照补充 consumed/sent 序号。验收时制造命令发布后、RT 消费前的可控停顿，日志不得声称已发送。

### P2-2：`ifname` 不选择实际 IgH 主站

[ethercat_adapter_igh.cpp#L116](../src/motors/src/protocol/ethercat/ethercat_adapter_igh.cpp#L116) 丢弃 `ifname` 并固定
`ecrt_request_master(0)`。RK3588 新脚本能按物理 GMAC/MAC 配置 master 0，因此当前固定 profile 可以工作；但上层打印的
接口名并非选择依据，现场改名也不会切换总线。最小修复是把配置项改为明确的 master index，启动日志同时打印 IgH
master 绑定的实际 MAC／物理设备。验收需故意给一个不同的上层接口名，程序必须拒绝不一致或明确显示仍使用 master 0。

### P2-3：reset 启动 FK 失败会静默改用目标姿态

[action_processor.cpp#L523](../src/inference/src/robot/action_processor.cpp#L523) 从真实踝电机角求启动脚姿态；不可达时
[L538](../src/inference/src/robot/action_processor.cpp#L538) 把插值起点设成目标默认 pitch/roll 并继续成功。单次编码器姿态
位于求解域外或求解不收敛时，复位轨迹的起点不再代表当前脚位。最小修复是启动阶段快速失败并报告两电机角和求解结果；
无硬件测试输入一个有限但不可解的反馈，必须拒绝 reset 且不发送插值动作。

## 6. 当前 RK3588 板端只读证据

本次未安装或修改系统。板端是 `Linux 6.1.99-rt36-rk3588 PREEMPT_RT`，CPU0–7 在线。启动参数已经包含
`isolcpus=domain,managed_irq,6-7 rcu_nocbs=6-7 irqaffinity=0-5`，有效 isolated CPU 为 6–7，unbound
workqueue mask 为 `3f`；这部分符合新 profile。

| 检查项 | 当前读回 | 判定 |
|---|---|---|
| cpufreq policy0/4/6 | 均为 `ondemand` | 不符合 `performance` 要求，也是预检首个失败项 |
| 目标 EtherCAT 物理 GMAC | `fe1c0000.ethernet` → `eth0`, MAC `fa:fd:53:a0:a5:55` | 物理解析和永久 MAC 正常 |
| IgH `main_devices` 配置／已加载参数 | 均为 `f6:fd:53:a0:a5:55`（当前 `eth1`） | 与目标 `fe1c`/`eth0` 不一致 |
| CAN0 IRQ 77 | effective affinity `7` | 期望 CPU2 |
| eth1 IRQ 142/143 | effective affinity `0` | 当前主站仍指向非目标 GMAC；目标 eth0 未出现可核对 IRQ |
| NetworkManager unmanaged | 已按目标 MAC `fa:...` 排除 | 此项匹配 |
| NetworkManager guard drop-in | 缺失 | 不符合 |
| `/run/robot-rt-layout.ready` | 缺失 | 本次启动未通过 oneshot 验收 |
| `/dev/EtherCAT0` | 缺失 | IgH 用户态设备不可用 |

只读运行 `bash deploy/rk3588/robot-rt-setup.sh check` 返回 1：

```text
[robot-rt] ERROR: cpufreq policy0 governor is ondemand, expected performance
```

独立 Debug 构建的 `policy_test deploy.yaml` 同样在
[policy_test.cpp#L84](../src/inference/examples/policy_test.cpp#L84) 预检处返回 1，尚未打开 Xbox、加载模型或初始化电机。
按 [RK3588 部署说明](../deploy/rk3588/README.md) 完成 `install`、受控重启及 `check` 后，才能继续硬件验收；
本次没有执行这些改变系统状态的步骤。

## 7. 最小整改顺序

1. **先恢复部署前提。** 按现有脚本安装、重启并使 `robot-rt-setup.sh check` 全部通过；保留当前 fail-fast 预检。
2. **修正数据有效性。** P1-1 让 WKC 决定 PDO 样本是否可更新时间，P1-2 让控制计算按 last-valid 反馈年龄停止。
3. **修正周期和 IMU 一致性。** P1-3 跳过错过的 EtherCAT 周期并暴露 miss；P1-4 完成同样本配帧、真实接收时间和四元数边界检查。
4. **关闭模型／硬件版本契约。** 用训练导出材料决定速度与 gait 参数，用驱动器回读决定 PDO 版本；没有证据前不猜默认值。
5. **补齐诊断语义。** 精确命名 command 状态，明确 master 0 的实际设备，reset FK 不可达时快速失败。
6. **最后做实机放行。** 在支撑台架按下一节验收，确认后再进入正常行走测试。

## 8. 已执行验证

所有 C/C++ 构建均使用全新的 `/tmp/robot_review_20260914_*` 目录和 `CMAKE_BUILD_TYPE=Debug`，避免 Release
下 `assert` 被移除造成误判。

| 项目 | 结果 |
|---|---|
| `src/base` Debug build + CTest | 3/3：SPSC、RingBuffer、thread runtime |
| `src/imu` Debug build + CTest | 3/3：Xsens parser、SocketCAN config、A100 parser |
| `src/motors` Debug build + CTest | 4/4：motor realtime channel 及 base 三项 |
| `src/inference` Debug build + CTest | 14/14：配置、IMU、动作／踝、RK3588 profile/preflight、Torch threading、motors/base 等 |
| `src/xbox_control` Debug build + CTest | 1/1：摇杆映射 |
| `bash deploy/rk3588/test_robot_rt_setup.sh` | 通过：apply 幂等、check、物理接口解析、过期 MAC、NetworkManager guard、IRQ/nohz 等 fixture |
| `python3 src/inference/src/recorder/plot_motor_tracking_test.py` | 通过；测试用缺列电机产生预期 skip warning |
| 实际 `policy.pt` CPU 检查 | PyTorch 2.5.1；705 输入得到 `[1,12] float32` 且全有限，675 输入被拒绝 |
| `policy_runtime_threading_test` | 通过；ATen intra-op=2、inter-op=1、OpenBLAS=1、OpenMP dynamic=off 均读回一致 |
| 当前工作区 `git diff --check` | 通过 |

构建配置阶段仍有可选 `kineto_LIBRARY-NOTFOUND` 警告，但链接和上述测试全部成功，因此不列为当前部署缺陷。
现有测试已经很好覆盖 STOP、命令时间戳、单轴故障、线程配置和主机 fixture；尚缺的是 P1-1 至 P1-5 所列的
数据有效性／配置语义断言，而不是更多病态压力测试。

## 9. 必要实机验收项

以下只在电机离地或有机械支撑、急停可用的台架进行：

1. `robot-rt-setup.sh check` 全通过；读回 `policy_main`、`policy_cmd`、`ecat_rt` 的 CPU、调度类和优先级，
   分别符合 CPU4–5/OTHER、CPU6/FIFO70、CPU7/FIFO80；CAN 和 EtherCAT IRQ 分别在 CPU2/CPU3。
2. 核对 12 轴固件与实际 PDO mapping，达到稳定完整 WKC；注入一次短暂 WKC 丢失，确认旧 PDO 不续新时间且动作年龄保护生效。
3. 正常策略负载下记录 EtherCAT wake-up lateness、周期执行时间、命令反馈年龄、策略推理时间和端到端动作年龄的最大值／高分位；
   一次现实调度延迟后无背靠背追赶。
4. Xsens 单字段丢帧测试确认不跨样本组合，内核接收时间和设备 SampleTime 的关系可解释；零／异常范数姿态不能进入策略。
5. 在支撑台架逐项触发 STOP、策略超时、命令线程停止、单轴 offline/故障/失能/模式错；所有轴进入 quick-stop，
   STOP 只有在有效 PDO 确认失能后完成，故障消失不得自动 RESTART。
6. 使用训练端黄金 705 维观测核对速度范围、gait、历史顺序、缩放、关节映射和 12 维动作，再进行非零速度测试。
7. 正常启动姿态覆盖踝机构实际工作域；reset FK 不可达必须拒绝动作并给出可定位日志。

完成条件是：P1-1 至 P1-6 已修复或取得明确版本证据，板端预检全通过，上述台架验收没有数据新鲜度、
deadline miss、WKC、STOP 或全身故障锁存违约。达到这些条件前，本报告只支持继续无硬件回归和受保护台架整改，
不支持无保护行走放行。
