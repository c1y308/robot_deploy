# RK3588 当前代码务实审阅（2026-10-22）

## 1. 结论与审阅基线

本次以 [2026-09-14 报告](rk3588_static_review_2026-09-14.md) 为线索，重新检查当前源码及实际调用链，未直接继承旧报告的缺陷结论。

基线为 `112992eda06cce05d4ea43d53ab35af9fafb56e3` **加审阅时的工作区改动**，包括已迁移的 base 头文件、测试文件删除，以及尚未提交的 reset FK 快速失败修复。本文评价当前磁盘上的代码，不等同于只评价该 Git 提交。

**本次确认 4 项需要处理的问题：2 项 P1、2 项 P2。** P1 表示优先修复的构建阻断或周期执行问题，P2 表示特定正常故障场景下的退出问题或诊断误导；不将这些等级等同于实机危险程度评分。

| 编号 | 优先级 | 问题 | 证据强度 |
|---|---|---|---|
| N1 | P1 | 删除测试源码后，CMake 仍无条件引用，导致 inference 和 Xbox 独立工程无法生成构建系统 | 干净目录配置复现 |
| N2 | P1 | EtherCAT 循环迟到后仍追赶历史期限，出现多个背靠背周期 | 实际基类线程循环注入 4 ms 停顿复现 |
| N3 | P2 | 首次推理前持续断线，STOP 无法确认且通信故障不锁存，退出会反复等待 | FakeAdapter 复现底层状态，上层退出路径静态确认 |
| N4 | P2 | `command_applied` 实际只表示提交到 latest 通道，不能证明 RT 已应用 | 完整调用链核对 |

旧报告 P1-1、P1-2、P2-2、P2-3 已有对应修复。旧 P1-4、P1-5、P1-6 按本次用户确认的设计边界排除。旧 P1-3 和 P2-1 分别对应本文 N2 和 N4。

本轮只新增审阅报告，没有修改运行代码、恢复已删除测试或改变系统配置。不沿用旧报告的 5.9 分，也不将 9 月 14 日的主机读回结果当作当前状态。

## 2. 范围及明确排除项

检查范围包括策略入口、观测构建、模型加载/推理、命令 worker、复位、EtherCAT 反馈及 STOP/故障处理、Xsens 接收和发布、线程配置、部署脚本、日志与 CMake。

按用户要求，本次采用以下约定：

- **旧 P1-4 不作为问题：** IMU 使用各字段最新值，不要求 quaternion、rate、SampleTime 属于同一采样帧，也不要求 IMU 与电机严格同步。仍核对实际接收时间及已有年龄限制是否有效，但不增加同帧配对要求。
- **旧 P1-5 不作为问题：** YAML 速度范围不约束当前输入，以及 gait 配置表达能力，不作为本次缺陷或整改前置条件。另外，当前加载器已支持 `observations.gait_phase` 和 `params.period`，旧文中“完全无法表达”的描述也不再准确，见 [deploy_config.cpp](../../src/inference/src/config/deploy_config.cpp#L676)。
- **旧 P1-6 不作为问题：** 不根据仓库 ESI padding 对象差异推断现用设备有错误，不将其列入待整改项。
- 不讨论恶意内部输入、无限灌满队列、多种故障同时发生、数值回绕、任意更换异常模型等场景。不以缺少理论防御分支作为缺陷。
- 一次数毫秒调度延迟、启动期间一次实际断线、正常重新构建和日志分析属于本次范围。

无硬件测试使用临时目录及 FakeAdapter。未运行 `policy_test`、真实 EtherCAT/CAN 收发、安装脚本或主机调参；未执行实际主机 `check`，因此没有本次板端预检通过/失败结论。

## 3. 旧报告逐项复核

| 旧编号 | 当前结论 | 当前源码依据与说明 |
|---|---|---|
| P1-1：坏 WKC 刷新反馈时间 | **已修复** | [myact_motor_controller.cpp](../../src/motors/src/drivers/myact/myact_motor_controller.cpp#L351) 只有 `process_data_ok && comm_ok` 才读取 PDO、更新 observed 和 `last_valid_host_timestamp_ns`；[反馈发布](../../src/motors/src/drivers/myact/myact_motor_controller.cpp#L994)使用该有效时间。已执行 1–9 个坏周期、恢复完整周期、10 个坏周期锁存等定向验证。 |
| P1-2：worker 无限复用旧反馈 | **已修复** | [policy_command_worker.cpp](../../src/inference/src/robot/policy_command_worker.cpp#L225) 取最老电机时间，在计算前后检查年龄；[命令期限](../../src/inference/src/robot/policy_command_worker.cpp#L145)还封顶到反馈时间加年龄上限。超过 20 ms 的旧反馈不能通过刷新命令产生时间续命。本项主要为源码复核，未重新执行完整 worker 端到端停顿测试。 |
| P1-3：EtherCAT 追赶历史周期 | **仍存在** | 见 N2；实际循环复现。 |
| P1-4：IMU 跨样本组合 | **按设计排除** | 不要求同帧。附带的旧时间戳问题已有修复：SocketCAN 使用内核 RX 时间；组合时间取 quaternion/rate 接收时间较早者。未要求进一步同帧化。 |
| P1-5：速度范围/gait | **按设计排除** | 当前仍读取但不应用速度范围；用户确认接受。gait 已有 YAML 入口，不再沿用旧版本描述。 |
| P1-6：ESI padding 差异 | **按设计排除** | 不要求本次修改映射或补充版本证明。 |
| P2-1：`command_applied` 含义 | **仍存在** | 见 N4。 |
| P2-2：忽略 `ifname` | **已修复** | `38e7675` 已删除 `ifname` 调用链；[适配器](../../src/motors/src/protocol/ethercat/ethercat_adapter_igh.cpp#L128)明确使用 master 0，网卡绑定由系统主站配置决定。固定单主站本身不列为问题。 |
| P2-3：reset FK 失败回退目标 | **当前工作区已修复** | [action_processor.cpp](../../src/inference/src/robot/action_processor.cpp#L453) 在任一脚踝失败时返回 false，输出侧别、电机角、reachable/pitch/roll；[调用方](../../src/inference/src/robot/robot_interface.cpp#L248)在插值前退出。正常重建及左右失败回归通过。 |

旧报告归档说明曾将 reset FK 回退列为接受行为；本次以用户随后明确要求的快速失败修复及当前源码为准。

其他已确认的行为：

- STOP 仍使用独立请求槽；确认前拒绝重启，整机离线故障会锁存。定向执行的 STOP 退休旧 setpoint、单轴 offline 整机锁存验证通过。
- [SocketCAN](../../src/imu/src/drivers/socket_can_port.cpp#L57)启用 `SO_TIMESTAMPNS_NEW`，通过 [recvmsg](../../src/imu/src/drivers/socket_can_port.cpp#L175)读取控制消息；读取排队数据不会直接赋予出队时刻作为接收时间。
- [Xsens 发布](../../src/imu/src/protocol/xsens_mti/can_parser.cpp#L186)保留独立字段更新，使用两字段接收时间的最小值；[四元数处理](../../src/imu/src/protocol/xsens_mti/can_parser.cpp#L131)会拒绝零/过小范数。本次不要求扩展为任意异常姿态的防御框架。
- [RK3588 配置](../../src/inference/include/robot/robot_config.hpp#L163)已合并到 `robot_config.hpp`：推理 CPU4–5/OTHER，命令 CPU6/FIFO70，EtherCAT CPU7/FIFO80，IMU CPU2，后台 CPU0–1，计算库线程数 2/1/1。
- 当前策略采用“观测到动作年龄不超过 40 ms 才准入，已发布目标最多保持 60 ms，控制命令另受 10 ms 与反馈年龄限制”的规则，见 [准入](../../src/inference/src/robot/robot_interface.cpp#L362)、[发布](../../src/inference/src/robot/policy_command_worker.cpp#L111)及[命令期限](../../src/inference/src/robot/policy_command_worker.cpp#L145)。这是当前明确的时间预算，不直接套用旧报告中从最老观测起算目标期限的描述。

## 4. 当前问题及最小建议

### N1 / P1：残留测试目标阻断正常构建

**位置：** [motors/CMakeLists.txt](../../src/motors/CMakeLists.txt#L55)、[xbox_control/CMakeLists.txt](../../src/xbox_control/CMakeLists.txt#L25)。

当前工作区已删除：

```text
src/motors/tests/motor_realtime_channel_test.cpp
src/xbox_control/tests/xbox_controller_mapping_test.cpp
```

对应 `add_executable`、链接和 `add_test` 仍在。inference 无条件引入 motors 子目录，因此不是只影响测试运行，正常配置 inference 也会失败。

**实际复现：**

```bash
cmake -S src/inference -B /tmp/rk3588-review-20261022.DKwpww/inference -DCMAKE_BUILD_TYPE=Debug
cmake -S src/xbox_control -B /tmp/rk3588-review-20261022.DKwpww/xbox -DCMAKE_BUILD_TYPE=Debug
```

两者均在生成阶段失败，分别报告缺少 `tests/motor_realtime_channel_test.cpp`、`tests/xbox_controller_mapping_test.cpp` 及目标没有源文件。Torch 的可选 kineto 警告不是这次失败的原因。

**影响：** 清理构建目录、换目录构建或正常重新配置即可触发；已有旧二进制能够运行并不能证明当前源码可以重新构建。这是当前工作区删除操作与 CMake 不一致造成的问题，不据此认定基线提交本身同样失败。

**最小建议：** 按已确定的删除测试意图，移除这两个孤立目标的 `add_executable`、`target_link_libraries` 和 `add_test`。不需要恢复整个测试目录，也不需要增加新的构建抽象。inference 直接创建的 `xbox_control_core` 不受 Xbox 独立测试目标影响，不应误报为第二条 inference 阻断原因。

**验收：** 上述两个干净目录配置成功，并实际构建目标库及示例程序。

### N2 / P1：迟到后执行多个追赶周期

**位置：** [MotorControllerBase::thread_func](../../src/motors/src/motor_base/motor_controller_base.cpp#L183)。

当前每轮执行完回调后，仅对 `next_period` 加固定周期再绝对睡眠：

```cpp
realtime_cycle_callback();
add_period_ns(next_period, rt_options_.rt_period_ns);
clock_nanosleep(kClockToUse, TIMER_ABSTIME, &next_period, nullptr);
```

没有在迟到后跳过已经过去的期限；睡眠返回码也未处理。只要一次调度延迟或回调超时使期限落在过去，后续几次睡眠便不能恢复均匀周期。

**实际复现：** 使用真实 `MotorControllerBase` 线程循环，派生回调在第 5 次执行时休眠 4 ms，其余回调为空操作。配置 1 ms 周期、普通调度，不需要权限或硬件。实际记录的相邻回调间隔（微秒）：

```text
4076 16 18 15 871 999
```

这证明当前循环会连续追赶。该数字是无硬件测试的一次观测，不是板端 FIFO80 的 WCET 或发生频率测量。

**影响：** EtherCAT 收发会短时间密集执行，按 `discrete_cmd_tick_` 计数的状态机检查/重试也会在更短的墙钟时间内推进。不能把这个问题描述成 `policy_cmd` 的踝力矩滤波器一定被重复推进：该滤波器在另一个线程，两条循环应分别分析。

**最小建议：** 对同一期限处理 `EINTR`；超过完整周期时把下一期限推进到未来，避免追赶已错过的周期。可记录 miss 次数和最大迟到量，便于实测；不凭本次注入测试决定一次 miss 就停机的阈值，也不改 FIFO 为 RR。

**验收：** 用相同的 4 ms 单次停顿验证恢复后不再出现连续追赶周期，再测正常负载下的实际周期分布。

### N3 / P2：启动期断线导致退出路径反复等待

**现实场景：** EtherCAT 已进入 RT 收发，第一次正式策略推理尚未开始，电机供电或总线连接持续中断；随后初始化失败或策略入口失败，程序进入 shutdown。不需要叠加另一种故障。

**调用链证据：**

1. [initialize](../../src/inference/src/robot/robot_interface.cpp#L55) 使用 `defer_communication_protection=true`。保护直到 [policy_step](../../src/inference/src/robot/robot_interface.cpp#L458) 即将执行正式推理才开启。
2. [通信看门狗](../../src/motors/src/drivers/myact/myact_motor_controller.cpp#L449)在保护关闭时不断清零坏周期计数。即使 `current_communication_fault` 已为 LinkDown/WkcIncomplete，`communication_fault_latched` 仍为 false。
3. [STOP 确认](../../src/motors/src/drivers/myact/myact_motor_controller.cpp#L908)要求完整 PDO、通信正常且驱动已失能；持续断线无法满足。
4. [shutdown](../../src/inference/src/robot/robot_interface.cpp#L103)只有“通信故障已锁存且当前仍故障”才允许走未确认释放路径。上述状态导致每次都返回 `RetryRequired`。
5. [析构](../../src/inference/src/robot/robot_interface.cpp#L35)会一直重试 shutdown；每轮还要经过默认 4 s 的 STOP 等待。首次推理已不可能继续，保护开关也不会自行打开。

**定向验证：** FakeAdapter 先产生完整反馈，然后保持 LinkDown/incomplete WKC，关闭启动通信保护，提交 STOP 并推进 100 个实际控制逻辑周期，得到：

```text
STOP_pending=1 comm_latched=0 current_fault=1
```

保持断线，开启保护后推进 10 个周期，通信故障能够锁存。动态测试覆盖了导致退出条件无法成立的底层状态；上层反复等待结论来自以上代码路径，没有通过启动真实机器人或让测试进程永久阻塞来验证。

**影响：** 进程可能持续占用主站及资源，错误后不能按现有 `ReleasedAfterCommLoss` 设计正常结束。本项没有证据说明电机会继续运动，不能从退出挂起推断实际机械状态。

**最小建议：** 在已经启动 RT 的退出路径恢复通信故障检测，使后续持续断线可以进入现有锁存和释放分支；保留初始化期间容忍瞬态的既定设计。另一个可选做法是在 shutdown 中按持续通信丢失时间判定，但无需同时引入两套机制。不能仅按任意一次 STOP 超时直接强制释放。

**验收：** 首次推理前持续断线后，STOP 不被伪报成功，退出能得到明确结果并释放资源；短暂恢复后的有效 PDO 仍可正常确认 STOP。

### N4 / P2：日志 `command_applied` 不代表 RT 已应用

**位置：** [policy_command_worker.cpp](../../src/inference/src/robot/policy_command_worker.cpp#L276)、[send_policy_setpoint](../../src/motors/src/motor_base/motor_controller_base.cpp#L351)、[日志字段](../../src/inference/include/recorder/inference_record.hpp#L58)。

`apply_impedance_setpoints_realtime()` 成功只表示 `send_policy_setpoint()` 已把数据发布到 SPSC latest 通道。worker 随后立即设置 `command_applied=true`。此时 RT 可能尚未消费；如果下个 worker 周期先发布新值，旧值可以正常地被覆盖，或者被随后到来的 STOP 阻止应用。

latest 覆盖机制符合当前设计，问题是日志名称和“电机接收的目标位置”注释容易被当作执行确认，导致分析动作延迟、丢命令或跟踪误差时得出错误结论。

**最小建议：** 将字段及 CSV 列统一改为 `command_published` 或 `command_enqueued`，目标值注释改为已提交值。只有确实需要测量 RT 应用时间时，再从 RT 回传消费序号/时间；本次不建议为改名扩展新通道。

**验收：** 发布后、RT 消费前的记录只表明发布成功，不宣称已发送或执行。本项为源码确定的语义问题，未新增端到端日志确认机制测试。

## 5. 验证结果与限制

本次临时构建和证据目录：`/tmp/rk3588-review-20261022.DKwpww`。临时目录可能被系统清理；源码位置和复现步骤是报告的长期依据。

| 验证 | 结果 | 说明 |
|---|---|---|
| inference 干净 CMake 配置 | **失败** | N1：motors 测试源已删除但目标仍在 |
| Xbox 独立工程干净 CMake 配置 | **失败** | N1：Xbox 测试源已删除但目标仍在 |
| base Debug 构建 | **通过** | 3 个测试可执行文件均生成 |
| base CTest | **2/3 通过** | SPSC、RingBuffer 通过；thread_runtime 的错误优先级断言失败，详见下文 |
| IMU Debug 构建 | **通过** | A100/Xsens 库及示例均编译链接成功，没有运行硬件示例 |
| Xsens parser 定向回归 | **通过** | 从基线 Git 内容取已删除测试到 `/tmp`，与当前 parser 编译；没有恢复工作区文件 |
| SocketCAN timestamp mock 回归 | **通过** | 同样从基线提取测试，与当前端口源码和 syscall wrappers 链接；不打开真实 CAN。错误打印是预期注入路径 |
| WKC last-valid 反馈 | **通过** | 调用当前 MyAct 控制代码，验证无效周期冻结数值/时间、恢复后刷新及 10 周期锁存 |
| STOP 退休旧目标及单轴 offline 整机锁存 | **通过** | 重用临时提取的 FakeAdapter 定向场景 |
| 4 ms 停顿后的周期追赶 | **缺陷复现** | 见 N2；普通调度下实际基类循环 |
| 启动期断线/STOP pending/不锁存 | **条件复现** | 见 N3；不是完整 RobotInterface 实机退出测试 |
| reset FK 快速失败回归 | **通过** | 当前源码重新编译；左右失败、正常实际姿态重建、失败后有效重试均通过 |
| `bash deploy/rk3588/test_robot_rt_setup.sh` | **通过** | fixture 使用假 proc/sys/etc 路径；不代表实际主机已通过预检 |

FK 测试还检查了非有限输入拒绝。本次修复的主要行为证据是有限输入 `upper=1.0 rad, lower=1.7 rad` 在指定初值下求解失败时返回 false，不再回退到目标；这是函数级失败分支验证，不主张该输入是正常工作域内常见姿态，也不新增关节工作域问题。

### base 测试失败如何解释

当前 `thread_runtime_test` 报错：

```text
stack prefault failure did not identify the cause
```

[测试](../../src/base/tests/thread_runtime_test.cpp#L73)同时传入大于 1 MiB 的栈预触碰量和 FIFO 优先级 -1，要求先报告栈错误。然而当前 [configure_current_thread](../../src/base/include/rt_thread_config/thread_runtime.hpp#L144)先校验/设置调度，再在后面预触碰栈，因此先返回优先级错误；头部关于执行顺序的注释也与实现不同。

这是已有测试、注释与实现顺序不一致的确凿验证结果。**按本次排除极端和不支持输入的要求，不把“双非法配置时哪条错误优先”列为运行缺陷，也不据此认定正常线程启动失败。** 后续确定预触碰与调度的预期顺序后，应让实现、注释和测试保持一致。

### 未验证内容

本次没有完整 inference 链接成功的证据，因为配置阶段即被 N1 阻断；也没有当前板端主机预检、真实模型推理延迟、FIFO 下周期分布、真实断线和电机运动结果。未执行的验证不记为通过，也不凭缺少实机数据额外制造缺陷。

## 6. 建议处理顺序

1. 先清理 N1 的两个残留测试目标，恢复可重复构建。
2. 修正 N2 的迟到周期推进，用同一个 4 ms 注入场景验证。
3. 补齐 N3 的退出期通信保护衔接，保留启动瞬态容忍及 STOP 真实确认语义。
4. 处理 N4 的日志命名，并统一线程配置顺序的实现/注释/测试。

P1-4、P1-5、P1-6 不出现在整改队列中；已修复的 WKC 时间、反馈年龄、ifname 和 reset FK 不重复要求重构。
