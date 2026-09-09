**RK3588 人形机器人嵌入式框架深度静态审阅报告**

审阅日期：2026-09-07。基线：`75306df887d05dd18405ef75c03dc669f5e9b26a`，按当前工作区行号引用；保留用户在 `inference_record.hpp` 中已有的空格修改。本次只新增报告；测试构建、探针和证据放在 `/tmp/robot_deploy_audit_20260907`。

审阅对象是主机部署框架：驱动器执行 FOC，主机 EtherCAT 周期为 1 ms，策略周期为 20 ms。当前没有 RKNN/ONNX 推理后端，因此 NPU 部分是迁移设计建议。没有执行真实电机程序、总线写操作、系统调参或带电故障注入。下文的“复现”均指无硬件模拟或离线输入验证，不代表已经测得整机实时性能。

**1. 总体健康度评分与阻塞性缺陷**

**总体评分：4.6 / 10。当前版本不满足无外部支撑行走的部署放行条件。**

评分针对可见主机代码的部署成熟度，五项等权。摔倒风险有明确代码路径；是否发生电机过热、驱动器烧毁取决于电流、温度、制动和功率级保护，不能由本仓库单独断言。

| 维度 | 分数 | 主要依据 |
|---|---:|---|
| 执行安全 | 3 | 指令没有有效期；STOP 可被普通队列丢弃；单轴故障不触发整机保护且可自动恢复输出 |
| 实时调度 | 4 | EtherCAT 使用绝对时间和 FIFO80，但 1 kHz 踝力矩工作线程未设置实时调度；无代码绑核、内存锁定、超期保护 |
| 数据同步 | 4 | 有主机单调时间、传感器年龄检查，但 CAN 排队时间不可见，分帧可跨采样拼接，WKC 异常反馈仍被刷新时间 |
| 策略边界 | 6 | 输入尺寸、动作有限性和限幅已有防护；训练语义缺少对照，步态参数存在隐式默认，推理完成后未校验动作时效 |
| 可验证性 | 6 | 12 项现有无硬件测试通过，但关键失效链路未覆盖，日志不能证明当前策略帧已写入 PDO |

值得保留的实现：三槽 SPSC 通道使用 acquire/release 所有权交换；策略与控制反馈分开发布；正常 EtherCAT 周期没有发现显式堆分配或阻塞日志；通信看门狗在连续 10 个坏周期后锁存；模式切换期间目标对齐反馈；重启会清除旧目标；策略输出有有限性校验；踝关节有行程检查、Jacobian 奇异性检查及力矩限幅。这些保护确实存在，以下问题是它们没有覆盖的失效路径。

**B1 — Blocker：策略／控制生产者失活，旧目标与旧力矩可无限期输出（已复现下游行为）**

证据：[策略工作线程](/home/cat/robot_deploy/src/inference/src/robot/robot_interface.cpp:398)、[策略目标发布](/home/cat/robot_deploy/src/inference/src/robot/robot_interface.cpp:456)、[控制线程消费目标](/home/cat/robot_deploy/src/motors/src/motor_base/motor_controller_base.cpp:565)、[PVT 输出](/home/cat/robot_deploy/src/motors/src/drivers/myact/myact_motor_controller.cpp:523)。

```cpp
if (policy_target_channel_.try_consume_latest(latest_target)) {
    current_target_q_model_rad = latest_target;
}
// 没有新目标时仍用 current_target_q_model_rad 计算并发布命令。
```

触发条件：策略 `forward()` 卡住、调用者停止调用 `policy_step()`、日志锁阻塞策略，或者踝力矩工作线程长时间未得到调度。目标通道只有角度数组，没有策略帧号、生成时间或截止时间；底层 `ControlCommand` 同样没有有效期。主机通信继续正常时，通信看门狗不会触发。

危险传播：策略卡住时，工作线程继续对旧姿态闭环；工作线程卡住时，EtherCAT 继续重复最后的 PVT。踝关节的 `kp=kd=0`，输出是主机计算的纯力矩，冻结后的力矩不会随新反馈更新。推理即使最终返回，`policy_step()` 也不重新检查其所依据观测的年龄。

无硬件探针仅提交一次 `111‰` 力矩，200 ms 后仍观察到 `target_torque=111, control_word=15`。这个结果证明底层缺少命令超时，不把 200 ms 当作生产环境时延测量值。

最小修复：策略目标携带 `policy_seq / observation_time / valid_until`；控制命令携带生产时间和源策略序号；由仍在运行的 EtherCAT 周期独立检查两层有效期，并在超期时进入锁存的整机保护状态。工作线程重复发布不能延长源策略的有效期；推理返回后的过期结果必须丢弃。保护动作应由驱动器能力与机器人受力状态确定。

验证：分别冻结策略生产者和命令生产者，保持模拟总线 WKC 正常；在明确配置的截止期后，全部轴进入保护输出，恢复生产者后不会自行重新使能。

**B2 — Blocker：STOP 走普通队列且未确认完成；停机还依赖日志／线程退出（已复现队列丢停）**

2026-09-09：复核曾在默认容量 16 下确认普通 STOP 丢失；随后已实施独立 STOP 状态、有效回读确认及先停机后清理的修复，motors 2/2、inference 10/10 测试通过。确认超时保留 RT，析构持续等待。以下为修复前历史记录；修复行为、测试及实机验收边界见 [B2 复核记录](b2_stop_recheck_2026-09-09.md)。

证据：[提交成功仅表示入队](/home/cat/robot_deploy/src/motors/src/motor_base/motor_controller_base.cpp:340)、[单轴队列满后标记失败](/home/cat/robot_deploy/src/motors/src/motor_base/motor_controller_base.cpp:608)、[仅服务队头](/home/cat/robot_deploy/src/motors/src/motor_base/motor_controller_base.cpp:645)、[stop 返回逻辑](/home/cat/robot_deploy/src/inference/src/robot/robot_motor_session.cpp:165)、[整机 shutdown 顺序](/home/cat/robot_deploy/src/inference/src/robot/robot_interface.cpp:108)。

```cpp
stop_policy_command_worker();
initialized_.store(false);
unload_policy();               // 包含 recorder.stop() / join()
imu_session_.deinitialize();   // 等待采集线程退出
motor_session_.deinitialize();// 到这里才提交 STOP
```

触发条件：存在尚未完成的 RESTART／SET_MODE，单轴命令队列满，或者停机时记录器写盘、flush、线程 join 阻塞。队头正常确认也需要两个间隔为 20 tick 的验证点；超时常量为 4000 tick。STOP 没有抢占路径。[deinitialize](/home/cat/robot_deploy/src/inference/src/robot/robot_motor_session.cpp:151) 忽略 `stop()` 结果，固定睡眠 50 ms 后停止主站循环；50 ms 不能证明驱动器已确认停止。[故障处理](/home/cat/robot_deploy/src/inference/src/robot/robot_interface.cpp:668) 还在请求停机前输出日志并 join 工作线程。

无硬件探针使用容量为 2 的单轴队列，加速构造满队列：STOP 返回 ACCEPTED，随后其完成状态变成 FAILED，两轴仍保持原力矩和 `0x000f`。生产默认容量为 16，改变的是达到满队列所需命令数量，没有消除同一缺陷。

最小修复：建立独立、不可被普通命令队列挤掉的停止／故障锁存，RT 周期在普通命令前检查；锁存后取消待执行的运动使能命令。先请求并观察驱动器保护状态，再卸载模型、flush 日志、join 非关键线程。严格区分“请求已接收”“主机已发送”“驱动器已确认”；超时进入已定义的硬件兜底流程，不能返回停机成功。

验证：队列满、模式切换未完成、存储写阻塞及通信断开四种情况下，停机请求均具有有界响应；日志不可用不得延迟保护输出。完整断线时主机无法把最后一帧送到驱动器，需要验证驱动器自身通信看门狗动作。

**B3 — Blocker：单轴故障只关闭本轴，其他轴保持旧输出；故障清除后自动恢复（已复现）**

证据：[单轴故障分支](/home/cat/robot_deploy/src/motors/src/drivers/myact/myact_motor_controller.cpp:443)、[全体目标被拒绝时直接 return](/home/cat/robot_deploy/src/motors/src/drivers/myact/myact_motor_controller.cpp:736)、[观测仅验证数值等条件](/home/cat/robot_deploy/src/inference/src/robot/observation_builder.cpp:175)、[通信看门狗条件](/home/cat/robot_deploy/src/motors/src/drivers/myact/myact_motor_controller.cpp:298)。

```cpp
if (motor.observed.sw_faulted || motor.rx.error != 0) {
    motor.step = MyactMotorStep::FAULT;
    motor.tx.control_word = CMD_SHUTDOWN;
    // 清零本轴输出；没有清除 desired.enabled 或锁存整机故障。
    return;
}
```

触发条件：一轴发生驱动故障、失去使能或模式不再确认，但 EtherCAT WKC 仍完整。上层虽然拿到了 `faulted / enabled / comm_ok`，却未用这些字段做全轴运行健康门控。连续目标接口返回 ACCEPTED 只证明发布到 SPSC；RT 消费时才发现一轴未就绪，整组新目标被丢弃，其他轴继续执行旧目标。

探针结果：给轴 0 注入错误后，轴 1 继续输出 `111‰`；提交 `222‰` 新目标返回 ACCEPTED，但轴 1 仍输出 `111‰`。清除轴 0 故障后，不发送 RESTART，它自行恢复 `111‰`。

最小修复：任一承重轴出现故障、非预期失能或模式失配，进入统一锁存保护；清除使能意图并作废旧目标。重新使能必须经过明确恢复操作和全部轴就绪检查；RT 拒绝整组目标需要向控制者反馈，而不只是后台事件打印。

验证：WKC 保持完整时注入单轴故障、失能、模式失配，验证全体轴的保护状态；故障清除不自动恢复力矩。此缺陷的整机后果取决于当时接触状态，但承重关节退出、其他关节继续旧运动是明确的摔倒路径。

**硬件放行仍需补齐的证据**

- PVT 增益单位、额定力矩对应电机侧还是输出侧、踝编码器每圈计数翻倍的依据。当前常量为普通轴 131072 count/rev、踝轴 262144 count/rev、踝额定力矩 10.5 N·m，软件力矩上限 2000‰，即按该换算为 21 N·m。不能仅凭“200%”认定会烧毁，也不能认定可持续输出。
- 驱动器 0x6072、实际电流限制、温度保护、持续／峰值时间、制动器和失去通信后的动作。本仓库没有完成这些参数的上电回读验收；ESI 中存在 Max Torque 对象不证明现场参数正确。
- `CMD_DISABLE_OPERATION=0x0007` 与 `CMD_QUICK_STOP=0x0002` 的实际受力行为。去使能不等于机器人能站稳，软件停机动作必须与制动、支撑及驱动器实现对应。
- 提供的电机与 IMU PDF 本次未完成全文提取，因此没有据其确认量纲或帧发送次序；已逐项解析仓库 ESI XML，并发现下述版本差异。

**2. 架构性建议与具体修改路径**

**当前链路与部署事实**

```text
Xsens CAN 接收线程 ─SPSC─→ 策略线程（默认 50 Hz，CPU TorchScript）
                              │ 观测／模型／动作缩放
                              ▼
                      策略目标 SPSC（目前无有效期）
                              ▼
                 踝力矩工作线程（目标 1 kHz，普通调度）
                              │ FK／Jacobian／滤波／力矩
                              ▼
                      电机 setpoint SPSC
                              ▼
                 EtherCAT 周期线程（FIFO80，1 kHz）
                              │ PDO／状态机／DC
                              ▼
                         驱动器 FOC
```

EtherCAT 反馈分别发布给策略和力矩工作线程；当前两个 1 kHz 循环各自用独立唤醒时间，没有共同周期编号或相位约定。

| 只读观测 | 实际结果 | 含义 |
|---|---|---|
| 内核 | `6.1.99-rt36-rk3588`，`CONFIG_PREEMPT_RT=y`，高精度定时器开启 | 已安装 RT 内核，不能再把“未打 RT 补丁”当作问题 |
| CPU 能力 | CPU0–3 capacity=414，CPU4–7=1024 | 绑定策略需要针对小核／大核，而非任意选 CPU 编号 |
| 隔离 | `/sys/devices/system/cpu/isolated` 为 `7` | CPU7 调度隔离有效，应用仍需显式绑定并检查 cpuset 限制 |
| 动态 tick | 命令行有 `nohz_full=7`；`CONFIG_NO_HZ_FULL` 未开启，sysfs 无 `nohz_full` | 命令行参数没有实现 full dynticks；并不意味着 1 kHz 定时器本身失效 |
| 调频 | 八个 CPU 的 governor 均为 `ondemand` | 频率变化是需测量的时延来源，不能假设固定大核算力 |
| IRQ | can0 IRQ77 有效亲和性为 CPU7；eth0 IRQ142/143 有效 CPU0 | CAN 在隔离控制核上产生干扰；eth0 IRQ 不足以代表原生 EtherCAT 驱动的收包路径 |
| EtherCAT | 本地头文件 API 1.6.0；加载模块含 `ec_master`、`ec_stmmac` | 应核对原生驱动轮询路径，不能直接套用普通网卡 IRQ 方案 |
| 实时预算 | `sched_rt_runtime_us=950000` | 保留 RT throttling；需要测量是否触发，不能盲目设为 -1 |

未在当前可见进程列表中取得正在运行的策略／电机线程，不能报告其实际 FIFO、TID 或绑核已验证。审阅进程允许 CPU0–6，这也不能代替生产服务的 cpuset 取证。[环境证据](/tmp/robot_deploy_audit_20260907/environment.json)。full dynticks 的前提见 [Linux NO_HZ 文档](https://www.kernel.org/doc/html/latest/timers/no_hz.html)。

**建议的首轮线程布局**

| 角色 | CPU | 调度起点 | 修改位置／要求 |
|---|---|---|---|
| EtherCAT 周期 | 7 | 保留 FIFO80 | `MotorControllerBase` 增加亲和性配置、线程名及启动屏障，在线程就绪后允许使能 |
| 踝力矩计算 | 6 | FIFO70，先验证 WCET | `start_policy_command_worker` 设置调度；绑定到电机反馈周期，必须检测重复、漏周期和状态年龄 |
| CAN 接收／解析 | 2 | FIFO60，按 IRQ 依赖验证 | 使用硬件实际采样率；将 can0 IRQ 从 CPU7 移至 CPU2，必要时令其 IRQ 线程高于接收线程 |
| CPU 推理／NPU 提交与结果处理 | 4–5 | SCHED_OTHER | 单独推理线程；CPU 后端先固定 intra-op=1、inter-op=1，测量后再调整 |
| 日志、手柄、诊断、一般系统任务 | 0–3 | SCHED_OTHER | 显式避免工作线程继承 CPU7 绑定；日志队列不得反向阻塞控制链 |

这是待测的部署起点，不是已经证明可调度的配置。若 EtherCAT 依赖 IRQ 线程或内核工作线程完成 I/O，需把相应依赖纳入 CPU 和优先级设计；不要让 FIFO80 线程等待一个被它饿死的低优先级依赖。PREEMPT_RT 的线程化中断与内核优先级继承机制见 [Linux 官方说明](https://www.kernel.org/doc/html/latest/core-api/real-time/theory.html)。用户态 `std::mutex` 不会因为安装了 RT 内核就自动变成优先级继承互斥锁。

具体实施顺序：

1. 先实现 B1–B3 的保护锁存及有效期；启动时建立传感器有效状态和全部轴反馈，再执行受控使能。当前 restart 在 IMU 启动前发生，并固定等待 500 ms 与 1 s，不能将这些 sleep 视为就绪证明。
2. 建立统一 `control_seq` 与单调时钟期限。EtherCAT 在周期 n 接收、检查反馈，发布状态；工作线程为明确的目标周期计算；EtherCAT 只接受满足序号与期限的结果。预算中显式计入当前消费顺序带来的反馈／命令流水线延迟。
3. 在启动非实时阶段完成模型加载、多次预热、固定缓冲区、线程栈与工作集预触碰；检查 `mlockall` 及资源限制的返回结果。禁止运行期任意扩容或再次加载模型。锁内存要覆盖实际模型与队列占用，不能只写一条调用而不验证效果。
4. 周期调度检查 `clock_nanosleep` 返回值；EINTR 重等同一截止时间；超期记录违约并跳过已错过的期限，禁止无界追赶历史周期。连续超期触发保护。滤波器不得把实际延迟数毫秒的更新继续假装成均匀 1 ms 数据。
5. 增加最小运行配置：控制／推理／IMU 的 CPU 和优先级、周期、数据年龄和命令有效期。现有 YAML 加载器只接受固定的训练配置顶层字段，不能假设在 YAML 中随意添加这些参数会生效。

**EtherCAT 与 IMU 同步**

- 保留已有 `ecrt_master_application_time`、每两周期参考时钟同步和每周期从时钟同步；新增 DC 偏差监测以及配置周期的一致性校验。`RealtimeOptions.rt_period_ns` 可变，而 DC 周期固定为宏 1000 Hz，当前默认一致，未来调参必须统一。
- 将应用时间采样放在一致的周期阶段，测量接收、计算、排队和发送耗时。当前使用 `CLOCK_MONOTONIC` 并不能单独证明 DC 同步错误；需要验证与从站的时间偏移、Sync0 相位及漂移，不建议不加分析地替换为可能跳变的墙钟。[IgH 官方接口说明](https://docs.etherlab.org/ethercat/1.6/doxygen/group__ApplicationInterface.html)。
- WKC 不完整时保留最后有效反馈的采样时间并显式标无效；通信看门狗可以有容忍窗口，控制计算不能把窗口内旧 PDO 当作新测量。使用 elapsed time 辅助故障期限，避免“10 次循环”在线程停顿时被误认为固定 10 ms。
- CAN 使用 `recvmsg` 获取内核接收时间和丢包统计；仅订阅所需 CAN ID，并监测 bus-off／错误帧。软件或硬件时间戳是否可用要看驱动；`SO_TIMESTAMPNS` 时间域不是自动的 `CLOCK_MONOTONIC`，必须转换并处理时钟调整。[Linux timestamping](https://www.kernel.org/doc/html/latest/networking/timestamping.html)、[SocketCAN](https://www.kernel.org/doc/html/latest/networking/can.html)。
- 四元数、角速度与 SampleTime 以采样周期组成一致快照，分别保留字段更新时间；丢一类报文时整帧不可无限跨周期配对。设备计数需要回绕展开与主机时钟映射；Xsens 的 uint32、10 kHz tick 约 119.3 小时回绕。硬件同步不可用时，报告可测的时间映射误差，不能称接收时间接近就是采样同步。
- 默认 CAN 为 250 kbit/s。若每个采样包含 4、8、6 字节三帧，按标准 CAN 数据帧并计 3 bit 间隔、不计填充，至少约 285 bit／样本，即 1.14 ms／样本。因而不能假设这三帧组合能在当前速率下持续提供 1 kHz 完整新样本；这是带上述帧组合假设的链路预算，实际采样配置尚未确认。

**NPU 迁移路径与接口建议**

离线检查模型归档并用本地 PyTorch 2.5.1 验证：`705 → 512 → 256 → 128 → 12`，隐藏层 ELU、末尾 Tanh，normalizer 为 Identity。705 维零输入得到 `[1,12] float32` 且全有限；675 维输入被矩阵尺寸检查拒绝。代码默认编译选项 ON 与该模型一致。

1. 固定批量 1 和 705 维输入，从训练导出端生成 ONNX，再用匹配目标 BSP 的 RKNN-Toolkit2 转为 RKNN。先采用非 INT8 的基准配置，再决定量化；显式核对 ELU、Tanh、数据类型、布局与算子支持。转换、Runtime 和内核 RKNPU 驱动版本一起记录。
2. 保存训练端黄金样本：按字段排序的 15 帧历史、关节方向、站立偏置、重力坐标系、步态相位、原始上一动作。依次比较 TorchScript、ONNX、RKNN 的动作和最终力矩；尺寸相同不代表训练语义相同。
3. 第一版使用独立推理线程和单个 RKNN context，同步完成该线程内部的输入／run／取回输出；控制线程继续独立周期运行。使用预分配输入输出与有界 latest 通道，不积压每一帧。目标是受控的端到端动作年龄，不只看 NPU 每秒推理数。
4. 推荐新增固定大小的 `PolicyInputFrame{seq, sensor_time_ns, deadline_ns, observation}` 和 `PolicyResult{seq, sensor_time_ns, infer_start_ns, infer_end_ns, raw_action, status}`。缓冲区所有权到推理完成才归还；跨线程有 acquire/release，NPU 内存按 SDK 要求处理 cache sync／stride。普通原子内存屏障不能替代 DMA 缓存一致性 API。
5. 如后续启用 `RKNN_FLAG_ASYNC_MASK`，必须验证输入与输出帧号匹配：官方 API 明确该模式可能取得前一帧结果；`rknn_run_extend`／输出扩展的 frame ID 和 timeout 要按实际 SDK 使用。不能给旧输出贴上当前观测时间。多线程解耦本身不要求打开该标志。[RKNN C API](https://raw.githubusercontent.com/airockchip/rknn-toolkit2/master/rknpu2/runtime/Linux/librknn_api/include/rknn_api.h)。
6. NPU 超时／返回错误时停止发布动作，并由 B1 的独立期限检查保护；恢复 Runtime 不自动恢复电机使能。对这个约 52.6 万 MAC 的小型 MLP，先比较 CPU 与 NPU 总时延，包括提交、复制和排队；不能仅由芯片 TOPS 推断延迟收益。

**3. 关键代码片段问题：行级分析**

以下 P1 表示需要在部署前修复或取得明确证据，P2 表示影响确定性、兼容性或验证可信度。B1–B3 已在第一部分给出完整触发路径。

| 编号／等级 | 文件、行与关键代码 | 触发与影响 | 最小修改及验证 |
|---|---|---|---|
| R1 / P1 | [robot_interface.cpp:347](/home/cat/robot_deploy/src/inference/src/robot/robot_interface.cpp:347) 仅创建 `std::thread`；[374](/home/cat/robot_deploy/src/inference/src/robot/robot_interface.cpp:374) 以 `filter_dt_s` 为周期；[442](/home/cat/robot_deploy/src/inference/src/robot/robot_interface.cpp:442) 独立 `sleep_until` | 真正计算踝力矩的 1 kHz 线程没有像 EtherCAT 一样设 FIFO，且与反馈周期独立。负载下延迟、复用旧反馈、滤波采样时间失真 | 配置并验证工作线程调度／亲和性，使用周期序号与反馈有效期；用调度跟踪测等待时间和错过周期 |
| R2 / P1 | [motor_controller_base.cpp:298](/home/cat/robot_deploy/src/motors/src/motor_base/motor_controller_base.cpp:298) `add_period_ns` 后直接 `clock_nanosleep(...)` | 忽略返回值和截止期违约；线程暂停若干毫秒后可能连续执行多个过期周期。高优先级追赶会放大 IRQ／其他线程饥饿 | 处理 EINTR、记录超时、跳过历史周期并有界保护；模拟暂停后检查周期间隔，不能只统计平均 Hz |
| R3 / P1 | [socket_can_port.cpp:140](/home/cat/robot_deploy/src/imu/src/drivers/socket_can_port.cpp:140) `read` 后在 [156](/home/cat/robot_deploy/src/imu/src/drivers/socket_can_port.cpp:156) 取 `monotonic_now_ns()` | 读取线程延迟时，积压的旧报文得到新时间；50 ms 年龄守卫可被绕过。没有内核过滤和接收溢出统计会进一步隐藏排队问题 | recvmsg 接收时间／丢包统计，映射设备采样时间；注入接收队列滞留，验证样本年龄仍反映原到达时间 |
| R4 / P1 | [can_parser.cpp:121](/home/cat/robot_deploy/src/imu/src/protocol/xsens_mti/can_parser.cpp:121)、[136](/home/cat/robot_deploy/src/imu/src/protocol/xsens_mti/can_parser.cpp:136) 任一字段刷新统一时间；[170](/home/cat/robot_deploy/src/imu/src/protocol/xsens_mti/can_parser.cpp:170) 仅检查两个 fresh 标志 | 100 ms 前的四元数可与当前角速度配成一帧，统一时间显示“新鲜”；SampleTime 未参与配对且可以为零 | 分字段时戳／采样代号，限制配对跨度；探针已复现，增加丢帧、乱序、回绕、设备重启测试 |
| R5 / P1 | [can_parser.cpp:140](/home/cat/robot_deploy/src/imu/src/protocol/xsens_mti/can_parser.cpp:140) 不检查四元数范数；[164](/home/cat/robot_deploy/src/imu/src/protocol/xsens_mti/can_parser.cpp:164) 仅检查重力分量有限性 | 全零四元数得到 `(0,0,-1)` 且 valid=true，设备初始化／异常输出可伪装成正常直立姿态 | 边界校验范数，量化小误差允许归一化，零值和严重偏差拒绝；探针已复现全零情形 |
| R6 / P1 | [myact_motor_controller.cpp:299](/home/cat/robot_deploy/src/motors/src/drivers/myact/myact_motor_controller.cpp:299) 计算 WKC 健康；[305](/home/cat/robot_deploy/src/motors/src/drivers/myact/myact_motor_controller.cpp:305) 却用从站 online/operational 决定接收；[930](/home/cat/robot_deploy/src/motors/src/drivers/myact/myact_motor_controller.cpp:930) 每周期刷新反馈时间 | 从站 OP 不等于本周期 PDO 有效；前 9 个 WKC 异常周期可带新时戳发布旧／部分有效数据。9坏1好的模式还能持续重置故障计数 | WKC 与反馈有效性合并，保留最后有效时间；用交替坏周期和单域部分更新验证，另统计滑动窗口错误率 |
| R7 / P1 | [robot_interface.cpp:108](/home/cat/robot_deploy/src/inference/src/robot/robot_interface.cpp:108)、[inference_recorder.cpp:283](/home/cat/robot_deploy/src/inference/src/recorder/inference_recorder.cpp:283) 先等待记录线程，再停电机 | 正常退出时工作线程已停，电机仍保持最后输出；flush／磁盘阻塞使这段等待无上界 | 与 B2 一并调整生命周期：保护优先，资源释放随后；用阻塞日志 sink 验证 |
| R8 / P2 | [ankle_motor_fk.hpp:82](/home/cat/robot_deploy/src/inference/include/kinematics/ankle_motor_fk.hpp:82)、[122](/home/cat/robot_deploy/src/inference/include/kinematics/ankle_motor_fk.hpp:122)、[142](/home/cat/robot_deploy/src/inference/include/kinematics/ankle_motor_fk.hpp:142) 最多 50 次迭代 × 5 个阻尼值 × 12 次线搜索 | 循环有界但耗时依输入变化；两踝在每个 1 ms 工作周期调用，近奇异位形可能显著增加执行时间。未实测，不能宣称已超 1 ms | 扫描实际工作空间并测 release 构建尾部耗时；限制求解预算和有效工作域，必要时优化导数。Jacobian 转置的力矩映射本身未发现转置错误 |
| R9 / P2 | [inference_recorder.cpp:248](/home/cat/robot_deploy/src/inference/src/recorder/inference_recorder.cpp:248) `lock_guard`；[258](/home/cat/robot_deploy/src/inference/src/recorder/inference_recorder.cpp:258) `deque.push_back`；[policy_runtime.cpp:155](/home/cat/robot_deploy/src/inference/src/policy/policy_runtime.cpp:155) Tensor 包装与 forward | 位于 50 Hz 策略路径，不是 EtherCAT RT 周期；仍可能因互斥、分配及运行库内部调度推迟策略更新 | 固定记录环形缓冲、非阻塞发布、预热和固定 Torch 线程数；测分配、缺页和端到端动作年龄。from_blob 包装不等于必然复制数据，contiguous 也不必然每次分配 |
| R10 / P2 | [robot_interface.cpp:640](/home/cat/robot_deploy/src/inference/src/robot/robot_interface.cpp:640) 发布新目标后立刻读旧工作线程日志；[652](/home/cat/robot_deploy/src/inference/src/robot/robot_interface.cpp:652) 用 worker healthy 代替 `command_applied` | 同一 CSV 行可能混有当前策略动作和上一个实际命令；“已应用”为真也不能证明 RT 消费或 PDO 发送成功 | 全链路使用策略序号／控制序号，RT 输出提交回执；拆分 enqueued、accepted_by_rt、sent、drive_confirmed；记录时间单调关系测试 |
| R11 / P2 | [deploy_config.cpp:649](/home/cat/robot_deploy/src/inference/src/config/deploy_config.cpp:649) 白名单不允许 gait_phase；[deploy.yaml:54](/home/cat/robot_deploy/src/inference/config/deploy.yaml:54) 注释；[observation_builder.cpp:272](/home/cat/robot_deploy/src/inference/src/robot/observation_builder.cpp:272) 705 分支仍插入步态项 | 当前模型尺寸正确，但 gait 参数只能隐式取默认；用户解除注释会加载失败，无法通过这份配置表达训练语义 | 根据编译分支显式加载／拒绝步态项；固定黄金输入历史。探针确认原配置接受、解除注释后报 unknown key |
| R12 / P2，模型替换时升级 | [policy_runtime.cpp:23](/home/cat/robot_deploy/src/inference/src/policy/policy_runtime.cpp:23) 仅启动 dry-run 验证 numel；[160](/home/cat/robot_deploy/src/inference/src/policy/policy_runtime.cpp:160) 每帧直接取 Tensor 并复制 12 项 | 当前静态 MLP 输出固定，未复现越界；若替换为数据相关输出形状的模型，启动通过后运行时输出缩短可越界读取 | 每次外部模型返回后验证类型／形状／numel，再取指针；构造“零输入输出12、其他输入输出6”的测试模型验证拒绝 |
| R13 / P1，版本待核实 | [ethercat_adapter_igh.cpp:68](/home/cat/robot_deploy/src/motors/src/protocol/ethercat/ethercat_adapter_igh.cpp:68)、[76](/home/cat/robot_deploy/src/motors/src/protocol/ethercat/ethercat_adapter_igh.cpp:76) 填充对象为 0x2ffd/0x2ffe；[ESI:1867](</home/cat/robot_deploy/src/motors/datasheet/myact/EtherCAT ESI/MT-Device 250702.xml:1867>)、[1918](</home/cat/robot_deploy/src/motors/datasheet/myact/EtherCAT ESI/MT-Device 250702.xml:1918>) 为 0x5ff1/0x5ff2 | 仓库两份 PDO 定义不一致；主数据字段和位宽基本对应，不能直接推断已错位烧毁。按固件能力，可能表现为配置失败或版本不匹配 | 上电前回读实际固件、PDO assignment／mapping，与选定 ESI 固定为同一版本；不能只靠 sizeof 断言 |
| R14 / P2 | [ethercat_adapter_igh.cpp:118](/home/cat/robot_deploy/src/motors/src/protocol/ethercat/ethercat_adapter_igh.cpp:118) `(void)ifname; ecrt_request_master(0)` | 上层打印的 enp8s0 不选择实际网卡；生产排错容易核对错接口，或错误认为更改名字已切换总线 | 明确使用 master index 并核查主站配置的物理设备／从站身份；显示实际配置，保留驱动原生收发路径 |
| R15 / P2 | [action_processor.cpp:522](/home/cat/robot_deploy/src/inference/src/robot/action_processor.cpp:522) reset 时 FK 不可达则把起点设为目标默认姿态 | 启动插值的起点不再代表当前真实脚位，可能丢失预期的渐变效果；不在正常 1 kHz 路径 | FK 不可达时拒绝自动复位，报告原始电机角及位形；在启动阶段注入不可解但编码器有限的反馈验证 |

**关于 ARM 并发与误报边界**

- [spsc_latest_channel.hpp:60](/home/cat/robot_deploy/src/base/include/spsc_latest_channel/spsc_latest_channel.hpp:60) 和 [72](/home/cat/robot_deploy/src/base/include/spsc_latest_channel/spsc_latest_channel.hpp:72) 的交换使用 `memory_order_acq_rel`，在单生产者、单消费者且 reset 不并发的契约下，没有发现需要额外手写 ARM `dmb` 的问题。正常启动／停止流程先结束线程再 reset；公开 API 的并发调用约束仍应写明。
- [107](/home/cat/robot_deploy/src/base/include/spsc_latest_channel/spsc_latest_channel.hpp:107) 起的槽和索引未做缓存行隔离，存在伪共享优化空间；这不是已确认的数据竞态。用目标 ARM64 的 lock-free 查询和反汇编确认原子实现，再依据测量决定 padding，避免空耗内存。
- 普通状态缓存与回调互斥位于后台发布线程。MyAct 覆盖了基类的 queue-full／failed 回调，实际通过事件队列发送；不能把基类 `printf` 默认实现直接算作当前 MyAct 周期内打印。
- 启动时的 500 ms、1 s、20 ms 插值 sleep 与运行时 1 kHz 抖动是不同问题；前者影响启动安全顺序，后者需要分析工作线程周期。
- 当前没有进行 TSAN 或证明所有公开 API 可任意并发。现有 FakeAdapter 测试中部分 rx setter 与 receive 使用不同步存储，未来做 TSAN 时应先修复测试夹具，不能把其报告直接归因于生产驱动。

**训练语义尚不能从本仓库证明的部分**

当前观测是按字段分组的 15 帧历史，每个字段内部从旧到新；首次用当前值填满历史；上一动作是 raw action。A100 对安装角做了固定旋转补偿，Xsens 直接使用四元数；传感器安装矩阵和训练坐标系需要比对。模型归档能确认层结构，不能确认所有训练观测定义。需提供训练导出配置或黄金数据，逐项核对历史顺序、步态门控、缩放、关节顺序／方向、踝并联机构模型以及控制延迟。不能把目前“尚未核实”写成“已经确认顺序错误”。

**4. 验证方案、已执行结果与验收标准**

**已执行的无硬件检查**

在独立 Debug 构建中运行，保留断言；现有 Release 构建会定义 NDEBUG，[ring_buffer_test.cpp:14](/home/cat/robot_deploy/src/base/tests/ring_buffer_test.cpp:14) 把 push 操作写在 assert 内，因此 Release 下打印 passed 不能代表该测试有效。

| 检查 | 结果／证据 |
|---|---|
| CTest 注册的 7 项：CSV、动作 IK、踝 Jacobian、IMU 配置、Xsens 解析、CAN 配置、A100 解析 | 7/7 通过；[test_0.log](/tmp/robot_deploy_audit_20260907/test_0.log) |
| 手工运行 RingBuffer、SPSC、motor_realtime_channel_test | 3/3 通过；[test_1.log](/tmp/robot_deploy_audit_20260907/test_1.log)、[test_2.log](/tmp/robot_deploy_audit_20260907/test_2.log)、[test_3.log](/tmp/robot_deploy_audit_20260907/test_3.log) |
| Xbox 纯映射测试、Python 电机跟踪绘图测试 | 2/2 通过；[Xbox 日志](/tmp/robot_deploy_audit_20260907/xbox_controller_mapping_test.log)、[绘图日志](/tmp/robot_deploy_audit_20260907/test_4.log) |
| 无硬件故障探针 | 复现 B1 下游旧输出、B2 丢停、B3 单轴故障／自动恢复、R4 跨时刻配帧、R5 零四元数；[源码](/tmp/robot_deploy_audit_20260907/fault_probe.cpp)、[原始日志](/tmp/robot_deploy_audit_20260907/fault_probe.log) |
| 配置探针 | 原 YAML 接受且 observation_size=705；解除 gait 注释后拒绝；[日志](/tmp/robot_deploy_audit_20260907/config_probe.log) |
| 实际 policy.pt 离线 CPU 输入／输出检查 | PyTorch 2.5.1：705 输入通过，675 输入拒绝；未测 NPU 或整机闭环 |

独立构建使用 `cmake -S src/inference -B /tmp/robot_deploy_audit_20260907/build -DCMAKE_BUILD_TYPE=Debug -DROBOT_POLICY_ENABLE_GAIT_PHASE_OBS=ON`，只构建上述测试目标。配置有 `kineto_LIBRARY-NOTFOUND` 警告，但这些目标构建与运行成功；不能将其误报为构建失败。

`ankle_inverse_kinematics_test` 位于 tests 目录但会初始化真实电机，已识别并跳过；也没有运行 policy_test、motors_test、id_test 或硬件延迟示例。当前 CTest 没有注册上述手工运行的底层测试。建议在 CI 中注册无硬件测试，明确区分硬件示例，增加保护链路回归；本次没有修改这些源码。

**故障注入矩阵**

| 场景 | 注入方式 | 应观察的验收行为 |
|---|---|---|
| 策略不再发布／推理延迟超过有效期 | 模拟策略暂停，EtherCAT 与工作线程保持运行 | 源策略期限到达后保护；重复发布旧目标不能续期；迟到推理结果丢弃 |
| 工作线程失活 | 仅暂停命令生产者 | EtherCAT 独立检测命令年龄，不能无限重复纯力矩 |
| 队列满／切模式期间 STOP | 内存 FakeAdapter 暂停周期后批量入队 | STOP 独立于普通队列；不会被丢弃，后续 RESTART 不能越过故障锁存 |
| 单轴故障、失能、模式错 | 保持 WKC 完整，只改一轴反馈 | 整机进入保护，明确报告原因；故障消失不自行恢复 |
| IMU 一类报文丢失／跨周期重排 | 直接向解析器 feed，不向真实 can0 发帧 | 过期组合不发布；零／异常范数四元数拒绝；重复 SampleTime 不刷新采样有效期 |
| 时间戳零值、倒退、回绕、设备重启 | 离线数据和虚拟时钟 | 明确失效或重建映射；不能把缺失时间当年龄为零直接放行 |
| WKC 9坏1好、10坏、单轴 offline | FakeAdapter 脚本 | 无效 PDO 不获新采样时间；连续坏周期保护保持；错误率监控可识别反复掉帧 |
| 存储阻塞／日志满 | 测试程序使用受控的阻塞输出 sink | 日志不阻塞保护请求，记录丢失有计数；shutdown 不先等待日志 |
| 周期唤醒迟到 | FakeAdapter 或离线循环注入延迟 | 记录 deadline miss 并跳过旧期限；无持续突发追赶 |
| 模型／训练语义迁移 | 同一黄金观测输入各后端 | 逐层动作及最终力矩误差受控，所有输出序号与源观测一致 |

**板端工具与命令模板（以下尚未执行）**

当前环境有 `trace-cmd`；没有发现 `perf`、`cyclictest`、`rtla` 可执行文件。应安装与 BSP 配套的工具；部分 RTLA 功能依赖内核配置。以下采集和压力负载在专用台架上运行，先使用输出隔离的控制回放程序；不能把冻结整个生产进程当作本报告已执行的验证。

先识别实际进程与线程，填写测试变量；下面的 PID/TID 示例值需要替换：

```bash
ps -eLo pid,tid,psr,cls,rtprio,comm
robot_pid=1234
ecat_tid=1235
command_tid=1236
policy_tid=1237
taskset -pc "$ecat_tid"
taskset -pc "$command_tid"
chrt -p "$ecat_tid"
chrt -p "$command_tid"
cat /proc/irq/77/effective_affinity_list
zcat /proc/config.gz | grep -E 'CONFIG_(PREEMPT_RT|NO_HZ_FULL|HIGH_RES_TIMERS|RCU_NOCB_CPU)'
```

调度分析使用 `perf sched record` 与 `timehist`；不要把 `perf sched trace` 当作当前通用子命令。[perf 官方手册镜像](https://man7.org/linux/man-pages/man1/perf-sched.1.html)。

```bash
sudo perf sched record -a -o /tmp/robot-sched.data -- sleep 30
sudo perf sched timehist -i /tmp/robot-sched.data -t "$ecat_tid,$command_tid,$policy_tid" -w -M
sudo perf stat -t "$ecat_tid,$command_tid,$policy_tid" \
  -e task-clock,context-switches,cpu-migrations,page-faults,minor-faults,major-faults -- sleep 30
```

关注 R1/R2/R8/R9：线程已被唤醒到实际运行之间的等待、工作线程迁核、周期内长执行、缺页。报告唤醒等待和执行时间两项，不能把它们混成一个平均“推理时间”。

```bash
sudo cyclictest -a 7 -t 1 -p 80 -m -i 1000 -D 30m -h 2000 -q
sudo trace-cmd record -o /tmp/robot-rt.dat \
  -e sched:sched_switch -e sched:sched_wakeup -e sched:sched_migrate_task \
  -e irq:irq_handler_entry -e irq:irq_handler_exit \
  -e irq:softirq_entry -e irq:softirq_exit \
  -e power:cpu_frequency -- sleep 30
sudo trace-cmd report -i /tmp/robot-rt.dat
```

cyclictest 单独测内核唤醒基线；不要在同一核同时用高优先级 cyclictest 与生产控制线程比较后宣称是无干扰测量。先用 `cyclictest --help` 确认板端版本参数；直方图还要查看超范围样本。trace-cmd 事件应以本机 `trace-cmd list -e` 为准，某事件不存在时记录缺项，不能悄悄当作已测；用调频事件关联 ondemand 抖动。[trace-cmd 官方手册](https://www.trace-cmd.org/Documentation/trace-cmd/trace-cmd-record.1.html)。

执行顺序为无负载、CPU 推理、日志写入、内存／I/O 负载、NPU 推理组合。压力只加在台架的指定 CPU，例如 `taskset -c 0-5 stress-ng --cpu 6 --timeout 30m --metrics-brief`；内存与 I/O 负载另行限制内存量和写入目录，并记录温度、频率、负载参数。相同负载下比较修改前后最大值、P99/P99.9、违约次数；不把 30 分钟通过当作硬实时上界证明。

EtherCAT／CAN 的只读取证模板：

```bash
ethercat version
sudo ethercat master
sudo ethercat slaves -v
sudo ethercat domains -v
ip -details -statistics link show can0
timeout 30s candump -t a -e -x 'can0,005:7FF,021:7FF,032:7FF,#FFFFFFFF'
```

`candump` 用于观察报文间隔与错误；应核对本机 can-utils 的过滤参数。驱动器 PDO、Sync0、看门狗和限流回读按确认的对象字典完成，避免对未知对象直接写入。DC 偏差需在控制应用中使用匹配版本 IgH 的监测 API 或专用台架观测 Sync0；一次 `ethercat slaves` 快照不能证明微秒级同步。

建议追加 RT 固定大小统计：计划唤醒、实际唤醒、receive 完成、计算完成、send 返回、有效 WKC、策略序号、命令序号、最后有效反馈时间、故障状态。由后台导出，避免每周期打印或每周期写 trace_marker。

NPU 验证在接入 RKNN 后执行：用 Toolkit2 的 `eval_perf` 和 SDK 示例检查逐层耗时与总耗时；C API 开启 `RKNN_FLAG_COLLECT_PERF_MASK` 后通过 `RKNN_QUERY_PERF_DETAIL / RKNN_QUERY_PERF_RUN` 取性能数据，同时查询 SDK 版本。性能采集会增加开销，应另做关闭 profiler 的端到端测量；不虚构一个通用的 `rknn_profiler` 命令。比较单 NPU 核与多核配置，验证返回帧号、超时行为、输入输出 buffer 复用和长期温升。[RKNN 官方 API](https://raw.githubusercontent.com/airockchip/rknn-toolkit2/master/rknpu2/runtime/Linux/librknn_api/include/rknn_api.h)。

**验收预算与放行门槛**

下面数值是 1 kHz 台架回归的初始工程预算，不是电机或机器人已认证的安全阈值；需要根据驱动器 Sync0／看门狗、IMU 采样率和闭环稳定性结果收紧。

| 项目 | 初始验收目标 | 判定限制 |
|---|---|---|
| EtherCAT 1 ms 周期 | 唤醒延迟 ≤50 μs、主机 receive 到 send 返回 ≤200 μs，定义的交付截止期违约为 0 | send 返回不等于从站已作用；还需验证链路传输和 Sync0 建立时间 |
| 踝工作线程 | 计算 ≤300 μs，数据移交／相位预算 ≤100 μs，剩余约 350 μs 裕量 | 这些是统一流水线内的预算分配，不能分别测平均值后相加冒充最坏值 |
| 1 kHz 命令／反馈年龄 | 模拟测试先以 3 ms 为超时门限 | 真实阈值由闭环稳定性及驱动器允许保持时间决定；必须测实际有效反馈时间 |
| 50 Hz 策略结果 | 20 ms 周期内完成；源观测到动作有效期初始取 60 ms | 到期进入保护，旧目标重发不续期；60 ms 只是测试起点 |
| IMU 成帧一致性 | 四元数、角速度属于同一采样周期；缺帧不能跨无限周期拼接 | 当前 50 ms 年龄、30 ms skew 仅是宽松守卫，不能证明 1 ms 同步；最终误差阈值需实测 |
| STOP／单轴故障／过期命令 | 保护请求到 RT 保护输出 ≤2 个控制周期 | 驱动器实际停机时间另测；断线兜底由驱动器看门狗证明 |
| 记录与内存 | 稳态关键控制线程零 major/minor fault，无阻塞文件 I/O；应用于测量的动作必须能追到策略序号 | 对 profiler 额外开销单独记录；日志 dropped 不得被解释成控制正常 |
| Torch → ONNX → RKNN | 使用黄金样本逐项比较 raw action、目标角和最终力矩，并完成仿真／台架回放 | 误差限值按闭环稳定性确定；INT8 不沿用 FP32 容差，不仅比较均方误差 |

放行顺序：先消除 B1–B3 并加入回归；修复 R3–R7 的状态有效性和停机顺序；核对 ESI／驱动器参数与训练语义；随后验证调度预算和 NPU 迁移。静态审查与上述无硬件测试已经完成，真实板端负载时延、DC 偏差、驱动器制动／热保护和真实闭环稳定性尚未验证。
