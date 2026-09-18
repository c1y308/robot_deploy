# 当前人形机器人部署框架精简审查（2026-09-18）

审查基线：`38e7675febce08e12f68358c7d327bfefd9e48b7`。审查开始时工作区干净。
本次交付为审查文档和历史报告归档，没有修改控制源码、部署参数或系统设置。

## 审查结论与适用边界

**当前框架已有足够明确的部署主链，最值得精简的是外围通用性、日志对控制数据的侵入，以及重复维护的状态。不能把名称相近的 timeout、ready 或 fault 全部视为重复。**

本次依据仓库中的实际入口、配置和消费者，审查固定 RK3588、MyAct/IgH master 0、12 电机、Xsens CAN、当前 TorchScript 模型及既有线程模型。策略名义周期 20 ms，IMU 按用户说明以 400 Hz 反馈，踝控制和 EtherCAT 为 1 kHz。不要求跨线程同周期、不要求 IMU 字段同一采样帧、不设计动态硬件选择或运行中恢复。

用户明确接受的三项行为，均不列为缺陷、不设整改或放行门槛：

- **Xsens 分帧可以跨样本组合。** 保证各字段 latest-value 语义和现有数据年龄即可；不增加共同样本编号、配帧窗口、重排或等待机制。
- **仓库 ESI 与代码 PDO padding 对象版本的差异。** 当前使用正常，本次不调整映射、不要求额外版本取证。
- **reset 起始 FK 失败后采用默认目标角度。** 按用户确认，这是可接受的安全复位行为，保留；不改成拒绝复位。

这也修正了前两份报告对上述行为过严的评价。历史报告中的评分、风险等级及实机放行判断不能直接套用于当前版本。

证据分为两类：源码可直接证明的死字段、重复状态和调用关系；需要同步缩窄工具或兼容能力才能删除的设计。后者明确列出受影响消费者，不能冒称“全仓库无人使用”。本次没有执行真实 CAN/EtherCAT 收发或电机动作。

## 实际运行链路

```text
policy_test 主线程：YAML → 固定 RK3588 profile → 主机预检 → 配置 policy_main
  → 打开 Xbox → 创建 RobotInterface
  → mapping/action/observation 初始化 → CPU 模型加载、dry-run、Torch worker 验证
  → MCL_ONFAULT 锁页、栈预触碰 → 启动 recorder
  → EtherCAT connect → 主线程收发并等待 OP/WKC → 创建电机后台线程和 ecat_rt
  → STOP 确认 → 设置模式 → RESTART 确认 → IMU 线程 → 平滑 reset
  → policy_cmd 接管复位保持
  → Xbox 后台线程 → 主线程按 20 ms 调用 policy_step

policy_step：电机/IMU latest 快照 → 年龄检查 → 15 帧观测 → inference
  → raw action clip/scale/offset clip → 40 ms admission → 发布 target 或 drop

policy_cmd（CPU6/FIFO70）：target latest + 电机反馈 latest
  → 踝 FK/Jacobian/滤波/力矩 + 其它轴阻抗 → 下发带最终截止期的命令

ecat_rt（CPU7/FIFO80）：STOP → 普通离散命令 → 连续命令消费 → 再检查 STOP
  → EtherCAT receive → 驱动状态/故障/watchdog → send → 反馈 fan-out

日志：policy 组装记录 → worker 补首条命令 → policy 收回记录 → recorder 后台 CSV
退出：先请求 STOP、停止 worker 刷新 → 确认 STOP → join → 卸载/关闭/释放
```

这里有两个用途不同的 1 kHz 线程。`policy_cmd` 做数值计算，`ecat_rt` 做周期通信和最后命令期限检查。没有计算耗时及相位的实测依据时，把它们合并不是本轮最小修改。

### 时效和状态机制的语义核对

| 机制 | 当前语义及消费者 | 判断 |
| --- | --- | --- |
| IMU 年龄 50 ms | policy 入口判断传感器是否停止更新 | 保留 |
| 电机反馈年龄 20 ms | policy 和 worker 检查最后有效 PDO；参与最终命令期限 | 保留，同一上限应用于不同消费者 |
| IMU/电机时间差 30 ms | policy 入口另加相对时间差限制 | 可简化，见 15；不意味着要求同帧 |
| 结果准入 40 ms | inference 后判断候选结果是否可发布；超龄 drop | 保留 |
| 目标保持 60 ms | 从实际发布起算；worker 重复生成命令不能续期 | 保留 |
| 首帧 60 ms | 从首次正式 inference 起算，约束尚未取得 target 的复位保持 | 保留首次推理起点，启动段可减分支，见 19 |
| 连续命令 10 ms | worker 不再生成命令时，RT 端独立停机 | 保留 |
| 连续坏 WKC 10 周期 | EtherCAT 链路持续不完整，RT 锁存通信故障 | 保留，不替代反馈年龄 |
| 从站过程数据 watchdog 约 100 ms | 主机不再发 PDO 时由从站处理 | 保留，软件线程不能替代 |
| 离散命令重试/4 s 超时/上层4 s等待 | 驱动状态确认、再次应用及调用方结束等待 | 可局部减状态，见 23；不删除闭环确认 |

控制命令最终期限已取以下最小值：

```text
min(target 发布时间 + 60 ms,
    command 产生时间 + 10 ms,
    最老有效电机反馈时间 + 20 ms)
```

它将不同来源的约束汇总到一个 RT 消费期限，已经是合理的简化，不应再添加独立的“总链路 deadline”或额外序号门禁。

## 逐项精简建议

### 01｜已经关闭的主循环计时统计仍持续计算

- **位置 / 模块：** [policy_test.cpp](../../src/inference/examples/policy_test.cpp#L160)，主循环及 `kPrintPolicyTiming`。
- **当前设计：** 每轮读取起止时间、累计平均/最大耗时，并判断一秒报告窗口；报告代码已全部注释，`last_report` 也不再更新。
- **问题类型：** 冗余、极低价值复杂度。
- **是否建议修改：删除。**
- **理由：** 当前没有统计输出消费者，recorder 已记录 inference 时间；空报告分支不能帮助实机排障。
- **最小修改方案：** 删除该开关、`last_report`、`total_step_ms`、`max_step_ms`、`step_start/end/ms` 和空报告分支；保留 `steps`，因为失败文本和退出总步数仍使用它。删除旁边已经弃用的速度切换注释。

### 02｜模型可读性在 Torch 加载前重复检查

- **位置 / 模块：** [policy_test.cpp](../../src/inference/examples/policy_test.cpp#L36)，`file_readable()` 及调用处。
- **当前设计：** 先用 `ifstream` 打开模型检查可读，再由 `PolicyRuntime::load()` 真正加载并报告错误。
- **问题类型：** 重复保护。
- **是否建议修改：删除。**
- **理由：** 模型加载本来就在硬件初始化前，文件不存在/权限不够/归档损坏均由真实加载边界处理；提前打开一次没有额外保护效果。
- **最小修改方案：** 删除辅助函数和对应分支，沿用 `load_policy()` 的失败和错误文本。

### 03｜训练导出元数据被要求存在，却从不进入部署逻辑

- **位置 / 模块：** [deploy_config.cpp](../../src/inference/src/config/deploy_config.cpp#L490)，`load_commands()`、`load_action()`；[deploy.yaml](../../src/inference/config/deploy.yaml#L7)。
- **当前设计：** 解析速度 ranges 到 `ignored_range`；验证 `joint_names`、`joint_ids: null` 和两组 delay 范围后丢弃。控制不使用这些值，也不模拟 delay。
- **问题类型：** 冗余、极低价值复杂度。
- **是否建议修改：简化。**
- **理由：** 当前真正的速度映射在 Xbox，delay 等只是训练配置残留。这里应该减少无效 schema，而不是补上部署延迟模拟或新的速度限制功能。
- **最小修改方案：** 配套删除当前 YAML 中这些无人消费的字段以及对应解析、必填检查、helper 和允许键。保留实际使用的 action clip/scale/offset 和 observation 参数。若外部流程必须直接提供原始训练导出 YAML，则先仅取消这些元数据的必填及值校验；仓库未包含该外部流程，不声称可以无条件改变其输入格式。

### 04｜配置加载器为了验证创建 mapping，启动时又创建一次

- **位置 / 模块：** [deploy_config.cpp](../../src/inference/src/config/deploy_config.cpp#L355)，`load_joint_topology()` 和成员 `mapping_`；[robot_interface.cpp](../../src/inference/src/robot/robot_interface.cpp#L138)。
- **当前设计：** YAML 先验证 12 项 permutation 和固定踝电机集合，再创建完整 `JointMapping`；加载器的 mapping 不被其它函数使用。RobotInterface 启动时重新创建并验证相同 mapping。
- **问题类型：** 冗余、重复保护。
- **是否建议修改：删除。**
- **理由：** 原始 YAML 的 permutation 检查仍有价值；加载器构造出的对象只做一次重复验证，没有运行期消费者。
- **最小修改方案：** 删除加载器中的 `mapping_` 成员及 `JointMapping::create()` 调用。保留 YAML 原始映射检查，并保留启动时真实 mapping 的一次完整构造验证，仍在接触硬件前失败；不新增“已验证配置”包装类或跨模块缓存。

### 05｜JointMapping 重复保存原配置与可直接从配置读取的镜像

- **位置 / 模块：** [joint_mapping.hpp](../../src/inference/include/robot/joint_mapping.hpp#L31)，`config_`、方向镜像、左右踝镜像；[joint_mapping.cpp](../../src/inference/src/robot/joint_mapping.cpp#L160)。
- **当前设计：** 完整保留配置，又复制方向和两个踝映射；私有 `configure()` 只在工厂构造时调用，却先清空状态、构造 next 容器，最后批量提交。
- **问题类型：** 冗余、过度设计。
- **是否建议修改：简化。**
- **理由：** mapping 是启动后不可变的固定表，不存在运行中重新配置或失败回滚的需求。
- **最小修改方案：** getter 直接读 `config_` 的方向及踝映射，删除三个镜像成员；保留实际需要推导的直驱索引表/踝标记。删除仅为重复 configure 服务的清空和事务式 next 临时变量。共享只读 mapping 和现有类边界可以继续保留。

### 06｜运行配置总开关制造多组默认分支，并决定日志失败策略

- **位置 / 模块：** [robot_config.hpp](../../src/inference/include/robot/robot_config.hpp#L129)，`RuntimeThreadingConfig::enabled`；[robot_interface.cpp](../../src/inference/src/robot/robot_interface.cpp#L179)、各 session/worker。
- **当前设计：** 总开关决定线程 options、三个库线程数、是否验证 worker CPU；recorder 启动失败又按该开关选择退出或继续。
- **问题类型：** 过度设计、极低价值复杂度。
- **是否建议修改：简化。**
- **理由：** 部署入口固定启用 profile；工具和测试的默认 options/线程数已经能表达未配置行为。线程设置与日志失败是否容忍没有相同语义。
- **最小修改方案：** 直接传递现有 options、线程数及 CPU 列表，删除覆盖为默认值的三元分支和总开关；保留空 CPU 集合/零线程数供现有独立工具使用。日志显式开启而启动失败时统一返回初始化失败。保留 memory-lock、host-preflight 的明确用途，不添加另一组总开关。

### 07｜线程设置结果中有无人使用的错误码，线程名读回也没有比较

- **位置 / 模块：** [thread_runtime.hpp](../../src/base/include/tool/thread_runtime.hpp#L37)，`ThreadSetupResult::error_code` 和 `configure_current_thread()` 内的 `actual_name`。
- **当前设计：** 错误文本已含错误码解释，却又保存一个生产调用方从不读取的数值字段；`pthread_getname_np()` 填入 `actual_name` 后不检查内容。
- **问题类型：** 冗余。
- **是否建议修改：删除。**
- **理由：** 名称来自仓库内部固定字符串，读取一个不消费的名字不验证实时调度。成功布尔值和错误文本足够完成启动握手。
- **最小修改方案：** 删除结果中的 `error_code` 字段及赋值；删除线程名读回和临时缓冲。保留名称设置、affinity/调度设置失败检查及必要读回；保留 promise/future 启动握手，不改为新的启动状态机。

### 08｜PolicyRuntime 的 loaded 标志重复表达模型对象存在

- **位置 / 模块：** [policy_runtime.cpp](../../src/inference/src/policy/policy_runtime.cpp#L244)，`loaded_`、`is_loaded()`、`unload()`；[policy_runtime.hpp](../../src/inference/include/policy/policy_runtime.hpp#L56)。
- **当前设计：** 模型指针赋值时 `loaded_=true`，失败/卸载时两者都清掉；`is_loaded()` 同时检查标志、永不主动清空的 `impl_` 和 module。
- **问题类型：** 冗余。
- **是否建议修改：删除。**
- **理由：** 构造成功后 Impl 固定存活，加载状态可直接由 module 指针表示；不存在“module 存在但 loaded=false”的有效运行态。
- **最小修改方案：** 删除 `loaded_` 和重复的 Impl 空值分支，`is_loaded()` 返回 module 是否存在。保留 PImpl，避免 Torch 头文件扩散到非 Torch 模块；不扩大到 PImpl 重构。

### 09｜一次启动链仍携带旧 session 清理和重新加载准备

- **位置 / 模块：** [robot_interface.cpp](../../src/inference/src/robot/robot_interface.cpp#L43)，`initialize()`、`load_policy()`、`initialize_policy_runtime_state()`。
- **当前设计：** initialize 先 shutdown；load_policy 又 shutdown 模型、stop recorder、重置处理器。处理器刚构造时也已 reset；`PolicyRuntime::load()` 自己还执行 unload。
- **问题类型：** 冗余、过度设计。
- **是否建议修改：简化。**
- **理由：** 当前 main 只初始化一次，失败就退出，不热加载模型，也不在同一对象上重复启动。
- **最小修改方案：** 按当前入口写明一次 initialize 的调用约束，删掉 initialize 前的旧 session 清理、load_policy 的重复 stop/reset。保留初始化失败后的 shutdown，以及可重复调用的 shutdown/析构清理，因为现有失败出口和析构确实会重复清理；不能将所有生命周期容错一起删除。

### 10｜策略所有者线程内部使用了不必要的 mutex 和 atomic

- **位置 / 模块：** [robot_interface.hpp](../../src/inference/include/robot/robot_interface.hpp#L89)，`target_velocity_mutex_`、`initialized_`；[robot_imu_session.hpp](../../src/inference/include/robot/robot_imu_session.hpp#L65)。
- **当前设计：** 当前 main 在同一线程中调用 `set_target_velocity()`、`policy_step()` 和启停；worker 不读取 `target_velocity_` 或 RobotInterface 的 initialized。IMU session 的 initialized 也只由所有者生命周期函数读写。
- **问题类型：** 不可能场景防御、冗余。
- **是否建议修改：简化。**
- **理由：** Xbox 已通过自己的锁安全交付 command，这并不使 RobotInterface 的速度缓存成为跨线程变量。
- **最小修改方案：** 删除速度 mutex，两个所有者 initialized 改普通 bool，写清现有串行调用约束。**不要连带修改** MotorSession 的 initialized/motion_enabled、worker running/failed、共享首推理时间和故障 latch；这些确有跨线程访问。

### 11｜私有启动函数重复防御不可能的未初始化调用顺序

- **位置 / 模块：** [robot_interface.cpp](../../src/inference/src/robot/robot_interface.cpp#L274)，`reset_joints()`；[action_processor.cpp](../../src/inference/src/robot/action_processor.cpp#L99)，reset 内部尺寸检查。
- **当前设计：** reset 是私有函数，仅在成功创建处理器、成功初始化/RESTART 后调用，仍检查电机初始化、处理器空指针，并提示“先调用 initialize”。内部 reset 数组尺寸也来自同一个已确定的 12 DOF。
- **问题类型：** 不可能场景防御、重复保护。
- **是否建议修改：删除。**
- **理由：** 私有函数不能由用户在 initialize 前直接调用；上游失败已立即退出。
- **最小修改方案：** 删除这些纯调用顺序/固定尺寸防御，结合第 14 项使用固定数组。保留跨线程 STOP 可能改变的运动许可、反馈有效性、实际 IK/FK 成败及物理限位；这些不由调用顺序保证。

### 12｜固定 EtherCAT 拓扑仍保留未被构建入口选择的单电机版本

- **位置 / 模块：** [ethercat_adapter_igh.hpp](../../src/motors/include/protocol/ethercat/ethercat_adapter_igh.hpp#L13)，`MYACTUA_ECAT_NO_FORWARDERS`；`SlaveOffsets` 的两个 reserved offset。
- **当前设计：** 默认实际 12 轴拓扑外，还编译支持 `{0}` 单电机拓扑；仓库 CMake/入口没有选择宏。两个 padding offset 没有注册、写入或读取。
- **问题类型：** 过度设计、冗余。
- **是否建议修改：删除。**
- **理由：** 当前部署物理拓扑确定，unused offset 也不参与当前 PDO 数据布局。
- **最小修改方案：** 留下正在使用的 `kSlavePositions`，删除另一分支及宏；删除两个 unused offset 成员。**不改 PDO entries 的 padding 对象/位宽/映射。** FakeAdapter 的少轴测试与真实拓扑宏无关，不需要删。

### 13｜adapter 中存在同线程状态的原子镜像和内部越界防御

- **位置 / 模块：** [ethercat_adapter_igh.cpp](../../src/motors/src/protocol/ethercat/ethercat_adapter_igh.cpp#L277)，physical 收发、`get_bus_health()`、`send/receive()`；[ethercat_adapter_igh.hpp](../../src/motors/include/protocol/ethercat/ethercat_adapter_igh.hpp#L78)。
- **当前设计：** 每周期把 master/domain 状态拷到三个 atomic，getter 再读回来；从站 configured 也存 atomic。初始化成功之后的内部循环仍检查指针组合和固定索引。
- **问题类型：** 冗余、不可能场景防御。
- **是否建议修改：简化。**
- **理由：** 实际调用者只有启动等待和 MyAct RT update；前者结束后 RT 才接管，两者不并发。公共状态线程读取的是发布后的快照，不读取 adapter。
- **最小修改方案：** `get_bus_health()` 直接返回 master/domain 状态，configured 从已有 sc_state 推导；删除 atomic 镜像。将成功 init 后才调用收发、内部索引有效作为契约，删内部重复检查。保留 init 中所有真实 IgH 资源/API 失败检查，以及外部提交入口的索引验证。保留 `tx_shadow`，它承担统一落 PDO 的实际作用。

### 14｜固定 12 轴快照及首帧保持仍走动态 vector 路径

- **位置 / 模块：** [robot_motor_session.hpp](../../src/inference/include/robot/robot_motor_session.hpp#L33)，`MotorStateSnapshot`；[robot_motor_session.cpp](../../src/inference/src/robot/robot_motor_session.cpp#L320)，`apply_targets_rad()`；[observation_builder.cpp](../../src/inference/src/robot/observation_builder.cpp#L183)。
- **当前设计：** 底层已有固定反馈数组，策略层转成五个 vector，再拷贝位置/速度到固定数组。首帧前 worker 每 1 ms 构造 vector 阻抗目标；reset 也反复创建固定长度 vector。
- **问题类型：** 冗余、极低价值复杂度。
- **是否建议修改：简化。**
- **理由：** 当前没有可变电机数量，动态长度既引入分配，又引入额外尺寸检查和转换接口。
- **最小修改方案：** 部署层快照/复位/保持目标统一为 `std::array<...,12>`，直接调用已存在的 fixed 阻抗构造入口；ObservationBuilder 直接引用固定位置/速度数组。删除 reserve/push_back、range template 和 vector→array 中转。底层 FakeAdapter 少轴测试仍可保留自己的 count，不必同时改整个电机库。

### 15｜单独的 sensor skew 门限对 latest-value 部署增加第三种时序判据

- **位置 / 模块：** [robot_interface.cpp](../../src/inference/src/robot/robot_interface.cpp#L369)，`validate_policy_sensor_timing()`；[robot_config.hpp](../../src/inference/include/robot/robot_config.hpp#L99)。
- **当前设计：** IMU 和电机各自检查年龄，再独立要求两者时间差不超过 30 ms。
- **问题类型：** 极低价值复杂度。
- **是否建议修改：简化。**
- **理由：** **这不是数学上的重复检查**：现有 50/20 ms 年龄约束不能推出 30 ms skew。删除会放宽一部分现有准入范围。但当前要求是独立 latest 值，不要求同帧；没有证据说明 30 ms skew 是当前模型必须满足的额外同步条件。
- **最小修改方案：** 保留两种源数据年龄和最老 observation 时间；去掉 skew 阈值配置及失败判据。时间差如仍用于排障可临时计算，不需要保留第三个停机条件。若该数值来自已确认的训练/闭环要求，则保留原实现；不能仅凭“实时系统必须同步”继续扩展配帧机制。

### 16｜策略轮次由两个计数器维护

- **位置 / 模块：** [robot_interface.hpp](../../src/inference/include/robot/robot_interface.hpp#L100)，`next_policy_seq_`；[observation_builder.cpp](../../src/inference/src/robot/observation_builder.cpp#L126)，`frame_index_`。
- **当前设计：** 每次正式推理分配 policy_seq；成功返回的一步，包括 drop，都会推进 frame_index。前者从 1 开始，后者从 0 开始；frame_index 还驱动 gait phase。
- **问题类型：** 冗余。
- **是否建议修改：合并。**
- **理由：** 当前没有并发多次 inference、推理失败后继续运行或其它帧推进入口，正常轮次和 drop 轮次中始终满足 `policy_seq = frame_index + 1`。
- **最小修改方案：** 保留 ObservationBuilder 的计数器用于 gait 和 CSV，推理序号由它加一生成；删除独立 next_policy_seq/reset。保留日志中的两列以免影响现有绘图，列值可派生；失败推理立刻停机，不需要为失败后的“下一轮”维护计数器。

### 17｜逐阶段诊断维护了一套非控制状态

- **位置 / 模块：** [robot_interface.cpp](../../src/inference/src/robot/robot_interface.cpp#L451)，`set_policy_step_phase()` 和失败输出；[robot_interface.hpp](../../src/inference/include/robot/robot_interface.hpp#L57)。
- **当前设计：** 每次 policy_step 进入七个阶段时保存阶段和单调时间，仅在本线程处理失败时输出阶段名称及耗时。
- **问题类型：** 极低价值复杂度。
- **是否建议修改：删除。**
- **理由：** 这套状态曾服务于排查，确有诊断价值，但不参与保护；多数失败调用点已有明确原因，inference 耗时也已记录。正常路径持续维护它的价值有限。
- **最小修改方案：** 删除 phase enum、两个状态、setter 及各处调用；保留具体失败消息、inference 起止时间和 worker 超时文本。若正在复现尚未定位的阻塞，可暂留至排查结束；不把它升级为跨线程阶段监视或新 watchdog。

### 18｜控制目标承载完整日志，且元数据重复

- **位置 / 模块：** [policy_command_worker.hpp](../../src/inference/include/robot/policy_command_worker.hpp#L26)，`PolicyTargetFrame`；[robot_interface.cpp](../../src/inference/src/robot/robot_interface.cpp#L554)；[policy_command_worker.cpp](../../src/inference/src/robot/policy_command_worker.cpp#L109)。
- **当前设计：** target 内嵌完整 InferenceRecord；policy_seq 在外层和记录各存一次，valid_until 也有两份。policy 局部 record 再复制给局部 target，随后复制到写槽；worker 回传完整记录。
- **问题类型：** 冗余、过度设计。
- **是否建议修改：简化。**
- **理由：** 头文件离线探针实测 `InferenceRecord=3592 B`、`PolicyTargetFrame=3616 B`。worker 控制只需要 12 个模型目标角和期限；完整记录扩大了两个线程的耦合。**这不是已证实的性能故障**：target 仅在有新帧时复制，正常约 50 Hz；首条命令回传也每个策略序号一次。
- **最小修改方案：** 先消除重复 policy_seq/valid_until 和局部整帧中转，使用现有写槽完成发布；recorder 关闭时不填充、不回传非控制日志字段。首条命令对齐和电机 target 列有真实绘图/排障消费者，可以暂保留现有往返通道。若当前部署不再需要这些列，直接删除 worker 补日志及完成记录通道，policy 自己记录输入/输出即可；不要为保持旧 CSV 再增加序号 join 表、回执队列或匹配状态机。

### 19｜首帧超时错误构造了虚假的完整 target

- **位置 / 模块：** [policy_command_worker.cpp](../../src/inference/src/robot/policy_command_worker.cpp#L200)，首帧前保持分支、`startup_policy_target_expired()`。
- **当前设计：** 同一保持轮次多次读取首次推理时间、计算 deadline；失败时为了复用日志格式，创建一个默认 PolicyTargetFrame，倒推出 published_at，再传入普通目标错误函数。
- **问题类型：** 极低价值复杂度、过度设计。
- **是否建议修改：简化。**
- **理由：** 首帧保持确实需要独立期限，但不存在已发布 target；为一条错误消息伪造 3.6 KB target 没有控制用途。
- **最小修改方案：** 保留共享首次 inference 时间，当前轮次直接计算首帧 deadline 并判断；失败时直接报告首帧超时及已有 deadline，不构造 target。deadline 对复位命令的封顶继续保留，避免主线程阻塞时无限保持；不增加 startup-ready 标志。

### 20｜RT 缓存完整 active command，但后续只读取 timing

- **位置 / 模块：** [motor_controller_base.cpp](../../src/motors/src/motor_base/motor_controller_base.cpp#L550)，`process_latest_setpoint_commands()`；[motor_controller_base.hpp](../../src/motors/include/motor_base/motor_controller_base.hpp#L287)。
- **当前设计：** 每次接收命令复制完整 `active_setpoint_`，后续只读取 valid_until 和 source_policy_seq；实际目标已经写入驱动 desired state，并不靠 active payload 重放。
- **问题类型：** 冗余。
- **是否建议修改：简化。**
- **理由：** 实测 ControlCommand 为 632 B，CommandTiming 仅 24 B。这是正常 1 kHz 命令路径中可以明确减少的存储/复制。
- **最小修改方案：** active 缓存改为已有 `CommandTiming`；没有 active 时用零 deadline 表达，删除 `has_active_setpoint_`。保持当前“先检查旧命令过期、再消费新命令”的规则，主动全身 STOP 时清空缓存；不顺手改变故障锁存顺序。

### 21｜离散队列条目携带连续控制数组

- **位置 / 模块：** [discrete_command_channel.hpp](../../src/motors/include/motor_base/discrete_command_channel.hpp#L27)，SubmissionQueue::Entry；[command_types.hpp](../../src/motors/include/motor_base/command_types.hpp#L103)。
- **当前设计：** 普通离散命令只需要类型、mode、目标索引和 ID，却把含两组连续目标数组、timing、payload 标志的完整 ControlCommand 放进队列。
- **问题类型：** 过度设计、冗余。
- **是否建议修改：简化。**
- **理由：** 离散和连续路径已经使用不同队列，统一大对象并没有统一实际处理，反而制造无效字段组合。
- **最小修改方案：** 现有 Entry 只保存上述离散字段，提交时提取，消费时直接用于入轴队列；保留 API 的必要目标验证和结果追踪。无需增加 interface/factory/variant。632 B 连续对象的其它模式兼容可以另行处理，不必本轮一起重写全部命令 API。

### 22｜固定全身启动仍拆成十二个 SET_MODE 命令

- **位置 / 模块：** [robot_motor_session.cpp](../../src/inference/src/robot/robot_motor_session.cpp#L133)，初始化设置模式；`restart()` 与 `wait_for_stop()`。
- **当前设计：** 全部轴使用同一模式，却分配 12 个 command_id、提交 12 个命令；STOP 和 RESTART 又各自实现结果轮询/截止期/sleep。
- **问题类型：** 冗余、极低价值复杂度。
- **是否建议修改：合并。**
- **理由：** 现有命令已经支持 all-motors，结果 tracker 也支持整机目标掩码；不需要为固定部署拆开操作。
- **最小修改方案：** 用一条现有 `set_mode(mode)` 覆盖全部轴；保留每轴物理状态推进。将现有 `wait_for_stop()` 改成 STOP/RESTART 共用的完成等待，保留调用处的运动许可更新及具体错误信息；不增加命令执行器类。

### 23｜离散状态机有仅存在一个分支的过渡阶段和每命令恒定参数

- **位置 / 模块：** [command_types.hpp](../../src/motors/include/motor_base/command_types.hpp#L252)，QUEUED、max_retries；[motor_controller_base.cpp](../../src/motors/src/motor_base/motor_controller_base.cpp#L665)、`service_discrete_commands()`。
- **当前设计：** 入队先设 QUEUED，取出时无条件改 APPLY_PENDING；max_retries 每次都赋同一常量。cur_retry、deadline_tick、verify/retry tick 共同管理确认。
- **问题类型：** 冗余、过度设计。
- **是否建议修改：简化。**
- **理由：** QUEUED 不等待任何额外事件，max_retries 也没有按命令选择的消费者。
- **最小修改方案：** 初始直接 APPLY_PENDING，删除 QUEUED 和重复初始化；删除 max_retries 成员，直接使用已有常量。保留重试、验证和有界等待。最大重试约束与 4 s 超时都能结束失败流程，但触发时间不同；本轮不为了合并而改变已有启动等待预算。上层墙钟等待也不等同 RT tick 超时，不能一并删除。

### 24｜诊断快照与终端打印之间多了一条 1 ms 发布线程

- **位置 / 模块：** [myact_motor_controller.cpp](../../src/motors/src/drivers/myact/myact_motor_controller.cpp#L189)、`realtime_start_callback()`；[status_channel.hpp](../../src/motors/include/motor_base/status_channel.hpp#L285)；[motor_status_monitor.hpp](../../src/motors/include/motor_base/motor_status_monitor.hpp#L131)。
- **当前设计：** RT 发布 MotorState → motor_diag 每 1 ms 消费、更新有锁 vector 缓存 → motor_mon 每 20 ms 读取并打印。
- **问题类型：** 不必要抽象、极低价值复杂度。
- **是否建议修改：合并。**
- **理由：** diagnostics 只有 monitor 一个消费者，两级发布缓存并没有服务第二个实际读者。部署入口关闭电机打印时这两条线程已不启动，不能算成当前一直消耗 CPU 的线程。
- **最小修改方案：** 现有 monitor 线程直接消费诊断 latest 快照并打印，删除 motor_diag 发布线程、有锁缓存及其启停分支。公共 motor_status 另有 reset/状态回调消费者，先保留；不要将终端打印移进 RT，也不要新增统一 telemetry 子系统。

### 25｜Shell 和 C++ 各维护一套固定主机预检规则

- **位置 / 模块：** [rk3588_host_preflight.cpp](../../src/inference/src/robot/rk3588_host_preflight.cpp#L363)；[robot-rt-setup.sh](../../deploy/rk3588/robot-rt-setup.sh#L761)。
- **当前设计：** 各自解析 CPU 集合、cmdline、governor、IRQ、驱动、主站 MAC、NetworkManager 配置和 ready 文件。C++ 约 499 行，脚本已经提供部署安装后的 `check` 命令；两套检查也不完全相同。
- **问题类型：** 冗余、过度设计。
- **是否建议修改：合并。**
- **理由：** 重复的是规则实现，不是“预检没有用”。当前板卡固定，维护两套 parser 和 profile 常量会造成漂移。
- **最小修改方案：** 保留部署脚本的 check 作为规则来源，现有 C++ 入口启动时调用固定的 `/usr/local/sbin/robot-rt-setup check` 并使用退出状态，删除重复 parser；不创建通用命令执行框架。现有部署服务本来就安装该脚本；同时将测试集中到现有 fixture，保留应用对失败状态的测试。不要只读一个 ready 文件就省掉当前 live 状态检查，也不要删掉真实绑核/调度失败处理。

### 26｜SocketCAN 时钟映射的整数极限检查超出当前时间域

- **位置 / 模块：** [socket_can_timestamp.hpp](../../src/imu/src/drivers/socket_can_timestamp.hpp#L22)，`subtract_ns()` 和 timestamp_to_ns 的 int64 极限分支。
- **当前设计：** 每个时钟转换/差值都通过通用有符号溢出 helper，检查数百年量级的边界；同时验证 recvmsg 附属数据及 realtime/monotonic 映射。
- **问题类型：** 不可能场景防御。
- **是否建议修改：简化。**
- **理由：** 当前系统年份、正常运行时长及内核时钟来源远离 int64 纳秒极限，不需要将任意 int64 测试输入都纳入部署合同。
- **最小修改方案：** 正常时间域内直接做纳秒差值和转换，删除 subtract_ns 及极限分支/对应极端测试。保留真实 kernel RX timestamp、CLOCK_MONOTONIC 映射、recvmsg 长度/附属信息检查和错误返回。NTP/系统校时不由仓库保证绝不发生，本次**不直接删除**映射变化检查，也不新增恢复状态机；400 Hz 分帧 latest-value 语义与内核排队时间是真实且不同的概念。

### 27｜Xsens 的原始 IMU callback 是没有数据出口的空接口

- **位置 / 模块：** [can_parser.hpp](../../src/imu/include/protocol/xsens_mti/can_parser.hpp#L16)，IMUCallback_t、imu_callback_；[xsens_reader.cpp](../../src/imu/src/drivers/xsens_mti/xsens_reader.cpp#L123)；[socket_can_port.cpp](../../src/imu/src/drivers/socket_can_port.cpp#L208)。
- **当前设计：** Xsens 保存并转发 IMU callback，却从不生成 IMUData 或调用它；另有无人调用的 SocketCanPort::read() 包装和可选空 timestamp 指针路径。
- **问题类型：** 不必要抽象、冗余。
- **是否建议修改：删除。**
- **理由：** 当前 Xsens 只提供部署需要的 AHRS 输出，不应为了与 A100 形式一致宣称支持一个空数据接口。
- **最小修改方案：** 删除 Xsens parser 的 callback 类型/成员/setter；若暂保留通用 ReaderBase，其该项 override 只能作为明确的空兼容入口，不再保存 callback。结合 29 删除部署层基类后可彻底删掉。删除无人消费的 SocketCanPort::read()；read_nonblocking 时间输出改必传引用，配套更新实际 reader/时间戳测试，删 nullable 分支。不删除真正使用的 AHRS callback 和打印/get_ahrs_data 接口。

### 28｜当前单一 705 维模型仍携带 675 维历史版本

- **位置 / 模块：** [policy_observation_config.hpp](../../src/inference/include/policy/policy_observation_config.hpp#L21)，两种长度；[observation_builder.cpp](../../src/inference/src/robot/observation_builder.cpp#L215)，offset/历史分支；[deploy_config.cpp](../../src/inference/src/config/deploy_config.cpp#L684)。
- **当前设计：** YAML 是否出现 gait_phase 决定 675/705，影响 history offsets、Torch 输入长度及 recorder 列数。当前 YAML 包含 gait_phase，仓库只有一份部署 policy.pt。
- **问题类型：** 过度设计、极低价值复杂度。
- **是否建议修改：删除。**
- **理由：** 当前模型的 705→12 已离线执行验证，675 分支没有当前部署模型消费者；它是历史兼容，不是需要保留的 gait 功能。
- **最小修改方案：** 固定 705 和相应 offsets，删 gait.enabled 及无 gait 的长度选择、CSV 长度组合和兼容测试。保留 gait 的 period/门控和 15 帧训练布局，并仍在启动 dry-run 验证模型输出。此项需同步改配置/模型输入/日志头，收益主要是维护，不应排在小范围删除之前。

### 29｜部署 session 为已确定的 Xsens 选择第二种 reader

- **位置 / 模块：** [robot_imu_session.cpp](../../src/inference/src/robot/robot_imu_session.cpp#L30)，reader switch；[robot_config.hpp](../../src/inference/include/robot/robot_config.hpp#L39)；[inference/CMakeLists.txt](../../src/inference/CMakeLists.txt#L108)。
- **当前设计：** 部署对象支持 A100 串口与 Xsens CAN，两套参数放在同一配置。imu_only_test 确实能选择 A100，且独立 A100 解析器/示例仍存在。
- **问题类型：** 不必要抽象。
- **是否建议修改：简化。**
- **理由：** 不能说 ReaderBase 全仓库只有一个实现；但当前机器人只接 Xsens，部署路径不需要 type/switch/baudrate 组合。
- **最小修改方案：** 部署 session 固定 Xsens，删其 A100 include、选择分支、串口字段转发和 A100 链接。同步缩窄 imu_only_test，A100 调试由已存在的 `src/imu/examples/a100_test.cpp` 承担；暂保留独立 A100 模块及测试，不把“部署无需链接”扩大为“源文件都没人用”。不建立新的 reader factory。

### 30｜已经弃用的 NetworkManager guard 模板仍在仓库

- **位置 / 模块：** [networkmanager-ethercat-guard.conf](../../deploy/rk3588/networkmanager-ethercat-guard.conf#L1)；[test_robot_rt_setup.sh](../../deploy/rk3588/test_robot_rt_setup.sh#L55) 的旧 fixture guard、`__check-nm-guard` 分派。
- **当前设计：** 模板的 ExecStartPre 调用 `__check-nm-guard`，正式脚本已没有该命令，install 也不再复制模板。fixture 自行模拟旧内部命令并创建旧 guard 文件。
- **问题类型：** 冗余、极低价值复杂度。
- **是否建议修改：删除。**
- **理由：** 当前部署已经刻意让 NetworkManager 独立于机器人预检；旧模板既无安装消费者，照抄安装又会执行一个不存在的命令。
- **最小修改方案：** 删除模板和 fixture 中的伪旧命令/无用 guard 文件，测试需要时直接调用现有 check_networkmanager_config；合并内容相同的 run_setup/run_internal_setup wrapper。保留 unmanaged-by-MAC 配置及检查，不扩展为系统网络恢复机制。本次仅审查，不更改已安装的系统配置。

### 31｜硬件示例没有遵循现有命令有效期，是实际接口迁移遗漏

- **位置 / 模块：** [motor_response_latency_test.cpp](../../src/motors/examples/motor_response_latency_test.cpp#L40)，20 ms 周期及 [10 ms deadline](../../src/motors/examples/motor_response_latency_test.cpp#L366)；[ankle_inverse_kinematics_test.cpp](../../src/inference/tests/ankle_inverse_kinematics_test.cpp#L28) 的 50 Hz 下发；[motors_test.cpp](../../src/inference/examples/motors_test.cpp#L71)、[id_test.cpp](../../src/motors/examples/id_test.cpp#L108) 的单次下发后长期等待。
- **当前设计：** latency/ankle 工具每 20 ms 发一次，而连续命令只有效 10 ms；其它示例发一次后 sleep 数秒或进入只监控循环。latency 的提交 helper 还忽略 send_policy_setpoint 返回值。
- **问题类型：** 冗余、极低价值复杂度。
- **是否建议修改：简化。**
- **理由：** 正常时间安排下旧命令就在下一次刷新之前过期，RT 的锁存符合接口语义；不能为了让旧工具继续运行而关闭机器人 watchdog。这是工具问题，不是主策略链的同样缺陷；没有执行实机复现。
- **最小修改方案：** 要继续使用的工具把已有保持/监控循环改为小于 10 ms 的刷新间隔，例如 5 ms，并处理真实提交失败；单次下发后 sleep 的旧演示段删掉或在原循环内刷新。没有现行用途的整份旧示例可直接移除其源文件和 CMake target。不要给工具新建线程、恢复器或第二套 timeout。

### 32｜Xbox 的 fd 锁覆盖了由启停顺序保证的访问

- **位置 / 模块：** [xbox_controller.cpp](../../src/xbox_control/src/xbox_controller.cpp#L53)，open/is_open、polling_loop、read_available_events、close_device；[xbox_controller.hpp](../../src/xbox_control/include/xbox_controller.hpp#L95)。
- **当前设计：** io_mutex 保护 fd；但 fd 在启动 worker 前打开，close_device 先 stop/join 再关闭，运行中不替换设备。
- **问题类型：** 不可能场景防御。
- **是否建议修改：删除。**
- **理由：** 当前主线程串行管理生命周期，polling 线程运行时 fd 不变，读写不存在竞争；复制 fd 时加锁也没有保护整个 poll 调用。
- **最小修改方案：** 删 io_mutex 和相关锁，保持现有 open→start→stop/join→close 约束。state_mutex 继续保留，command/错误确由 polling 线程写、policy 主线程读。手柄读取线程继续独立，不将设备 I/O 合并进策略循环。

## 看起来复杂、但建议保留的部分

下表逐项给出保留理由及边界。这里的“保留”不是建议再添加保护。

| 编号 / 位置 | 当前设计 | 问题类型 | 是否建议修改 | 当前部署理由 | 最小方案 |
| --- | --- | --- | --- | --- | --- |
| K01：[worker timing](../../src/inference/src/robot/policy_command_worker.cpp#L154) / [admission](../../src/inference/src/robot/robot_interface.cpp#L411) | 40 ms 准入、60 ms hold、10 ms 命令期限、首帧起点 | 可以保持 | **保留** | 分别覆盖迟到结果、无新 target、worker 失活；已存在真实 freshness 排查记录。RT 最终期限不能被重复发布旧命令续期 | 不改变门限、drop 语义和旧命令过期优先顺序 |
| K02：[valid PDO](../../src/motors/src/drivers/myact/myact_motor_controller.cpp#L363) / [feedback age](../../src/inference/src/robot/policy_command_worker.cpp#L236) | 完整 WKC 才更新时间；各轴 last-valid，worker 算最老时间并封顶 deadline | 可以保持 | **保留** | 不完整周期不应被包装成新反馈；通信线程与计算线程可以分别停顿。策略入口检查不能覆盖独立 worker | 不新增周期号、同周期 barrier、额外 last-valid 镜像 |
| K03：[feedback fan-out](../../src/motors/src/motor_base/motor_controller_base.cpp#L512) / [SPSC](../../src/base/include/spsc_latest_channel/spsc_latest_channel.hpp#L67) | 同一反馈发布到两条 SPSC，消费者无新帧时使用各自缓存 | 可以保持 | **保留** | policy 和 command 是两个真实消费者；一条 SPSC 不能被两人消费。没新帧不等于旧帧立即无效 | 保留缓存与首次就绪语义，按年龄限制；不手写 ARM barrier 或新多读者通道 |
| K04：[STOP mailbox](../../src/motors/src/motor_base/motor_controller_base.cpp#L274) / [whole-body fault](../../src/motors/src/drivers/myact/myact_motor_controller.cpp#L383) | STOP 绕普通队列，RT 再次检查；请求/确认/释放 ID 与全身锁存 | 可以保持 | **保留** | main 和 worker 都会请求 STOP，属于实际多生产者；普通模式/RESTART 可与 STOP 排队交错，不能忽略旧命令失效。各轴停机回读也不同时完成 | 保留 CAS、ID 和确认屏障；本轮不改整套 STOP 状态。单轴兼容可待明确删掉相关工具/API 后集中缩窄，不零散删状态 |
| K05：[driver control_ready](../../src/motors/src/drivers/myact/myact_motor_controller.cpp#L95) / [mode switching](../../src/motors/src/drivers/myact/myact_motor_controller.cpp#L667) | desired/observed、enabled/control_ready、多步模式切换 | 可以保持 | **保留** | 期望使能不等于驱动已使能，模式写入不等于回读确认；启动就真实需要 0x6060/0x6061/0x6040/0x6041 推进。worker 快照检查和 RT 接受检查发生在不同时间 | 保留已有闭环。CSP 仍被 id_test 使用，不能仅因 policy 用 PVT 删除；CSV/CST 未见生产下发消费者，可随命令 API 精简另行移除 |
| K06：[ankle control](../../src/inference/src/robot/action_processor.cpp#L316) / [observation ankle](../../src/inference/src/robot/observation_builder.cpp#L337) | 两条线程各自 FK/Jacobian，历史解连续性和力矩滤波状态 | 可以保持 | **保留** | 50 Hz 观测和 1 kHz 力矩使用不同时间的最新反馈，不能共享一个有状态 solver；速度/力矩两处相似数学代码有不同输出用途 | 不引入公共踝服务或第三线程。保留有界求解、奇异检查、物理限位、最终数值/PDO 表示范围检查；零初始化/恒定 count 可局部删，无须重写数值方法 |
| K07：[watchdog](../../src/motors/src/drivers/myact/myact_motor_controller.cpp#L446) / [slave watchdog](../../src/motors/src/protocol/ethercat/ethercat_adapter_igh.cpp#L169) | 通信连续坏周期、命令 freshness、从站硬件 watchdog | 可以保持 | **保留** | 分别看到总线状态、命令生产停止、主机收发停止；任一个都不能完全替代另两个 | 不增滚动错误率 watchdog 或“总安全状态机” |
| K08：[shutdown](../../src/inference/src/robot/robot_interface.cpp#L106) / [communication latch](../../src/motors/src/drivers/myact/myact_motor_controller.cpp#L470) | 先 STOP，再 join；未确认保留 RT；持续通信丢失时区分资源释放；通信 fault 与 terminal fault 各有状态 | 可以保持 | **保留** | RT 若先退出就无法继续停机确认。通信 latch 还决定未确认释放，与通用 terminal latch 并非同一消费者语义；这里的 RetryRequired 用于释放确认，不是运行中恢复 | 不将两个 latch 盲目合并成大 fault enum；不删结果类型或将未确认说成成功，不添加恢复分支 |
| K09：[configured thread](../../src/base/include/tool/thread_runtime.hpp#L199) / [Torch setup](../../src/inference/src/policy/policy_runtime.cpp#L206) / [memory lock](../../src/inference/src/robot/robot_interface.cpp#L205) | 启动握手、affinity/FIFO、Torch/OpenMP/OpenBLAS 配置及新 worker 检查，按需锁页+栈预触碰 | 可以保持 | **保留** | 设置被权限/cpuset 拒绝是现实边界；Torch 创建的线程不同于手工创建线程。MCL_ONFAULT 有已记录的板端回归依据 | 仅删第 07 项死字段；不为形式统一重写线程 helper，不删锁页或扩大 warm-up 次数 |
| K10：[adapter interface](../../src/motors/include/protocol/ethercat/ethercat_adapter.hpp#L21) / [sessions](../../src/inference/include/robot/robot_interface.hpp#L80) / [PolicyRuntime Impl](../../src/inference/src/policy/policy_runtime.cpp#L192) | 通信 adapter、会话分工、Torch PImpl、一个控制器基类 | 可以保持 | **保留** | FakeAdapter 是真实的无硬件测试实现；session 有资源/线程所有权，PImpl 隔离大型依赖。控制器基类虽只有一个生产型号，大合并会搬动已经验证的 STOP/命令逻辑 | 本轮不合并整个基类/驱动、session/RobotInterface；不为第二种硬件继续增加抽象 |
| K11：[Xsens publish](../../src/imu/src/protocol/xsens_mti/can_parser.cpp#L186) / [reset fallback](../../src/inference/src/robot/action_processor.cpp#L465) | 两种字段 fresh 后发布，主机时间取较老字段；reset FK 不可解用默认姿态 | 可以保持 | **保留** | fresh 标志不强制同一采样帧，较老时间避免长期未更新字段伪装成新数据；用户接受默认姿态复位 | 不增加帧号/配帧。保留简单四元数无效值检查，不要求收紧到复杂范数/重新归一化策略；PDO padding 也保持当前映射 |
| K12：[AHRS snapshot](../../src/inference/include/robot/robot_imu_session.hpp#L17) / [recorder](../../src/inference/src/recorder/inference_recorder.cpp#L347) / [Xbox state](../../src/xbox_control/include/xbox_controller.hpp#L21) | Euler、设备 sample timestamp、原始手柄轴/has 标志、完整观测 CSV，后台队列/flush/状态 | 可以保持 | **保留** | Euler 被 imu_only_test 打印，sample 时间用于其频率统计及 CSV；手柄原始字段被 xbox_input_test 使用。CSV target/反馈被绘图消费；logger 的运行和停止请求语义不同，文件 I/O 需隔离 | 不冒称死字段。保留现有有界队列、文件错误检查、后台 recorder 与必要状态；不因低概率磁盘阻塞新建落盘恢复器或统一无锁日志系统 |

另外，EtherCAT 的绝对睡眠在迟到后会追赶过去周期（[thread_func](../../src/motors/src/motor_base/motor_controller_base.cpp#L184)）；源码可证明这种行为，但本次没有取得它造成当前部署问题的时延证据。**本轮先保留，不增加 deadline-miss 计数、迟到状态机或新的停机门限。** 如果真实运行证实追赶有害，最小修改是复用现有 policy_cmd 的跳过过去唤醒点做法；不扩大成实时调度框架。

## 从整个系统看，应该怎样精简

### 1. 最大的五个复杂度来源

| 复杂度来源 | 必要部分 | 额外部分 |
| --- | --- | --- |
| 离散命令、STOP 与驱动模式推进 | 多生产者停机、物理回读确认、全身 fault、期望与实际区分 | 大命令对象进离散队列、恒定参数入状态、无等待意义的 QUEUED、全身操作拆成12个命令 |
| 踝机构与两个1 kHz线程 | 并联踝 FK/IK/Jacobian/滤波/力矩；计算与总线职责分离 | 固定长度 vector、中转复制、adapter 同线程原子镜像；不把两份有状态 FK 当成可随意合并的重复 |
| freshness/admission/首帧保持 | 三种故障的时间预算、首帧阻塞约束、最后有效反馈期限 | 独立 skew 判据的需求不明确、伪 target 错误包装、重复计数/active payload/阶段状态 |
| 主机准备和库线程控制 | 固定绑核/FIFO、锁页、Torch worker 约束及启动失败检查 | 两套完整主机规则 parser、总 enabled 分支、无用线程名读回、旧网络 guard 模板 |
| 观测模型兼容与日志 | 当前705维训练布局、15帧历史、实际 CSV 排障/绘图、后台写盘 | 675兼容、部署 A100 选择、无人消费的训练 metadata、日志和 target 元数据重复、诊断两级线程 |

这些来源按代码和状态耦合判断，不代表已经测得某个模块占用最多 CPU，也不代表数值求解慢于实时预算。

### 2. 实时部署真正需要的复杂度

需要保留的是能在现有链路中说明消费者和故障来源的机制：最后有效 PDO、命令最终 deadline、STOP 绕普通队列、全身故障锁存、实际驱动状态确认、踝求解及限位、线程配置握手、锁页和后台 I/O。它们防的是当前硬件/线程实际会遇到的问题，而非为了覆盖任意用户、任意模型、任意并发调用。

不用为了“所有理论风险”继续扩展这些机制。现有 single-producer 契约、启动所有权交接、先 join 再关 fd、同线程设置速度等，应当直接作为调用合同使用。

### 3. 明显的 over-engineering

最明确的是：运行不用却强制验证的训练 metadata（03）、不变表的事务式配置与镜像（05）、同线程状态 atomic 镜像（13）、仅供错误输出的阶段/伪 target 状态（17/19）、离散和连续目标塞入一个大载荷（21）、只有一个诊断读者却使用两级线程（24）、两套固定主机规则（25），以及无当前模型/硬件需求的兼容分支（28/29）。

有两个 1 kHz 线程、存在多个期限、存在 interface 本身都不足以证明 over-engineering。要看真实消费者、哪个线程还能继续运行、以及合并是否搬动了必要职责。

### 4. 现在可以放心删除的内容

以下删除不缩窄当前策略部署行为；除纯死代码外仍需同步更新类型引用和相关测试：

- 主循环不输出的计时统计、空报告分支及弃用注释（01），模型重复可读检查（02）。
- 配置 loader 的临时 mapping 对象（04）；mapping 的方向/踝镜像（05）。
- ThreadSetupResult 未使用错误码、无消费者的线程名读回（07）；PolicyRuntime loaded 标志（08）。
- 当前一次启动合同中的旧 session 清理/重复 reset（09）；同线程速度 mutex、仅所有者访问的 atomic（10）。
- 私有 reset 的初始化顺序/已保证尺寸防御（11），固定拓扑之外的未选择宏分支及 unused offset（12）。
- adapter 健康状态 atomic 镜像及成功 init 后内部重复防御（13），Xbox fd mutex（32）。
- 第二个策略计数器（16）、纯诊断阶段状态（17）、重复 target 元数据（18）、首帧伪 target（19）。
- RT active command 中无人重读的 payload/独立 has-active（20），QUEUED 过渡阶段和 max_retries 镜像参数（23）。
- Xsens 不生成数据的原始 IMU callback 存储、无人调用的 CAN read 包装（27）。
- 不被安装且调用已删除命令的 NetworkManager 模板及相关伪 fixture（30）。

**需要配套缩窄消费者或改变输入/日志格式的删除另列：** metadata schema（03）、skew 判据（15）、日志完成通道（18 的后续方案）、675 兼容（28）、部署 A100 选择（29）、无用途的实机工具（31）。这些不能冒称为简单删一行而对所有现有接口零影响。

### 5. 看起来复杂，但本轮不应继续动的地方

先保持 STOP/mailbox/ID/确认与旧命令失效顺序、通信原因与终端 fault 的区分、shutdown 未确认时的资源保留、驱动模式状态机、踝数值求解及其线程独占状态、两条反馈 SPSC，以及 model dry-run/线程数/affinity/MCL_ONFAULT 链路。也保持用户已确认的三项行为。

不要为了架构整齐一次性合并所有 session、删控制器基类、将计算塞进 EtherCAT RT、设计统一 fault 状态机或新 telemetry 中心。收益尚不确定，代码搬动和验证成本很明确。

### 6. 只能精简一轮时的优先顺序

| 优先级 | 建议内容 | 为什么先做 | 最小验证 |
| --- | --- | --- | --- |
| 第一批：小范围删减 | 01/02/04/07/08/10/16/17/19/20/23/30/32 | 删除死逻辑和重复状态，易逐项审阅，基本不改控制行为；20明确减少1 kHz缓存复制 | 重新编译受影响目标；现有 timing、thread、recorder、Xbox 测试；不写镜像式新测试 |
| 第二批：固定数据和启动逻辑 | 05/06/09/11/12/13/14/21/22/24 | 减少动态分配、命令体积、运行分支和一个诊断线程 | 保留/运行 SPSC、motor realtime、动作/观测/CSV、初始化顺序回归；台架核对启动、reset、STOP |
| 工具同步 | 31 | 现有刷新周期与deadline直接冲突，继续使用会给部署调试错误反馈 | 无硬件确认刷新间隔/返回值；要使用的工具再在既有台架流程验证，不关闭watchdog |
| 最后按范围取舍 | 03/15/18后续/25/26/27/28/29 | 涉及输入格式、日志语义、独立工具或启动环境依赖，应按当前实际使用范围成组缩窄 | 对照当前 YAML→705输入→12动作、CSV消费者、脚本fixture；不增加未来兼容矩阵 |

一轮修改的目标应当是：**正常策略链减少无用状态和复制，启停/STOP/故障/踝控制的已有语义仍可直接验证。** 不以删掉多少 watchdog、引入多少统一接口或新架构作为成果指标。

## 本次验证与交付边界

- 对当前 tracked 生产源码、关键配置、构建入口、部署脚本及相关测试/调试消费者进行静态追踪；工具不可用 `rg`，以文件列表、文本搜索和小脚本交叉核对引用。
- 实际 CPU 离线加载当前 `policy.pt`，输入 `(1,705)`，输出 `(1,12)`、float32，输出有限；仅验证当前零输入运行及形状，不宣称完成训练语义或闭环性能验收。
- 使用当前头文件编译无硬件大小探针，测得 InferenceRecord 3592 B、PolicyTargetFrame 3616 B、ControlCommand 632 B、CommandTiming 24 B、ImpedanceSetpoint 40 B。
- 已运行 `bash deploy/rk3588/test_robot_rt_setup.sh`，通过。该脚本将 proc/sys/etc/dev/install 路径指向临时 fixture，跳过真实 systemd；未运行正式 install/__apply/check、硬件示例或真实电机测试。
- 本次没有改源码，所以没有将旧 build 二进制的通过结果当成最新全量回归，也没有为了纯文档改动运行整个硬件部署程序。以上建议尚未实施，其验证列是后续精简的最小验收范围。
- 两份历史报告移到本目录，修正因移动改变的相对链接；历史正文结论保留，另加归档说明，当前判断以本报告为准。
