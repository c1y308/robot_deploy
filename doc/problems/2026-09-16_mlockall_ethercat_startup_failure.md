# RK3588 锁内存引起 EtherCAT 启动失败的排查与修复记录

- 日期：2026-09-16。
- 对照基线：`41213ce6078f065bd888c912102f1a9812a248fb`（添加故障锁存诊断标准）。
- 环境：RK3588，Linux `6.1.99-rt36-rk3588`，IgH EtherCAT。
- 状态：关闭锁内存的板端对照已恢复正常；已恢复按需锁页并完成编译和无硬件测试，板端长期稳定性尚未完成独立验收。
- 范围：记录启动通信异常、初始化失败及锁内存修复，不展开随后出现的运行期策略 freshness 超时。

## 1. 问题现象

增加 CPU 隔离、模型初始化前置和进程内存锁定后，启动阶段先后观察到：

```text
[MYACTUA] motor fault latched on motor 1, reason=OFFLINE, configured=0
[ERROR] robot.initialize() failed.
```

用户反馈基线版本之前可以正常运行。当前 `HEAD` 即上述基线，新增差异来自工作区未提交修改；排查中没有整体回退，也没有覆盖既有故障诊断和停机修改。

为减少启动瞬态的终端锁存，策略启动链已将 OFFLINE/通信 watchdog 保护延后到第一次正式 `policy_step()` 推理前开启。此前带连续就绪计数、命令门禁等逻辑的复杂启动豁免方案已撤销，不属于最终实现。

延后锁存后仍出现初始化失败，说明不能仅用“保护提前开启”解释：通信保护开关不会恢复从站 OP，也不会使 WKC 完整，STOP/RESTART 确认条件仍然有效。

## 2. 最近一次初始化失败的内核日志

2026-09-16 15:58 这次运行的关键时间线：

```text
15:58:41.510212  EtherCAT: Requesting master 0...
15:58:41.520615  Domain0: expected working counter 36.
15:58:56.265225  Slave states on main device: PREOP, OP.
15:58:56.284211  EtherCAT ERROR 0-3: AL status message 0x001A: "Synchronization error".
15:58:56        Domain 0: 6 working counter changes - now 14/36.
15:58:56–15:59:00  多个从站报告 0x001A，并被确认退到 SAFEOP。
15:59:00.925216  EtherCAT 0: Releasing master...
```

这些日志确认初始化期间存在真实的同步异常和过程数据不完整，并非只有应用层误报。`configured` 要求从站同时 `online && operational`，所以退出 OP 也会显示 `configured=0`，不能直接解释成物理断线。

应用终端的完整前置错误没有留存，因此没有唯一确定 `initialize()` 的返回分支。`RobotMotorSession::initialize()` 中启动 RT 后的 `if (!stop()) return false;` 是可疑点，但不能据此排除启动前等待或 RESTART 失败。

## 3. 与基线的关键差异

| 项目 | 基线 | 出现问题时的实现 |
| --- | --- | --- |
| 初始化顺序 | EtherCAT RT 启动后加载模型 | 先加载模型、单次 dry-run、锁内存，再初始化 EtherCAT |
| 进程内存锁定 | 无新增进程锁定要求 | `mlockall(MCL_CURRENT \| MCL_FUTURE)` |
| 栈预触碰 | 无新增预触碰配置 | policy/cmd/motor 128 KiB，IMU 64 KiB |
| 内核隔离 | CPU6–7；IRQ CPU0–5；workqueue `3f` | CPU4–7；IRQ CPU0–3；workqueue `0f` |
| 用户线程布局 | policy CPU4–5，cmd CPU6/FIFO70，ecat CPU7/FIFO80 | 相同 |
| 启动前就绪判断 | 一次全部电机 OP | 连续十次全部电机 OP 且 link/WKC 正常 |
| 从站 watchdog | 无显式 divider/intervals 设置 | 新增显式约 100 ms 配置 |

DC 设置、控制周期和部署模型配置没有变化。主线程在模型加载前绑定 CPU4–5、检查 Torch 新增 worker TID 的机制在基线中已经存在，不能将回归简单归因于新增用户线程绑核。

上述差异均作为候选分别检查，没有同时回退多个变量来掩盖原因。

## 4. 无硬件线程创建对照

代码检查发现，启动链保持：

```text
connect → 主线程物理收发并等待就绪 → start() → RT 线程接管物理收发
```

等待结束后，`MotorControllerBase::start()` 依次启动状态发布、事件分发、诊断和 EtherCAT RT 线程。在主线程最后一次物理收发到 RT 第一次物理收发之间，没有持续 PDO 交换。这一交接空档在基线中已经存在，锁内存可能将其放大。

独立探针采用同样的四线程创建顺序、CPU 绑定及启动握手，不连接 EtherCAT、不运行模型，也不设置 FIFO。四个线程保持同时存活；分别以新进程运行三次，避免上一轮线程栈缓存干扰。

实测每个线程的默认栈映射为 **8 MiB**：

| 对照方式 | 四线程创建总耗时 |
| --- | ---: |
| 不锁内存、不预触碰栈，模拟基线 | 0.34–0.58 ms |
| 仅对最后一个线程预触碰 128 KiB 栈 | 0.36–0.39 ms |
| `MCL_CURRENT \| MCL_FUTURE`，保留预触碰 | 11.46–12.10 ms |

后续增加 `MCL_ONFAULT` 的三轮对照：

| 对照方式 | 四线程创建总耗时 |
| --- | ---: |
| `MCL_CURRENT \| MCL_FUTURE` | 12.74–13.17 ms |
| `MCL_CURRENT \| MCL_FUTURE \| MCL_ONFAULT` | 0.36–0.38 ms |

结论：大额开销主要来自创建新映射时的整栈填充和锁定，而不是显式预触碰的 128 KiB。上述数据是独立探针结果，不是实际 EtherCAT 交接耗时或 RT 调度延迟测量。

## 5. 板端单变量确认

只修改 RK3588 profile：

```cpp
profile.require_process_memory_lock = false;
```

用户反馈：**“恢复正常”。**

该对照将启动回归基本定位到新增锁内存路径，支持“后续线程创建开销放大收发交接空档”的判断。因此没有继续放宽通信保护，也没有修改 DC 参数、CPU 布局或安全超时。

证据边界：板端开关对照确认锁内存路径与故障相关，独立探针确认整栈锁定的大额创建开销；具体 DC 失同步因果仍未通过实际交接时间或调度 trace 直接测量。

## 6. 最终最小修复

重新启用锁内存：

```cpp
profile.require_process_memory_lock = true;
```

锁定参数增加 `MCL_ONFAULT`：

```cpp
mlockall(MCL_CURRENT | MCL_FUTURE | MCL_ONFAULT);
```

当前驻留页立即锁住；其余当前/未来映射的页面在首次触及时锁住，不因创建映射而一次性填充整个默认线程栈。

保留现有行为：

- 模型只执行原有单次 dry-run，不增加模型预热循环。
- 锁定发生在模型加载和 Torch worker 校验后、硬件初始化前。
- 锁定失败输出 `errno`，初始化立即失败，不静默降级为不锁内存。
- policy_main 在锁定后再次预触碰 128 KiB 栈。
- 新线程按名称/affinity、栈预触碰、调度策略设置、启动握手、业务循环的顺序执行。
- 普通 shutdown 不调用 `munlockall()`。
- 不修改策略 20 ms 周期和 60/10 ms 安全超时，也不增加新的保护状态机。

源码添加了说明问题的注释：整栈填充可能拉长 EtherCAT 启动收发交接空档；按需锁页配合业务循环前的栈预触碰用于避开这笔大额启动开销。

恢复锁内存时的代码修复涉及：

- [RK3588 profile](../../src/inference/include/robot/rk3588_runtime_profile.hpp)：恢复锁内存开关。
- [RobotInterface](../../src/inference/src/robot/robot_interface.cpp)：增加 `MCL_ONFAULT`，同步错误信息和问题说明注释。
- [内存锁定初始化测试](../../src/inference/tests/robot_initialization_memory_lock_test.cpp)：断言三个锁定标志及失败前不启动硬件/recorder。
- [部署说明](../../deploy/rk3588/README.md)：同步按需锁页语义和限制。

## 7. 验证记录与限制

执行过：

```bash
cmake --build src/inference/build -j2
ctest --test-dir src/base/build --output-on-failure
ctest --test-dir src/motors/build --output-on-failure
ctest --test-dir src/inference/build --output-on-failure
git diff --check
```

结果：

- inference 全量编译成功，包含 `policy_test`。
- base：3/3 通过。
- motors：4/4 通过。
- inference：15/16 通过；锁内存初始化、profile 和 Torch threading 测试均通过。
- 唯一失败为既有 `deploy_config_model_order_test`：`raw_action_clip: null must disable raw action clipping`。部署配置实际为 `raw_action_clip: 1.0`，没有为通过测试修改控制配置。
- `git diff --check` 通过。
- 自动验证没有启动真实电机或 EtherCAT；初始化锁内存测试包装了 `ecrt_request_master`，不会取得硬件。

`MCL_ONFAULT` 从 Linux 4.4 起支持，当前内核和头文件支持该标志。账号仍须具备足够的 `RLIMIT_MEMLOCK` 或 `CAP_IPC_LOCK`。

按需锁页不保证所有页面提前驻留：未触及且非驻留页首次访问仍可能缺页；`malloc/mmap`、allocator bookkeeping 和 lazy operator initialization 仍可能产生运行时开销。固定图 dry-run 未覆盖的数据相关分支仍存在首次分配风险。

本记录不能作为六小时零超时、最大推理时间或 DC 长期稳定性的验收结论。

## 参考

- [Linux mlock/MCL_ONFAULT 说明](https://man7.org/linux/man-pages/man2/mlock.2.html)：锁页、按需填充和实时线程栈预触碰语义。
- [IgH Application Interface](https://docs.etherlab.org/ethercat/1.6/doxygen/group__ApplicationInterface.html)：应用时钟及从站 watchdog 配置接口。
