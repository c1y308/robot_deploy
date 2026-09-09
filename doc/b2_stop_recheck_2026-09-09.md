# B2 STOP 路径复核（2026-09-09）

## 修复实施结果

已按确认的方案实施软件修复。`send_discrete_command(STOP)` 现在使用独立固定状态槽，普通队列容量不影响 STOP；同目标未完成请求复用 ID，未完成 STOP 不受普通命令的 64 项结果缓存淘汰影响。每个目标只保留最新 STOP，已完成请求被同目标的新请求替换后，旧 ID 可返回 UNKNOWN。

- [STOP 提交与 RT 服务](/home/cat/robot_deploy/src/motors/src/motor_base/motor_controller_base.cpp:585)：RT 在普通命令处理前及输出周期前处理 STOP。目标轴上的旧 RESTART/SET_MODE 按 ID 截止值失效，完成状态为 FAILED；全轴 STOP 未全部确认前，任何目标轴都不能先行 RESTART。已完成的普通命令历史仍沿用原有有界缓存。
- [驱动确认](/home/cat/robot_deploy/src/motors/src/drivers/myact/myact_motor_controller.cpp:847)：要求有效 link/WKC、目标轴配置及 operation-enabled 清除，保留间隔 20 tick 的两次成功验证。每周期发现无效回读会撤销该轴尚用于聚合的确认，避免旧轴确认与另一轴稍后的确认拼成整机成功。通信故障锁存本身不再表示 STOP 完成。
- [会话接口](/home/cat/robot_deploy/src/inference/src/robot/robot_motor_session.cpp:141)：增加不等待完成的 request_stop，stop 改为确认等待；deinitialize 返回 bool，超时保留 RT/控制器/adapter。STOP 不因普通命令的 4000 tick 超时退出，确认恢复后仍可成功；等待使用现有 `discrete_command_completion_timeout_ms`，默认 4000 ms。
- [整机停机](/home/cat/robot_deploy/src/inference/src/robot/robot_interface.cpp:110)：STOP → 关闭生产标志 → 确认 → worker join → 策略/记录器 → IMU → 电机线程释放。故障入口在诊断和 join 前发布 STOP；初始化先确认初始 STOP，再提交模式配置，不能覆盖上次停机失败后保留的控制器。
- 两个上层析构函数持续重试确认；**持续断链时，析构/进程正常退出可能一直等待**，这是本次明确选择的行为。底层 MotorControllerBase::shutdown 仍是线程释放原语，上层只在确认后调用它。
- 全轴普通 STOP 会作废已停用的运动目标并丢弃等待期间的 setpoint，避免正常停止等待超过默认 10 ms 后被旧目标的有效期额外锁死。已经发生的过期/通信故障锁存不清除，部分轴停止时仍保留其余运行轴的新鲜度保护。

修复实施时的历史验证结果（下述新增测试已按要求删除，数量不代表当前测试集）：

| 检查 | 结果 |
| --- | --- |
| `cmake -S src/motors -B /tmp/b2-motors-build` 与 `cmake --build /tmp/b2-motors-build -j 4` | 通过 |
| `ctest --test-dir /tmp/b2-motors-build --output-on-failure` | 2/2 通过；[日志](/tmp/b2-motors-ctest.log) |
| `cmake -S src/inference -B /tmp/b2-inference-build` 与 `cmake --build /tmp/b2-inference-build -j 4` | 通过；Torch 配置有可选 kineto 静态库缺失警告，未影响构建 |
| `ctest --test-dir /tmp/b2-inference-build --output-on-failure` | 10/10 通过，包含上述 motors 测试；[日志](/tmp/b2-inference-ctest.log) |
| `git diff --check` | 通过 |

当时的 `motor_stop_test` 覆盖：初始 STOP/模式配置/重启、单轴容量 2 和 16 实际填满、入口 64 填满、不对称队列、单轴停止、在途模式取消、结果缓存绕回、重复及并发 STOP、重叠请求、全轴部分确认与失效回读、4000 tick 后迟到确认、断链/WKC 故障及恢复、短有效期目标后的正常 STOP/RESTART。`robot_shutdown_test` 覆盖：RT 暂停时有界返回失败、资源保留/禁止覆盖、阻塞 worker join、两条故障入口的阻塞日志 sink、两个上层析构等待和恢复。测试使用内存 adapter 和受控线程屏障，没有连接硬件；真实驱动器响应与机械停止仍需实机验收。

按要求已删除此次新增的 STOP/shutdown 测试、FakeAdapter 辅助文件、探针入口、测试友元和 CMake 注册。功能修复及原有测试对新接口/语义的适配保留。下面的原始复核表是**修复前历史证据**。

## 修复前复核记录

结论：B2 仍成立，建议保持 Blocker。普通 STOP 可以在提交入口被拒绝，也可以在返回 ACCEPTED 后因单轴队列满而失败；未满队列也会阻塞 STOP。上层未等待停止确认便关闭 RT 循环。当前已有独立的新鲜度过期保护，因此旧报告关于日志阻塞后持续输出的描述需要补充适用条件。

复核对象为 HEAD `a08bb97f3650c2070b652d2571789321d344ee4a` 加当时工作区。开始复核时 `robot_motor_session.hpp/.cpp` 已有未提交的 setpoint API 改动；复核阶段未修改生产代码。旧报告 `/tmp/robot_deploy_audit_20260907/fault_probe.cpp` 已不存在，当时重新建立了无硬件探针（现已按要求删除）。

## 代码证据与判定

| 项目 | 当前代码 | 判定 |
| --- | --- | --- |
| 提交成功仅表示入口入队 | [motor_controller_base.cpp:369](/home/cat/robot_deploy/src/motors/src/motor_base/motor_controller_base.cpp:369) 分配 ID、初始化结果、尝试 `cmd_queue_.try_push()`，成功即返回 ACCEPTED | 成立。入口满时 STOP 与其他命令一样返回 QUEUE_FULL |
| 二级队列可丢 STOP | [motor_controller_base.cpp:794](/home/cat/robot_deploy/src/motors/src/motor_base/motor_controller_base.cpp:794) 单轴 `push_back()` 失败只标记失败并发送事件 | 成立。全轴 STOP 按轴分别入队，可部分执行、部分失败 |
| STOP 不抢占普通命令 | [motor_controller_base.cpp:615](/home/cat/robot_deploy/src/motors/src/motor_base/motor_controller_base.cpp:615) 每周期按预算分发；[831](/home/cat/robot_deploy/src/motors/src/motor_base/motor_controller_base.cpp:831) 每轴只处理队头 | 成立。默认入口容量 64、单轴容量 16、每周期分发 16，见 [RealtimeOptions](/home/cat/robot_deploy/src/motors/include/motor_base/motor_controller_base.hpp:31) |
| 两个 20 tick 验证点 | [command_types.hpp:398](/home/cat/robot_deploy/src/motors/include/motor_base/command_types.hpp:398)、[状态机](/home/cat/robot_deploy/src/motors/src/motor_base/motor_controller_base.cpp:868) | 成立。命令在 tick t 应用，理想情况下 t+20、t+40 两次满足才完成；前驱完成当周期不会继续服务新的队头 |
| 4000 tick 超时 | [入轴队列时计算 deadline](/home/cat/robot_deploy/src/motors/src/motor_base/motor_controller_base.cpp:783) | 成立，但这是从进入单轴队列开始计时，并非 STOP 的延迟保证；300 tick 重试间隔、最多 10 次也可更早导致失败。tick 只有在默认 1 ms 周期下才对应名义 ms |
| `stop()` 返回 true 未确认停止 | [robot_motor_session.cpp:157](/home/cat/robot_deploy/src/inference/src/robot/robot_motor_session.cpp:157)、[submit_command:349](/home/cat/robot_deploy/src/inference/src/robot/robot_motor_session.cpp:349) | 成立。command_id 被丢弃，`motion_enabled_=false` 仅是应用层门控。RESTART 已有结果等待，STOP 没有 |
| 50 ms 后关闭 RT | [robot_motor_session.cpp:143](/home/cat/robot_deploy/src/inference/src/robot/robot_motor_session.cpp:143) 忽略 `stop(-1)` 返回值，sleep 50 ms，调用 shutdown；[基类 shutdown](/home/cat/robot_deploy/src/motors/src/motor_base/motor_controller_base.cpp:285) 先置 running=false 再 join | 成立。shutdown 不排空 STOP、不补发最后一次停止 PDO；[MyAct 停止回调](/home/cat/robot_deploy/src/motors/src/drivers/myact/myact_motor_controller.cpp:287) 仅关闭监控/诊断并打印 |
| STOP 排在日志和线程退出之后 | [RobotInterface::shutdown](/home/cat/robot_deploy/src/inference/src/robot/robot_interface.cpp:107)、[worker join](/home/cat/robot_deploy/src/inference/src/robot/robot_interface.cpp:381)、[unload_policy](/home/cat/robot_deploy/src/inference/src/robot/robot_interface.cpp:294) | 成立。记录器 [stop](/home/cat/robot_deploy/src/inference/src/recorder/inference_recorder.cpp:282) 等待执行写盘/flush 的 worker；[IMU deinitialize](/home/cat/robot_deploy/src/inference/src/robot/robot_imu_session.cpp:84) 也先停止 reader，最后才进入电机 deinitialize |
| 等待 SUCCEEDED 即可证明停机 | [myact_motor_controller.cpp:848](/home/cat/robot_deploy/src/motors/src/drivers/myact/myact_motor_controller.cpp:848) 在通信故障锁存时直接对 STOP 返回 SATISFIED | **不成立，是本次补充发现。** 该分支在检查通信和 `operation_enabled` 之前返回；不能把这种 SUCCEEDED 当成驱动器回读确认 |

## 无硬件复现结果

探针使用真实 `MotorControllerBase` 和 `MyActMotorController`，仅将 EtherCAT adapter 替换为内存实现。双轴先完成 IMPEDANCE 模式设置和 RESTART 确认，再施加原始前馈力矩 123、456。FakeAdapter 在周期末设置屏障，批量提交后按指定周期数推进；Rx 默认持续报告 operation-enabled，只有指定确认用例才改为 disabled。

保留默认 1 ms 周期、入口容量 64 和每周期分发预算 16；仅针对容量 2 用例改变单轴容量。使用 `rt_priority=0`，不需要 RT 权限，不连接物理主站。为独立观察普通 STOP，前几例将 debug setpoint 有效期设为 5 秒；该设置不是生产配置，也不是证明默认 10 ms 新鲜度保护失效。另设 100 ms 有效期用例，单独验证过期保护仍生效。

| 场景 | 实测提交/结果 | 输出 PDO |
| --- | --- | --- |
| 每轴容量 2，2 条全轴 RESTART 后追加 STOP，推进 2 tick | ACCEPTED → FAILED | 两轴保留 123/456，均为 `0x000f` |
| **默认每轴容量 16**，16 条全轴 RESTART 后追加 STOP，推进 2 tick | ACCEPTED → FAILED | 两轴保留 123/456，均为 `0x000f` |
| 仅轴 0 填满默认容量 16，再提交全轴 STOP | ACCEPTED → FAILED | 轴 0 为 123 / `0x000f`；轴 1 为 0 / `0x0007` |
| 队列未满，2 条可正常确认的 RESTART 后追加 STOP，推进 50 tick | ACCEPTED → PENDING | 两轴仍为 123/456、`0x000f`；STOP 尚未应用 |
| 空队列 STOP，但 Rx 不确认 disabled，推进 50 tick | ACCEPTED → PENDING | 两轴为 0 / `0x0007`；已输出停止指令不等于驱动器已确认 |
| 上一例随后提供 disabled 回读，再推进 31 tick | SUCCEEDED | 两轴继续为 0 / `0x0007` |
| 填满默认入口队列 64 后提交 STOP | QUEUE_FULL，无 command_id | STOP 在入口即被拒绝 |
| STOP 已因默认单轴队列满而 FAILED，随后使已消费的 setpoint 过期 | STOP 仍为 FAILED，freshness latch=true | 两轴变为 0 / `0x0007`；旁路保护有效 |
| 总线健康改为 link-down，持续 Rx operation-enabled，watchdog 锁存后提交 STOP | **ACCEPTED → SUCCEEDED** | 内存 Tx 为 0 / `0x0002`；没有 disabled 回读，断链下也不能证明驱动器收到该 PDO |

所有探针断言通过。这里的 PASS 表示复现了表中缺陷/边界，并非生产代码通过安全验收。50 tick 的用例验证的是状态机进展，不能用其替代真实 50 ms 调度或机械停止时间的测量。原始力矩为 PDO 单位，不是 N·m。

历史复现命令（探针源码现已删除，不能直接在当前工作区运行）：

```bash
g++ -std=c++17 -O2 -pthread \
  -I src/motors/include -I src/base/include -I /home/cat/ethercat/include \
  doc/probes/b2_stop_queue_probe.cpp \
  src/motors/src/motor_base/motor_controller_base.cpp \
  src/motors/src/motor_base/rt_event_dispatcher.cpp \
  src/motors/src/drivers/myact/myact_motor_controller.cpp \
  src/motors/src/drivers/myact/myact_debug_printers.cpp \
  -o /tmp/b2_stop_queue_probe
timeout 15s /tmp/b2_stop_queue_probe
```

该编译只使用 `ecrt.h` 的类型定义，不链接/实例化 IGH adapter。当前执行的[完整日志](/tmp/b2_stop_queue_probe.log)可供查看；关键结果已持久化在上表。另用同一组编译参数和源文件，将 probe 源替换为 `src/motors/tests/motor_realtime_channel_test.cpp`，编译为 `/tmp/b2_motor_realtime_channel_test` 并运行 `timeout 30s`，退出码为 0；[日志](/tmp/b2_motor_realtime_channel_test.log)。现有测试通过未排除本次新增场景。

## 对旧报告的必要修正

“STOP 没有抢占路径”应限定为**公开的普通 STOP 请求**。当前 [process_latest_setpoint_commands](/home/cat/robot_deploy/src/motors/src/motor_base/motor_controller_base.cpp:632) 在已有 active setpoint 到期时锁存，并调用 [apply_safety_stop](/home/cat/robot_deploy/src/motors/src/motor_base/motor_controller_base.cpp:739) 直接对全轴施加 STOP，绕开离散队列。通信 watchdog 也有直接输出 quick-stop 的路径。这些旁路没有接入普通 `stop()`，也没有给普通 STOP 补充完成确认。

“故障处理在请求停机前输出日志并 join”已不是当前代码的完整描述：[handle_policy_step_failure](/home/cat/robot_deploy/src/inference/src/robot/robot_interface.cpp:726) 当前打印、置 initialized=false、停止/join worker，**函数中没有显式 STOP 请求**，注释说明由 RT 执行保护；[fail_policy_command_worker](/home/cat/robot_deploy/src/inference/src/robot/robot_interface.cpp:524) 同样只停生产者并保存/打印错误。shutdown 的显式 STOP 仍在日志、IMU 释放之后。

因此日志/flush/join 阻塞会推迟显式 STOP 和资源释放，但在“RT 持续推进、已有 active setpoint、生产者停止刷新”的条件下，过期保护可以独立改变输出，不能再无条件宣称旧力矩会随日志阻塞无限保持。当前 [控制命令新鲜度默认预算](/home/cat/robot_deploy/src/inference/include/robot/robot_config.hpp:95) 为 10 ms；这是有效期预算，不是机械停机时间，也不是在 RT 阻塞情况下的执行保证。尚无 active setpoint 时，过期条件不会触发；故障日志之前生产者也可能尚未被停止。

此次未注入真实磁盘/日志阻塞、未执行整机 shutdown 的动态集成测试、未接硬件；关于 shutdown 阻塞顺序的结论来自代码路径。内存 PDO 变化只证明主机侧控制器生成了对应输出，不能证明驱动器收到、已去使能或机械系统停止。

## 修复应覆盖的范围

1. 为普通 STOP/故障停机建立无需普通队列空位的请求路径，由 RT 在输出前优先处理。重叠目标轴上的旧 RESTART/SET_MODE 必须失效，完成状态应可追踪，防止旧命令随后重新使能。
2. 区分提交接受、RT 已施加保护、驱动器已确认停止。修正通信故障锁存下直接 SATISFIED 的判据；缺失/失效通信反馈不能冒充停止确认。仅给现有 `stop()` 添加 SUCCEEDED 轮询不足以修复。
3. shutdown 和策略故障入口先发起停机并禁止继续生产运动命令，再做日志、join 和资源释放；确认期间保留 RT 收发。以有界的明确完成/失败结果替换固定 50 ms 等待，规定断链、超时后的处理及驱动器 watchdog 配合。
4. 验收至少覆盖两级队列满、单轴队列不对称、RESTART/SET_MODE 在途抢占、停止后的旧命令、正常/缺失/失效回读、日志阻塞和 RT 过期保护。软件输出时限与实机去使能/机械停止分别验证。

上述为复核后的修复边界，此次未实施生产逻辑修复。
