# SocketCAN 接收时间与 Xsens 组合数据年龄修复

日期：2026-09-18。按冻结方案实施，保持公开函数签名、`AHRSData` 布局、策略门限和 A100 路径兼容。

## 缺陷与定性调整

原 SocketCAN 路径在 `read()` 完成后调用主机单调时钟，因此积压在接收队列中的旧帧被重新标记为当前时间。Xsens parser 的 Q 或 Rate 帧还会覆盖统一接收时间，使完整 AHRS 的年龄只反映最后到达字段，而不是组成字段中最老字段。parser 原先直接修改公开数据，未读取的完整数据可能被后续半更新污染；全零四元数仍能算出有限重力并被标记有效。

此前 60 秒被动抓流包含 141516 帧。SampleTime 每次增加 25 tick，但实际 CAN 流存在 Rate 先于 Q、同一区间重复 Rate，以及待组装字段跨 SampleTime 到达的交错。各字段没有共同采样编号，抓流和已知消息定义不能建立严格 SampleTime 分组语义。因此“跨 SampleTime 配对”不再单独作为已证实的 P1 缺陷，本次保留已有 latest-value pairing。

本次关闭确定的接收时间、字段年龄、完整快照隔离与非法四元数问题。设备输出裁剪留作独立优化，不增加组内 skew 窗口、周期学习、SampleTime 分组或 RX overflow 接口。

## 实现行为

Socket 创建后、绑定前启用 `SO_TIMESTAMPNS_NEW`，失败则关闭 fd。`recvmsg()` 接收 Classic CAN 帧，通过正确对齐的 ancillary buffer、level/type/长度检查及 `memcpy` 提取 `__kernel_timespec`。`EINTR` 重新初始化消息并重试；`EAGAIN/EWOULDBLOCK` 返回 0，成功返回 1，其他错误返回 -1。输出接收时间在入口置零，只有全部检查成功后才赋值。缺失、截断或非法时间戳不会回退到当前主机时间。

打开时和每个成功接收后，各采样一次 `M1 → CLOCK_REALTIME → M2`，使用 `M1 + (M2-M1)/2` 计算 offset。baseline 仅校验跳变；`kernel_rx_realtime - current_offset` 才是实际转换。offset 与 baseline 的差超过 1 ms 时失效，恰好 1 ms 允许。读钟失败、采样顺序异常或半跨度达到 1 ms，按映射不可信处理；转换必须为正，未来偏差最多允许 1 ms。映射失效持续报错直到 close/open，不重新标记旧队列，也不添加 poll 时钟检查或永久补偿。

Xsens Q 和 Rate 各保存 `std::int64_t` 接收时间。两路自上次发布后都 fresh 时，使用最新待组装值发布，并取 `min(Q_rx, Rate_rx)`；发布后清除两路 fresh。Q Q Q R 使用最后一个 Q；R R Q 使用最后一个 R。SampleTime 在发布时复制最近 uint32 tick × 100000 ns，仅作为元数据，无值、重复、零值和回绕不影响配对，也不表示共同采样时刻。

pending 与 published 两份 `AHRSData` 隔离，只有完整发布才替换 published；同步回调与 getter 均取得 published。四元数在姿态和重力计算前检查 `norm_sq` 有限且大于 0.25，失败计入既有 `error_frames`，清除 pending、字段接收时间和两路 fresh，保留已有快照与 ready。合法长度的接收帧继续计入 `total_frames`；仅有效完整发布增加 `ahrs_frames`。不归一化或过滤四元数。

已核实 feed、同步回调和打印 getter 在 CAN 接收线程中执行；策略线程经现有 `SpscLatestChannel` 获取快照副本。无需增加锁。用户原有 reader 缩进与 parser 打印格式修改均保留。

接收错误仍使用 reader 的既有停止路径，最后有效快照由已有年龄守卫接管；不增加立即停电机通道。REALTIME 到 MONOTONIC 的转换仍有有限采样精度，本次验收只证明队列滞留时间没有被读取时刻覆盖。

## 验证

IMU 使用独立 Debug 构建目录 `/tmp/robot-deploy-imu-debug`，无硬件 CTest 4/4 通过。inference 全量构建（含 `policy_test`）和全部 CTest 18/18 通过，包含策略时序、线程、通道及内嵌 IMU 回归。实际 parser 用旧 Q 与旧 Rate 形成快照，停止输入，在 100 ms 后检查；电机时间为当前时间，只在该测试中放宽 skew 为 200 ms，默认 50 ms IMU 年龄守卫拒绝。已有 50 ms 与 50 ms + 1 ns 边界保持。`git diff --check` 通过。

自动回归覆盖 NEW ancillary 提取与长度／缺失／截断，current offset 实际转换，正负 clock step、1 ms 边界、读钟失败、不可信采样、转换溢出／非正／过晚、映射错误锁存和显式重开，以及 `EINTR` 消息重置。系统调用通过仅用于测试目标的链接包装模拟，不修改主机墙钟或增加公开注入接口。

parser 回归覆盖字段两种顺序、连续同类覆盖、旧字段年龄、回调与 getter 的完整快照、半更新和非法 Q 对未读快照的保护、全零／低范数拒绝及恢复、reset、无 SampleTime、复用／零／回绕／重复元数据。实测日志第 40–48 行的交错 Q/Rate/SampleTime 数据作为回归输入，不增加 SampleTime 发布门槛。

真实 can0 被动测试未初始化或使能电机，未配置接口或发送数据。接口维持 250 kbps、UP、ERROR-ACTIVE；验收前 RX/TX 错误、丢弃、bus errors 均为 0，TX 为 0。确认可读后停读 100 ms，两次测试首帧转换后的主机年龄分别为 **100.112 ms** 和 **100.115 ms**，满足至少 95 ms。

最终构建连续接收 **5.00035 s**、**11834 帧**，Q **400.172 Hz**，Rate **395.972 Hz**，AHRS **395.972 Hz**；无时间映射错误，parser errors 为 0。平均成功接收调用耗时 **2.866 μs**，最长 **547.757 μs**。此前一轮 Q 为 399.990 Hz、Rate/AHRS 为 393.591 Hz，与历史抓流约 400 Hz Q / 393 Hz Rate 的量级一致；短时未观察到新增节流或持续映射错误，此结果不替代长时间运行验证。

积压验收只判断首帧时间，没有要求 live drain 后最终 AHRS 被守卫拒绝。持续输入可能让最终快照变新，这是合理行为；旧快照拒绝由受控单元测试独立验证。真实设备测试目标不注册普通 CTest，也不引入 vcan 配置依赖。

## 复现命令

```bash
cmake -S src/imu -B /tmp/robot-deploy-imu-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/robot-deploy-imu-debug -j4
ctest --test-dir /tmp/robot-deploy-imu-debug --output-on-failure

cmake --build src/inference/build -j4
ctest --test-dir src/inference/build --output-on-failure

# 需要 can0 已启动并持续输入；只被动接收。
/tmp/robot-deploy-imu-debug/socket_can_timestamp_integration_test --interface can0
```

实现见 [SocketCAN 接收](../../src/imu/src/drivers/socket_can_port.cpp)、[内部时间辅助函数](../../src/imu/src/drivers/socket_can_timestamp.hpp)、[Xsens parser](../../src/imu/src/protocol/xsens_mti/can_parser.cpp)。使用约定见 [IMU README](../../src/imu/README.md)。
