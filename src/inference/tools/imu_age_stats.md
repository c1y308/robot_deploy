# policy_test 的 IMU 数据年龄统计

在项目根目录执行，无需重新编译，也不启动电机：

```bash
# 默认分析 src/inference/log 下修改时间最新的 CSV
python3 src/inference/tools/imu_age_stats.py

# 指定一次运行；建议 policy_test 停止并完成日志写入后分析
python3 src/inference/tools/imu_age_stats.py src/inference/log/1017_1742.csv

# 无硬件验证
python3 src/inference/tests/imu_age_stats_test.py
```

终端输出样本数、Min/Mean/P50/P90/P95/P99/P99.9/Max（ms）、最大值所在帧及
CSV 行号、直方图。默认在输入 CSV 同目录的 `<文件名>.imu_age/` 下保存
`summary.json` 和 `histogram.csv`。也可以用 `--output-dir` 指定目录。
分位数采用 nearest rank：排序后取第 `ceil(N*p)` 个样本。

## 口径

`age_ns = inference_start_ns - imu_receive_timestamp_ns`

这是当前策略帧使用的 IMU 快照，从主机接收到模型推理开始之间的年龄。
当前 Xsens 驱动的接收时间来自 SocketCAN 内核 RX 时间戳，转换到主机
单调时钟；四元数与角速度组成快照时采用两者中较早的接收时间。
因此包含内核接收队列等待、解析发布、快照缓存等待和观测构建等时间。
这里统计的是本帧最新输入的年龄，历史堆叠观测中的旧帧不在此口径内。

设备 `imu_sample_timestamp_ns` 与主机时钟没有对齐，不能直接相减。
此结果不包含传感器内部处理及 CAN 帧到达主机之前的传输时间，不能当作
从物理采样到模型输入的完整延迟。历史日志的接收时间精度取决于当时使用的驱动版本。

## 覆盖范围

- 每个已记录策略帧贡献一个样本，包括 `policy_result_dropped=1` 的推理结果。
- 同一 IMU 快照被多次使用时不去重，否则会掩盖停更时年龄增长。
- 报告帧索引范围内的缺帧数，以及首帧之前未记录的帧数。
- 零时间戳、负年龄、截断行和重复策略帧会报错，不会静默丢弃。
- 当前 recorder 在推理结果产生后记录：传感器过期守卫、推理或控制线程故障
  若在记录前触发，该次尝试不在 CSV 中。即使帧索引连续，也不能证明最后一次
  故障尝试被保存。因此报告中的最大值严格限定为**已记录帧的最大年龄**。
- 若控制台出现日志队列／写盘错误，该日志不能作为完整运行的统计结论。
