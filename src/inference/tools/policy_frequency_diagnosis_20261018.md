> 历史记录：本文测量的是迁移前的 LibTorch 版本，文中的环境变量、`.pt` 路径和 benchmark 命令不适用于当前 ORT CPU 部署。当前构建与验证见 [inference README](../README.md)。

# policy_test 约 42 Hz 的定位结果（2026-10-18）

## 实机复测更新：PASSIVE 未通过通信稳定性验证

用户确认 `1018_1744.csv` 使用 `OMP_WAIT_POLICY=PASSIVE`，因通信故障退出；
普通启动的 `1018_1745.csv` 没有出现同样的运行期通信问题。

| 实机日志 | 已记录帧数 | 记录跨度 | 实际频率 | 平均推理耗时 | 结果 |
|---|---:|---:|---:|---:|---|
| `1018_1744.csv`，PASSIVE | 141 | 约 2.8 s | 49.999 Hz | 1.829 ms | 通信故障退出 |
| `1018_1745.csv`，普通启动 | 2092 | 约 49.9 s | 41.906 Hz | 23.825 ms | 用户反馈通信正常 |

两次 PASSIVE 启动对应的 `1018_1739.csv`、`1018_1740.csv` 没有策略帧，
故障发生在接入策略前。内核日志在故障运行中记录了 WKC 从 36/36 下降到 0/36、
响应从站从 15 降到 11，以及释放主站前的同步错误；1745 对应运行在达到稳定
36/36 后、释放主站前没有记录 WKC 下降。释放主站后的同步错误不计作运行期故障。

**撤回将 PASSIVE 直接用于实机的建议。当前先使用下面的普通启动命令。**
离线基准只验证了推理加速，不能证明与硬件控制链路同时运行的稳定性。
当前可确认两种启动方式存在实机结果差异，尚不能确认 PASSIVE 通过哪个具体
环节触发通信异常，也不能据此把问题单独归因于线缆或供电。进一步定位需要
同步记录 EtherCAT 周期、唤醒延迟、收发耗时及从站状态。

```bash
sudo ./src/inference/build/policy_test ./src/inference/config/deploy.yaml
```

下面保留离线定位数据及曾测试的环境设置作为证据。

## 离线推理耗时定位

当前 oneDNN/ACL 计算路径中的 OpenMP 忙等待，与隔离核心上的线程集中运行叠加，
使推理调用持续耗时约 24 ms。离线基准中，启动前设置 `OMP_WAIT_POLICY=PASSIVE` 可在
保持同一模型、同一后端、同样 2 个 ATen 线程的条件下，将离线推理降到约 1.8 ms，
并按 20 ms 周期运行。实机通信稳定性复测结果见上文。

## 历史实机对照命令（已出现通信故障，不作为运行建议）

本次失败对照使用以下命令，会执行完整电机使能、复位及策略流程：

```bash
sudo env OMP_WAIT_POLICY=PASSIVE ./src/inference/build/policy_test \
  ./src/inference/config/deploy.yaml
```

这项环境设置无需重新编译。变量在进程启动前设置，使 OpenMP 库初始化时读到。
当前推理代码没有覆盖 `OMP_WAIT_POLICY`。本次未把环境调整写入生产启动流程。
无需改动模型、训练步长或 EtherCAT 控制周期。

## 证据

实机日志 `src/inference/log/1018_1658.csv`：3894 帧、93.083 秒，实际
41.823 Hz；平均推理耗时 23.871 ms，推理结束到下次开始平均仅 0.039 ms。

新增的 `policy_benchmark` 直接复用 `PolicyRuntime::load/infer`，加载同一个
deploy.yaml 与 policy.pt，采用相同的 CPU4/5、SCHED_OTHER、OpenBLAS=1、
interop=1。它不会初始化 RobotInterface、IMU 或电机。基准采用有限非零输入，
先预热 20 次，再统计下列样本；每个线程方案使用独立进程。

| 配置 | 样本数 | 平均推理 ms | P99 ms | 最大 ms | 运行频率 Hz |
|---|---:|---:|---:|---:|---:|
| 原配置：2 线程、oneDNN、连续执行 | 500 | 23.844 | 24.784 | 24.951 | 41.931 |
| 原配置，按原主循环逻辑尝试 20 ms 周期 | 100 | 23.860 | 24.731 | 25.122 | 41.903 |
| 只减少 ATen 为 1 线程 | 500 | 17.362 | 18.876 | 18.974 | 57.582 |
| 2 线程，显式分配主线程 CPU4、工作线程 CPU5 | 300 | 15.718 | 16.878 | 17.068 | 63.603 |
| 原配置，仅启动前设 OMP_WAIT_POLICY=PASSIVE | 200 | 1.708 | 1.828 | 1.875 | 584.110 |
| 原配置，仅启动前设 GOMP_SPINCOUNT=0 | 200 | 1.721 | 1.981 | 2.068 | 579.829 |
| 关闭 oneDNN，1 线程，连续执行 | 300 | 0.483 | 0.511 | 0.644 | 2061.283 |
| **PASSIVE、原后端/2 线程，20 ms 周期** | **1000** | **1.777** | **1.989** | **2.356** | **49.99985** |

最后一项持续 20.000060 秒，1000 次推理中没有一次超过 20 ms。
这里的 50 Hz 是带定时等待的实际执行频率；连续执行的 Hz 是吞吐量，不代表建议控制频率。

## 根因链路

1. 当前内核参数包含 `isolcpus=domain,managed_irq,4-7`，而策略线程及后端工作线程
   只设置了允许集合 `{4,5}`。采样的线程最后运行核心均为 CPU4；原配置测量期间，
   进程 CPU 时间/墙钟时间约 0.995，实际只用到约一个核心。
   允许集合包含两个隔离核心，并不保证线程自动分散到两个核心。
2. oneDNN verbose 明确显示 `runtime:OpenMP,nthr:2` 和 `matmul,gemm:acl`，
   模型四个全连接层走 ARM Compute Library 路径，同时可见权重 reorder。
   测试中还观察到后端短生命周期线程。因此 ATen 线程数不等于进程总线程数。
3. 默认等待策略包含忙等待。线程集中在同一个隔离核心时，等待线程继续占用 CPU，
   延长其他线程推进工作的时间。相同计算路径下，只将等待策略改为 PASSIVE，
   平均耗时从 23.844 ms 降到 1.708 ms；GOMP_SPINCOUNT=0 独立对照也得到约 1.7 ms。
   这一对照确认忙等待是主要的可消除开销，显式分配核心和切换后端的实验提供补充证据。

内核文档说明 domain 隔离会移除通用负载均衡；GNU libgomp 文档说明默认会先忙等待，
PASSIVE 对应默认零自旋：

- [Linux 内核 CPU 隔离文档](https://cdn.kernel.org/doc/html/latest/admin-guide/cpu-isolation.html)
- [GNU libgomp GOMP_SPINCOUNT](https://gcc.gnu.org/onlinedocs/gcc-15.3.0/libgomp/GOMP_005fSPINCOUNT.html)

## 输出与环境校验

- 从 `1018_1658.csv` 每隔 120 帧抽取一组，共 32 组真实的 705 维观测、384 个动作值。
  使用同一个模型分别在默认等待和 PASSIVE 下运行，输出逐值完全相同，最大绝对差为 0。
- 关闭 oneDNN 的额外对照最大绝对差为 6.408e-7；它只用于定位，没有应用于生产配置。
- PyTorch 2.5.1 / oneDNN 3.5.3，RK3588 大核运行在 performance governor，读取频率
  2304 MHz，读取温度约 34–36°C。所用模型 SHA256：
  `34e4cb60baa8705100de5a95e2e29bf5be576b3b0bf3b31068cbc5c9a25d4824`。
- 下表基准由助手离线运行；用户后续的实机对照达到 50 Hz，但出现通信故障，
  因此没有通过完整硬件控制链路的验证。

原始记录位于项目根目录 `logs/policy_diagnosis_20261018/`，包括各组终端输出、
oneDNN verbose、输出一致性结果 `output_verification.json`、抽样输入和验证脚本。

## 离线复现

```bash
cmake -S src/inference -B src/inference/build
cmake --build src/inference/build --target policy_benchmark -j4

# 原配置，约 42 Hz；均不会使能电机
./src/inference/build/policy_benchmark --threads 2 --iterations 500

# 相同后端、相同线程数，验证 50 Hz 定时运行
OMP_WAIT_POLICY=PASSIVE ./src/inference/build/policy_benchmark \
  --threads 2 --iterations 1000 --period-ms 20
```

`--threads 1`、`--pin-workers`、`--no-mkldnn` 分别用于线程数、明确分配核心及
计算后端对照。这些参数仅作用于独立基准程序，不修改 policy_test 的运行配置。
