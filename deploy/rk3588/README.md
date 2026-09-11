# RK3588 实时 CPU 布局

该配置只为 `policy_test` 的固定硬件 profile 服务。通用库、`motors_test`、
`imu_only_test` 和无硬件测试仍继承默认调度与亲和性。

## 布局

| CPU | 用途 |
| --- | --- |
| 0–1 | Xbox、状态、诊断、事件、监控及 recorder 后台线程 |
| 2 | `imu_can_rx` / `imu_serial_rx` 与 CAN0 IRQ |
| 3 | eth0/EtherCAT IRQ、NAPI/NET_RX/IgH 路径的阶段一目标 |
| 4–5 | `policy_main`、ATen/OpenMP 推理线程 |
| 6 | `policy_cmd`, `SCHED_FIFO 70` |
| 7 | `ecat_rt`, `SCHED_FIFO 80` |

CPU3 不做启动级隔离。FIFO80 与 FIFO70 是各自固定 CPU 上的本地调度参数，
不表示跨 CPU 的全局串行优先关系。脚本不调用 `chrt`，不修改 IRQ、NAPI、
IgH 或其他内核线程的调度策略/优先级。

## 安装与验收

在板上执行：

```bash
sudo ./deploy/rk3588/robot-rt-setup.sh install
sudo reboot
sudo /usr/local/sbin/robot-rt-setup check
```

`install` 会：

- 将活动 uEnv 中旧的 `isolcpus`、`rcu_nocbs`、`irqaffinity`、`nohz_full`
  替换为 `isolcpus=domain,managed_irq,6-7 rcu_nocbs=6-7 irqaffinity=0-5`；
- 用 eth0 当前 MAC 修正 `ec_master main_devices`；
- 安装并启用 oneshot 服务；
- 配置 irqbalance 排除 CPU6–7。

首次修改会保留 `.pre-robot-rt` 备份。脚本不会自动重启。重启后的 oneshot
只在 workqueue、三个 cpufreq policy、动态发现的 CAN/eth IRQ、EtherCAT 设备
和所有回读均正确时，生成与本次 boot ID 绑定的
`/run/robot-rt-layout.ready`。`policy_test` 会在硬件初始化前复查同样条件。

若 `/dev/EtherCAT0` 未出现，先检查：

```bash
cat /sys/class/net/eth0/address
cat /etc/modprobe.d/ethercat.conf
dmesg | grep -Ei 'ethercat|ec_master|ec_stmmac'
```

## 路径测量

阶段一不主动切换 threaded NAPI。先保存线程和中断快照：

```bash
ps -eLo pid,tid,cls,rtprio,psr,comm,cmd | \
  grep -E 'policy_|ecat_rt|EtherCAT|ec_|irq/|napi/|ksoftirqd/3'
grep -E 'CPU|can0|eth0|stmmac|EtherCAT' /proc/interrupts
grep NET_RX /proc/softirqs
```

有 `trace-cmd` 时，在空载及 CPU0–5 压力负载下分别记录至少 30 分钟。先用
IRQ、NAPI、softirq 与调度 tracepoint 确认真实 execution context：

```bash
sudo trace-cmd record -o rk3588-ecat.dat \
  -e irq:irq_handler_entry -e irq:irq_handler_exit \
  -e napi:napi_poll -e irq:softirq_entry -e irq:softirq_exit \
  -e sched:sched_wakeup -e sched:sched_switch \
  sleep 1800
```

同时记录应用的 deadline-miss/通信看门狗事件。确认 CPU3 的实际竞争和调度等待
后，再决定是否约束 IgH `run_on_cpu`、隔离 CPU3 或调整内核线程优先级。
`/sys/module/ec_master/parameters/run_on_cpu` 不存在时，不得假定当前 IgH 构建
支持该能力。
