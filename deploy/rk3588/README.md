# RK3588 实时 CPU 布局

该配置只为 `policy_test` 的固定硬件 profile 服务。通用库、`motors_test`、
`imu_only_test` 和无硬件测试仍继承默认调度与亲和性。

## 布局

| CPU | 用途 |
| --- | --- |
| 0–1 | Xbox、状态、诊断、事件、监控及 recorder 后台线程 |
| 2 | `imu_can_rx` / `imu_serial_rx` 与 CAN0 IRQ |
| 3 | 普通内核任务和未显式固定的 IRQ；为网络路径保留余量 |
| 4–5 | `policy_main`、ATen/OpenMP 推理线程 |
| 6 | `policy_cmd`, `SCHED_FIFO 70` |
| 7 | `ecat_rt`, `SCHED_FIFO 80` |

CPU4–7 由启动参数隔离，普通系统负载限制在 CPU0–3。FIFO80 与 FIFO70 是各自固定 CPU 上的本地调度参数，
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
  替换为 `isolcpus=domain,managed_irq,4-7 rcu_nocbs=4-7 irqaffinity=0-3`；
- 通过 `/sys/class/net/*/device -> fe1c0000.ethernet` 找到 EtherCAT 物理
  GMAC，拒绝临时分配的 MAC，不依赖易变的 `eth0`/`eth1` 名称；
- 用该物理 GMAC 的永久 MAC 修正 `ec_master main_devices`；
- 写入 `99-ethercat-unmanaged.conf`，让 NetworkManager 按 MAC 永久忽略
  EtherCAT 网口；
- 只在 robot-rt ready 检查中验证 unmanaged 配置，不通过实时布局检查阻止
  NetworkManager 启动，保留管理网口和 Wi-Fi 的恢复能力；
- 安装并启用 oneshot 服务；
- 将 unbound workqueue 限制到 CPU0–3（掩码 `0f`）；
- 配置 irqbalance 排除 CPU4–7（CPU list `4-7`、掩码 `000000f0`）。

首次修改会保留 `.pre-robot-rt` 备份。脚本不会重启 NetworkManager，也不会
自动重启系统；因此应在安装命令成功返回后安排受控重启。重启后的 oneshot
只在 workqueue、三个 cpufreq policy、CAN IRQ、EtherCAT 专用驱动及设备
和所有回读均正确时，生成与本次 boot ID 绑定的
`/run/robot-rt-layout.ready`。`policy_test` 会在硬件初始化前复查同样条件。

`install` 会在写入 NetworkManager 配置后立即执行只读配置校验；重启后
使用 `robot-rt-setup check` 验收完整实时布局。

## 策略启动与内存锁定

`policy_test` 在任何 ATen/OpenMP 设置、模型加载和 dry-run 之前，将
`policy_main` 永久绑定到 CPU4–5 并设为 `SCHED_OTHER/0`。模型只执行现有的
一次 dry-run；该步骤既验证输出，也触发固定模型图所需的 Torch worker 创建。
随后程序枚举新增 TID，要求它们的 affinity 是 `{4,5}` 的子集且调度策略为
`SCHED_OTHER/0`。任一校验失败都会在 recorder、EtherCAT、电机和 IMU 启动前退出。

worker 校验后程序执行 `mlockall(MCL_CURRENT | MCL_FUTURE | MCL_ONFAULT)`。
`MCL_ONFAULT` 锁住当前驻留页，其余页面在首次触及时锁住；新映射不因锁定而
一次性填充整个默认线程栈，避免拉长 EtherCAT 启动收发交接空档并引发失同步。
RK3588 profile 采用 fail-closed，因此运行账号必须具备足够的 `RLIMIT_MEMLOCK`
或 `CAP_IPC_LOCK`，内核须支持 `MCL_ONFAULT`（Linux 4.4 起）。
锁定成功后，`policy_main`、`policy_cmd`、`ecat_rt` 分别预触碰
128 KiB 栈，IMU reader 预触碰 64 KiB；新线程会在切换到 FIFO 之前完成此步骤。
进程正常 shutdown 不主动 `munlockall()`。

未触及且尚未驻留的页面首次访问仍可能缺页，按需锁页不等价于所有页面预先驻留。
内存锁定只保证已锁页面不被换出，并不能消除 `malloc`、`mmap`、allocator
bookkeeping 或 lazy operator initialization 的运行时开销。应用也不能直接预触碰
Torch 内部 worker 的完整栈；单次 dry-run 未覆盖的数据相关分支仍可能有首次分配
抖动。本 profile 不增加多轮模型预热，也不提升 Torch worker 的实时优先级。

若 `/dev/EtherCAT0` 未出现，先检查物理映射、NetworkManager 排除规则和主站
MAC。下面的命令不假定 EtherCAT 一定叫 `eth0`：

```bash
for netdev in /sys/class/net/*; do
  printf '%s -> %s\n' "${netdev##*/}" "$(readlink -f "${netdev}/device")"
done
cat /etc/NetworkManager/conf.d/99-ethercat-unmanaged.conf
cat /etc/modprobe.d/ethercat.conf
cat /sys/module/ec_master/parameters/main_devices
dmesg | grep -Ei 'ethercat|ec_master|ec_stmmac'
```

若 unmanaged 配置不匹配，应修复上述文件并重新运行 `robot-rt-setup.sh install`。
失败时 robot-rt 不会 ready，但 NetworkManager 独立运行。

## B1 策略结果准入与目标保持

策略在 inference 和 action processing 完成后，紧邻准入判断采样
`decision_now`。最老的电机/IMU 主机观测年龄大于 40 ms 时丢弃本轮结果，
不发布目标、不续期、不提交该结果的 raw action；策略步和 gait phase 继续推进。
40 ms 是固定的初始测试门限。电机和 IMU 时间戳均须有效，原传感器年龄和
时间差保护保持生效。

准入后才组装固定大小日志。发布前单独采样 `published_at_ns`，目标最多
hold 60 ms；重复下发命令不能延长该期限。首帧前从第一次正式推理开始计
60 ms，复位命令也受其封顶。底层命令继续受独立 10 ms 截止期和目标截止期
共同限制。新目标须及时送达底层，发布本身不保证避开旧命令的到期锁存。

`policy_seq` 每轮正式推理递增，包括 drop；`target_seq` 仅有效发布时连续
递增，drop 行为 0。CSV 追加 `target_seq`、`obs_to_action_age_us`、
`target_hold_age_us`、`policy_result_dropped`。两项年龄在 admission 时采样，
hold age 表示更新前距上次有效发布时间的间隔；首帧前距首次推理起点计时。
事后日志处理不重算年龄或改变准入结论。CSV 不是连续 watchdog 采样，
超时应结合错误文本中的实时 hold age 判断。

## 路径测量

阶段一不主动切换 threaded NAPI。先保存线程和中断快照：

```bash
ECAT_NETDEV="$(for n in /sys/class/net/*; do
  [ "$(basename "$(readlink -f "$n/device")")" = fe1c0000.ethernet ] && basename "$n"
done)"
ps -eLo pid,tid,cls,rtprio,psr,comm,cmd | \
  grep -E 'policy_|ecat_rt|EtherCAT|ec_|irq/|napi/|ksoftirqd/3'
grep -E "CPU|can0|${ECAT_NETDEV}|stmmac|EtherCAT" /proc/interrupts
grep NET_RX /proc/softirqs
```

有 `trace-cmd` 时，在空载及 CPU0–3 压力负载下分别记录至少 30 分钟。先用
IRQ、NAPI、softirq 与调度 tracepoint 确认真实 execution context：

```bash
sudo trace-cmd record -o rk3588-ecat.dat \
  -e irq:irq_handler_entry -e irq:irq_handler_exit \
  -e napi:napi_poll -e irq:softirq_entry -e irq:softirq_exit \
  -e sched:sched_wakeup -e sched:sched_switch \
  sleep 1800
```

同时记录应用的 deadline-miss/通信看门狗事件，并用下面的命令逐 TID 验证运行时布局：

```bash
ps -eLo pid,tid,psr,cls,rtprio,comm
taskset -pc <tid>
```

要求 `policy_main` 和新增 Torch workers 只使用 CPU4–5，`policy_cmd` 只使用
CPU6/FIFO70，`ecat_rt` 只使用 CPU7/FIFO80。进入正式控制循环后还应持续观察这些
线程的 major/minor fault。确认 CPU3 的实际竞争和调度等待
后，再决定是否约束 IgH `run_on_cpu`、隔离 CPU3 或调整内核线程优先级。
`/sys/module/ec_master/parameters/run_on_cpu` 不存在时，不得假定当前 IgH 构建
支持该能力。

最终板端验收需要至少 6 小时策略运行：零 freshness latch、零策略截止期超时、
最大推理时间小于 30 ms。正式循环中上述三个应用线程的 major fault 保持 0，
稳态 minor fault 不持续增长；再人工阻塞策略线程超过 60 ms，确认现有 `reason=1`
安全停止仍生效。20 ms 策略周期和 60/10 ms 超时数值保持不变；60 ms 改为
有效目标发布后的 hold 上限，准入采用上述固定 40 ms 测试门限。验收还须确认
一次 stale drop 后及时恢复可继续运行、持续无有效 target 时仍停机。
