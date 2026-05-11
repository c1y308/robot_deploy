# EtherCAT OP 状态异常排障复盘

## 背景

在运行 `src/motors/build/ID_test` 时，程序会初始化 IGH EtherCAT 主站，然后等待 12 个电机从站进入 OP 状态。现场反馈是物理拓扑正确，但程序始终提示有 3 个从站无法进入 OP。

这次排查的目标不是直接修改控制逻辑，而是把问题从“3 个从站不进 OP”逐层收敛到可验证的根因。

## 现象

执行命令：

```bash
cd src/motors/build
sudo ./ID_test
```

终端输出的关键现象：

```text
[1/4] 正在初始化网卡...
[ECAT_DIAG] disabled, interval_cycles=500 (env: MYACTUA_ECAT_DIAG / MYACTUA_ECAT_DIAG_INTERVAL)
[2/4] 等待 12 个从站进入 OP...
  已就绪: 0/12
  ...
  已就绪: 9/12
[ready] 等待耗时 30000 ms
[错误] 从站未在 30s 内全部就绪，请检查接线/供电/物理位置映射。
```

初始判断有三个方向：

- 物理拓扑或 position 映射错误。
- EtherCAT 主站配置和从站实际对象字典不一致。
- 应用层等待逻辑误判了 OP 状态。

## 第一层排查：确认“已就绪”的含义

先看 `ID_test.cpp` 的等待逻辑。程序在 30 秒内循环调用：

```cpp
adapter->receivePhysical();
adapter->sendPhysical();

if (adapter->isConfigured(i)) {
    ++ready_count;
}
```

`isConfigured(i)` 在 `EthercatAdapterIGH` 中不是读 DS402 状态字，而是读 IGH 的 slave config state：

```cpp
const bool ok = sc_state[i].online && sc_state[i].operational;
slave_configured[i].store(ok, std::memory_order_relaxed);
```

因此，`9/12` 的含义是：12 个被配置的电机从站中，只有 9 个进入了 EtherCAT OP 状态，另外 3 个仍未 operational。这不是应用层模式切换失败，而是 EtherCAT AL 状态切换失败。

## 第二层排查：打开应用诊断，定位失败电机

代码里已经有诊断开关：

```bash
sudo MYACTUA_ECAT_DIAG=1 MYACTUA_ECAT_DIAG_INTERVAL=1000 ./ID_test
```

稳定后诊断输出显示：

```text
[ECAT_DIAG] cycle=26000 wc=27 wc_state=INCOMPLETE
  M0  status=0x0000 err=0x0000 op=0 cfg=0
  M1  status=0x1231 err=0x0000 op=0 cfg=1
  M2  status=0x1231 err=0x0000 op=0 cfg=1
  M3  status=0x1231 err=0x0000 op=0 cfg=1
  M4  status=0x1231 err=0x0000 op=0 cfg=1
  M5  status=0x1231 err=0x0000 op=0 cfg=1
  M6  status=0x0000 err=0x0000 op=0 cfg=0
  M7  status=0x0000 err=0x0000 op=0 cfg=0
  M8  status=0x1231 err=0x0000 op=0 cfg=1
  M9  status=0x1231 err=0x0000 op=0 cfg=1
  M10 status=0x1231 err=0x0000 op=0 cfg=1
  M11 status=0x1231 err=0x0000 op=0 cfg=1
```

由此可以把问题从“3 个从站”缩小为具体逻辑电机：

- `M0`
- `M6`
- `M7`

同时，Domain working counter 稳定在 `27/36`，说明只有 9 个电机从站参与了有效过程数据交换，剩余 3 个没有进入可交换 PDO 的 OP 状态。

## 第三层排查：映射逻辑索引到 EtherCAT 物理位置

`EthercatAdapterIGH.hpp` 中当前逻辑索引到物理 position 的映射是：

```cpp
inline constexpr std::array<uint16_t, 12> kSlavePositions = {
    1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13
};
```

所以失败逻辑电机对应的物理 position 是：

```text
M0 -> position 1
M6 -> position 8
M7 -> position 9
```

这一步把问题从应用逻辑索引落到了总线物理从站号。

## 第四层排查：读取内核日志，确认 AL 错误码

查看 IGH 主站内核日志：

```bash
sudo dmesg --ctime --since "2026-05-11 15:25:00"
```

关键日志：

```text
EtherCAT ERROR 0-1: Failed to set SAFEOP state, slave refused state change (PREOP + ERROR).
EtherCAT ERROR 0-1: AL status message 0x0025: "Invalid Output Mapping".

EtherCAT ERROR 0-8: Failed to set SAFEOP state, slave refused state change (PREOP + ERROR).
EtherCAT ERROR 0-8: AL status message 0x0025: "Invalid Output Mapping".

EtherCAT ERROR 0-9: Failed to set SAFEOP state, slave refused state change (PREOP + ERROR).
EtherCAT ERROR 0-9: AL status message 0x0025: "Invalid Output Mapping".
```

这一步确认：

- 三个失败从站确实是 `0-1`、`0-8`、`0-9`。
- 失败发生在 PREOP 切 SAFEOP 阶段。
- 从站拒绝状态切换的原因是 `Invalid Output Mapping`。

因此，问题重点从“拓扑/链路”转向“主站配置的输出 PDO 和从站实际支持的 PDO 不一致”。

## 第五层排查：确认物理拓扑是否正确

使用系统匹配当前内核模块版本的 `ethercat` 工具：

```bash
sudo /usr/bin/ethercat slaves -v
```

总线枚举结果为 15 个从站：

```text
Slave 0  : Junction
Slave 1  : MT_Device
Slave 2  : MT_Device
Slave 3  : MT_Device
Slave 4  : MT_Device
Slave 5  : MT_Device
Slave 6  : MT_Device
Slave 7  : Junction
Slave 8  : MT_Device
Slave 9  : MT_Device
Slave 10 : MT_Device
Slave 11 : MT_Device
Slave 12 : MT_Device
Slave 13 : MT_Device
Slave 14 : Junction
```

与代码映射一致：

- `0/7/14` 是 Junction/转发站。
- `1-6` 和 `8-13` 是 12 个电机。
- 代码跳过 Junction，只配置 12 个电机。

因此，物理拓扑和 position 映射不是主因。

## 第六层排查：读取实际 PDO 映射

当前代码对所有电机统一配置：

```cpp
RxPDO 0x1601:
  0x6040, 0x607A, 0x60FF, 0x6071,
  0x2000, 0x2001, 0x6060, padding

TxPDO 0x1A00:
  0x6041, 0x6064, 0x606C, 0x6077,
  0x603F, 0x6061, padding
```

关键是失败从站是否真的支持 `0x1601`。

读取成功从站 position 2：

```bash
sudo /usr/bin/ethercat upload -p 2 -t uint8 0x1601 0
sudo /usr/bin/ethercat upload -p 2 -t uint32 0x1601 8
```

结果：

```text
0x08 8
0x5ffe0008
```

说明成功从站支持 `0x1601`，并且 `0x1601` 有 8 个映射项。

读取失败从站 position 1：

```bash
sudo /usr/bin/ethercat upload -p 1 -t uint8 0x1601 0
sudo /usr/bin/ethercat upload -p 1 -t uint32 0x1601 8
```

结果：

```text
SDO transfer aborted with code 0x06020000: This object does not exist in the object directory
```

说明失败从站没有 `0x1601` 这个 PDO mapping object。

继续读取失败从站旧映射：

```bash
sudo /usr/bin/ethercat upload -p 1 -t uint8 0x1600 0
sudo /usr/bin/ethercat upload -p 1 -t uint32 0x1600 5
sudo /usr/bin/ethercat upload -p 1 -t uint32 0x1600 6
sudo /usr/bin/ethercat upload -p 1 -t uint32 0x1600 7
```

结果：

```text
0x07 7
0x60720010
0x60600008
0x5ffe0008
```

这说明失败从站支持的是旧 `0x1600` 映射，且第 5 项是 `0x6072:00/16`，不是当前代码需要的 `0x2000` 和 `0x2001`。

对 position 8 和 position 9 做同样读取，结果一致：它们也支持旧 `0x1600`，不支持当前代码统一使用的 `0x1601`。

## 关于 0x5FFE / 0x2FFD 的试验结论

排查过程中曾验证过 padding 对象差异，例如 `0x5FFE`、`0x2FFD`、`0x2FFE`。这个方向可以解释日志中关于 padding 的 warning，但不能解释三台失败电机没有 `0x1601` 的事实。

因此，不能把 `0x5FFE` 或 `0x2FFD/0x2FFE` 的替换当成最终修复。真正的根因是 PDO mapping object 和对象字典版本不一致。

## 根因

三台无法进入 OP 的电机从站是旧对象字典或旧固件版本：

```text
M0 -> position 1 -> 不支持 0x1601，只支持旧 0x1600
M6 -> position 8 -> 不支持 0x1601，只支持旧 0x1600
M7 -> position 9 -> 不支持 0x1601，只支持旧 0x1600
```

当前 `EthercatAdapterIGH` 对所有 12 台电机统一配置 `0x1601`，并要求过程数据中包含：

```text
0x2000 PVT_KP
0x2001 PVT_KD
```

但旧从站的 `0x1600` 中没有 `0x2000/0x2001`，而是包含：

```text
0x6072 Max Torque
```

因此，这三台电机在 PREOP 切 SAFEOP 时校验输出 PDO 失败，返回：

```text
AL status 0x0025: Invalid Output Mapping
```

## 结论

物理拓扑是正确的，代码中的电机 position 映射也是正确的。问题不是线序、供电或 Junction 拓扑，而是同一条总线上混用了两类电机对象字典：

- 新版本电机支持 `0x1601`，可进入 OP。
- 旧版本电机只支持 `0x1600`，当前统一配置下拒绝 SAFEOP/OP。

## 修复方向

优先推荐：

```text
统一三台失败电机的固件或对象字典版本，使 12 台电机都支持同一套 0x1601 PDO。
```

软件兼容方案：

```text
在 EthercatAdapterIGH 中做混合 PDO 配置：
- position 1/8/9 使用旧 0x1600 映射。
- 其它电机继续使用新 0x1601 映射。
- 写过程数据时，旧映射不能写 0x2000/0x2001，需要按旧 PDO 中的 0x6072 处理。
```

如果系统长期维护，推荐优先做固件/对象字典统一。混合 PDO 兼容可以作为短期过渡方案，但会增加控制层对不同电机版本的分支处理。

