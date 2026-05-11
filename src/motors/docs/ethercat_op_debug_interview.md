# EtherCAT OP 排障面试材料

## 一分钟项目讲述稿

在 MYACTUA EtherCAT 电机控制项目里，我遇到过一个典型的现场问题：12 个电机从站中只有 9 个能进入 OP，另外 3 个一直超时。最开始现象很像拓扑或接线问题，但我没有直接改线，而是先看程序等待条件，确认 `9/12` 表示 IGH slave config 中只有 9 个从站 operational。

接着我打开应用层诊断，定位到失败的是逻辑电机 `M0/M6/M7`，再结合代码中的 `kSlavePositions` 映射，把它们对应到 EtherCAT 物理位置 `1/8/9`。之后看内核 `dmesg`，三台从站都在 PREOP 到 SAFEOP 时返回 `AL status 0x0025: Invalid Output Mapping`。这说明不是应用层状态机问题，而是 PDO 输出映射不匹配。

最后我用 `ethercat slaves -v` 确认总线拓扑正确，15 个从站中 `0/7/14` 是 Junction，12 个电机正好在 `1-6` 和 `8-13`。再用 SDO 读取实际 PDO 映射，发现失败的三台不支持当前代码使用的 `0x1601`，只支持旧 `0x1600`，而成功电机支持 `0x1601` 和 `0x2000/0x2001`。所以根因是三台电机对象字典或固件版本不一致，不是物理拓扑问题。

## 三分钟技术深挖版

这个问题的关键是把“从站不进 OP”拆成 EtherCAT 状态机、过程数据、对象字典三个层次来看。

第一步，我先确认程序里“已就绪”的定义。`ID_test` 等待 12 个从站进入 OP，它并不是通过 DS402 状态字判断电机是否 enable，而是调用 `adapter->isConfigured(i)`。继续追到 `EthercatAdapterIGH` 后发现，这个标志来自 `ecrt_slave_config_state`，条件是 `online && operational`。所以 `9/12` 代表的是 EtherCAT 从站配置层面只有 9 台进入 OP，而不是电机还没使能。

第二步，我打开 `MYACTUA_ECAT_DIAG`，让程序打印每个逻辑电机的状态。稳定后发现只有 `M0/M6/M7` 的 `cfg=0`，且状态字一直是 `0x0000`。Domain working counter 稳定在 `27/36`，和 9 台成功电机对应。这样就从“有 3 台失败”缩小到具体逻辑索引。

第三步，我用代码里的物理映射表：

```cpp
kSlavePositions = {1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13};
```

把失败逻辑电机换算成总线 position：

```text
M0 -> 1
M6 -> 8
M7 -> 9
```

第四步看 `dmesg`，三台对应的日志都是：

```text
Failed to set SAFEOP state, slave refused state change (PREOP + ERROR)
AL status message 0x0025: "Invalid Output Mapping"
```

这一步很关键，因为它把问题从“物理链路”转成“PDO 输出映射”。EtherCAT 从 PREOP 到 SAFEOP 会检查 Sync Manager、FMMU 和 PDO 映射，如果输出映射与从站对象字典不匹配，从站就会拒绝进入 SAFEOP。

第五步，我用 `ethercat slaves -v` 证明拓扑没问题。总线有 15 个从站，`0/7/14` 是 Junction，`1-6`、`8-13` 是 12 个电机，和代码跳过 Junction 的设计一致。

最后一步是读实际对象字典。成功从站 position 2 支持：

```text
0x1601:00 = 8
0x1601:08 = 0x5ffe0008
```

失败从站 position 1/8/9 读取 `0x1601` 返回：

```text
object does not exist in the object directory
```

但它们支持旧映射：

```text
0x1600:00 = 7
0x1600:05 = 0x60720010
0x1600:06 = 0x60600008
0x1600:07 = 0x5ffe0008
```

当前代码统一配置 `0x1601`，并要求 `0x2000 PVT_KP` 和 `0x2001 PVT_KD`。失败的三台没有这套 `0x1601` 映射，所以报 `Invalid Output Mapping`。最终结论是：现场混用了新旧对象字典版本的电机，物理拓扑正确，但 PDO 版本不一致。

## STAR 版本

**Situation：**  
在 EtherCAT 多电机控制项目中，运行 `ID_test` 时 12 个电机从站只有 9 个进入 OP，程序 30 秒超时，现场确认物理拓扑正确。

**Task：**  
需要定位 3 个从站无法进入 OP 的真正原因，区分是拓扑、主站配置、PDO 映射还是应用层状态机问题，并给出可执行修复方向。

**Action：**  
我先追踪程序就绪条件，确认 `9/12` 来自 IGH 的 `online && operational`。然后打开 `MYACTUA_ECAT_DIAG`，定位失败逻辑电机为 `M0/M6/M7`。结合 `kSlavePositions` 映射，将其对应到物理 position `1/8/9`。随后读取 `dmesg`，发现三台均返回 `AL status 0x0025: Invalid Output Mapping`。最后使用 `ethercat slaves -v` 和 `ethercat upload` 验证总线拓扑正确，并确认失败从站不支持 `0x1601`，只支持旧 `0x1600` 映射。

**Result：**  
将问题从“3 台从站不进 OP”精确定位为“position 1/8/9 三台电机对象字典版本旧，不支持当前代码统一配置的 `0x1601` PDO”。修复方向明确为统一固件/对象字典版本，或在软件中实现新旧 PDO 混合兼容。

## 面试官高频追问

### 为什么一开始不能直接判断是物理拓扑问题？

因为总线仍然能稳定识别 15 个从站，而且 9 个电机可以进入 OP。如果是链路断开、Junction 接错或物理顺序大范围错误，通常会表现为从站数量不对、后续从站不可见、链路端口异常，或者 position 映射整体错位。本次问题是固定的 `1/8/9` 三台拒绝 SAFEOP，更像配置或对象字典差异。

### 为什么 `9/12` 可以对应到具体 3 台？

`ID_test` 的 ready count 来自 `adapter->isConfigured(i)`，而 `isConfigured` 来自每个 slave config 的 `online && operational`。打开 `MYACTUA_ECAT_DIAG` 后，每个逻辑电机都会打印 `cfg`，所以可以直接看到 `M0/M6/M7` 是 `cfg=0`，其它 9 台是 `cfg=1`。

### 为什么需要看 `dmesg`？

应用层只能看到“没有进入 OP”，但看不到从站拒绝状态切换的 AL 状态码。IGH master 在内核日志里会打印从 PREOP、SAFEOP、OP 切换失败的原因。本次 `dmesg` 给出的 `AL status 0x0025: Invalid Output Mapping` 是定位方向转折点，说明问题在输出 PDO 映射，而不是应用层使能流程。

### `AL status 0x0025` 说明什么？

它表示 Invalid Output Mapping，即主站下发或声明的输出 PDO 映射与从站实际允许的输出映射不一致。从站在 PREOP 切 SAFEOP 时会校验 Sync Manager 和 PDO 配置，校验失败就拒绝进入 SAFEOP，因此也不可能进入 OP。

### 为什么说物理拓扑是正确的？

`ethercat slaves -v` 枚举到 15 个从站：

```text
0/7/14: Junction
1-6, 8-13: MT_Device 电机
```

代码里 `kSlavePositions` 正好跳过 `0/7/14`，只配置 `1-6` 和 `8-13` 的 12 个电机。因此拓扑结构和代码映射一致。

### 如何证明不是 padding 对象 `0x5FFE` / `0x2FFD` 的问题？

排查中确实看到 padding 对象相关 warning，但进一步读 SDO 后发现更根本的问题：失败从站没有 `0x1601` 这个 PDO mapping object。也就是说，即使处理了 padding 名称差异，只要仍然统一配置 `0x1601`，这三台旧对象字典从站仍会失败。因此 padding 不是最终根因。

### 成功电机和失败电机的对象字典有什么差别？

成功电机支持新映射：

```text
0x1601:00 = 8
0x1601:08 = 0x5ffe0008
```

失败电机不支持 `0x1601`：

```text
SDO abort: object does not exist
```

但失败电机支持旧映射：

```text
0x1600:00 = 7
0x1600:05 = 0x60720010
0x1600:06 = 0x60600008
0x1600:07 = 0x5ffe0008
```

这说明失败电机用的是旧 PDO，其中包含 `0x6072 Max Torque`，没有当前代码要求的 `0x2000 PVT_KP` 和 `0x2001 PVT_KD`。

### 为什么当前代码会触发这个问题？

当前 `EthercatAdapterIGH` 对所有 12 台电机统一使用 `RxPDO 0x1601`，并在 PDO 中声明：

```text
0x6040, 0x607A, 0x60FF, 0x6071,
0x2000, 0x2001, 0x6060, padding
```

这对新对象字典电机是有效的，但对旧对象字典电机无效。旧电机没有 `0x1601`，只支持 `0x1600`，所以拒绝进入 SAFEOP。

### 后续如何修复？

推荐方案是统一硬件固件或对象字典版本，让 12 台电机都支持同一套 `0x1601` PDO。这是长期维护成本最低的方案。

如果现场短期无法升级，则软件需要做混合 PDO 兼容：

```text
position 1/8/9 使用旧 0x1600 映射
其它 position 使用新 0x1601 映射
```

同时控制层要识别旧 PDO 中没有 `0x2000/0x2001`，不能继续按新 PVT 参数写入，而要按旧映射中存在的 `0x6072` 等对象处理。

### 这个排查体现了什么工程能力？

它体现了三个能力：

- 分层定位：从应用 ready count 追到 EtherCAT AL 状态，再追到对象字典。
- 证据闭环：用诊断日志定位逻辑电机，用映射表定位物理 position，用 `dmesg` 定位 AL 错误，用 SDO 读取证明 PDO 版本差异。
- 风险控制：没有把试验性的 padding 改动当成最终修复，而是继续验证到对象字典层面，避免误修。

## 可量化亮点

可以在简历或面试中这样表述：

```text
定位并复盘一次 EtherCAT 多从站 OP 切换失败问题：
将“12 台电机仅 9 台进入 OP”的笼统现象，
逐步收敛为 M0/M6/M7 -> position 1/8/9 -> AL 0x0025 -> 0x1601 不存在、仅支持旧 0x1600 映射，
最终确认根因是现场混用新旧对象字典版本，而非物理拓扑错误。
```

## 简历压缩版

```text
负责 EtherCAT 多电机控制链路排障，针对 12 台从站中 3 台无法进入 OP 的问题，
通过应用诊断、IGH 内核日志、从站拓扑枚举和 SDO 对象字典读取，
将问题定位到 position 1/8/9 三台电机 PDO 版本不一致：
失败从站不支持当前主站配置的 0x1601，仅支持旧 0x1600，
并给出固件统一与混合 PDO 兼容两类修复方案。
```

