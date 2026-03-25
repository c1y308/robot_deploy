# 脉塔电机控制-`ethercat`

## PDO结构

~~~
esheep@Helper:~$ sudo /home/esheep/SOEM/build/install/bin/slaveinfo enp12s0 -map
SOEM (Simple Open EtherCAT Master)
Slaveinfo
Starting slaveinfo
ecx_init on enp12s0 succeeded.
1 slaves found and configured.
Calculated workcounter 3

Slave:1
 Name:MT_Device
 Output size: 128bits
 Input size: 128bits
 State: 4
 Delay: 0[ns]
 Has DC: 1
 DCParentport:0
 Activeports:1.0.0.0
 Configured address: 1001
 Man: 00202008 ID: 00000000 Rev: 00010000
 SM0 A:1000 L: 128 F:00010026 Type:1
 SM1 A:1080 L: 128 F:00010022 Type:2
 SM2 A:1100 L:  16 F:00010064 Type:4
 SM3 A:1400 L:  16 F:00010020 Type:3
 FMMU0 Ls:00000000 Ll:  16 Lsb:0 Leb:7 Ps:1400 Psb:0 Ty:02 Act:01
 FMMU1 Ls:00000010 Ll:  16 Lsb:0 Leb:7 Ps:1100 Psb:0 Ty:01 Act:01
 FMMU2 Ls:00000020 Ll:   1 Lsb:0 Leb:7 Ps:080d Psb:0 Ty:01 Act:01
 FMMUfunc 0:1 1:2 2:3 3:0
 MBX length wr: 128 rd: 128 MBX protocols : 04
 CoE details: 27 FoE details: 00 EoE details: 00 SoE details: 00
 Ebus current: 0[mA]
 only LRD/LWR:0
PDO mapping according to CoE :
  SM2 inputs
     addr b   index: sub bitl data_type    name
  [0x0010.0] 0x6040:0x00 0x10 UNSIGNED16   control word
  [0x0012.0] 0x607A:0x00 0x20 INTEGER32    target position
  [0x0016.0] 0x60FF:0x00 0x20 INTEGER32    target velocity
  [0x001A.0] 0x6071:0x00 0x10 INTEGER16    Target Torque
  [0x001C.0] 0x6072:0x00 0x10 UNSIGNED16   Max Torque
  [0x001E.0] 0x6060:0x00 0x08 INTEGER8     modes of operation
  [0x001F.0] 0x5FFE:0x00 0x08
  SM3 outputs
     addr b   index: sub bitl data_type    name
  [0x0000.0] 0x6041:0x00 0x10 UNSIGNED16   status word
  [0x0002.0] 0x6064:0x00 0x20 INTEGER32    position actual value
  [0x0006.0] 0x606C:0x00 0x20 INTEGER32    velocity actual value
  [0x000A.0] 0x6077:0x00 0x10 INTEGER16    torque actual value
  [0x000C.0] 0x603F:0x00 0x10 UNSIGNED16   error code
  [0x000E.0] 0x6061:0x00 0x08 INTEGER8     modes of operation display
  [0x000F.0] 0x5FFE:0x00 0x08
End slaveinfo, close socket
End program
~~~



~~~
addr b   index: sub bitl data_type    name
~~~

它的真实含义是：

> **“某个对象（0x6040 等）在 PDO 内存中的位置、长度、类型、语义”**



例子（你表里的）：

```
[0x0010.0] 0x6040:0x00 0x10 UNSIGNED16 control word
```

拆解 `[0x0010.0]`

| 部分     | 含义                        |
| -------- | --------------------------- |
| `0x0010` | **字节偏移（byte offset）** |
| `.0`     | **bit 偏移（bit offset）**  |

 `[0x0010.0]` = **第 16 个字节，从 bit0 开始**





| 对象              | 含义            |
| ----------------- | --------------- |
| `0x6040`          | Control Word    |
| `0x6041`          | Status Word     |
| `0x6060 / 0x6061` | 运行模式 / 显示 |
| `0x607A`          | Target Position |
| `0x60FF`          | Target Velocity |
| `0x6071`          | Target Torque   |





### 输出PDO

~~~
Byte 0-1   : Control Word        (0x6040)
Byte 2-5   : Target Position    (0x607A)
Byte 6-9   : Target Velocity    (0x60FF)
Byte 10-11 : Target Torque      (0x6071)
Byte 12-13 : Max Torque         (0x6072)
Byte 14    : Mode of Operation  (0x6060)

~~~

### 输入PDO

~~~
Byte 0-1   : Status Word        (0x6041)
Byte 2-5   : Position Actual   (0x6064)
Byte 6-9   : Velocity Actual   (0x606C)
Byte 10-11 : Torque Actual     (0x6077)
Byte 12-13 : Error Code        (0x603F)
Byte 14    : Mode Display      (0x6061)

~~~

