# `IGH`绑定网卡

停掉网卡上的协议栈

~~~
sudo ip link set enp12s0 down
sudo ip addr flush dev enp12s0
sudo ip link set enp12s0 up
sudo ethercatctl start
~~~





rk3588_LubanCat

start_ethercat.sh

~~~
#!/bin/bash

# 基础目录
BASE_DIR="/home/cat/ethercat_board"
# 核心模块路径
MASTER_KO="${BASE_DIR}/ec_master.ko"
# 网卡驱动模块路径
GENERIC_KO="${BASE_DIR}/ec_generic.ko"
# 服务启动脚本
SERVICE_SCRIPT="${BASE_DIR}/output/etc/init.d/ethercat"

echo "--- 启动 EtherCAT 环境 ---"

sudo ifconfig eth1 down
echo "[1/3] 网卡 eth1 已关闭"

sudo insmod $MASTER_KO main_devices=82:94:09:28:de:ec
sleep 1

sudo insmod $GENERIC_KO
echo "[2/3] 内核驱动模块已加载"

sudo $SERVICE_SCRIPT start
sleep 1
sudo ifconfig eth1 up
echo "[3/3] EtherCAT 服务已启动"


echo "--- 环境准备就绪！ ---"
# 最终检查状态
sudo ethercat master
sudo ethercat slaves
~~~

