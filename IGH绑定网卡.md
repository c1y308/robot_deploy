# `IGH`绑定网卡

停掉网卡上的协议栈

~~~
sudo ip link set enp12s0 down
sudo ip addr flush dev enp12s0
sudo ip link set enp12s0 up
sudo ethercatctl start
~~~

