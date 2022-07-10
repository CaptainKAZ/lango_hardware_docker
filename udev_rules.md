# 使用udev rule机制来根据gs_usb设备(CandoCandlelight等)的序列号指定名称

udev是Linux内核中主管设备变更(热插拔事件)的主要程序. 通过编写udev rules可以对满足条件的设备进行自定义的操作.我们需要通过udev rules实现以下功能:

1. 对USB设备进行免ROOT权限的访问
2. 对gs_can设备,根据不同的设备序列号进行识别 并且赋予他们独一无二的网络设备名称

udev rules编写指南可以参考 [英文原文](https://linuxconfig.org/tutorial-on-how-to-write-basic-udev-rules-in-linux#:~:text=Udev%20rules%20are%20defined%20into%20files%20with%20the,rules%2C%20%2Fetc%2Fudev%2Frules.d%2F%20is%20reserved%20for%20custom%20made%20rules.) [中文翻译](https://blog.csdn.net/rong11417/article/details/102881398)

用户定义的udev rules在`/etc/udev/rules.d/`文件夹下,按照rule的优先级(越小越靠前)生效

我们先在其中复制candleLight固件中的[原始rules文件](https://github.com/candle-usb/candleLight_fw/blob/master/70-candle-usb.rules)`70-candle-usb.rules`

```
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", MODE="660", GROUP="users", TAG+="uaccess"
```
这条rules的意思是在usb子系统中找到idVendor=1d50和idProduce=606f的硬件,赋予660的访问权限(chmod)然后修改用户组为users(不为root)以及添加用户组tag

我参考[这篇问答](https://unix.stackexchange.com/questions/204829/attributes-from-various-parent-devices-in-a-udev-rule)写的rules

```bash
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", MODE="660", GROUP="users", TAG+="uaccess", GOTO="CAN"
SUBSYSTEMS=="usb",GOTO="ENDCAN"

LABEL="CAN"
SUBSYSTEMS=="usb", ATTRS{serial}=="758A559A0E52",GOTO="LANGO_CAN0"
# 在这添加其他的CAN设备 示例：
# SUBSYSTEMS=="usb", ATTRS{serial}=="xxxxx",GOTO="LANGO_CAN1"
LABEL="LANGO_CAN0"
KERNEL=="can*", NAME="lango_can0",GOTO="ENDCAN"
# 在这添加LABEL和指定名称 示例：
# LABEL="LANGO_CAN1"
# KERNEL=="can*", NAME="lango_can1",GOTO="ENDCAN"
LABEL="ENDCAN"
```
就是加了一个条件跳转

至于序列号`ATTRS{serial}`怎么获取 有几种办法
一种是看'lsusb -v'下面iSerial后面的字符串
另外一种方法是看`udevadm info -ap /sys/class/net/can0`里面有详细的各个属性

要使得udev rules生效,需要运行以下指令
```bash
sudo service udev restart
sudo udevadm control --reload
```