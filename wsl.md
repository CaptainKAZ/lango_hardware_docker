# 给WSL2加入SocketCAN支持

微软编译的WSL Linux内核并没有加入SocketCAN以及USB2CAN的支持，所以我们还需要重新编译内核，然后生成ko文件的内核模块（补丁）整个过程还挺长的。

主要参考内容为WSL下的GitHub [Issue](https://github.com/microsoft/WSL/issues/5533)

```bash
git clone https://github.com/microsoft/WSL2-Linux-Kernel
cd WSL2-Linux-Kernel
git checkout `uname -r`
cat /proc/config.gz | gunzip > .config
make prepare modules_prepare -j$(nproc)
make menuconfig  # 在这里进入Network->CAN里面把里面的有关CAN的模块全部打开
# 或者直接复制我设置好的.config文件到编译目录下
make modules -j$(nproc)
sudo make modules_install
sudo modprobe can-raw
sudo modprobe can
sudo modprobe gs_usb
```

实测最后modprobe必出问题，报错为module notfound主要原因可能是内核版本的localversion后面多了个加号 原版版本是5.10.102.1-microsoft-standard-WSL2，我们编译的版本可能多了个加号5.10.102.1-microsoft-standard-WSL2+

[Issue](https://github.com/microsoft/WSL/issues/5533)里面有人说，直接一条路走到黑直接替换内核
```bash
make -j$(nproc)
sudo make install
cp vmlinux /mnt/c/Users/<yourwindowsloginname>/ #把内核复制出来
```

然后在`.wslconfig`（参照readme建立）下面加上自定义内核
```
kernel=C:\\Users\\<yourwindowsloginname>\\vmlinux
```