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

然后重启wsl 再运行modprobe

然后再`ifconfig -a`应该可以看到usb2can的设备了

关于去除加号然后编译。我也找了[教程](https://blog.csdn.net/weixin_34421376/article/details/116871095#:~:text=kernel%E7%89%88%E6%9C%AC%E5%87%BA%E7%8E%B0%E4%B8%80%E4%B8%AA%E5%8A%A0%E5%8F%B7%20%28plug%20sign%29%E7%9A%84%E5%8E%9F%E5%9B%A0%E5%8F%AF%E8%83%BD%E6%98%AF%E5%A6%82%E4%B8%8B%E4%B8%A4%E7%82%B9%EF%BC%8C%E5%BD%93%E7%84%B6%E5%89%8D%E6%8F%90%E6%98%AF%E4%BD%BF%E7%94%A8Linux%E7%9A%84GIT,repository%EF%BC%8C%E4%B8%94CONFIG_LOCALVERSION_AUTO%E5%92%8CLOCALVERSION%E9%83%BD%E6%B2%A1%E6%9C%89%E8%AE%BE%E7%BD%AE%E3%80%82%20%281%29%E5%A6%82%E6%9E%9C%E5%BD%93%E5%89%8Drepository%E7%9A%84commit%20ID%E4%B8%8D%E6%98%AF%E6%9F%90%E4%B8%80%E4%B8%AAtag%EF%BC%8C%E5%88%99%E9%BB%98%E8%AE%A4%E6%9C%89%E4%B8%80%E4%B8%AA%E5%8A%A0%E5%8F%B7%E3%80%82)，但是并没有成功