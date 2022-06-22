# 使用 Docker 在 Windows 11 WSLg 环境下部署 Lango 开发环境

## 准备

1. Windows 11 专业版操作系统 启用 Hyper-V 功能与 WSL 功能（控制面板-添加或删除Windows功能）
2. Windows Terminal（终端用的舒服）
3. Docker Desktop
4. Visual Studio Code

## 部署

### WSL环境准备
在 Windows Terminal 的 Powershell 中输入

```powershell
wsl --update #更新 WSL 内核
wsl --install -d ubuntu #安装 Ubuntu(20.04)
wsl # 启动 WSL 首次启动会要求输入密码等
```

WSL 连接 USB 参考[微软官方教程](https://docs.microsoft.com/en-us/windows/wsl/connect-usb)

PowerShell中
```powershell
winget install --interactive --exact dorssel.usbipd-win
usbipd wsl list #查看 Windows 下的设备
usbipd wsl attach --busid <busid> #按照设备 ID 连接
```
WSL bash 中
```bash
sudo apt install linux-tools-5.4.0-77-generic hwdata
sudo update-alternatives --install /usr/local/bin/usbip usbip /usr/lib/linux-tools/5.4.0-77-generic/usbip 20
```

连接完成之后可以在 WSL 中使用 `lsusb` 查看 USB 设备

## Docker环境准备

安装并启动 Docker Desktop 即可在 WSL 和 Windows 下直接使用 `docker` 命令

## Docker 构建
默认构建的是开发环境，包括一堆GUI工具 通过更改构建阶段为deploy可以仅包含狗上必要环境

```bash
sh build_docker.sh
```

有个坑就是WSL默认给的内存大小是 6GB 编译的时候不太够用 需要重新设置

在 Windows 用户目录 ( Win+R 运行-> %UserProfile% )下创建`.wslconfig`文件，输入以下内容

```
[wsl2]
memory=10GB
swap=8GB
```
个人测试勉强够用，浏览器多几个标签页还得关一下

## Docker 运行

在Docker里面调用WSLg可以参考[WSL官方文档](https://github.com/microsoft/wslg/blob/main/samples/container/Containers.md)
```bash
sh run_docker.sh #已经处理好 WSLg 的问题了（大概）
```

WSL中运行上述指令可以进入docker内的 bash 可以通过 `glxinfo -B` 查看显卡信息，如果显示 LLVM-pipe 就是没有GPU加速 需要debug

运行仿真
```bash
source install/setup.bash
source /opt/ros/foxy/setup.bash #不知道为啥没有source上 回头看看Dokcerfile哪里写的不对

ros2 launch quadruped_controllers locomotion.launch.py
```
不出意外可以看到 Gazebo 和 Rviz
如果遇到 Gazebo 卡启动，可以运行 `gazebo --verbose` 看看日志

如果出现什么`QStandardPaths: wrong ownership.`之类的报错 应该是 Docker 内用户 UID 和 WSL 下的UID不匹配造成的

Docker中运行以下指令可以设置UID GID

Docker外 （WSL中）
```bash
id -u ${USER} #查看 PID
id -g ${USER} #查看 GID
```

Docker内 更改PID UID
```bash
usermod -u PID lango && usermod -g GID lango
```