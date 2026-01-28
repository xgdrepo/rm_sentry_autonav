# 串口通信测试流程

## 终端1：创建虚拟串口
```bash
sudo apt-get install socat
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```
选择显示出的 `/dev/pts/2` 其中一个作为输入端口

## 终端2：启动ROS核心
```bash
roscore
```

## 终端3：运行串口收发节点
```bash
cd ~/ws
source devel/setup.bash
# 在 serial_com.launch 中更改串口号为 /dev/pts/2
roslaunch serial_com serial_com.launch
```

## 终端4：运行测试节点
```bash
rosrun serial_com test_node
```

## 终端5：查看接收数据
```bash
sudo apt install minicom
minicom -D /dev/pts/3 -b 115200
```

## 终端6：发送测试数据
```bash
# 发送血量数据HP：100
echo -e -n '\xA6\x64\xC2' > /dev/pts/3
# 发送血量数据HP：8
echo -e -n '\xA6\x08\xAE' > /dev/pts/3
```