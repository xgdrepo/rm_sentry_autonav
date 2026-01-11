终端1 发送测试：

安装工具：
sudo apt-get install socat
开启虚拟端口：
socat -d -d pty,raw,echo=0 pty,raw,echo=0
选择显示出的dev/pts/2其中一个作为输入端口

终端2：启动roscore
roscore

终端3 在serial_com.launch中更改串口号为dev/pts/2,运行串口收发节点：
cd ~/ws
source devel/setup.bash
roslaunch serial_com serial_com.launch

终端4：运行节点发布/cmd和spin
rosrun serial_com test

终端5：查看另一个串口号dev/pts/3接受到的数据，确认dev/pts/2是否接收到serial_com发送的数据
安装工具：
sudo apt install minicom
接受dev/pts/3端的数据：
minicom -D /dev/pts/3 -b 115200

终端6：从发送血量数据到/dev/pts/3串口，serial_com再从/dev/pts/2中读取血量数据，例如： 0xA6 0x64 0xXX（XX为校验和），假设血量是 100 (0x64)，计算校验和：0xA6 ^ 0x64 = 0xC2，发送之后，可以在终端3中看到接受到的HP数据

echo -e -n '\xA6\x64\xC2' > /dev/pts/3







