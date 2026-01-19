启动导航

roslaunch robot nav.launch

启动决策

roslaunch decision decision.launch


启动虚拟串口

socat -d -d pty,raw,echo=0 pty,raw,echo=0

启动串口通信

roslaunch serial_com serial_com.launch

发送血量数据HP：8
echo -e -n '\xA6\x08\xAE' > /dev/pts/3