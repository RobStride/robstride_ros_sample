# ROS package for RobStride motor control
This routine was reposted by RobStride Dynamics from DR.MuShibo. Sincere gratitude goes to DR.MuShibo for their development and sharing.

### USB2CAN Hardware:Canable
- canable (cantact clone): http://canable.io/ (STM32F042C6)
- 灵足的串口转CAN模块只适用于灵足的上位机，Ubuntu上使用需要额外的canable模块。

## Dependency:
- 注意自己的ros2版本号，自行修改
```shell
sudo apt-get install net-tools
sudo apt-get install can-utils
sudo apt-get install ros-humble-can-msgs
sudo apt-get install ros-humble-socketcan-bridge
```

### Ubuntu
```shell
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev

sudo ip link set can0 type can bitrate 1000000 

sudo ip link set can0 up
sudo ifconfig can0 txqueuelen 100
```

### Launch the launch file for the demo
- 在工作空间中运行如下命令: 
```shell
colcon build 
source ./install/setup.zsh (or bash)
ros2 run rs_motor_ros2 rs_motor_ros2
```