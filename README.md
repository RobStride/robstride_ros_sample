# ROS package for RobStride motor control
This routine was reposted by RobStride Dynamics from DR.MuShibo. Sincere gratitude goes to DR.MuShibo for their development and sharing.

### USB2CAN Hardware:Canable

-canable (cantact clone): http://canable.io/ (STM32F042C6)

## Dependency:
```shell
sudo apt-get install net-tools
sudo apt-get install can-utils
sudo apt-get install ros-noetic-can-msgs
sudo apt-get install ros-noetic-socketcan-bridge
```

### Use of slcan in Linux Ubuntu

1. Connect the can module to the ubuntu system, and send commands to query the device:
   
```shell
ls /dev/ttyACMx
```

2. Map ttyACM0 to can0, and set the bit rate to 1000K:

```shell
sudo slcand -o -c -s8 /dev/ttyACM0 can0
```

The corresponding setting of the bit rate is :
-s8 = 1M

3. Enable CAN:

```shell
sudo ifconfig can0 up
```

4. Configure the storage length of the transmission data buffer of txqueuelen:

```shell
sudo ifconfig can0 txqueuelen 100
```

### Launch the launch file for the demo

```shell
roslaunch motor_control motor_test.launch
```
