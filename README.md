# dji_rs2_ros_controller

This repo contains a ROS node to interface with the DJI RS2 gimbal.
In addition to this package, you will need the socketcan_bridge node to provide the ROS to CAN interface.

Parts of this code are based on the [DJI R SDK demo software](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/DJI%20R%20SDK/SDK%20demo%20software.zip).

## Install

### Setup ROS, catkin workspace and system dependencies
Please install ROS and setup your catkin workspace according to the official ROS tutorials.

Install additional system dependencies:
```bash
sudo apt install can-utils

```

### Clone and build
```bash
cd ~/catkin_ws/src/
git clone git@github.com:ceinem/dji_rs2_ros_controller.git --recursive
catkin build hough2map
```

### Hardware setup
As the only interface with feedback the DJI RS2 gimbal only has a CAN port.
To interface with this, you need the [DJI Focus Wheel](https://store.dji.com/ch/product/ronin-s-focus-wheel) and connected it to a hardware CAN interface connected to your computer.
I tested with a [CANusb](http://www.can232.com/?page_id=16) CAN to Serial interface as well as a [SLCANUINO](https://github.com/kahiroka/slcanuino).

Follow the instructions of DJI [wiring diagram](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/DJI%20R%20SDK/External%20interface%20diagram.pdf) to connect the gimbal to the CAN interface.

### Run
For the CANusb bring up the can interface using:
```bash
sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0
sudo ip link set up slcan0
```

For the SLCANUINO bring up the can interface using:
```bash
sudo slcan_attach -f -s6 -o /dev/ttyUSB0
sudo slcand -S 1000000 ttyUSB0 can0
sudo ifconfig can0 up
```

Make sure to always specify the correct USB-port.

Launch the ROS-node using
```
roslaunch dji_rs2_ros_controller gimbal_control.launch
```

Make sure to specify the correct CAN-bus in the launch file.
