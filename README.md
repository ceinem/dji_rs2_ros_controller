# dji_rs2_ros_controller

This repo contains a ROS node to interface with the DJI RS2 gimbal.
In addition to this package, you will need the socketcan_bridge node to provide the ROS to CAN interface.

As a third point, you need a hardware CAN interface which is connected to the gimbal.
I used a [CANusb](http://www.can232.com/?page_id=16) CAN to Serial interface as well as a [SLCANUINO](https://github.com/kahiroka/slcanuino).

Follow the instructions of DJI [wiring diagram](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/DJI%20R%20SDK/External%20interface%20diagram.pdf) to connect the gimbal to the CAN interface.

### Run

For the CANusb bring up the can interface using:
```
sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0
sudo ip link set up slcan0
```

For the SLCANUINO bring up the can interface using:
```
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
