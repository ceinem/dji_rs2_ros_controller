cmake_minimum_required(VERSION 3.0.2)
project(dji_rs2_ros_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
  socketcan_bridge
)


add_service_files(DIRECTORY srv
  FILES
  SendJointPos.srv
  SendJointSpeed.srv
  )




###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gimbal_control
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS message_runtime
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)




catkin_install_python(PROGRAMS
  scripts/can_bus_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
