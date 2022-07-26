cmake_minimum_required(VERSION 3.0.2)
project(insp_rail_pkg)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  message_generation
  geometry_msgs
)


## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES insp_rail_pkg
#  CATKIN_DEPENDS roscpp roslib rospy sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 catkin_install_python(PROGRAMS
   scripts/img_listener_UDP.py
   scripts/UDP_server.py
   scripts/command_publisher.py
   scripts/cmd_vel_publisher.py
   scripts/detectron2_ros.py
   scripts/frames2video.py
   scripts/pc_drone_controller.py
   scripts/drone_controller_simulation.py
   scripts/PID_drone_controller.py
   scripts/Lyapunov_drone_controller.py
   
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )



## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
