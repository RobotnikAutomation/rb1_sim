cmake_minimum_required(VERSION 2.8.3)
project(rb1_arm_joystick)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_runtime
  roscpp
  sensor_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(FILES ArmRefRb1.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   sensor_msgs
   geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rb1_arm_joystick
#  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(rb1_arm_joystick src/rb1_arm_joystick.cpp)
add_dependencies(rb1_arm_joystick rb1_arm_joystick_generate_messages_cpp)

target_link_libraries(rb1_arm_joystick
   ${catkin_LIBRARIES}
)
