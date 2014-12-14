cmake_minimum_required(VERSION 2.8.3)
project(rb1_base_joystick)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    message_generation
    nav_msgs 
    robotnik_msgs
    roscpp 
    sensor_msgs 
    geometry_msgs 
    )
    
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv 
   FILES
    enable_disable.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   nav_msgs robotnik_msgs sensor_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS 
    nav_msgs 
    robotnik_msgs
    roscpp 
    sensor_msgs 
    geometry_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
)

## Declare a cpp executable
add_executable(rb1_base_joystick src/rb1_base_joystick.cpp)

add_dependencies(rb1_base_joystick robotnik_msgs_generate_messages_cpp)
add_dependencies(rb1_base_joystick rb1_base_joystick_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(rb1_base_joystick
   ${catkin_LIBRARIES}
 )

