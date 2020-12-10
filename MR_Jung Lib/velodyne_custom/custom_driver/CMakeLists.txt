cmake_minimum_required(VERSION 3.0.2)
project(custom_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  velodyne_msgs
  custom_msgs
  encoder_msgs
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   VelodyneCustom.msg
#   RingIndex.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#  DEPENDENCIES
#  std_msgs
#  sensor_msgs
#  geometry_msgs
# )

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
  INCLUDE_DIRS include
  LIBRARIES custom_driver
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs pcl_ros
  DEPENDS PCL
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

add_executable(custom_driver src/custom_driver.cpp src/main.cpp)

target_link_libraries(custom_driver ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
  
#############
## Install ##
#############
