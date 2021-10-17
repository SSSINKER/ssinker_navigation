cmake_minimum_required(VERSION 3.0.2)
project(ssinker_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ebimu_odometry
  move_base
  nmea_navsat_driver
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointfollower src/pointfollower.cpp)
target_link_libraries(pointfollower ${catkin_LIBRARIES})
