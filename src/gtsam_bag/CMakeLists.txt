cmake_minimum_required(VERSION 3.0.2)
project(gtsam_bag)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(GTSAM REQUIRED)

if(GTSAM_FOUND)  
  message(STATUS "Found GTSAM ${GTSAM_VERSION}")
else()
  message(STATUS "GTSAM Not Found")
endif()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRSs}
)

add_executable(gtsam_bag src/gtsam_bag.cpp)
target_link_libraries(gtsam_bag gtsam)
target_link_libraries(gtsam_bag ${catkin_LIBRARIES})
