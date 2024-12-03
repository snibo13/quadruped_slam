cmake_minimum_required(VERSION 3.0.2)
project(gtsam_bag)

## Compile as C++17
add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  sensor_msgs
  nav_msgs
  std_msgs
)

## Find GTSAM
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)


if(GTSAM_FOUND)
  message(STATUS "Found GTSAM ${GTSAM_VERSION}")
else()
  message(FATAL_ERROR "GTSAM not found. Please ensure GTSAM is installed and available.")
endif()

## Declare the catkin package
catkin_package(
  CATKIN_DEPENDS rosbag roscpp sensor_msgs nav_msgs std_msgs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare the executable and link libraries
add_executable(gtsam_node src/gtsam_node.cpp)
target_link_libraries(gtsam_node
  ${catkin_LIBRARIES}
  ${GTSAM_LIBRARIES}
  gtsam
)

## Install targets
install(TARGETS gtsam_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
