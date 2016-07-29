cmake_minimum_required(VERSION 2.8.3)
project(aruco2_ros)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  aruco2_catkin
  aruco2_msgs
  cv_bridge
  image_transport
  roscpp
  geometry_msgs
  tf
  eigen_conversions
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(aruco2_detector_node src/detector_node.cpp src/utils.cpp)

target_link_libraries(aruco2_detector_node
   ${catkin_LIBRARIES}
)
