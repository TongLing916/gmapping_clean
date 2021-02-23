cmake_minimum_required(VERSION 2.8.3)
project(openslam_gmapping)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -O3 -Wall")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(catkin REQUIRED)

find_package(Glog REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/grid/point_accumulator.cc
  
  src/grid_fast_slam/grid_slam_processor.cc
  src/grid_fast_slam/motion_model.cc
  
  src/scan_matcher/grid_line_traversal.cc
  src/scan_matcher/scan_matcher.cc
  
  src/sensor/odometry_sensor.cc
  src/sensor/range_reading.cc
  src/sensor/range_sensor.cc

  src/utility/random_helper.cc
)
target_link_libraries(${PROJECT_NAME}
  glog
)