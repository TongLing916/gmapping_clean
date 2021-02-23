cmake_minimum_required(VERSION 2.8.3)
project(gmapping)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -O3 -Wall")

find_package(catkin REQUIRED 
COMPONENTS 
    roscpp
    rospy
    std_msgs
    tf 
    sensor_msgs 
    nav_msgs
    openslam_gmapping
)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/../openslam_gmapping/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
    app/main.cc
    
    src/slam_gmapping.cc
)

target_link_libraries(${PROJECT_NAME}_node
  -pthread
  ${catkin_LIBRARIES}
)