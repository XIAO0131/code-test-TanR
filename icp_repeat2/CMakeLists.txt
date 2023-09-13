cmake_minimum_required(VERSION 3.0.2)
project(icp_repeat2)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "RELEASE")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_msgs
  pcl_ros
  sensor_msgs
  roscpp
  std_msgs
  ndt_omp
  fast_gicp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)


catkin_package(
CATKIN_DEPENDS
ndt_omp
fast_gicp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS} 
)


link_directories(
  ${PCL_LIBRARY_DIRS}
  ${Python3_LIBRARY_DIRS})

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
# add_executable(pcl_sample src/pcl_sample.cpp)
# target_link_libraries(pcl_sample ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# add_executable(publish_velocity src/publish_velocity.cpp)
# target_link_libraries(publish_velocity ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(astar src/AStar.cpp)

add_executable(pcl_icp src/pcl_icp.cpp)
target_link_libraries(pcl_icp astar ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Python3_LIBRARIES})
# add_executable(imu_try src/imu_try.cpp)
# target_link_libraries(imu_try ${catkin_LIBRARIES} ${PCL_LIBRARIES})
