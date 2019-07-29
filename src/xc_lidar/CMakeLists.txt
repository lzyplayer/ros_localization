cmake_minimum_required(VERSION 2.8.3)
project(lidar_odometry)

# -mavx causes a lot of errors!!
add_definitions(-std=c++11)
# add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
# set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pcl_ros
  roscpp
  rospy
  tf
  sensor_msgs
  ndt_omp
  message_generation
  message_filters
)

# find_package(OpenMP)
# if (OPENMP_FOUND)
#     set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#     set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# endif()

find_package(PCL 1.7 REQUIRED)
include_directories(include ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
# message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
# message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(EIGEN_PACKAGE EIGEN3)

catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs message_runtime eigen_conversions
  DEPENDS ${EIGEN_PACKAGE}
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES hdl_scan_matching_odometry
#  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(lidar_matching 
              src/lidar_odometry/lidar_odometry.cpp
              )

add_executable(lidar_odom 
              src/lidar_odometry/main.cpp
              src/lidar_odometry/lidar_odometry.cpp
              )

target_link_libraries(lidar_odom ${catkin_LIBRARIES})

add_dependencies(lidar_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



