cmake_minimum_required(VERSION 2.8.3)
project(slamwithimu)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  message_filters
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  dbw_mkz_msgs
  )


#roslint_cpp()
#add_message_files(FILES
#  Zpre.msg
#)
#generate_messages(DEPENDENCIES std_msgs)
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    eigen_conversions
    geometry_msgs
    message_filters
    message_runtime
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    dbw_mkz_msgs

)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

# Library definitions
# add_library(filter_utilities src/robot_localization/filter_utilities.cpp)
# add_library(filter_base src/robot_localization/filter_base.cpp)
# add_library(ekf src/robot_localization/ekf.cpp)
# add_library(ros_filter_utilities src/robot_localization/ros_filter_utilities.cpp)
# add_library(localization src/CTRA.cpp)

# Dependencies
#add_dependencies(filter_base ${PROJECT_NAME}_gencpp)

# Executables

add_executable(lidarimumode src/lidarodom.cpp )
add_executable(slamimumode src/mailunca.cpp)
add_executable(mailunca_debug src/mailunca_debug.cpp)
target_link_libraries(mailunca_debug ${catkin_LIBRARIES})
target_link_libraries(slamimumode ${catkin_LIBRARIES})
target_link_libraries(lidarimumode ${catkin_LIBRARIES})

