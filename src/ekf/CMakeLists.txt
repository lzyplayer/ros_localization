cmake_minimum_required(VERSION 2.8.3)
project(syq_ekf)

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

# Attempt to find Eigen using its own CMake module.
# If that fails, fall back to cmake_modules package.
# find_package(Eigen3 REQUIRED)
# message("!!!!!!!!!!")
# message("${EIGEN3_INCLUDE_DIR}")
# message("!!!!!!!!!!")

set(EIGEN_PACKAGE EIGEN3)

#roslint_cpp()

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

add_executable(bicycleslamlidarnode src/bicycleslamlidar.cpp)
add_executable(akermanslamlidarnode src/akermanslamlidar.cpp)
add_executable(akermanlidarnode src/akermangpslidar.cpp)
add_executable(akermangpsnode src/akermangps.cpp)
add_executable(akermannode src/akerman-un.cpp)
add_executable(bicyclenode src/bicycle-un.cpp)
add_executable(akermangpsvelnode src/akermangpsvel.cpp)


target_link_libraries(bicycleslamlidarnode ${catkin_LIBRARIES})
target_link_libraries(akermanslamlidarnode ${catkin_LIBRARIES})
target_link_libraries(akermanlidarnode ${catkin_LIBRARIES})
target_link_libraries(akermangpsnode ${catkin_LIBRARIES})
target_link_libraries(akermangpsvelnode ${catkin_LIBRARIES})
target_link_libraries(akermannode ${catkin_LIBRARIES})
target_link_libraries(bicyclenode ${catkin_LIBRARIES})