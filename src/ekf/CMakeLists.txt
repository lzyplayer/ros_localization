cmake_minimum_required(VERSION 2.8.3)
project(ekf)

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
  tf
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
    tf
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

add_executable(bicycleslamnode src/bicycleslam.cpp)
add_dependencies(bicycleslamnode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bicycleslamnode ${catkin_LIBRARIES})

add_executable(akermanslamnode src/akermanslam.cpp)
add_dependencies(akermanslamnode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(akermanslamnode ${catkin_LIBRARIES})
# target_link_libraries(bicyclelrslamlidarnode ${catkin_LIBRARIES})

# target_link_libraries(bicyclelrnode ${catkin_LIBRARIES})
