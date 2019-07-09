cmake_minimum_required(VERSION 2.8.3)
project(prm_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rospy sensor_msgs pcl_ros pcl_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL  REQUIRED)
include_directories(
  include ${catkin_INCLUDE_DIRS}${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})




###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES prm_localization
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
#nodelets
add_library(prm_localization_nodelet src/prm_localization_nodelet.cpp)
target_link_libraries(prm_localization_nodelet
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_library(globalmap_provider_nodelet src/globalmap_provider_nodelet.cpp)
target_link_libraries(globalmap_provider_nodelet
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )
#
#add_executable(prm_localization
#        src/prm_localization_nodelet.cpp
#        )
#add_dependencies(prm_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(prm_localization
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
#add_executable(globalmap_provider
#        src/globalmap_provider_nodelet.cpp
#        )
#add_dependencies(globalmap_provider ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(globalmap_provider
#        ${catkin_LIBRARIES}
#        ${PCL_LIBRARIES}
#        )
add_executable(blank
        src/blank.cpp
        )
add_dependencies(blank ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(blank
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(rt_localization
  src/rt_localization.cpp
)
add_dependencies(rt_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rt_localization
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
