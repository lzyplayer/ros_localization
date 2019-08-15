cmake_minimum_required(VERSION 2.8.3)
project(prm_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(${PROJECT_NAME}_CATKIN_DEPS
        roscpp std_msgs rospy nodelet sensor_msgs pcl_ros pcl_msgs ndt_omp nav_msgs tf tf_conversions dbw_mkz_msgs
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS} )


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)
include_directories(
  include ${catkin_INCLUDE_DIRS}${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

#nvidva
#message(STATUS "CMAKE_SOURCE_DIR:" ${CMAKE_SOURCE_DIR})
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/prm_localization/cmake/Modules/")
#find_package(CUDA REQUIRED)
#find_package(Threads REQUIRED)
#find_package(GLEW REQUIRED)
#find_package(Driveworks REQUIRED)

#set(Driveworks_INCLUDE_DIRS ${Driveworks_INCLUDE_DIRS} "/usr/local/driveworks/include/")
#set(CUDA_INCLUDE_DIRS ${CUDA_INCLUDE_DIRS} "/usr/local/cuda/include")
#set(Driveworks_LIBRARY_DIRS ${Driveworks_LIBRARY_DIRS} "/usr/local/driveworks/lib/")
#include_directories(
#        ${Driveworks_INCLUDE_DIRS}
#        ${CUDA_INCLUDE_DIRS}
#)
#link_directories(${Driveworks_LIBRARY_DIRS})

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES prm_localization
 CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
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
#    ${Driveworks_INCLUDE_DIRS}
#    ${CUDA_INCLUDE_DIRS}
)
#nodelets


add_library(globalmap_provider_nodelet src/globalmap_provider_nodelet.cpp)
target_link_libraries(globalmap_provider_nodelet
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_library(rt_localization_nodelet src/rt_localization_nodelet.cpp)
target_link_libraries(rt_localization_nodelet
        ${catkin_LIBRARIES}
#        ${PCL_LIBRARIES}
#        ${Driveworks_LIBRARIES}
        )
#node
# add_executable(blank
#         src/blank.cpp
#         )
# add_dependencies(blank ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(blank
#         ${catkin_LIBRARIES}
#         ${PCL_LIBRARIES}
#         )
# add_executable(rt_localization_debug
#         src/rt_localization_debug.cpp
#         )
# add_dependencies(rt_localization_debug ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(rt_localization_debug
#         ${catkin_LIBRARIES}
#         ${PCL_LIBRARIES}
#         )

# add_executable(hello_nvidva
#         src/hello_nvidva.cpp
#         )
# add_dependencies(hello_nvidva ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(hello_nvidva
#         ${catkin_LIBRARIES}
#         ${PCL_LIBRARIES}
        # ${Driveworks_LIBRARIES}
#         )

