cmake_minimum_required(VERSION 3.0.2)
project(particle_filter)


add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  geodesy
  nmea_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  interactive_markers
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES particle_filter
  LIBRARIES particle_filter_localization_nodelet
  #CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(particle_filter_localization_nodelet
  apps/particle_filter_localization_nodelet.cpp  
  src/particle_filter.cpp
   )

 target_link_libraries(particle_filter_localization_nodelet
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES}
)

add_executable(particle_filter_node src/main.cpp src/particle_filter.cpp)

target_link_libraries(particle_filter_node
${catkin_LIBRARIES}
${PCL_LIBRARIES}
)


# add_dependencies(particle_filter_localization_nodelet ${PROJECT_NAME}_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
 







