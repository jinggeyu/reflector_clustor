cmake_minimum_required(VERSION 2.8.3)
project(reflector_locate)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Debug") 
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  pcl_ros
  pcl_conversions
)
find_package(PCL REQUIRED COMPONENTS common io visualization )
## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   reflector_msgs.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES reflector_locate
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cluster_cloudpoint src/cluster_cloudpoint.cpp)
add_dependencies(cluster_cloudpoint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#remember to link ,if not ,will occur that function has no definition.
target_link_libraries(cluster_cloudpoint ${catkin_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES})

add_executable(reflector_extract src/reflector_extract.cpp)
add_dependencies(reflector_extract ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(reflector_extract ${catkin_LIBRARIES})

add_executable(register_locate src/register_locate.cpp src/icp_achieve.cpp)
add_dependencies(register_locate ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(register_locate ${catkin_LIBRARIES})
