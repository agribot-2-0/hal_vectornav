cmake_minimum_required(VERSION 2.8.3)
# Compile options so compiles with ROS Kinetic
# Future releases may want to 
add_compile_options(-std=c++17)
# add_compile_options(-std=c++17 -Wall -Wextra -pedantic)
project(hal_vectornav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  agribot_common
)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vectornav
   CATKIN_DEPENDS roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

#include(vnproglib-1.1/cpp/CMakeLists.txt)
add_subdirectory(vnproglib-1.1.5.0/cpp)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include vnproglib-1.1.5.0/cpp/include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
## Declare a cpp executable
add_executable(hal_vectornav src/vectornav_node.cpp src/VectornavNode.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(hal_vectornav
  libvncxx
  ${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS hal_vectornav
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
