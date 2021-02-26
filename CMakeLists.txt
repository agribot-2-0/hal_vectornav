cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++17)
project(hal_vectornav)

find_package(catkin REQUIRED
  roscpp
  tf
  tf2
  sensor_msgs
  agribot_common
)

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
