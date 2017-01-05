cmake_minimum_required(VERSION 2.8.3)
project(rail_object_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Object.msg
  Detections.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SceneQuery.srv
  ImageQuery.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
#  LIBRARIES object_detector
  CATKIN_DEPENDS
    cv_bridge
    image_transport
    message_runtime
    sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  libs/darknet/src
)

## Declare the C based darknet library
add_custom_target(darknet
  COMMAND make library
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/libs/darknet
)

## Declare a C++ library
add_library(rail_object_detector
  src/${PROJECT_NAME}/detector.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(rail_object_detector
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  darknet
)

## Declare a C++ executable
add_executable(detector_node
  src/detector_node.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(detector_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  rail_object_detector
)

## Specify libraries to link a library or executable target against
target_link_libraries(rail_object_detector
  ${catkin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/libs/darknet/libdarknet.a
)

target_link_libraries(detector_node
  ${catkin_LIBRARIES}
  rail_object_detector
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS detector_node rail_object_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY libs/darknet/cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/libs/darknet/cfg
  FILES_MATCHING PATTERN "yolo.cfg"
)

install(DIRECTORY libs/darknet/data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/libs/darknet/data
  FILES_MATCHING PATTERN "coco.names"
)
