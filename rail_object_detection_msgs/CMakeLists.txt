cmake_minimum_required(VERSION 2.8.3)
project(rail_object_detection_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

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
#  INCLUDE_DIRS
#    include
#  LIBRARIES object_detector
  CATKIN_DEPENDS
    sensor_msgs
    std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
# include_directories(
#   ${catkin_INCLUDE_DIRS}
#   include
#   libs/darknet/src
# )

# Add the definitions for darknet
# if(CMAKE_BUILD_TYPE MATCHES Debug|DEBUG|debug OR NOT CMAKE_BUILD_TYPE)
#   set(DARKNET_DEBUG 1 CACHE INTERNAL "")
# else()
#   set(DARKNET_DEBUG 0 CACHE INTERNAL "")
# endif()

# if(DARKNET_GPU EQUAL 1 AND DEFINED DARKNET_GPU_ARCH)
#   add_definitions(-DGPU=1)
# endif()

## Declare the C based darknet library
# add_custom_target(darknet ALL
#   COMMAND GPU=${DARKNET_GPU} DEBUG=${DARKNET_DEBUG} GPU_ARCH=${DARKNET_GPU_ARCH} make library
#   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/libs/darknet
# )

## Declare a C++ library
# add_library(rail_darknet_detector
#   src/darknet_detector.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(rail_darknet_detector
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
#   darknet
# )

## Declare a C++ executable
# add_executable(darknet_node
#   nodes/darknet_node.cpp
# )

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(darknet_node
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
#   rail_darknet_detector
# )

## Specify libraries to link a library or executable target against
# if(DARKNET_GPU EQUAL 1 AND DEFINED DARKNET_GPU_ARCH)
#   target_link_libraries(rail_darknet_detector
#     ${catkin_LIBRARIES}
#     ${PROJECT_SOURCE_DIR}/libs/darknet/libdarknet.a
#     -L/usr/local/cuda/lib64
#     -lcuda
#     -lcudart
#     -lcublas
#     -lcurand
#   )
# else()
#   target_link_libraries(rail_darknet_detector
#     ${catkin_LIBRARIES}
#     ${PROJECT_SOURCE_DIR}/libs/darknet/libdarknet.a
#   )
# endif()

# target_link_libraries(darknet_node
#   ${catkin_LIBRARIES}
#   rail_darknet_detector
# )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
# install(TARGETS darknet_node rail_darknet_detector
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Install the python scripts at the appropriate location
# catkin_install_python(PROGRAMS nodes/drfcn_node
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#   FILES_MATCHING PATTERN "*.launch"
# )

# install(DIRECTORY libs/darknet/cfg/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/libs/darknet/cfg
#   FILES_MATCHING PATTERN "yolo.cfg"
# )

# install(DIRECTORY libs/darknet/data/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/libs/darknet/data
#   FILES_MATCHING PATTERN "coco.names"
# )
