cmake_minimum_required(VERSION 2.8.3)
project(joy_map_saver)

find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs std_msgs message_generation)



#######################################
## Declare ROS messages and services ##
#######################################

## Generate services in the 'srv' folder
add_service_files(
	FILES
	SaveMap.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map_saver
# CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${scitos_teleop_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(map_saver
#   src/${PROJECT_NAME}/map_saver.cpp
# )

## Declare a cpp executable
 add_executable(joy_map_saver src/ap_map_saver.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

add_dependencies(joy_map_saver joy_map_saver_gencpp)


## Specify libraries to link a library or executable target against
 target_link_libraries(joy_map_saver
   ${catkin_LIBRARIES}
   ${scitos_teleop_LIBRARIES}
 )

#############
## Install ##
#############


## Mark executables and/or libraries for installation
 install(TARGETS joy_map_saver
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_map_saver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
