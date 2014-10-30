cmake_minimum_required(VERSION 2.8.3)
project(message_store_map_switcher)


find_package(catkin REQUIRED COMPONENTS
  mongodb_store
  map_server  
  nav_msgs
  message_generation
)


# yaml workaround adapted from https://github.com/ros-planning/navigation/blob/hydro-devel/map_server/CMakeLists.txt
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.2)


if(NOT (YAMLCPP_VERSION LESS 0.5))
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif()

find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)

# Generate services in the 'srv' folder
add_service_files (
  FILES
  SwitchMap.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages()

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
#  INCLUDE_DIRS include
#  LIBRARIES message_store_map_switcher
#  CATKIN_DEPENDS map_server nav_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  ${SDL_INCLUDE_DIRS}
  ${SDL_image_INCLUDE_DIRS}
)

# Declare a cpp executable
add_executable(message_store_map_saver src/message_store_map_saver.cpp)

# Add cmake target dependencies of the executable/library
# as an example, message headers may need to be generated before nodes
add_dependencies(message_store_map_saver mongodb_store_msgs_generate_messages_cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(message_store_map_saver	
  ${SDL_LIBRARIES}
  ${SDL_image_LIBRARIES}
  ${YAMLCPP_LIBRARIES}  
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############


catkin_install_python(PROGRAMS  scripts/message_store_map_server
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark executables and/or libraries for installation
install(TARGETS message_store_map_saver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

