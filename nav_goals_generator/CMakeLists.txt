cmake_minimum_required(VERSION 2.8.3)
project(nav_goals_generator)

find_package(catkin REQUIRED COMPONENTS 
    geometry_msgs
    message_generation
    nav_msgs 
    rospy
    tf
    std_msgs
    visualization_msgs
)

add_service_files(
  FILES
    NavGoals.srv
    WeightedNavGoals.srv
    ROINavGoals.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    geometry_msgs
    message_generation
    nav_msgs 
    rospy
    tf
    std_msgs
    visualization_msgs  
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(
    PROGRAMS scripts/generate_nav_goals.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
