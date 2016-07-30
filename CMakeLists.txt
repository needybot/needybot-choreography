cmake_minimum_required(VERSION 2.8.3)
project(needybot_choreography)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  rospy
  geometry_msgs 
  needybot_msgs
  message_generation
)

find_package(PkgConfig)

catkin_python_setup()

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY
    action
  FILES
    Animator.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs 
    needybot_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    rospy
    geometry_msgs 
    message_runtime
    needybot_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/choreography.test)
endif()
