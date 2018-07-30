cmake_minimum_required(VERSION 2.8.3)
project(controlling_omni_robots)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  geometry_msgs
  robotino_msgs
  std_msgs
  message_generation
)

add_service_files(
  FILES
  GoToGoal.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  robotino_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robotino_stuff
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)