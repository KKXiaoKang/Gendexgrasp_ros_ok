cmake_minimum_required(VERSION 3.0.2)
project(motion_capture_ik)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  geometry_msgs
)

add_message_files(
  FILES 
  robotArmQVVD.msg
  ikSolveError.msg
  handPose.msg
  armHandPose.msg
  twoArmHandPose.msg
)
add_service_files(
  DIRECTORY srv
  FILES
  changeArmCtrlMode.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motion_capture_ik
 CATKIN_DEPENDS rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
