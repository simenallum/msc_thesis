cmake_minimum_required(VERSION 3.0.2)
project(anafi_uav_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  rospy
  std_msgs
  geometry_msgs
  message_generation
  actionlib_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  AttitudeCommand.msg
  AttitudeSetpoint.msg
  CameraCommand.msg
  EkfOutput.msg
  EulerPose.msg
  Heading.msg
  MoveByCommand.msg
  MoveToCommand.msg
  PointWithCovarianceStamped.msg
  PositionSetpointRelative.msg
  ReferenceStates.msg
  SaturationLimits.msg
  PoseStampedEuler.msg
  Float32Stamped.msg
  SkyControllerCommand.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  SetDesiredPosition.srv
  SetPlannedActions.srv
)

# Generate actions in the 'action' folder
add_action_files(
  FILES
  GoTo.action
  Land.action
  TakeOff.action
  SearchArea.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs 
  geometry_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS 
  message_runtime
  actionlib_msgs
)