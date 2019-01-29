cmake_minimum_required(VERSION 2.8.3)
project(speech_recognition)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roslint
  rospy
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES speech_recognition
#  CATKIN_DEPENDS roslint rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

