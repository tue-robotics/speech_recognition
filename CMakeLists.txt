cmake_minimum_required(VERSION 2.8.3)
project(speech_recognition)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_add_env_hooks(setup SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package()

