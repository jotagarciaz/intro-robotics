# Introduction-to-robotics

## In order to create a package:

1- cd ~/catkin_ws/src
2- catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
3- cd ..
4- catkin_make
5- . ~/catkin_ws/devel/setup.bash

## In order to access to the package you need to use:

roscd 

## To supervise which first-order dependencies are in our package we should use:

rospack depends1 beginner_tutorials 

## To check indirect dependencies we shall use:

rospack depends1 rospy

## To Build a package 

cd ~/catkin_ws/

catkin_make Ho

## How to build a publisher node

- We have to create scripts directory:

mkdir scripts
cd scripts

## Add this to CMakeLists.txt

catkin_install_python(PROGRAMS scripts/talker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
