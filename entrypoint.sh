#!/bin/bash

# Create package directory
catkin_create_pkg put_odometry std_msgs rospy roscpp robot_localization

cp /shared/smb_navigation.bag /program/
chmod +x smb_navigation.bag

mv put_odometry src/put_odometry

# Create launch files
cd src/put_odometry
mkdir launch
cp /shared/config.launch launch/
cp /shared/config.yaml launch/

# Add odometry script
cp /shared/display.py .
chmod +x display.py

cd /program/

# Build package
catkin_make
source devel/setup.bash


# Run CAN
roscore &
roslaunch --wait put_odometry config.launch
