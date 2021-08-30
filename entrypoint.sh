#!/bin/bash

# Create package directory
catkin_create_pkg put_odometry std_msgs rospy roscpp robot_localization

# Install virtual CAN
cd src
git clone https://github.com/PUT-Motorsport/PUTM_DV_recruitment_task_2020.git dv_task/
# Replace CAN files
cp -f /shared/CANreceiver.py ros2can/scripts/CANreceiver.py
cp -f /shared/CANsender.py ros2can/scripts/CANsender.py
# Enable execution
chmod +x ros2can/scripts/CANreceiver.py
chmod +x ros2can/scripts/CANsender.py

cd /program
cp /shared/smb_navigation.bag /program/
chmod +x smb_navigation.bag

mv put_odometry src/put_odometry

# Create launch files
cd src/put_odometry
mkdir launch
cp /shared/config.launch launch/
cp /shared/config.yaml launch/
cd /program/

# Build package
catkin_make
source devel/setup.bash


# Run CAN
roscore &
rosrun rviz rviz &
roslaunch --wait put_odometry config.launch
