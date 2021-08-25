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

# Build package
mv put_odometry src/put_odometry
catkin_make
source devel/setup.bash


# Run CAN
roscore &
rosrun ros2can CANreceiver.py &
rosrun ros2can CANsender.py
