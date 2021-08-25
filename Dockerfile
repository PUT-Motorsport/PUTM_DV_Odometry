FROM ros:melodic

# Update outdated registry key
RUN sudo apt-get -y update && sudo apt-get -y upgrade

# Install ROS essentials
RUN sudo apt-get -y install libeigen3-dev libcppunit-dev python3-psutil python3-future \
                            python3-pip python-pip curl wget

# Install robot-localization dependencies
RUN sudo apt-get -y install ros-melodic-robot-localization ros-melodic-rviz ros-melodic-cv-bridge

# Create directory for program and files shared with host
RUN mkdir -p /program/src
RUN mkdir -p /shared

# Install python requirements
RUN pip3 install --upgrade pip
COPY requirements.txt /program/requirements.txt
RUN pip3 install -r /program/requirements.txt

# Install virtual CAN
COPY ros2can /program/src/ros2can

# Copy startup and refresh scripts
COPY entrypoint.sh /program/
RUN chmod 777 /program/entrypoint.sh
COPY refresh.sh /program/
RUN chmod 777 /program/refresh.sh

WORKDIR /program
