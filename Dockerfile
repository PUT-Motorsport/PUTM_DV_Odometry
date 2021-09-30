# use image with cuda drivers and vulkan
FROM adamrehn/ue4-runtime:18.04-cudagl10.2-virtualgl

USER root
# Install sudo
RUN sed -i s/archive/pl.archive/ /etc/apt/sources.list
RUN apt-get update \
	&& apt-get upgrade -y \
	&& rm -rf /var/lib/apt/lists/*

# Set time zone
ENV TZ=Europe/Warsaw
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN apt-get upgrade -y && apt-get update
RUN apt-get install -y tzdata

# Install AirSim requirements
RUN apt-get install -y --no-install-recommends \
	python3 \
	python3-pip \
	sudo \
	libglu1-mesa-dev \
	xdg-user-dirs \
	pulseaudio \
	sudo \
	x11-xserver-utils \
	git \
    wget \
	rsync \
	unzip \
	g++ \
	vim \ 
	nano

ENV PATH="/home/ue4/miniconda3/bin:${PATH}"
ARG PATH="/home/ue4/miniconda3/bin:${PATH}"

RUN rm -rf /var/lib/apt/lists/*

# Install missing tool
RUN apt-get update
RUN apt-get install -y lsb-release python3-tk

# Allow the ue4 user to use sudo without a password
RUN passwd -d ue4 && usermod -aG sudo ue4

# Create not-sudo account
USER ue4
WORKDIR /home/ue4

RUN wget \
    https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh \
    && mkdir /home/ue4/.conda \
    && bash Miniconda3-latest-Linux-x86_64.sh -b \
    && rm -f Miniconda3-latest-Linux-x86_64.sh 

# Install python libs
RUN pip3 install setuptools wheel
RUN python3 -m pip install --upgrade pip
RUN pip3 install scikit-build airsim
RUN pip3 install --quiet matplotlib jupyter rospkg pyyaml pyquaternion scipy
RUN conda install -c anaconda tensorflow-gpu==2.4.1 -y

# Install ros melodic and requirements
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    sudo apt-get update && \
    sudo apt-get install -y ros-melodic-desktop-full && \
	sudo apt-get install -y ros-melodic-tf2-geometry-msgs \
							python-catkin-tools \
							python3-tk \
							python3-future \
							ros-melodic-rqt-multiplot \
							ros-melodic-joy \
							ros-melodic-cv-bridge \
							ros-melodic-image-transport \
							ros-melodic-ros-numpy \
							libyaml-cpp-dev \
							libcurl4-openssl-dev \
							libeigen3-dev \
							libcppunit-dev \
							python3-psutil \
							ros-melodic-robot-localization \
							ros-melodic-rviz \
							ros-melodic-map-server \
							ros-melodic-xacro

# Add ros melodic to startup
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# Get FSDS
RUN git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git --recurse-submodules

WORKDIR /home/ue4/Formula-Student-Driverless-Simulator

ADD ./settings.json settings.json
ADD ./shared/config.rviz ros/src/fsds_ros_bridge/rviz/default.rviz
ADD ./cameralauncher.py ros/src/fsds_ros_bridge/scripts/cameralauncher.py
ADD ./fsds_ros_bridge_camera.cpp ros/src/fsds_ros_bridge/src/fsds_ros_bridge_camera.cpp

# Build FSDS
RUN cd AirSim && \
    ./setup.sh && \
    ./build.sh

# Clone race evaluator
RUN cd ros/src && \
	git clone https://github.com/bartoszptak/ros-driving-track-evaluator.git -b sim/fsds

# Clone Formula Student Driverless and Autonomous Vehicle project repository
RUN cd ros/src && \
	git clone https://github.com/MatPiech/Autonomous_Cars_FSDS_project.git fsds_utils/

# Build ROS bridge
RUN cd ros && \
	catkin config --extend /opt/ros/melodic  && \	
    catkin init && \
	LINE=`sed -n '/.publish(go_signal_msg)/=' ./src/fsds_ros_bridge/src/airsim_ros_wrapper.cpp` && \
	sed -i "$LINE"' s/^/\/\//' ./src/fsds_ros_bridge/src/airsim_ros_wrapper.cpp && \
    catkin build

# Add FSDS env to startup
RUN echo "source /home/ue4/Formula-Student-Driverless-Simulator/ros/devel/setup.bash" >> ~/.bashrc

# Install python requrements
RUN cd python && \
	pip3 install -r requirements.txt

RUN wget "https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.0.0/fsds-v2.0.0-linux.zip" && \
	unzip fsds-v2.0.0-linux.zip && \
	mv fsds-v2.0.0-linux/* . && \
	rm fsds-v2.0.0-linux.zip

COPY ./convert.py /home/ue4/Formula-Student-Driverless-Simulator/ros/src/fsds_ros_bridge/scripts/convert.py

USER root
RUN echo "ue4:ue4" | chpasswd
USER ue4
