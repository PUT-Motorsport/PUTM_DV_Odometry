# PUTM_DV_Odometry 2020 (With Formula Student Simulator)
Odometry branch with integrated Formula Student Simulator.  
Based on:
- [Formula-Student-Driverless-Simulator](https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/latest/getting-started/)
- [ROS-melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [Unreal Engine docker image with CUDA, VirtualGL and Vulkan](https://github.com/adamrehn/ue4-runtime)
- [robot_localization](http://wiki.ros.org/robot_localization)

## Requirements

### Hardware

*  recommended system requirements:
    - 8 core 2.3Ghz CPU
    - 12 GB memory
    - 30GB free SSD storage
    - NVidia card with Vulkan support and 3 GB of memory
* testing machine:
    - OS: Ubuntu 20.04.1 LTS 64-bit
    - Intel® Core™ i5-8400 CPU @ 2.80GHz × 6
    - RAM 15,6 GiB
    - GeForce GTX 1060 6GB
* docker image size is about 6 GB


### Software

- docker 19.03
- NVIDIA GPU - [nvidia-docker2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker) section installation on Ubuntu

## Instalation

0. Clone repository

```bash
git clone https://github.com/PUT-Motorsport/PUTM_DV_Odometry_2020.git
git checkout plotting
cd PUTM_DV_Odometry
```

1. Build docker image (on testing machine build time is 13m9,531s)

```bash
docker build -t odom:latest .
```

2. Add docker access to nvidia (it's require sudo privileges to execute)

```bash
chmod +x xauth.sh
./xauth.sh
```

3. Define shared folder
```bash
export SHARED="location here"
```

3. Run docker image

```bash
./start.sh
```

4. You can connect to the machine from another terminal using

```bash
./run_container.sh
```

## Usage

1. Start Formula Student simulation

```bash
./FSDS.sh -nosound
```

2. Start ros bridge

```bash
roslaunch fsds_ros_bridge fsds_ros_bridge.launch
```

3. Start autonomous driving
```bash
python3 python/examples/autonomous_example.py
```

4. Start python script converting FSDS output to robot_localization
```bash
python3 /home/ue4/share/convert.py
```

5. Run visualization and sensor fusion
```
roslaunch /home/ue4/share/start.launch
```
