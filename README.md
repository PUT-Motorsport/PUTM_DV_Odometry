# PUT_DV Odometry
## GPS + IMU fusion using `robot_localization`

This branch contains files used for GPS + IMU fusion on either `Groove` GPS and `Ardusimple RTK` GPS.
* [Groove GPS](https://wiki.seeedstudio.com/Grove-GPS/)
* [Ardusimple RTK](https://eu.mouser.com/new/ardusimple/ardusimple-simplertk2b-boards/)
* [IMU (RSX UM7)](https://www.robotshop.com/uk/um7-orientation-sensor.html)

## Installation

The scripts run on ROS `Noetic`, but they should work on other distributions as well.

If you want to run the **Groove GPS** version, you need to download `nmea_navsat_driver` for you ROS version.
In example, for ROS Noetic:

```bash
$ sudo apt-get install ros-noetic-nmea-navsat-driver
```

If you are running the **Ardusimple RTK GPS** version, you need to download the `ublox` package.
In example, for ROS Noetic:
```bash
$ sudo apt-get install ros-noetic-ublox
```

For IMU, you need to install `um7` package. If you have any of its supported ROS versions (such as melodic), install
it normall, in example for melodic:
```bash
$ sudo apt-get install ros-melodic-um7
```

Otherwise use the modified version present in this repository inside of `um7` folder. If you want to modify it yourself
download the official `um7` repository [here](https://github.com/ros-drivers/um7).

You also need `robot_localization` package, which you can download normally via package manager, in example for Noetic:

```bash
sudo apt-get install ros-noetic-robot-localization
```

## Usage

Copy the `odom` (and optionally `um7`) folder into your main package location, run `catkin_make`, setup
the configuration files so they fit your system and setup and then use:

`roslaunch odom start.launch` for Groove GPS or

`roslaunch odom ardu_start.launch` for Ardusimple RTK GPS.

## Configuration

You need to set the `config/ekf_global.yaml` properly in order to setup **robot_localization** properly. The most important
parameters are:

* `frequency` - frequency at which **robot_localization** will publish data, ideally it should be faster than your GPS frequency
(because IMU needs to publish messages faster)

* `transform_time_offset` - how much time to wait before calculating transform, used to avoid problems with imu data coming too fast

* `print_diagnostics` - whether or not to print data about the node (if it's working or not)

Then, modify the `launch` file of your choice. You need to make sure that the `port` argument is correct in the `um7` package.

If you are using `Ardusimple RTK` GPS, also modify `config/ardu.yaml` file.

## ROSBags

In `bags` folder you will find recorded rosbags. Files following the name `ardu*.bag` have been recorded using **Ardusimple RTK**,
and `groove*.bag` bags have been recorded using **Groove** GPS. Every one of them is 10 minutes long.

## Important notes

1. `altitude` in GPS message (so **/gps/fix** or **/fix**) cannot be assigned to `NaN`! If you do that, `nmea_navsat_driver` will ignore it and not convert GPS data into Odometry, which means no fusion will take place! The `convert.py` script will replace `NaN` with `0.0`, which will solve the problem. If you are using Ardusimple RTK GPS, it will return altitude, so you don't need to worry about it and don't have to run the script.

2. IMU recordings (and UM7 in general) does not conform to **REP-105** standard (more about it [here](http://docs.ros.org/en/kinetic/api/robot_localization/html/integrating_gps.html#imu-data)), which is why you need to keep the inversion in config file:
```
<param name="yaw_offset" value="1.5707963"/>
```
Otherwise you will get wrong results!

3. By default, IMU's frame id is `imu_link` and GPS's is `gps`, so you need to cast them into `base_link` using `tf2` package:
```
<node pkg="tf2_ros" type="static_transform_publisher" name="bl_to_imu" args="0 0 0 0 0 0 1 base_link imu_link" />
<node pkg="tf2_ros" type="static_transform_publisher" name="bl_to_gps" args="0 0 0 0 0 0 1 base_link gps" />
```

4. The feedback loop from **robot_localization** to **nmea_navsat_driver** is intentional! GPS will listen for IMU message to set its heading and then it will unsubscribe from the topic, so it's important you do not remove it.


Reference libraries:
* [robot_localization](http://docs.ros.org/en/api/robot_localization/html/index.html)
* [ublox](https://github.com/KumarRobotics/ublox)
* [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver)
* [um7](https://github.com/ros-drivers/um7)
