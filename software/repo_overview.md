---
title: Repository overview
parent: Software
has_children: false
nav_order: 1
---

# Repository overview

### Basic functionality

### [cmr_os](https://github.com/MarvinStuede/cmr_os)

is the basic repository of the robot.  This repository includes all `.launch` files necessary to start the robot (sensors and actuators). The URDF model (`cmr_description`) is also located in this repository.
Here, the transformations between the coordinate systems described on page [Sensor topics and transformations](/Sobi/software/Sensor_topics_and_transformations.html) are defined. Also included are the package for running the Gazebo simulation (`cmr_gazebo`) and software interfaces for using the robot (`cmr_api`).

### [cmr_msgs](https://github.com/MarvinStuede/cmr_msgs)

defines custom ROS messages, services and actions which are needed system-wide and usually on every computer that works with or within the robot.

### [cmr_neo](https://github.com/MarvinStuede/cmr_neo)

contains additional code for the Neobotix MP-500 mobile platform, e.g. to decrease the velocity based on sonar measurements or battery monitoring.


### Social interaction
### [cmr_led](https://github.com/MarvinStuede/cmr_led)

contains drivers to interface Sobi's LED-Panel and strips with the ROS Framework. The drivers provide Subscribers and Service to play specific animations or set colors.



### Mapping, localization and perception
### [cmr_localization](https://github.com/MarvinStuede/cmr_localization)

contains starters and configuration to run RTAB-Map with Sobi. Also contains the map manager, which is further described [here](map_manager.html).

### [cmr_lidarloop](https://github.com/MarvinStuede/cmr_lidarloop)

a method to extend Graph based SLAM (ie. RTAB-Map) by Lidar based loop detection. [Documentation](lidarloop.html)

### [cmr_people_perception](https://github.com/MarvinStuede/cmr_people_perception)

people perception and tracking pipeline tailored to Sobi. The pipeline is heavily based on the [SPENCER Framework](https://github.com/spencer-project/spencer_people_tracking) and several State-of-the-art people detectors.

### Navigation and long term autonomy
### [cmr_monitoring](https://github.com/MarvinStuede/cmr_monitoring)

a framework to monitor system variables and ROS nodes. Based on the [https://github.com/luhrts/monitoring](monitoring ROS package) system errors (e.g. CPU or Network overloads) as well as unpingable Nodes and Topics out of tolerance bands can be detected and the recovery behaviors executed. Also contains a package to log ROS topics to a MongoDB.
