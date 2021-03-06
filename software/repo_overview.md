---
title: Repository overview
parent: Software
has_children: true
nav_order: 1
---

# Repository overview

### Basic functionality

### [cmr_os](https://github.com/MarvinStuede/cmr_os)

is the basic repository of the robot.  This repository includes all `.launch` files necessary to start of the robot (sensors and actuators). The URDF model (`cmr_description`) is also located in this repository.
Here, the transformations between the coordinate systems described on page [system communcation](404) (TODO) are defined. Also included are the package for running the Gazebo simulation (`cmr_gazebo`) and software interfaces for using the robot (`cmr_api`).

### [cmr_msgs](https://github.com/MarvinStuede/cmr_msgs)

defines custom ROS messages, services and actions which are needed system-wide and usually on every computer that works with or within the robot.

### [cmr_neo](https://github.com/MarvinStuede/cmr_neo)

contains additional code for the Neobotix MP-500 mobile platform, e.g. to decrease the velocity based on sonar measurements or battery monitoring.


### Social interaction
TODO


### Perception, localization and navigation
### [cmr_localization](https://github.com/MarvinStuede/cmr_localization)

contains starters and configuration to run RTAB-Map with Sobi. Also contains the map manager, which is further described [here](map_manager.html)
