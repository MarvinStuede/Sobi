---
title: Software
has_children: true
nav_order: 2
---

# Software
The code base of the robot consists of multiple packages, which are organized within separate repositories to keep the system as modular as possible. Under [Repository overview](repo_overview.html)  you can find a list of repositories with a short function description.


## Prerequisites
The framework is currently implemented under Ubuntu 16.04 (ROS Kinetic) and Ubuntu 18.04 (ROS Melodic).
Therefore, to use the packages the following prerequisites must be met:

* [Ubuntu 16.04](http://releases.ubuntu.com/16.04/) and [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [Ubuntu 18.04](http://releases.ubuntu.com/18.04/) and [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
* [Configured ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) - ROS workspace created and configured

## General installation instructions
If not further specified the dependencies can be installed via wstool and rosdep.
From the workspace's root folder (e.g. ~/cmr_ws) run:
```
wstool init src
```
Clone the repository
```
git clone git@github.com:MarvinStuede/[REPO_NAME].git src/[REPO_NAME]
```
Where you replace [REPO_NAME] with the specific repository.
Merge the rosinstall file and fetch code for dependencies
```
wstool merge -t src src/[REPO_NAME]/[ROSINSTALL_NAME].rosinstall
```
```
wstool update -t src
```
Download binary dependencies for all new packages
```
rosdep update
```
```
rosdep install --from-paths src --ignore-src -r -y
```
