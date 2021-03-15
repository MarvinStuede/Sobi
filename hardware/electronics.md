---
title: Devices and components
parent: Hardware
has_children: true
nav_order: 1
---

# Electronic devices and components

This chapter deals with the general structure of the robot's hardware.
In the following, the entire components and their position in the robot system will be discussed.
The following figure shows a general overview of all hardware-related components of the robot:

## General System Setup
[ ![component_overview](/Sobi/images/components_overview.png) ](/Sobi/images/components_overview.png)

The structure of the following section is divided into the subsections base, torso and head.

## The base of Sobi
Sobi's base consists of the purchased Neobotix MP500 mobile platform.

{{cmrobot:base.png?240 }}\

The platform, shown here in blue, consists of two drive wheels and a running wheel.
Both drive wheels have a motor, so that a pure rotation of the robot is possible.
A control computer is installed in the base, to which the ultrasonic sensors are also connected via the Neobotix USBoard.
These three ultrasonic sensors are marked here with a blue arrow. They are used to send a signal to block the drives as soon as there is an obstacle in a certain radius around the robot (see also [Communication structure](/cmrobot/communication_structure)).  
There is also a small display at the back of the base. This display shows the current status as well as the battery level of the robot.
The status of the robot can have a total of four different values. The statuses are shown in the following table with a description and a link to the solution of the problem.

| Status | Description | Link |
| ------ | ----------- | ---- |
| Emergency Stop | The emergency stop has been activated. This must be removed again to get control the platform motors | [Getting Started](/cmrobot/getting_started/) |
| Waiting | The robot's sensors have not been started yet |[Getting Started](/cmrobot/getting_started/)|
| Ready | The robot is ready to navigate autonomously | - |
|Invalid Data |Error - The entire robot must be restarted | [Getting Started](/cmrobot/getting_started/)|

{{cmrobot:sobi.png?240 }}\

In the figure on the left, the entire robot is shown.
A S300 2D safety laser scanner from SICK is attached to the bottom of the base.
The laser sensor and the drives are a self-contained system, so that it can block or limit the drives independently of the overall system.
The sensor is divided into three detection areas, which are defined by the respective distance to the sensor. A distinction is made between the first detection range, in which there is no intervention in the drives.
In the second detection range, which is a subset of the first, the drives are limited. Finally, the critical range is defined, in which the drives block directly as soon as an obstacle is detected in this range.
The sensor data of the 2D laser sensor are nevertheless available in the ROS framework, which will be discussed in more detail in the chapter of [Communication structure of the robot](/cmrobot/communication_structure).

**IMPORTANT**: ATTENTION! The robot is powered at 24 volts by four lead-acid batteries. This means that there is a permanent voltage at the batteries even if the robot is switched off!

## The torso and the head of SOBI
The torso and the head were designed by the Institute for Mechatronic Systems (imes). In them there are some components, which are shown in the following pictures.

{{cmrobot:sobit_front.png?300}}
{{cmrobot:sobi_back.png?265}}

The following table describes the components in ascending order.

| Sensor | Description |
| -----  | ----------- |
| 2D Laser Scanner |The 2D laser scanner (SICK S300) is primarily used for safety-related functions.
It is a safety system decoupled from the ROS framework. Nevertheless, the laser scan data can be accessed in the ROS Framework. More about this in chapter [Communication structure of the robot](/cmrobot/communication_structure). |
|Vecow EVS1000 |The embedded computer Vecow EVS 1000 is the main computer of the entire robot system. The Rosmaster of the entire communication system of the robot is executed on this computer.|
|Tablet |The tablet is a Samsung Galaxy Tab A (10.1 inch, 2016), which is an interface between man and machine for the outside users of the robot system. |
|Microphone |The microphone is an array microphone (ReSpeaker Mic Array v2). In addition to normal speech recognition, it can detect the direction from which the sound is coming via direction-of-arrival (function currently unused). Furthermore, algorithms like beamforming and static noise suppression are supported.  |
|RGBD cameras |In addition, two RGB-D cameras (Intel Realsense D435) are mounted in the head of the robot. These are depth imaging cameras and thus provide the depth image in addition to the RGB image.  |
|Jetson Nano |The two Realsense cameras are connected to the Jetson Nano, which is installed in the head. On the Jetson the corresponding drivers and ROS nodes are executed to provide the RGB, depth image and the point clouds to the ROS framework via Ethernet.  |
|Raspberry Pi |In addition, a Raspberry Pi is installed in the head. This is a single board (SB) computer and also part of the whole ROS framework. Here the LED panel and the servo motors are connected, which are also controlled by individual ROS nodes.  |
|Emergency Stop Switch |Two emergency stop switches are attached to the robot, which directly block the wheels when pressed.  |
|IMU/AHRS |The Inertial Measurement Unit (IMU) or the AHRS (Attitude Heading Reference System) is located at the bottom of the torso and is used for motion detection of the robot. An integrated Kalman filter directly provides the absolute orientation.  |
|BLDC Motors |The BLDC (Brushless Direct Current) motors are used to drive the arms. They are equipped with a gearbox (reduction ratio of 1:64) and an encoder. The motors are controlled via the underlying ODrive.  |
|Velodyne 3D Laser Scanner |The 3D laser scanner (Velodyne VLP 16 PUCK) is located in the neck. It provides three-dimensional laser data in the ROS framework as a point cloud including the measured intensities.  |
|Ears |In the head there are two modeling servo motors, which are responsible for the movement of the ears. These are controlled using the Raspberry Pi and the pigpio library.  |
|LEDs |The ears and arms are also equipped with RGB LED strips that can be controlled with the Arduino. The strips offer the possibility to display any RGB colors in 8-bit. In the head is an additional LED panel, which animates the eyes of the robot. The panel can display any BMP files in 64x32 Px format as well as GIF files in the same format. The panel is controlled by the Raspberry Pi using the [Rpi RGB LED Matrix Library](https://github.com/hzeller/rpi-rgb-led-matrix) |
