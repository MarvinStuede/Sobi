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
<!-- [[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_speech|cmr_speech]]
enthält die Software zur Anbindung der Speech-to-Text, Text-to-Speech und Dialogfunktionalität des Roboters. Die Skripte kommunizieren mit den Google Cloud Diensten STT, TTS und Dialogflow.

[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_tablet_app|cmr_tablet_app]]
enthält das Android Projekt für die Tablet Applikation des Roboters. Das Tablet ist über rosjava mit dem ROS-System verbunden und versendet entsprechende Messages/Services zur Bedienung des Roboters.

[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_js_server|cmr_js_server]]
enthält die HTML Dokumente mit dem JavaScript Code zur Erzeugung der 3D Karte auf dem Roboterdisplay.

[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_motor_drivers|cmr_motor_drivers]]
enthält den Code zur Ansteuerung der Servos zur Bewegung der Ohren sowie der BLDC Motoren zur Bewegung der Arme.

[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_led|cmr_led]]
enthält den Code zur Ansteuerung des LED-Panels sowie der LED-Streifen in den Ohren und Armen des Roboters. -->

### Perception, localization and navigation
TODO
<!-- [[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_localization|cmr_localization]] beinhaltet  ein Repo zur Lokalisierung des Roboters mittels RTAB-Map. Für genauere Informationen siehe die Einträge zu [[/cmrobot/rtab-map_slam|RTAB-Map]] und dem [[/cmrobot/map_manager|Map Manager]].

[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_lidarloop|cmr_lidarloop]] beinhaltet eine Erweiterung von RTAB-Map um Schleifenschlüsse basierend auf Laserscans vorzunehmen. Genauere Informationen finden sich unter [[/cmrobot/cmr_lidarloop|Schleifenschluss mit cmr_lidarloop]]. Der  Code basiert auf der [[https://seafile.projekt.uni-hannover.de/f/f4dbf9e8f4964dfc8f1b/|Masterarbeit von Tim-Lukas Habich]]


[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_people_perception|cmr_people_perception]]
enthält Pakete zur Personendeketion. Es wird hier auf verschiedene Algorithmen zur Detektion zurück gegriffen und das Tracking erfolgt über das SPENCER Framework. Der Code basiert auf der [[https://seafile.projekt.uni-hannover.de/f/9535e011d3cb4012a60f/|Masterarbeit von Alexander Petersen]].

[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_social_navigation|cmr_social_navigation]] beinhaltet verschiedene Plug-ins für die Costmap von Move Base, welche Kosten anhand von detektierten Personen erzeugen. Der Code basiert auf der [[https://seafile.projekt.uni-hannover.de/f/9535e011d3cb4012a60f/|Masterarbeit von Alexander Petersen]].

[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_pedsmin|cmr_pedsim]] ist ein Repo zur Simulation von getrackten Personen in vordefinierten Räumen und Bewegungen. -->
