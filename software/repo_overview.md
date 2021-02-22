---
title: Repository overview
parent: Software
has_children: true
nav_order: 1
---

# Repository overview

### Basic functionality
####[cmr_os](https://github.com/MarvinStuede/cmr_os)
is the basic repository of the robot.  In this repository are all necessary `.launch` files for the start of the robot (sensors and actuators). The URDF model (`cmr_description`) is also located in this repository.
Here, the transformations between the coordinate systems described on page [system communcation](404) (TODO) are defined. Also included are the package for running the Gazebo simulation (`cmr_gazebo`) and software interfaces for using the robot (`cmr_api`).

<!--
[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_utils|cmr_utils]] beinhaltet verschiedene Skripte zur Einrichtung neuer Computer und User. Unter anderem können neue Workspaces mit dem benötigtem Code installiert oder SSH Verbindungen eingerichtet werden. Desweiteren befindet sich eine Datenbank für KeePass2 mit den benötigten Kennwörtern für den Roboter in diesem Repo.

[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_os|cmr_os]]
ist das grundlegende Repository des Campusroboters.  In diesem befinden sich alle notwendigen ''.launch'' Dateien für den Start der Sensorik. Darüber hinaus befindet sich in diesem Repository das URDF-Modell (//cmr_description//). In diesem werden die Transformationen zwischen der in Kapitel [[/cmrobot/communication_structure// |Systemkommunikation]] beschriebenen Koordinatensysteme definiert. Außerdem sind das Paket zur Ausführung der Gazebo Simulation (//cmr_gazebo//) sowie Softwareschnittstellen zur Verwendung des Roboters (//cmr_api//) enthalten.

[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_msgs|cmr_msgs]]
definiert allgemein benötigte Messages, Services und Actions. Systemweit benötigte Nachrichten können hier definiert werden. Für mehrere Nachrichten, die sich spezifisch auf eine Teilaufgabe (z.B. Personendetektion) beziehen, empfiehlt sich das Erstellen eines eigenständiges Pakets

[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_neo|cmr_neo]] beinhaltet zusätzlichen Code, der für die Neobotix MP-500 Plattform benötigt wird, beispielsweise um die Geschwindigket des Roboters  mittels Ultraschall- und Lasersensoren zu begrenzen.-->

### Social interaction
[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_speech|cmr_speech]]
enthält die Software zur Anbindung der Speech-to-Text, Text-to-Speech und Dialogfunktionalität des Roboters. Die Skripte kommunizieren mit den Google Cloud Diensten STT, TTS und Dialogflow.

[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_tablet_app|cmr_tablet_app]]
enthält das Android Projekt für die Tablet Applikation des Roboters. Das Tablet ist über rosjava mit dem ROS-System verbunden und versendet entsprechende Messages/Services zur Bedienung des Roboters.

[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_js_server|cmr_js_server]]
enthält die HTML Dokumente mit dem JavaScript Code zur Erzeugung der 3D Karte auf dem Roboterdisplay.

[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_motor_drivers|cmr_motor_drivers]]
enthält den Code zur Ansteuerung der Servos zur Bewegung der Ohren sowie der BLDC Motoren zur Bewegung der Arme.

[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_led|cmr_led]]
enthält den Code zur Ansteuerung des LED-Panels sowie der LED-Streifen in den Ohren und Armen des Roboters.

### Perception, localization and navigation
[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_localization|cmr_localization]] beinhaltet  ein Repo zur Lokalisierung des Roboters mittels RTAB-Map. Für genauere Informationen siehe die Einträge zu [[/cmrobot/rtab-map_slam|RTAB-Map]] und dem [[/cmrobot/map_manager|Map Manager]].

[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_lidarloop|cmr_lidarloop]] beinhaltet eine Erweiterung von RTAB-Map um Schleifenschlüsse basierend auf Laserscans vorzunehmen. Genauere Informationen finden sich unter [[/cmrobot/cmr_lidarloop|Schleifenschluss mit cmr_lidarloop]]. Der  Code basiert auf der [[https://seafile.projekt.uni-hannover.de/f/f4dbf9e8f4964dfc8f1b/|Masterarbeit von Tim-Lukas Habich]]


[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_people_perception|cmr_people_perception]]
enthält Pakete zur Personendeketion. Es wird hier auf verschiedene Algorithmen zur Detektion zurück gegriffen und das Tracking erfolgt über das SPENCER Framework. Der Code basiert auf der [[https://seafile.projekt.uni-hannover.de/f/9535e011d3cb4012a60f/|Masterarbeit von Alexander Petersen]].

[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_social_navigation|cmr_social_navigation]] beinhaltet verschiedene Plug-ins für die Costmap von Move Base, welche Kosten anhand von detektierten Personen erzeugen. Der Code basiert auf der [[https://seafile.projekt.uni-hannover.de/f/9535e011d3cb4012a60f/|Masterarbeit von Alexander Petersen]].

[[https://gitlab.projekt.uni-hannover.de/imes-projekt-cmg_roboter/cmr-code/cmr_pedsmin|cmr_pedsim]] ist ein Repo zur Simulation von getrackten Personen in vordefinierten Räumen und Bewegungen.
