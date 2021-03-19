---
title: Map Manager
parent: Software
has_children: true
nav_order: 2
---

# Map Manager
<iframe width="560" height="315" src="https://www.youtube.com/embed/fBbt1CZmmzU" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

In this section the software _Map Manager_ is presented, which was developed in the context of a master thesis at IMES. With the Map Manager, several maps can be created and connected to each other at certain points (so-called link points).

Since Sobi has to operate in different environments on campus (e.g. indoor in corridors and entrance halls, in elevators or outdoor between the institute buildings) it is necessary to create several maps between which the robot can switch. This not only reduces the size of the individual maps and thus improves the localization and the resource utilization of the computer, but also allows to use different sensor configurations for different environments .

The chapter is divided into the following sections:

1. [Basic principle of the Map Manager](mm_basic_principle.html)

2. [Mapping with the Map Manager on Sobi](mm_mapping.html)

3. [Automated mapping with the map manager](mm_auto_mapping.html)

4. [Saving the maps in .pgm and .yaml](mm_save_maps.html)

5. [Visualization in Rviz](mm_visualization.html)

6. [Troubleshooting](mm_troubleshooting.html)

## Basic principle of the Map Manager

Grundsätzlich ist der Map Manager dem SLAM-Verfahren (in diesem Fall RTAB-MAP) übergeordnet. Der Algorithmus, verwaltet von RTAB-MAP erstellte Maps und  verbindet diese durch Link-Points. So kann gewährleistet werden, dass der Roboter später über mehrere Maps planen und navigieren kann. Die Unterteilung des Einsatzbereiches des Roboters in mehrere Karten hat den Vorteil, dass der Roboter unterschiedliche SLAM-Konfigurationen für unterschiedliche Umgebungen (z.B. indoor/outdoor) nutzen kann. Weiterhin verbessern kleinere Karten die Map Qualität, da RTAB-MAP weniger Nodes in dem Long-Term-Memory (LTM) hat und somit bereits eine Vorauswahl an Nodes, die für Loop-Closures in Frage kommen, getätigt wird.
Im Folgenden wird das Grundprinzip und die Funktionsweise des Map Managers erklärt. Zuerst wird auf die topologische Graph Struktur eingegangen, die aus der Anordnung der Karten und der Link-Points abstrahiert werden kann. Die Tor-Detektion und anschließende Auswahl der optimalen SLAM-Konfiguration wird ebenfalls beschrieben. Anschließend wird die Detektion von topologischen Loop-Closures durch Feature-Matching erklärt. Abschließend wird das Tracking der globalen Roboterposition beschrieben und die Optimierung des topologischen Graphen nach einem Loop-Closure.

## Mapping with the Map Manager on Sobi

[ ![GrundprinzipDesMapManagers_Bild1](/Sobi/images/GrundprinzipDesMapManagers_Bild1.png) ](/Sobi/images/GrundprinzipDesMapManagers_Bild1.png)

###### Abb. 1: Grundprinzip von dem Map-Manager mit mehreren Maps, die durch Link-Points verbunden sind, welche in dem Überlappungsbereich von zwei Maps liegen.

Jede Map repräsentiert einen Raum bzw. Outdoor-Bereich, die durch Türen voneinander getrennt sind. In Abb. 1 sind vier Maps (zwei indoor und zwei outdoor) präsentiert. Die einzelnen Maps sind durch farbig gestrichelte Linien gekennzeichnet. Für jede Map sind die Koordinatensysteme (CF) eingetragen. Die Link-Points sind durch Kreise in dem Überlappungsbereich von zwei Karten vor oder hinter der Tür gekennzeichnet. Die Link-Points werden durch Edges (farbige Pfeile) verbunden. Link-Points werden nur durch Edges verbunden, wenn sie sich auf der gleichen Karte befinden. Die Farbe der Pfeile stimmt mit der jeweiligen Farbe der Karte überein. Der gefahrene Roboter-Pfad für das Mapping ist durch die gestrichelte orange Linie gekennzeichnet.

Der Ablauf des Mappings gestaltet sich für dieses Beispiel wie folgt (die hier beschriebenen Methoden zum Feature-Matching und zur Tür-Detektion werden später noch genauer erläutert) :
Der Start ist auf Map 0 nachdem der Roboter über Laser-Scan Daten erkannt hat, dass die Indoor-Konfiguration (kleine Distanzen) für RTAB-MAP verwendet werden soll. Anschließend wird vor eine Tür gefahren und ein **Link-Point Candidate ** gesetzt, der später für das Loop-Closure wichtig ist und zu dem Link-Point L_3 wird, sobald der Roboter dort wieder von der anderen Map (3) durch die Tür fährt. Anschließend fährt der Roboter durch die Tür auf der anderen Seite des Raums.  Der Roboter erkennt, dass die Tür passiert wurde und beendet die alte Map 0 hinter der Tür. Die Umgebung wird durch die Laser-Scan Daten geprüft und die Outdoor-Konfiguration genutzt. Anschließend wird die Kartierung für eine neue Karte gestartet (Map 1). In dem Link-Point 1 haben also beide Karten eine Überschneidung. Der Roboter fährt über Map 1 und nachdem er die Tür passiert hat, beendet er Map 1 und prüft auf Basis der Kamera-Daten, ob der an diesem Ort bereits gewesen ist (bzw. ob dieser Link-Point bereits existiert) oder nicht. Anschließend beginnt er eine neue Karte in der Indoor-Konfiguration. Ein weiterer Link-Point Candidate wird gesetzt und darauffolgend wird die Tür in Richtung Map 3 passiert.  Link-Point 2 und Map 3 werden ebenfalls neu angelegt. Sobald der Roboter die Tür von Map 3 zu Map 0 durchfährt, wir dies automatisch erkannt und geprüft, ob der Roboter an diesem Ort bereits war. Da das Feature-Matching mit dem am Anfang gesetzten Link-Point Candidate eine hohe Übereinstimmung liefert, wird der Link-Point Candidate in einen Link-Point (L_3) umgewandelt. Weiterhin wird keine neue Karte begonnen, sondern wieder die alte Karte (Map 0) geladen. Nachdem der Roboter die Tür Richtung Map 2 durchfahren hat, wir ebenfalls der Link-Point Candidate in einen richtigen Link-Point umgewandelt, sodass der Topologische Graph nun vollständig ist.

### Tür-Detektion und Auswahl der SLAM-Konfiguration:

Um die Tür zu detektieren, werden die Laser-Scan Daten des Velodyne VLP-16 genutzt. Dazu wird die PointCloud2 zu Laser-Scan Daten (eine 2D-Projektion) mit dem ROS-Package [[ http://wiki.ros.org/pointcloud_to_laserscan | pointcloud_to_laserscan]] konvertiert. Die Laser-Scan Daten (Range) werden anschließend nach dem Winkel abgeleitet. Um Türen zu detektieren, wird nach großen Ableitungen in den Laser-Scan Daten in einem bestimmten Winkel und in einer bestimmten Distanz gesucht. Damit die Ableitung entsprechend groß wird, muss die Tür geöffnet sein.

[ ![GrundprinzipDesMapManagers_Bild2](/Sobi/images/GrundprinzipDesMapManagers_Bild2.png) ](/Sobi/images/GrundprinzipDesMapManagers_Bild2.png)

###### Abb. 2: Kriterien für die Tür-Detektion: Die geöffnete Tür verursacht hohe Ableitungen an der Position der Türpfosten. Dies kann sowohl von vorne als auch von hinten detektiert werden. Außerdem wird die Veränderung (Ableitung) der maximalen und minimalen Range-Data genutzt, um die Veränderung der Umgebung zu detektieren.

Für die zuverlässige Detektion einer Tür-Durchquerung gibt es drei Kriterien, von denen mindestens zwei erfüllt sein Müssen, damit die Tür detektiert wird. Das erste Kriterium ist die Detektion der Tür von vorne durch Analyse der Ableitungen der Laser-Scan Daten (Abb. 2 a). Das zweite Kriterium ist die selbe Detektion der Tür von hinten, also nachdem der Roboter durch gefahren ist  (Abb. 2 b). Wenn diese beiden Kriterien erfüllt sind, ist die Tür erfolgreich detektiert. Allerdings gibt es Türen, bei denen eine Wand im Rechten Winkel nah an dem Türrahmen steht, sodass die Ableitungen an dieser Stelle sehr klein werden. Dazu wurde das dritte Kriterium entwickelt: Die Laser-Scan Daten werden Tiefpass-gefiltert und anschließend eine imaginäre Bounding Box herumgelegt (min/max-Werte in x- und y-Richtung bestimmen). Die Änderung der Bounding Box wird über die Ableitung berechnet. Ist die Änderung sehr groß, deutet das darauf hin, dass man durch eine Tür in einen anderen Raum oder Outdoor-Umgebung gefahren ist. Dieses Kriterium kann also (höchstens) eine Tür-Detektion von vorne oder hinten ersetzen.

Für die Ermittlung der SLAM-Konfiguration für die neue Map werden ebenfalls Tiefpass-gefilterte Laser-Scan Daten verwendet. Wenn die Maximalen Abstände eine gewisse Grenze überschreiten, wird die Konfiguration für große Distanzen (outdoor) verwendet. Es gibt auch noch andere Kriterien, die Verwendet werden können und über das Config-File eingeschaltet  werden. Mögliche Kriterien sind noch das Rauschen der Tiefenbilder der Realsense-Kameras zu messen durch PSNR (Peak Signal-Noise-Ratio) oder SSIM (Structural Similarity). Ist das Rauschen zu stark (z.B. wegen zu hohen Distanzen) wir die Outdoor-Config genutzt. Außerdem ist ein CNN implementiert (ResNet18 basierend auf dem Places365 dataset), um die aufgenommene Szene der Front-Kamera in entweder //indoor// oder //outdoor// zu klassifizieren.

### Loop-Closure Detektion durch Feature-Matching:

damit der Roboter erkennen kann, ob der Link-Point bereits existiert, ein neuer Link-Point angelegt werden muss oder ein Loop-Closure mit einen vorhandenen Link-Point Candidate vorhanden ist, müssen die aktuellen Kamera-Bilder mit dem Kamera-Bild, welches in dem Link-Point (Candidate) gespeichert ist, verglichen werden. Dieser Vergleich erfolgt durch Feature-Matching mit SIFT-Features (siehe Abb. 3).

[ ![GrundprinzipDesMapManagers_Bild3](/Sobi/images/GrundprinzipDesMapManagers_Bild3.png) ](/Sobi/images/GrundprinzipDesMapManagers_Bild3.png)

###### Abb. 3: Feature-Matching zwischen einem vorher aufgenommenen Bild mit der Rück-Kamera (daher gedreht) von einem Link-Point Candidate und mit dem aktuellen Front-Kamera Bild des Roboters. Obwohl die Bilder verdreht sind, findet der Algorithmus viele Matches.

Wenn die Anzahl der übereinstimmenden Features (Matches) einen gewissen Grenzwert überschreitet, wird der Loop-Closure akzeptiert. Um robust gegen Bildrauschen zu sein, wird eine Bilder-Serie zum Feature-Matching verwendet. Der Median wird dann zu Bewertung der Ähnlichkeit herangezogen.  Zusätzlich wird bei der Schließung eines Loop-Closures mit einem Link-Point Candidate eine ICP-Registrierung zwischen der im Link-Point Candidate gespeicherten Point Cloud und der  aktuellen Point Cloud durchgeführt. Zum einen erhöht es die Sicherheit, dass kein falscher Loop-Closure erkannt wird, zum anderen lässt sich damit die aktuelle Position des Roboters im alten Map-Koordinatensystem berechnen, was für die Erstellung des Link-Points wichtig ist.

### Globales Positions-Tracking und topologische Graph-Optimierung

Bisher hat der Roboter die aktuellen Kamera-Bilder mit allen vorhandenen Link-Points und Link-Point Candidates verglichen. Dies kostet Zeit und ist ineffizient. Um die Effizient zu steigern, kann die globale Position des Roboters über mehrere Karten berechnet werden und nur die Link-Point (Candidates) in betracht ziehen, die sich innerhalb eines Suchradius um den Roboter befinden.
Um die globale Roboter-Position zu berechnen, müssen die relativen Positionen der Karten zueinander bekannt sein. Dies kann durch die Link-Points ausgerechnet werden, da sie die gleiche Stelle im globalen Koordinatensystem in den jeweiligen Koordinatensystemen der angrenzenden Karten beschreiben. Es kann also eine Transformation zwischen den beiden Ursprüngen der angrenzenden Karten berechnet werden. Der kürzeste topologische Pfad zwischen dem Welt-KS und dem Map-Ursprung in welcher die Roboter-Pose berechnet wird, kann durch den Dijkstra Algorithmus ermittelt werden.  

[ ![GrundprinzipDesMapManagers_Bild4](/Sobi/images/GrundprinzipDesMapManagers_Bild4.png) ](/Sobi/images/GrundprinzipDesMapManagers_Bild4.png)

###### Abb. 4: Globales Tracking der Roboter-Pose durch Verkettung von Transformationen. Da die relativen Transformationen mit einer Unsicherheit (Kovarianz) behaftet sind, vergrößert sich diese, je mehr Transformationen beteiligt sind (blauer Kreis). Diese Abbildung stellt einen Mapping-Status von einem früheren Zeitpunkt verglichen mit Abb. 2 dar.

In Abb. 4 wird die Verkettung von mehreren Transformationen und die damit verbundene Varianz-Fortpflanzung in der Position veranschaulicht. Da die Unsicherheit in der Position ansteigt, wird auch der Suchradius größer. In dem Beispiel liegt nur Link-Point Candidate 0 im Suchradius und wird somit als einzige Option zur Loop-Closure Detektion herangezogen. Die initiale Unsicherheit einer Position wird durch eine Schätzung (Varianz in X,Y und Theta) angegeben. Die Varianz-Fortpflanzung in den Posen wir mit Hilfe des ROS-Package [[ http://wiki.ros.org/pose_cov_ops | pose_cov_ops ]] berechnet.

Nachdem ein neuer Loop-Closure gefunden wurde, ist es möglich, die zusätzliche Topologische Information zu nutzen, um den Graphen hinsichtlich der globalen Positionen zu optimieren. Dies wird mit der C++ Library [[ https://openslam-org.github.io/g2o.html | g2o (A General Framework for Graph Optimization) ]] durchgeführt. Dadurch werden die globalen Positionen der Link-Points korrigiert und somit der Fehler durch die Varianz-Fortpflanzung verringert

##  Automated mapping with the Map Manager at CMR

  This article describes how automated mapping works with the Map Manager on the CMR. In the automated variant, the passing of doors is automatically detected and the link points and link point candidates are automatically set by the algorithm. The state machine for the automated mapping can be found in [[ https://phabricator.imes.uni-hannover.de/source/cmr_localization/browse/ehlers/map_manager/src/automated_sm_mapping.cpp | automated_sm_mapping.cpp ]] in the package ''map_manager''. The parameters for the map manager can be set in [[ https://phabricator.imes.uni-hannover.de/source/cmr_localization/browse/ehlers/map_manager/cfg/params.yaml | params.yaml ]].

  To start the automated mapping on the CMR the following steps have to be executed in the console:


    * ''start roscore'' (CMR).

    * ''bringup'' (base)

    * ''roslaunch map_manager sensor_model_full.launch'' (CMR) Launch the sensor and URDF model.

    * ''roslaunch map_manager utils_map_manager.launch'' (CMR) Launch the node pointcloud_to_laserscan as well as the CNN for scene classification (indoor or outdoor).

    * ''rosrun map_manager rtabmap_starter'' (CMR).

    * ''rosrun map_manager environment_detector'' (CMR) Starts the Environment Detector: detects the passage of doors and the changes from the environment by laser scan data. Automatically sets the link points and link point candidates and also automatically detects the loop closures. This node replaces the manual invocation of Ros services in non-automated mapping.

    * ''roslaunch map_manager automated_sm_mapping.launch'' (CMR) Starts State Machine and loads config parameters from .yaml file.

### State Machine automated mapping:

  State Machine states:

    * ''Initialize'': Initialize the State Machine: initialize important parameters; query the Working Directory; create from the SQL database where information about Link-Points, Link-Point Candidates and Maps are stored.

    * ''Start_Mapping'': State to start mapping: checks which SLAM configuration is optimal for the environment; sends message to ''rtabmap_starter'' to start RTAB map; calculates shortest topological path to the map the robot is currently on to initialize the ''Global Tracker''. Also starts Environment Detector via ROS message, which does not run if not mapped.

    * ''Mapping:'' State during mapping: here an initial pose is set at the beginning (if necessary); Continuously calling the ''Global Tracker'' to publish the Global Pose. During this state, the Environment Detector detects doors as they are passed through and changes in the environment. The Environment Detector exits the ''Mapping'' state as soon as the passage of a door is detected and a link point is stored or an already known link point is detected (by a loop closure).

    * ''Kill_mapping_mode'': Stops RTAB-MAP by killing the ROS nodes.

    * ''Main_Menu'': Main menu with choices: Record new map; Mapping on old map; Delete Map; Delete Link-Point; Abort.

  During the use one is led by text outputs by the program.

### Notes for the use of the Automated Map Manager:


    * To set link points, the robot must be moved in straight alignment in front of the open door. The door must be detected (to be on the safe side) three times in a row (happens automatically within about 2 seconds). Then the link point candidate is saved and the mapping can be continued normally.

    * In order for the Environment Detector to detect the passage of a door, the robot should be driven slowly and straight through the door so that the derivations at the door jambs are properly detected. For some doors, detection can be problematic if the door is made of glass (discharges may still be detected on the frame) or if walls are directly adjacent to the door, making the discharges very small. If the door could not be detected properly with several attempts, the (manual) state machine must be consulted.

    * For the doors to be detected, they must be open and no object must be directly in front of or behind the door.

### Parameters for Automated State Machine Mapping:

  The following parameters can be specified in [[ https://phabricator.imes.uni-hannover.de/source/cmr_localization/browse/ehlers/map_manager/cfg/params.yaml | params.yaml ]].

    * ''config_criterion'': criterion according to which the SLAM configuration should be determined automatically.

    * ''range_thr'': limit value for distance of laser scan data, from which the configuration should be used for high distances.

    * ''ssim_thr'' and ''psnr_thr'': limit values (PSNR and SSIM) to evaluate the noise of the cameras. If the noise in the depth image is too high, the depth image is calculated from the Velodyne data.

    * ''initial_variance_x'': Initial variance in the X position of the robot. Important to calculate the variance propagation in the global tracker.

    * ''initial_variance_y'': Initial variance in the Y position of the robot. Important to calculate the variance propagation at the global tracker.

    * ''initial_variance_theta'': Initial variance in the rotation of the robot. Important to calculate the variance propagation at the global tracker.

    * ''num_depth_imgs'': Number of depth images to be added for image noise detection.  

### Parameters for the Environment Detector:.

  The following parameters can be specified in [[ https://phabricator.imes.uni-hannover.de/source/cmr_localization/browse/ehlers/map_manager/cfg/params.yaml | params.yaml ]].

    * ''feature_matching_thr'': number of feature matches required to accept a loop closure.

    * ''num_last_imgs:'' number of images taken to detect feature matching with the stored image. When multiple images are taken, the feature matching becomes more robust against image noise, but it also increases the computation time.

    * ''NNDR_akaze'': Nearest neighbor distance ratio (NNDR) for AKAZE features.

    * ''NNDR_sift'': Nearest neighbor distance ratio (NNDR) for SIFT features.

    * ''icp_fitness_score_thr'': fitness score for ICP registration. If the fitness score is too large, i.e. the point clouds are too different in position and shape, the loop closure will not be accepted.

    * ''deriv_door_thr'': limit value for the derivation necessary to detect the door posts.

    * ''time_limit_door_detection'': time limit for the complete detection of a door. the timer starts as soon as the door has been detected from the front. If the door has not been completely detected after this time, the door detection will be reset.

    * ''deriv_range_thr'': threshold how high the bounding box derivative of the low pass filtered laser scan data must be to detect an environmental change by laser scan data (third criterion).

    * ''low_pass_filter_size_env'': filter size for the average low-pass filter for the laser scan data.


## Saving the maps in .pgm and .yaml

    This article describes how to save the corresponding .pgm and .yaml files based on the SQL databases created by RTAB-Map. In this procedure the conversion of the data has been automated. In principle, each individual database can also be manually converted to a .pgm and .yaml as described [[ https://answers.ros.org/question/217097/export-2d-map-from-rviz-andor-rtab-map/ | here]].

    To store the .pgm and .yaml automatically for each map in the working directory (must be specified at the beginning), the following must be done on the CMR:

      * ''roscore'' (CMR).

      * ''rosrun map_manager save_map'': Iterates through all maps present in the database and calls the ''map_saver'' through a ROS message. Additionally starts ''rtabmap_ros'' with the ''rtabmap_starter'' by sending a ROS message.

      * ''rosrun map_manager map_saver_starter'': starts the ''map_saver'' as soon as the signal comes from the node ''save_map''.

      * ''rosrun map_manager rtabmap_starter'': Starts ''rtabmap_ros'' as soon as the command from ''save_map'' comes.

    First start all nodes. Then enter the directory at the ''save_map'' node and wait until "//DONE, saved all maps!//" is displayed.
    After the ROS node ''save_map'' has iterated through all maps, all nodes can be turned off. Unfortunately, the commands have to be swapped out to three different nodes, otherwise automation in the code would not be possible. The saved .pgm and .yaml files can be found in the same directory as the SQL databases of RTAB-MAP.


## Visualization in Rviz

    With the help of the ROS node ''rvizgraph'' link points can be visualized in Rviz. To do this, the map manager must be running in mapping mode (as described in 3) and 4)). The node can be started by the command ''rosrun map_manager rvizgraph''. It gets the current map ID via the ROS topic ''/map_manager/cur_map_id''. This is necessary because the link points that are on the current map should be displayed in the map coordinate system. All other link points are represented in the world coordinate system, which is not as accurate.

    Therefore, the ROS node sends ''visualization_msgs::Marker'' on the topic ''map_manager/local_linkpoints_rviz'' or ''map_manager/global_linkpoints_rviz''. These markers can be easily added in Rviz. It is important to note that the markers can only be seen if the coordinate systems ''map'' and ''world'' also exist, i.e. if RTAB-MAP and the state machine for the mapping are running properly.


## Troubleshooting

    Here is a collection of problems that can occur in connection with the Map Manager and RTAB-MAP. This list will be extended continuously.



      * **RTAB-MAP does not get data (yellow warning appears):** As soon as RTAB-MAP does not get a required topic, RTAB-MAP as a whole does not work. Check if all topics with the appropriate names are published and available. Otherwise, check if the sensor drivers need to be restarted or the nodes ''rgbd_sync'' or ''register_velodyne'' are working properly.

      * **RTAB-MAP does not find loop closures**: Has the correct SLAM configuration been used? Has the same (or similar) path also been traversed in the correct orientation? Does the sensor system provide all required data? Has the appearance of the environment changed drastically (lighting conditions or objects that were not there before)?

      * **The Environment Detector does not detect the door:** Has the door been passed through straight and slowly? Is no one standing in the door, or can large deflections be formed? Is there a wall adjacent to the door so that the rejection becomes small? With glass doors the detection can be faulty. Sometimes it helps to restart the Environment Detector. If you can't get the problems under control, you have to use the State Machine for manual mapping.

      * **The Global Tracker publishes an incorrect global pose:** Make sure that the link points are not stored redundantly and that the topological graph is otherwise intact. Errors in the topological graph will also cause errors in the global tracker.

      * **No loop closure is detected by feature matching:** Have lighting conditions or the scene changed? Is the link point or link point candidate in the search radius of the robot (only relevant for automated mapping)?



### Citing
If you use this software for your research, please cite the following publication:
```
Map Management Approach for SLAM in Large-Scale Indoor and Outdoor Areas
Ehlers, S., Stuede, M., Nuelle, K., Ortmaier, T.
IEEE International Conference on Robotics and Automation (ICRA) 2020
```
