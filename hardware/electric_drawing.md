---
title: Electrical diagrams
has_children: true
parent: Devices and components
nav_order: 1
---

# Electrical circuit diagrams
This page provides information about the circuit diagrams of the different components. Diagrams are provided as PDF and, in the case of custom developed components, EAGLE files.

## Interconnection of the base with custom components
The mobile robot base is a commercial product, the Neobotix MP-500. Detailed electrical drawings of this components are not available for legal reasons. However, the available plans show the interconnection to the base via two connectors that were individually ordered.
The diagram for the individually produced MP-500 can be downloaded here:
[ESP-MP-500-Extension.pdf](/Sobi/download/ESP-MP-500-Extension.pdf)

The plan contains the connectors **X60** and **X61**, which represent the interface to the platform. The connector **X60** contains wiring for +24V, +12V, +5V and CAN Bus.
[ ![x60x61](/Sobi/images/X60X61.png) ](/Sobi/images/X60X61.png)
The **X61** connector has a line that is directly connected to the batteries and is used to extend the capacity by two batteries mounted on top of the mobile base.

As shown in the PDF, **X60** is connected to the Ultrasonic Board. It is also connected to a custom board, which provides power for e.g. the embedded computer (See second PDF)

## Power supply of custom components
The several components of the robot require different voltages. Therefore, there are two converter boards which contain 24V/12V and 24V/5V converters. The first board is powered by the +24V line of the **X60** connector of the base. When the mobile base is shut off, there is no voltage on the lines on the **X60** connector. The board therefore also drives a relay which shuts off the second converter board. Otherwise, the second board would not shut off because it is directly connected to the battery voltage.
The circuit diagram which shows the connection of the different components can be downloaded here
[ESP-MP-500-Sobi.pdf](/Sobi/download/ESP-MP-500-Sobi.pdf).

The first converter board, connected to **X60** is shown on page 3. The second converter board is shown on page 5.

## EAGLE files
The following EAGLE files are contained in a zip file, with schematics that can be used for production:
- **Sobi_Conv_Board_Base**: First converter board, connected to **X60**
- **Sobi_Conv_Board_Torso**: Second converter board, connected to **X61**
- **Sob_Raspi_LED_Fan_Shield**: A shield for the Raspberry Pi 3 that holds a connector for the LED Panel and the two Servos that move the Ears

Download the EAGLE files here:
[EAGLE_Files.zip](/Sobi/download/ESP-MP-500-Sobi.pdf)
