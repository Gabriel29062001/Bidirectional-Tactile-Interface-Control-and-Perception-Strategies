# Bidirectional-Tactile-Interface-Control-and-Perception-Strategies


Overview

This package provides software and hardware components for a haptic joystick system. The system utilizes ROS2 for communication between different modules and includes functionality for controlling movement 
using encoders and force sensors interfaced with an Teensy microcontroller.

Contents

haptic_joystick : ROS2 Communication Package: Contains ROS2 nodes and message types for communication between different parts of the system.
haptic_joystick_arduino : Arduino Control Module: Handles reading and sending with encoders and force sensors .
Vicon_measurents : treat csv files from Vicon system with plot, error and delay computation


Installation

ROS2 haptic_joystick:
Clone the repository to your ROS2 workspace.
Build the package using colcon build.

Arduino Control Module:
Upload the provided Arduino sketch (main_mutual.ino or main_sensor) to your Arduino board.
Connect the encoders and force sensors to the Arduino according to the provided documentation.

Usage

haptic_joystick : ROS2 Communication Package:

Launch the ROS2 nodes using the provided launch 
files for simultenous launch of all modules (ros2 launch haptic_joystic mutual_mode.launch.py)
Launch the ROS2 nodes using the provided nodes files  from setup.py for launch one of the module and debugging (ros2 run haptic_joystic user_command_pub 0)
Configure the ROS2 parameters as needed for your setup:
- user_command_pub : 0 or 1  depending on the module you want to control
- mapping : 0 for the direction module 0 to module 1 and 1 for the inverse direction to control
- talker, listener enable to send topic across computers via network and IP

Arduino Control Module:

Ensure that the Arduino is powered and connected to the system.
Start the Arduino sketch, which will begin reading data from the encoders and force sensors.
Configuration
Adjust ROS2 parameters to customize behavior, such as communication frequencies and sensor calibration.
Modify the Arduino sketch to accommodate different hardware configurations or communication protocols.
In particulary if you want to change the calibration value of load sensors at top bottom

Troubleshooting

Teensy port :
If you are not sure about serial number of Teensy, run port_recognition.py in haptic_joystic,
then change in the user_command_pub the teensy_serial_number.

Encoders :
If you have problems with encoders value: 
- check that encoders and legs are still connected mechanicaly correctly,
- check wires connexion
- check pins correspondances with  Arduino files, it might have some changes specially between Teensy and Arduino numeration

Motors :
- check wires connexion
- check pins correspondances with  Arduino files, it might have some changes specially between Teensy and Arduino numeration

Load sensors :
- check the wires connexion, if there is no more change in value from one encoder, its probably a wires connexion problem,
with the green connector. You can check with the Sparkfun code with Basics Readings codes.

MUX:
- if one of the load sensor is not even detected, check that you correctly connect the load cell to the mux ports
corresponding to Arduino code (myMux.setPorts())

ROS2 Node
If encountering issues with ROS2 communication, ensure that all nodes are properly launched and configured.
Check the Arduino connections and sensor readings if experiencing problems with hardware functionality.


Contact
For any inquiries or support, please contact gabriel.paffi@epfl.ch
