This project creates a butler like robot which can perform various tasks in a hotel environment

Packages created and their functions :-
	Butler Bringup – Defines the required launch files to launch the gazebo simulation
	Butler Description – Contains the URDF description of the robot
	Butler Controller – Contains the controller code for robot (Currently Empty or Only for testing purposes)
	Butler Behavior – Behavior tree implementation for the robot


Behavior tree:-
	The Behavior trees.cpp library is used to model the behaviour of the robot, the following behavior tree is used for this robot.  

![image](https://github.com/user-attachments/assets/7bba123c-c84d-4852-850c-0d28ca391574)

To run this project :-
	Run,
    1. ros2 launch butler_bringup butler-bringup-launch.py to launch gazebo and robot
    2. ros2 lifecycle set /map_server activate to activate the map
    3. ros2 run butler_behaviour leaf_nodes to start the bt

![Screenshot from 2025-02-22 23-36-39](https://github.com/user-attachments/assets/266c92b3-c4f1-48aa-8761-444642b0436d)

