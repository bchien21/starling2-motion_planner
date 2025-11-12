# ModalAI Starling 2 Max Motion Planning Simulation 

This repository contains a set of packages for using MoveIt Humble for collision-free motion planning of the ModalAI Starling 2 Development Drone. The URDF description package "starling2-description" was developed by a fellow JHU Robotics 
student named [Rahul KA](https://github.com/Rahul-K-A). 


## MoveIt Config Package

The package "starling2-moveit-config" was generated using the [MoveIt Setup Assistant](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html) and the URDF file from the description package. It contains 
launch files to launch the Rviz simulation. 

## Planner Package

The package "starling2-planner" contains nodes within the src folder that, when ran with "ros2 run", generates obstacles in the Rviz scene, plans a collision-free path for the quadcopter through those obstacles, and displays a visual of this path in Rviz. 

