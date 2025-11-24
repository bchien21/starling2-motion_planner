# ModalAI Starling 2 Quadcopter Motion Planning With MoveIt Humble

This repository contains a set of packages for using MoveIt Humble for collision-free motion planning of the ModalAI Starling 2 Development Drone. The URDF description package was developed by [Rahul K.A](https://github.com/Rahul-K-A), a JHU Robotics Master's Student. 

### MoveIt Config Package

This configuration was generated using the [MoveIt Setup Assistant](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html). It contains launch files for the RViz MoveIt simulation. 

### Planner Package

The package "starling2-planner" contains nodes to generate obstacles and plan a collision-free trajectory using the OMPL library, which is wrapped by MoveIt. It communicates this information over to the RViz simulation to visualize the planned path. 

