Robotics-Group39

## Assistive Robot for Safe Navigation and Object Detection for Visually Impaired Individuals with Audio Feedback

### Overview

This project implements an assistive robot designed to help visually impaired individuals, capable of detecting objects with audio feedback to warn the user and guide them safely through a room. The system implemented combines different algorithms, SLAM for mapping and localisation, A* for optimal path, YOLO for object detection, and Text-to-Speech (TTS) for audio feedback, all into one integrated system.

### Objective 
The objective of this project is to assess how integrating SLAM, A* pathfinding, YOLO object detection, LiDAR-based obstacle avoidance, and Text-to-Speech (TTS) impacts the robot’s ability to safely travel and provide visually impaired individuals with helpful navigational information. The robot’s ability to avoid collisions in a simulated indoor environment, detection accuracy and navigation efficiency are all measures to evaluate the system. The project’s goal is to determine whether combining these components into a single framework makes navigation safer and more reliable.

### Pre-programmed packages:
- pyttsx3 
- opencv-python
- numpy
- pillow
- ultralytics
- matplotlib
- math
- copy
- heapdict

### Group 39:
- Tarfah Almaghlouth txa250@student.bham.ac.uk
- Eesa Bazarwala exb651@student.bham.ac.uk
- Yixuan Zhu yxz1919@student.bham.ac.uk
- Sama Alzahrani sxa1705@student.bham.ac.uk

### Team Roles:
- **Tarfah Almaghlouth - SLAM**

    Responsible for implementing Simultaneous Localisation and Mapping (SLAM) to generate a real-time map of the environment and track robot’s position during navigation.
- **Eesa Bazarwala – A Star Pathfinding**

    Developed the A* algorithm to compute an optimal path using SLAM output and moved the robot using the efficient path from start to goal state. 
- **Yixuan Zhu – YOLO Object Detection**

    Integrated YOLO with the robot’s camera to perform real-time object detection and classification within the indoor environment
- **Sama Alzahrani – TTS & Object Avoidance**

    Integrated the Text-to-Speech for audio feedback and implemented the object avoidance module using LiDAR to prevent collisions.



