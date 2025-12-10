**Robotics-Group39**

## Assistive Robot for Safe Navigation and Object Detection for Visually Impaired Individuals with Audio Feedback

### Overview

This project implements an assistive robot designed to help visually impaired individuals, capable of detecting objects with audio feedback to warn the user and guide them safely through a room. The system implemented combines different algorithms, SLAM for mapping and localisation, A* for optimal path, YOLO for object detection, object avoidance to avoid collision, and Text-to-Speech (TTS) for audio feedback, all into one integrated system.

### Objective 
The objective of this project is to assess how integrating SLAM, A* pathfinding, YOLO object detection, LiDAR-based obstacle avoidance, and Text-to-Speech (TTS) impacts the robot’s ability to safely travel and provide visually impaired individuals with helpful navigational information. The robot’s ability to avoid collisions in a simulated indoor environment, detection accuracy and navigation efficiency are all measures to evaluate the system. The project’s goal is to determine whether combining these components into a single framework makes navigation safer and more reliable.

#### *This has been implmented in [Webot Simulation](https://cyberbotics.com/)*
To view the environment and the fully integrated system controller:<br/>
- **World:** [Group39_apartment.wbt](https://github.com/Samm-006/Robotics-Group39/blob/main/worlds/Group39_apartment.wbt)<br/>
- **Controller:** [Group39_Integrated_System.py](https://github.com/Samm-006/Robotics-Group39/blob/main/controllers/Group39_Integrated_System.py)<br/>

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
- **Tarfah Almaghlouth - SLAM**<br/>
    Responsible for implementing Simultaneous Localisation and Mapping (SLAM) to generate a real-time map of the environment and track robot’s position during navigation.
- **Eesa Bazarwala – A Star Pathfinding**<br/>
    Developed the A* algorithm to compute an optimal path using SLAM output and moved the robot using the efficient path from start to goal state. 
- **Yixuan Zhu – YOLO Object Detection**<br/>
    Integrated YOLO with the robot’s camera to perform real-time object detection and classification within the indoor environment
- **Sama Alzahrani – TTS & Object Avoidance**<br/>
    Integrated the Text-to-Speech for audio feedback and implemented the object avoidance module using LiDAR to prevent collisions.



