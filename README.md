# ROS 2 and Turtlebot3 Exploration and Experiments

This repository contains my personal coursework, experiments, and projects for **CS 7785: Introduction to Robotics Research** at Georgia Tech.  
It is intended solely as a record of my own work and for future personal reference.

## 🛠️ Tech & Tools
- **Languages**: Python, C++
- **Frameworks**: ROS2 Humble, OpenCV, PyTorch/TensorFlow
- **Platforms**: TurtleBot, Gazebo, RQT

## 🚀 Goals
- Build practical skills in robotics research methods
- Explore algorithms for perception, planning, and control
- Document progress for reproducibility and future learning

## 📂 Labs

### Lab 2 – ROS2 Perception and Object Tracking (`lab2_ws`)
This lab focuses on building a ROS2 workspace for perception experiments using a webcam or TurtleBot3’s camera.  
The main objectives are:
- Learn how to publish and subscribe to image topics in ROS2
- Apply **OpenCV-based vision techniques** for detecting and tracking a colored object
- Experiment with **HSV color thresholding**, morphological filtering, and contour detection
- Publish processed images and detection results as ROS2 topics for debugging
- Integrate object detection with the TurtleBot3 simulation in **Gazebo**
- Practice basic actuation by rotating the robot programmatically

The workspace includes:
- **`find_object.py`** – Python node for detecting a chosen color object in real-time, overlaying a marker, and publishing pixel coordinates
- **`rotate_robot.py`** – Python node that demonstrates simple motion control by publishing twist commands to rotate the TurtleBot3
- Launch/config files to start the perception and control pipelines
- Notes and experiments on tuning HSV thresholds, masking, tracking stability, and robot response

This lab was designed to build foundational skills in both perception and actuation, setting the stage for more advanced tasks in navigation and autonomy.


## ⚠️ Academic Integrity
All content in this repository represents my own work as part of CS 7785.  
It is **not a solutions manual** and is **not to be copied or submitted** for credit in any form.  
If you are a current or future student, please respect the **Georgia Tech Honor Code** and use this repository only for inspiration, reference, or learning after completing your own work.
