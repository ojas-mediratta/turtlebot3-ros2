# ROS 2 and TurtleBot3 — Coursework, Projects & Experiments

This repository contains my personal coursework, experiments, and research explorations with the TurtleBot3 platform under **ROS 2**, primarily for **CS 7785: Introduction to Robotics Research** at Georgia Tech.  
It is intended as a personal record and reference, **not** a solutions repository.

---

## 🛠️ Tech & Tools

- **Languages**: Python, C++
- **Frameworks / Libraries**: ROS 2 Humble (or compatible), OpenCV, PyTorch / TensorFlow  
- **Platforms / Simulators**: TurtleBot3 (real or simulated), Gazebo, RQT / RViz

---

## 🚀 Goals & Themes

- Build hands-on skills in robotics research workflows  
- Explore algorithms spanning perception, planning, and control  
- Maintain reproducible workflows and documented experiments  

---

## 📂 Repository Layout & Lab Overviews

- `lab2_ws/` — ROS2 workspace and code for Lab 2 experiments  
- `lab3_ws/` — ROS2 workspace and code for Lab 3  
- `lab4_ws/` — ROS2 workspace and code for Lab 4  
- `README.md` — this file  

---

<details>
<summary><b>Lab 2 – Perception & Object Tracking (`lab2_ws`)</b></summary>

**Purpose**: Set up a ROS2 perception pipeline to detect and track a colored object, publish detection outputs, and link to simple robot motion.

**Key files and modules:**
- `find_object.py` — Subscribes to an image topic, applies HSV thresholding and contour detection, and publishes both processed images and object coordinates.  
- `rotate_robot.py` — Publishes `Twist` commands to `/cmd_vel` to rotate the robot for verification.  
- Launch/config files — Bring up camera, perception, and control nodes with adjustable thresholds.

</details>

---

<details>
<summary><b>Lab 3 – Sensor Fusion, PID Control & Object Chasing (`lab3_ws`)</b></summary>

**Purpose**: Combine camera and LIDAR sensing for a closed-loop control system that chases a detected object.  

**Key files and modules:**
- **Vision / Detection Node** — Extracts the angular bearing of a tracked object.  
- **Range / Scan Node** — Processes `/scan` data to determine object distance.  
- **Chasing Controller** — Implements cascaded PIDs for angular and distance control, publishing `Twist` messages to `/cmd_vel`.  
- Launch/config files — Integrate all nodes with tuned parameters for stability and response.

</details>

---

<details>
<summary><b>Lab 4 – Go-to-Goal with Obstacle Avoidance (`lab4_ws`)</b></summary>

**Purpose**: Develop autonomous navigation behaviors using odometry and LIDAR data to reach a goal position while avoiding obstacles.

**Key files and modules:**
- `get_object_range.py` — Subscribes to `/scan`, detects nearby obstacles, and publishes a vector to the closest object as `/obstacle_vector` (`geometry_msgs/Vector3`).  
- `go_to_goal.py` — Subscribes to `/odom` and `/obstacle_vector`, computes velocity commands for goal-directed navigation with reactive obstacle avoidance, and publishes `/cmd_vel` (`geometry_msgs/Twist`).  
- `turtlebot3_bringup.launch.py` — Initializes onboard drivers and sensors, including `/scan` and `/odom` publishers.  
- Launch configuration — Integrates all nodes into a single pipeline for autonomous navigation.  
- See the `lab4_ws/README.md` for the full computational diagram.

</details>

---

## ⚠️ Academic Integrity & Usage

Everything in this repository is **my original work** for CS 7785.  
This is **not** a solutions manual and should **not** be used for direct submission in any course.  
If you are a student, please follow the **Georgia Tech Honor Code** — use this repository *only* for learning, reference **after** you’ve done your own work, or inspiration (not copy-paste).
