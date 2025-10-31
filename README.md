# ROS 2 and TurtleBot3 — Coursework, Projects & Experiments

This repository contains my personal coursework, experiments, and research explorations with the **TurtleBot3** platform under **ROS 2**, primarily for **CS 7785: Introduction to Robotics Research** at Georgia Tech.  
It is intended as a personal record and reference, **not** a solutions repository.

---

## 🛠️ Tech & Tools

- **Languages:** Python, C++  
- **Frameworks / Libraries:** ROS 2 Humble, OpenCV, Nav2, PyTorch / TensorFlow  
- **Platforms / Simulators:** TurtleBot3 (Burger / Waffle Pi), Gazebo, RViz, RQT  

---

## 🚀 Goals & Themes

- Build hands-on proficiency in ROS 2 architecture and robotics workflows  
- Explore the integration of **perception**, **control**, **mapping**, and **planning**  
- Maintain clear documentation and reproducible experiments for future research  

---

## 📂 Repository Layout & Lab Overviews

- `lab2_ws/` — Perception & Object Tracking  
- `lab3_ws/` — Sensor Fusion & PID Chasing  
- `lab4_ws/` — Go-to-Goal & Obstacle Avoidance  
- `lab5_ws/` — Mapping, Localization & Waypoint Navigation  
- `README.md` — this file  

---

<details>
<summary><b>Lab 2 – Perception & Object Tracking (`lab2_ws`)</b></summary>

**Purpose:** Establish a ROS 2 perception pipeline for detecting and tracking colored objects via camera input.

**Highlights:**
- Implemented **HSV-based segmentation** and contour detection using OpenCV.  
- Published centroid data and processed image streams to ROS topics.  
- Integrated **teleoperation and automated motion testing** using custom `Twist` publishers.  
- Developed modular launch files for quick sensor and node bring-up.

</details>

---

<details>
<summary><b>Lab 3 – Sensor Fusion & PID Object Chasing (`lab3_ws`)</b></summary>

**Purpose:** Fuse visual and LIDAR data for a cascaded PID controller that maintains distance and orientation to a moving object.

**Highlights:**
- Extracted angular bearing from vision node and range data from `/scan`.  
- Designed **cascaded proportional controllers** for angular alignment and linear velocity.  
- Demonstrated smooth pursuit behavior in both Gazebo and on real TurtleBot3 hardware.  
- Introduced feedback tuning methods and discrete-time controller analysis.

</details>

---

<details>
<summary><b>Lab 4 – Go-to-Goal with Reactive Obstacle Avoidance (`lab4_ws`)</b></summary>

**Purpose:** Develop autonomous goal navigation using odometry and LIDAR to reach a target pose while avoiding obstacles.

**Highlights:**
- Created an **obstacle-vector generator** from `/scan` data and integrated it with odometry feedback.  
- Implemented a **hybrid control policy** combining goal attraction and obstacle repulsion.  
- Added runtime parameterization via YAML config and dynamic re-launching for tuning.  
- Validated algorithm performance through real-time RViz visualization and motion logs.

</details>

---

<details>
<summary><b>Lab 5 – Mapping, Localization & Waypoint Navigation (`lab5_ws`)</b></summary>

**Purpose:** Integrate **SLAM, localization, and autonomous global navigation** using the **ROS 2 Nav2** stack. This lab transitions from custom controllers to a full-featured navigation framework, emphasizing configuration, tuning, and system-level understanding.

**Highlights:**
- **Mapping (SLAM):** Used `slam_toolbox` and teleoperation to generate occupancy maps in both real and simulated environments.  
- **Localization:** Configured **AMCL** (Adaptive Monte Carlo Localization) for robust pose estimation using laser scans and odometry.  
- **Path Planning:** Implemented **A\*** and **Dijkstra-based** global planning with **DWB** and **Pure Pursuit** local controllers, fine-tuned via YAML parameters.  
- **Waypoint Automation:** Developed a ROS 2 node that publishes `PoseStamped` goals to the `/goal_pose` topic, sequencing multiple global waypoints.  
- **Gazebo Testing:** Executed navigation in a custom maze environment with static and dynamic obstacles; verified real-world transferability.  
- **Parameter Tuning:** Adjusted costmap resolution, inflation radius, and control frequency for stable navigation performance.  
- **Evaluation:** Demonstrated full autonomous traversal between three arbitrary waypoints in both Gazebo and physical maze tests.

</details>

---

## 🧭 Looking Ahead

Future work will involve:
- Extending the navigation pipeline with **SLAM fusion** and **dynamic obstacle prediction**.  
- Experimenting with **semantic mapping** and **learned control policies** via reinforcement learning.  
- Integrating **ROS 2 Nav2 with onboard vision nodes** for multi-sensor navigation on the TurtleBot3.  

---

## ⚠️ Academic Integrity & Usage

Everything in this repository is **my original work** for **CS 7785 (Introduction to Robotics Research)**.  
This repository is for **learning and reference only** — please follow the **Georgia Tech Honor Code**.  
