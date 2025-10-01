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
- `README.md` — this file  

---

### Lab 2 – Perception & Object Tracking (`lab2_ws`)

**Purpose**: set up a ROS2 perception pipeline to detect and track a colored object, publish detection outputs, and link to simple robot motion.

**Key files and modules**:

- `find_object.py`  
  A ROS2 node (in Python) that subscribes to an image topic (camera), applies computer vision processing (e.g. HSV thresholding, morphological filtering, contour detection), identifies the object, and publishes:
  1. The processed image annotated with detection overlays  
  2. The pixel coordinates or centroid of the object  

- `rotate_robot.py`  
  A ROS2 control node that publishes `Twist` messages (to `/cmd_vel`) to rotate the robot (or simulated robot). Useful for verifying actuation and that your perception outputs can drive motion.

- Launch / config files  
  These set up the ROS2 node graph: launching the camera (or image source), `find_object.py`, `rotate_robot.py`, remappings, parameter settings for thresholds, etc.

---

### Lab 3 – Sensor Fusion, PID Control & Object Chasing (`lab3_ws`)

**Purpose**: fuse vision and LIDAR (or range sensor) data to chase a target object robustly via control.

**Key files and modules**:

- **Vision / Detection Module**  
  A node (perhaps still named `detect_object.py` or similar) that processes camera input to extract angular direction / bearing (in image coordinates or field of view angle) of the target object.

- **Range / Scan Fusion Module**  
  A node that subscribes to ROS2 `LaserScan` (e.g. `/scan`) and fuses that with the vision bearing to estimate distance to the target. This might involve filtering out spurious scans or selecting a nearest range in the approximate bearing direction.

- **Control / Chasing Node**  
  A ROS2 node implementing two PID controllers:
  1. **Angular / rotational PID** — to steer the robot so that its heading aligns with the target object  
  2. **Linear / translational PID** — to drive forward or backward to maintain a desired distance  

  This node publishes `Twist` messages to `/cmd_vel` to drive the robot toward (or maintain) the object.

- Launch / integration files  
  Launch scripts or `.py/.xml` files to bring up the full sensing + control pipeline, parameter definitions (PID gains, setpoints, thresholds), remappings, and simulation integration (e.g. in Gazebo with TurtleBot3).

- Supporting config & documentation  
  Config files (YAML or `.ini`) for PID gains, safe bounds, sensor fusion weights, and possibly a write-up or doc files explaining design choices, stability considerations, disturbances, and limitations.

---

## ⚠️ Academic Integrity & Usage

Everything in this repository is **my original work** for CS 7785.  
This is **not** a solutions manual and should **not** be used for direct submission in any course.  
If you are a student, please follow the **Georgia Tech Honor Code** — use this repository *only* for learning, reference **after** you’ve done your own work, or inspiration (not copy-paste).
