# AutoBots - ROS 2 TurtleBot3 Project

This is a ROS 2 Humble workspace for TurtleBot3 robotics coursework (CS/ME/ECE/AE/BME 7785 at Georgia Tech). The project contains multiple lab workspaces implementing progressively complex behaviors.

## Project Structure

- **lab2_ws**: Object tracking using camera vision (rotate toward red objects)
- **lab3_ws**: Object chasing using camera + LIDAR (maintain distance to objects)  
- **lab4_ws**: Autonomous navigation with obstacle avoidance and waypoint following
- Each workspace follows ROS 2 `ament_python` package structure with `src/`, `build/`, `install/`, `log/`

## Architecture Pattern

All packages follow a **multi-node pipeline architecture**:

1. **Sensor nodes** (camera, LIDAR) publish raw data via TurtleBot3 bringup
2. **Perception nodes** process sensor data and extract features (object location, obstacle vectors)
3. **Control nodes** implement controllers (PI, potential field) and publish velocity commands

Example from `lab3_ws`:
```
camera → detect_object → /object_angle → get_object_range → /object_location → chase_object → /cmd_vel
                            ↓                  ↑
                        LIDAR /scan ──────────┘
```

## Key Conventions

### Constants Module Pattern
Each package centralizes ALL tunable parameters in `<package_name>/constants.py`:
- Topic names (e.g., `OBJECT_COORD_TOPIC = '/object_px'`)
- HSV color thresholds for vision (`H_LOW`, `S_LOW`, `V_LOW`, etc.)
- Controller gains (`ANGULAR_KP`, `LINEAR_KI`)
- Physical parameters (`IMAGE_WIDTH`, `HORIZONTAL_FOV_RADIANS`)

**Important**: After editing `constants.py`, no rebuild is needed - just restart launch files.

### Launch File Pattern
Master launch files in `launch/` directories include TurtleBot3 bringup + all package nodes:
```python
# Always includes camera_robot.launch.py from turtlebot3_bringup
IncludeLaunchDescription(PythonLaunchDescriptionSource(camera_robot_launch_file))
# Then launches all package nodes
```

### Setup.py Entry Points
All executable nodes are registered in `setup.py`:
```python
entry_points={
    'console_scripts': [
        'find_object = autobots_object_follower.find_object_node:main',
        'rotate_robot = autobots_object_follower.rotate_robot_node:main',
    ],
}
```

## Critical Developer Workflows

### Build Process
```bash
cd ~/autobots/lab<N>_ws/
colcon build
source install/setup.bash
```

### Running on TurtleBot3 Robot
**CRITICAL**: Always set ROS_DOMAIN_ID=76 in every terminal:
```bash
export ROS_DOMAIN_ID=76
source ~/autobots/lab<N>_ws/install/setup.bash
ros2 launch <package_name> <master_launch_file>.launch.py
```

### Remote Visualization (from VM/computer)
```bash
export ROS_DOMAIN_ID=76
source /opt/ros/humble/setup.bash
rqt_image_view  # Select /image_processed/compressed topic
```

## Vision Processing Pattern

All vision nodes use this pipeline:
1. Subscribe to `/image_raw/compressed` with `BEST_EFFORT` QoS
2. Convert with `cv_bridge`: `compressed_imgmsg_to_cv2()`
3. HSV color masking with dual ranges (red wraps around 180° in HSV)
4. Morphological operations: `cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)`
5. Contour filtering by area ratio: `MIN_AREA_RATIO * (w * h)` to `MAX_AREA_RATIO * (w * h)`
6. Publish processed debug image to `/image_processed/compressed`

## Control Patterns

### Lab 2: Pure Rotation Controller
Proportional control: `angular_vel = TURN_SPEED * (x_pixel - IMAGE_WIDTH/2) / IMAGE_WIDTH`

### Lab 3: PI Controller (chase_object_node.py)
- **Separate PI loops** for linear (distance error) and angular (heading error)
- Integral windup protection with `INTEGRAL_CLAMP_LINEAR/ANGULAR`
- Deadbands (`DEL_D`, `DEL_A`) to prevent oscillation near setpoint
- **Timeout watchdog**: Publishes zero velocity if no target for >0.3s

### Lab 4: Potential Field Navigation (go_to_goal.py)
- **Attractive term**: Proportional to distance to goal
- **Repulsive term**: Based on `/obstacle_vector` from LIDAR (weighted when distance < `rep_cutoff`)
- **Odometry offset handling**: Uses rotation matrix from `Rotation_Script.py` to zero initial pose
- **Waypoint sequencing**: Loads from `wayPoints.txt`, different goal radii per waypoint index

## LIDAR Processing

Two distinct patterns:

**Lab 3** (`get_object_range_node.py`): 
- Correlates camera angle with LIDAR scan at that bearing
- **Angle normalization**: Camera gives -π to +π, LIDAR uses 0 to 2π indexing
- Publishes `Point(x=distance, y=angle)` to `/object_location`

**Lab 4** (`get_object_range.py`):
- Scans front arc (±35° configurable) for nearest valid obstacle
- Publishes `Vector3(x=ox, y=oy)` in robot frame to `/obstacle_vector`
- Filters by `min_valid_range` (0.05m) and `max_considered_range` (2.5m)

## Dependencies

Package.xml always includes:
```xml
<depend>rclpy</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>std_msgs</depend>
<depend>nav_msgs</depend>  <!-- If using odometry -->
```

External: `cv_bridge`, `turtlebot3_bringup`, `ros-humble-rqt-image-view`, `ros-humble-image-transport-plugins`

## Debugging Tips

- **QoS mismatches**: Sensor topics require `QoSReliabilityPolicy.BEST_EFFORT` and `QoSDurabilityPolicy.VOLATILE`
- **Image not appearing**: Check compressed image transport plugins are installed
- **Controller oscillation**: Tune deadbands (`DEL_D`, `DEL_A`) and integral clamps first
- **Stale data issues**: Implement timeout watchdogs (see `chase_object_node._tick()`)
- **Coordinate frame errors**: Ensure LIDAR angle normalization matches camera convention

## Lab 4 Specific Details

**Waypoint Navigation Requirements:**
- Load waypoints from `wayPoints.txt` (format: `x y` per line)
- Goal tolerances by index: [10cm, 15cm, 20cm] for waypoints 0, 1, 2
- Dwell at each waypoint for 10 seconds before proceeding
- **Hard speed limits**: |linear.x| ≤ 0.20 m/s, |angular.z| ≤ 1.50 rad/s

**State Machine Pattern** (`go_to_goal.py`):
- `DRIVE`: Navigate toward goal using potential field
- `DWELL`: Hold position for 10s after reaching waypoint
- `DONE`: All waypoints complete, publish zero velocity

**Odometry Zeroing** (from `Rotation_Script.py`):
- First odom message captures initial pose as offset
- All subsequent poses rotated/translated to start at (0,0,0)
- Uses rotation matrix to align with initial heading

**Lab 4 Launch Sequence:**
```bash
# Terminal 1 (robot bringup)
ros2 launch turtlebot3_bringup robot.launch.py

# Terminal 2 (navigation)
ros2 launch autobots_navigate_to_goal nav_lab4.launch.py
```

## File Organization

```
lab<N>_ws/
├── src/<package_name>/
│   ├── <package_name>/
│   │   ├── __init__.py
│   │   ├── constants.py          # ALL tunable parameters (lab2/3 only)
│   │   ├── *_node.py             # ROS node implementations
│   │   ├── *.py                  # Python modules (lab4: go_to_goal, get_object_range)
│   │   ├── Rotation_Script.py    # Odometry offset logic (lab4)
│   │   └── wayPoints.txt         # Waypoint data (lab4)
│   ├── launch/
│   │   └── *_master.launch.py    # Include bringup + all nodes
│   ├── setup.py                  # Entry points for executables
│   └── package.xml               # Dependencies
├── build/                        # Ignore (generated)
├── install/                      # Source this after colcon build
└── log/                          # Ignore (generated)
```
