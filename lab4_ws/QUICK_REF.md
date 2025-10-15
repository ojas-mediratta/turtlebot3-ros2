# Lab 4 Quick Reference Card

## 🚀 Launch Commands
```bash
# Terminal 1: Robot
export ROS_DOMAIN_ID=56
ros2 launch turtlebot3_bringup robot.launch.py

# Terminal 2: Navigation
export ROS_DOMAIN_ID=56
source ~/autobots/lab4_ws/install/setup.bash
ros2 launch autobots_navigate_to_goal nav_lab4.launch.py
```

## 📊 Where You'll See Output

**Terminal 2 (Navigation Launch) shows ALL node logs:**
- `[get_object_range]`: Obstacle detection, proximity warnings
- `[go_to_goal]`: "Reached waypoint X", "Dwelling for 10s", "Mission complete"
- This is your PRIMARY monitoring terminal during the demo

---

## 🖥️ Monitor Commands (FROM YOUR LINUX COMPUTER)

### Setup: Every terminal needs this first
```bash
export ROS_DOMAIN_ID=56
```

### CRITICAL: Speed Limit Verification (watch during entire demo)
```bash
ros2 topic echo /cmd_vel
# linear.x MUST stay in [-0.20, 0.20] m/s
# angular.z MUST stay in [-1.50, 1.50] rad/s
```

### Obstacle Detection (see what robot "sees")
```bash
ros2 topic echo /obstacle_vector
# x=0, y=0    → No obstacle
# x=0.5, y=0  → Obstacle 0.5m straight ahead
# x=0.5, y=-0.1 → Obstacle 0.5m ahead, slightly right
```

### Robot Position (verify navigation progress)
```bash
ros2 topic echo /odom --field pose.pose.position
# Compare to waypoints: (1.5, 0) → (1.5, 1.4) → (0, 1.4)
```

### LIDAR Check (verify sensor working)
```bash
ros2 topic hz /scan
# Should show ~5-10 Hz

ros2 topic echo /scan --field ranges --once | grep -v inf | head -20
# Shows valid distance readings
```

### Node Status
```bash
ros2 node list
# Should show: /get_object_range, /go_to_goal

ros2 topic list
# Should show: /scan, /odom, /cmd_vel, /obstacle_vector
```

### Emergency Stop
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

---

## 🧪 Recommended Testing Setup (4 Terminals on YOUR Computer)

### Terminal A: Navigation Launch (PRIMARY - watch this)
```bash
export ROS_DOMAIN_ID=56
source ~/autobots/lab4_ws/install/setup.bash
ros2 launch autobots_navigate_to_goal nav_lab4.launch.py
```
**You'll see:** All navigation logs, waypoint progress, obstacle warnings

### Terminal B: Speed Monitor (CRITICAL for grading)
```bash
export ROS_DOMAIN_ID=56
ros2 topic echo /cmd_vel | grep -A 1 "linear:\|angular:"
```
**You'll see:** Real-time velocity commands - verify limits never exceeded

### Terminal C: Obstacle Monitor
```bash
export ROS_DOMAIN_ID=56
ros2 topic echo /obstacle_vector
```
**You'll see:** What obstacles the robot detects and where

### Terminal D: Position Check (optional)
```bash
export ROS_DOMAIN_ID=56
watch -n 0.5 'ros2 topic echo /odom --field pose.pose.position --once'
```
**You'll see:** Robot position updating every 0.5 seconds

---

## 🎯 Key Parameters (if tuning needed)

### get_object_range.py
- `front_half_angle_deg = 50.0` - detection arc width
- `cluster_max_gap = 0.15` - obstacle clustering sensitivity
- `min_cluster_size = 5` - noise filtering

### go_to_goal.py
- `k_lin = 0.6` - speed toward goal
- `k_ang = 2.2` - turning aggressiveness
- `k_rep = 1.3` - obstacle avoidance strength
- `rep_cutoff = 0.7` - when to avoid obstacles
- `approach_slowdown_radius = 0.30` - precision approach zone

---

## ✅ Success Indicators

Watch Terminal A for these messages in sequence:
- [x] `[go_to_goal]: Odometry zeroed at raw pose...`
- [x] `[go_to_goal]: WP0: dist=1.50m → ... → dist=0.08m` (decreasing)
- [x] `[go_to_goal]: ✓ Reached waypoint 0 at (...). Dwelling for 10s...`
- [x] `[go_to_goal]: Dwell complete. Moving to next waypoint.`
- [x] (Repeat for WP1 and WP2)
- [x] `[go_to_goal]: ✅ All waypoints reached! Mission complete.`

Watch Terminal B: Speed limits NEVER exceeded
- [x] `linear.x` always in [-0.20, 0.20]
- [x] `angular.z` always in [-1.50, 1.50]

Watch Terminal C: Obstacle detection working
- [x] Shows (0, 0) when path clear
- [x] Shows obstacle position when purple box in view
- [x] Proximity warnings appear when <20cm

---

## 🔧 Quick Fixes

**Robot too aggressive?** → Increase `k_rep` to 1.5, increase `rep_cutoff` to 0.9

**Robot too slow?** → Increase `k_lin` to 0.8, decrease `approach_slowdown_radius` to 0.20

**False obstacles?** → Increase `min_cluster_size` to 8

**Oscillating?** → Decrease `k_ang` to 1.8

**Not detecting obstacle?** → Increase `front_half_angle_deg` to 60, check LIDAR with `ros2 topic hz /scan`
