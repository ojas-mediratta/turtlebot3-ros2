# Lab 4 Test Plan & Verification Guide

## Pre-Flight Checklist

### 1. Build & Source
```bash
cd ~/autobots/lab4_ws/
colcon build --symlink-install
source install/setup.bash
```

**Expected output:** Build completes without errors

### 2. Verify Waypoints File
```bash
cat ~/autobots/lab4_ws/src/autobots_navigate_to_goal/autobots_navigate_to_goal/wayPoints.txt
```

**Expected output:**
```
1.5 0
1.5 1.4
0 1.4
```

### 3. Set ROS Domain ID (CRITICAL - every terminal!)
```bash
export ROS_DOMAIN_ID=76
```

---

## Launch Sequence

### Terminal 1: Robot Bringup
```bash
export ROS_DOMAIN_ID=76
ssh burger@<robot_ip>
source /opt/ros/humble/setup.bash
ros2 launch turtlebot3_bringup robot.launch.py
```

**Expected output:**
- Camera node starts
- LIDAR node starts
- Motor controller starts
- No error messages

### Terminal 2: Navigation Launch
```bash
export ROS_DOMAIN_ID=76
source ~/autobots/lab4_ws/install/setup.bash
ros2 launch autobots_navigate_to_goal nav_lab4.launch.py
```

**Expected output:**
```
[get_object_range]: get_object_range started: scanning ±45° for obstacles in [0.05, 2.0]m
[go_to_goal]: go_to_goal started: 3 waypoints, max_v=0.2 m/s, max_w=1.5 rad/s
[go_to_goal]:   Waypoint 0: (1.50, 0.00) tol=10cm
[go_to_goal]:   Waypoint 1: (1.50, 1.40) tol=15cm
[go_to_goal]:   Waypoint 2: (0.00, 1.40) tol=20cm
[go_to_goal]: Odometry zeroed at raw pose: (...)
```

---

## Real-Time Monitoring

### Terminal 3: Topic Echo Commands

#### Check obstacle detection:
```bash
export ROS_DOMAIN_ID=76
ros2 topic echo /obstacle_vector
```

**Expected behavior:**
- When no obstacles nearby: `x: 0.0, y: 0.0, z: 0.0`
- When obstacle detected: `x: <distance*cos(angle)>, y: <distance*sin(angle)>, z: 0.0`
- Values should update at ~10-20 Hz

#### Check velocity commands:
```bash
ros2 topic echo /cmd_vel
```

**Expected behavior:**
- `linear.x` should be in range [-0.20, 0.20] m/s
- `angular.z` should be in range [-1.50, 1.50] rad/s
- **CRITICAL:** Values NEVER exceed these limits!

#### Check odometry:
```bash
ros2 topic echo /odom
```

**Expected behavior:**
- Position updates at ~30 Hz
- Starts near (0, 0) after zeroing

#### Check LIDAR:
```bash
ros2 topic echo /scan --once
```

**Expected behavior:**
- 360 range measurements
- Valid ranges typically 0.12m to 3.5m
- `inf` values for invalid readings

---

## Functional Tests

### Test 1: Speed Limit Verification
**Command:**
```bash
ros2 topic echo /cmd_vel | grep -E "linear|angular"
```

**Pass criteria:**
- All `linear.x` values ≤ 0.20 m/s (in absolute value)
- All `angular.z` values ≤ 1.50 rad/s (in absolute value)

### Test 2: Obstacle Detection Range
**Procedure:**
1. Place hand/object 0.5m in front of robot
2. Check `/obstacle_vector` output

**Pass criteria:**
- Vector magnitude ≈ 0.5m
- Vector points toward obstacle (x positive, y near zero for centered object)

### Test 3: Waypoint Tolerance
**Procedure:**
1. Let robot navigate to first waypoint (1.5, 0)
2. Watch terminal output

**Pass criteria:**
- Robot stops within 10cm of waypoint
- Terminal prints: `Reached waypoint 0 at (...). Dwelling for 10s...`

### Test 4: Dwell Timing
**Procedure:**
1. Time robot stop duration at waypoint
2. Use stopwatch or `time` command

**Pass criteria:**
- Robot holds position for exactly 10 seconds (±0.5s tolerance)
- No movement during dwell

### Test 5: Obstacle Avoidance
**Procedure:**
1. Place purple obstacle (or any object) in robot's path
2. Observe navigation behavior

**Pass criteria:**
- Robot detects obstacle before collision
- Robot steers around obstacle
- Robot returns to path toward goal after passing

### Test 6: Sequential Waypoint Navigation
**Procedure:**
1. Run full mission from start
2. Observe robot visit all 3 waypoints in order

**Pass criteria:**
- Waypoint 0: (1.5, 0) → 10cm tolerance → 10s dwell
- Waypoint 1: (1.5, 1.4) → 15cm tolerance → 10s dwell
- Waypoint 2: (0, 1.4) → 20cm tolerance → 10s dwell
- Terminal prints: `All waypoints reached! Mission complete.`

---

## RViz2 Visualization (Optional, from VM)

### Terminal 4: RViz2
```bash
export ROS_DOMAIN_ID=76
source /opt/ros/humble/setup.bash
rviz2
```

**Setup:**
1. Fixed Frame: `odom`
2. Add Display → LaserScan → Topic: `/scan`
3. Add Display → Odometry → Topic: `/odom`
4. Add Display → TF (to see robot frame)

**What to observe:**
- Red dots = LIDAR readings
- Blue arrow = robot pose from odometry
- Robot path traces toward waypoints
- Obstacle appears as LIDAR cluster

---

## Debugging Commands

### Check if nodes are running:
```bash
ros2 node list
```
**Expected:**
- `/get_object_range`
- `/go_to_goal`
- (plus turtlebot3_bringup nodes)

### Check topic connections:
```bash
ros2 topic info /obstacle_vector
ros2 topic info /cmd_vel
```

### Check message rates:
```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /cmd_vel
```

### Emergency stop (if needed):
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

---

## Common Issues & Solutions

### Issue: Robot spins in place
**Diagnosis:** Check `/obstacle_vector` - likely seeing false obstacles
**Solution:** Adjust `front_half_angle_deg` or `max_considered_range` in `get_object_range.py`

### Issue: Robot exceeds speed limits
**Diagnosis:** Check `/cmd_vel` values
**Solution:** Verify clamping logic in `go_to_goal.py` control loop

### Issue: Robot doesn't detect obstacles
**Diagnosis:** Check `/scan` topic
**Solution:** 
1. Verify LIDAR is publishing
2. Check `min_valid_range` and `max_considered_range` parameters
3. Ensure obstacle is within forward arc (±45°)

### Issue: Robot overshoots waypoints
**Diagnosis:** Check goal tolerance values
**Solution:** Tune `goal_radii` array in `go_to_goal.py`

### Issue: Odometry drift
**Diagnosis:** Position accumulates error over time (normal for dead reckoning)
**Solution:** This is expected. Lab uses odometry only. Keep waypoints close (<2m apart)

### Issue: Robot oscillates near goal
**Diagnosis:** Controller instability
**Solution:** 
1. Reduce `k_ang` (angular gain)
2. Increase goal tolerance
3. Add velocity smoothing

---

## Parameter Tuning Guide

### In `get_object_range.py`:
```python
self.front_half_angle_deg = 45.0   # Wider = more side detection, narrower = focus forward
self.min_valid_range = 0.05        # Lower = detect closer objects (but more noise)
self.max_considered_range = 2.0    # Higher = earlier detection, lower = only nearby
```

### In `go_to_goal.py`:
```python
self.k_lin = 0.5           # Higher = faster toward goal (but less stable)
self.k_ang = 2.0           # Higher = faster turning (but more oscillation)
self.k_rep = 1.2           # Higher = stronger obstacle avoidance
self.rep_cutoff = 0.9      # Distance when repulsion activates (m)
```

**Tuning workflow:**
1. Start with given values
2. If robot too aggressive → lower `k_lin`, `k_ang`
3. If robot too slow → raise `k_lin`, `k_ang`
4. If robot hits obstacles → raise `k_rep`, increase `rep_cutoff`
5. If robot avoids too much → lower `k_rep`, decrease `rep_cutoff`

---

## Success Criteria Summary

✅ **Builds without errors**
✅ **All speed commands within limits** (|v|≤0.2, |w|≤1.5)
✅ **Visits all 3 waypoints in order**
✅ **Stops within tolerance** (10/15/20cm)
✅ **Dwells for 10 seconds** at each waypoint
✅ **Avoids obstacles** without collision
✅ **Completes mission** and prints "Mission complete"

---

## Data Collection (for report)

### Record these during demo:
```bash
# Save velocity profile
ros2 topic echo /cmd_vel > velocity_log.txt

# Save odometry trajectory
ros2 topic echo /odom > odom_log.txt

# Save obstacle detections
ros2 topic echo /obstacle_vector > obstacle_log.txt
```

### Video capture:
- Use `rqt_image_view` to record `/image_raw/compressed` (if needed)
- Or use phone camera of robot navigation

---

## Quick Reference: Essential Commands

```bash
# Build
cd ~/autobots/lab4_ws && colcon build --symlink-install && source install/setup.bash

# Launch (after robot bringup)
ros2 launch autobots_navigate_to_goal nav_lab4.launch.py

# Monitor speed limits
ros2 topic echo /cmd_vel | grep -E "linear|angular"

# Monitor obstacle
ros2 topic echo /obstacle_vector

# Emergency stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once

# Kill nodes
Ctrl+C in launch terminal
```
