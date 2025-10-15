# Anti-Oscillation Fixes for Lab 4

## Problem Observed
Robot oscillated (turning left-right rapidly) when approaching obstacle between waypoints 1 and 2.

## Root Cause
When obstacle is directly between robot and goal, the attractive force (toward goal) and repulsive force (away from obstacle) fight each other, causing rapid angular velocity changes.

## Three Fixes Implemented

### Fix 1: Angular Damping Near Obstacles
**Location:** `go_to_goal.py` line ~270

**What it does:**
- When obstacle is closer than 0.5m, reduce angular velocity by 60% (multiply by 0.4)
- This prevents sharp turns that lead to oscillation

**Parameters added:**
```python
self.angular_damping_near_obstacle = 0.4    # reduce to 40% of normal
self.obstacle_proximity_threshold = 0.5     # activate when obstacle <0.5m
```

**Code:**
```python
if obs_dist > 0.01 and obs_dist < self.obstacle_proximity_threshold:
    w *= self.angular_damping_near_obstacle  # Dampen angular velocity
```

---

### Fix 2: Low-Pass Filter on Angular Velocity
**Location:** `go_to_goal.py` line ~278

**What it does:**
- Smooths angular velocity changes over time
- New angular velocity = 30% new command + 70% previous command
- Prevents sudden jerky turns

**Parameters added:**
```python
self.angular_smoothing_alpha = 0.3          # smoothing factor (0=no change, 1=instant)
self.prev_angular_vel = 0.0                 # stores previous command
```

**Code:**
```python
w = (self.angular_smoothing_alpha * w + 
     (1.0 - self.angular_smoothing_alpha) * self.prev_angular_vel)
self.prev_angular_vel = w
```

---

### Fix 3: Minimum Forward Progress
**Location:** `go_to_goal.py` line ~306

**What it does:**
- Ensures robot keeps moving forward when near obstacle
- Prevents getting stuck oscillating in place
- Minimum linear velocity = 0.08 m/s when obstacle nearby

**Parameters added:**
```python
self.min_forward_progress = 0.08            # minimum linear velocity near obstacles
```

**Code:**
```python
if obs_dist > 0.01 and obs_dist < self.obstacle_proximity_threshold:
    if abs(v) < self.min_forward_progress:
        v = self.min_forward_progress  # Force forward motion
```

---

## Expected Behavior After Fixes

### Before (Oscillating):
```
→ sees obstacle
→ turns left 15°
→ turns right 20°
→ turns left 18°
→ turns right 22°
→ (stuck oscillating, no forward progress)
```

### After (Smooth Avoidance):
```
→ sees obstacle at 0.6m
→ angular velocity damped by 60%
→ smoothed turn left 8° (gradual)
→ maintains 0.08 m/s forward speed
→ curves around obstacle smoothly
→ returns to path toward goal
```

---

## Tuning Guide

### If still oscillating:
```python
self.angular_damping_near_obstacle = 0.3  # More aggressive damping (30%)
self.angular_smoothing_alpha = 0.2        # Smoother filtering (20% new)
```

### If too slow near obstacles:
```python
self.min_forward_progress = 0.10          # Increase minimum speed
self.obstacle_proximity_threshold = 0.4   # Activate fixes closer to obstacle
```

### If avoiding too early:
```python
self.obstacle_proximity_threshold = 0.3   # Only activate when very close
```

---

## Build & Test

```bash
cd ~/autobots/lab4_ws/
colcon build --symlink-install
source install/setup.bash

# Terminal 1: Robot bringup (leave running)
export ROS_DOMAIN_ID=56
ros2 launch turtlebot3_bringup robot.launch.py

# Terminal 2: Navigation (Ctrl+C and rerun for each test)
export ROS_DOMAIN_ID=56
ros2 launch autobots_navigate_to_goal nav_lab4.launch.py
```

---

## What to Watch For

### Good signs ✅:
- Robot slows turning when obstacle detected
- Smooth curve around obstacle (not jerky)
- Terminal shows: `Obstacle close (0.45m) - damping angular velocity`
- Robot maintains forward motion throughout

### Still problematic ❌:
- Robot still oscillates → decrease `angular_damping_near_obstacle` to 0.3
- Robot stops moving → increase `min_forward_progress` to 0.12
- Robot turns too slowly → increase `obstacle_proximity_threshold` to 0.6

---

## Technical Details

The fixes work together:
1. **Damping** reduces the magnitude of oscillation
2. **Smoothing** prevents sudden changes between control cycles
3. **Forward progress** ensures robot doesn't get stuck

This is a standard approach in robotics for preventing potential field local minima and oscillations.

---

## Files Modified

- `go_to_goal.py` - Added 4 new parameters, 3 new state variables, modified control logic

## Build Status

✅ Build successful
✅ No syntax errors
✅ Ready for testing
