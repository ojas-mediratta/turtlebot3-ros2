#!/usr/bin/env python3
# go_to_goal.py
# Lab 4: Waypoint Navigation with Obstacle Avoidance
# Subscribes: /odom (Odometry), /obstacle_vector (Vector3)
# Publishes: /cmd_vel (Twist)

import math
import time
import numpy as np
import pathlib
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, Point

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        
        # ===== TUNABLE PARAMETERS =====
        # Control gains
        self.k_lin = 0.85           # linear gain for attractive force
        self.k_ang = 2.2           # angular gain for heading correction
        self.k_rep = 1.3           # repulsive force weight (higher = stronger avoidance)
        self.rep_cutoff = 0.75      # distance threshold for repulsion (m)
        
        # Approach behavior 
        self.approach_slowdown_radius = 0.25   # start slowing when within 25cm
        self.min_approach_speed = 0.075         # minimum speed when very close
        
        # Anti-oscillation parameters 
        self.angular_damping_near_obstacle = 0.3  # reduce angular gain when obstacle close
        self.obstacle_proximity_threshold = 0.5   # consider obstacle "close" at this distance
        self.min_forward_progress = 0.10          # minimum linear velocity to maintain forward motion
        self.angular_smoothing_alpha = 0.2        # low-pass filter for angular velocity (0-1)
        
        # Speed limits (LAB REQUIREMENTS - DO NOT EXCEED)
        self.max_v = 0.20          # m/s (lab hard limit)
        self.max_w = 1.50          # rad/s (lab hard limit)
        
        # Goal tolerances per waypoint index: [10cm, 15cm, 20cm]
        self.goal_radii = [0.10, 0.10, 0.10]
        self.dwell_time_sec = 5.0 # hold at waypoint for 5 seconds
        
        # Control loop rate
        self.control_rate_hz = 20.0  # 20 Hz
        # ==============================
        
        # Odometry offset state (zeroing at startup per Rotation_Script.py)
        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0
        self.globalPos = Point()
        self.globalPos.x = 0.0
        self.globalPos.y = 0.0
        self.globalAng = 0.0
        
        # Obstacle state (robot frame)
        self.obs_vec_robot = np.array([0.0, 0.0])
        
        # Anti-oscillation state (NEW: smooth angular velocity)
        self.prev_angular_vel = 0.0
        
        # Waypoint navigation state
        self.waypoints = self.load_waypoints()
        self.goal_idx = 0
        self.state = 'DRIVE'  # States: DRIVE, DWELL, DONE
        self.dwell_start = None
        
        # ROS I/O
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 20)
        self.sub_obs = self.create_subscription(Vector3, '/obstacle_vector', self.obstacle_callback, 20)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control loop timer
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_loop)
        
        self.get_logger().info(
            f'go_to_goal started: {len(self.waypoints)} waypoints, '
            f'max_v={self.max_v} m/s, max_w={self.max_w} rad/s'
        )
        for i, (x, y) in enumerate(self.waypoints):
            tol = self.goal_radii[min(i, len(self.goal_radii)-1)]
            self.get_logger().info(f'  Waypoint {i}: ({x:.2f}, {y:.2f}) tol={tol*100:.0f}cm')

    
    # ========== HELPER METHODS ==========
    
    def load_waypoints(self):
        """Load waypoints from wayPoints.txt in same directory."""
        here = pathlib.Path(__file__).parent
        txt = here / 'wayPoints.txt'
        wps = []
        
        try:
            with open(txt, 'r') as f:
                for line_num, line in enumerate(f, 1):
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    
                    parts = line.split()
                    if len(parts) != 2:
                        self.get_logger().warn(f'Skipping invalid line {line_num}: {line}')
                        continue
                    
                    x_str, y_str = parts
                    wps.append((float(x_str), float(y_str)))
            
            if not wps:
                self.get_logger().error('No waypoints loaded! Check wayPoints.txt')
            
            return wps
        
        except FileNotFoundError:
            self.get_logger().error(f'wayPoints.txt not found at {txt}')
            return []
        except ValueError as e:
            self.get_logger().error(f'Error parsing wayPoints.txt: {e}')
            return []
    
    def quat_to_yaw(self, q):
        """Convert quaternion to yaw angle (radians)."""
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    
    def apply_odom_offset(self, odom_msg):
        """
        Zero odometry at startup per Rotation_Script.py logic.
        Maintains stable (0,0,0) origin regardless of robot's power-on pose.
        """
        position = odom_msg.pose.pose.position
        yaw = self.quat_to_yaw(odom_msg.pose.pose.orientation)
        
        if self.Init:
            # First odom message: capture initial pose as offset
            self.Init = False
            self.Init_ang = yaw
            
            # Rotate initial position into frame aligned with initial heading
            M = np.array([
                [math.cos(self.Init_ang),  math.sin(self.Init_ang)],
                [-math.sin(self.Init_ang), math.cos(self.Init_ang)]
            ])
            p0 = M @ np.array([position.x, position.y])
            self.Init_pos.x, self.Init_pos.y = p0[0], p0[1]
            
            self.get_logger().info(
                f'Odometry zeroed at raw pose: ({position.x:.3f}, {position.y:.3f}, {yaw:.3f})'
            )
        
        # Apply rotation and offset to get zeroed global pose
        M = np.array([
            [math.cos(self.Init_ang),  math.sin(self.Init_ang)],
            [-math.sin(self.Init_ang), math.cos(self.Init_ang)]
        ])
        p = M @ np.array([position.x, position.y]) - np.array([self.Init_pos.x, self.Init_pos.y])
        
        self.globalPos.x = p[0]
        self.globalPos.y = p[1]
        self.globalAng = self.normalize_angle(yaw - self.Init_ang)
    
    def normalize_angle(self, angle):
        """Wrap angle to [-pi, pi]."""
        return (angle + math.pi) % (2*math.pi) - math.pi
    
    def angle_difference(self, target, current):
        """Compute shortest angular difference from current to target."""
        diff = target - current
        return self.normalize_angle(diff)
    
    # ========== ROS CALLBACKS ==========
    
    def odom_callback(self, msg: Odometry):
        """Update global pose from odometry."""
        self.apply_odom_offset(msg)
    
    def obstacle_callback(self, v: Vector3):
        """Update obstacle vector (robot frame)."""
        self.obs_vec_robot = np.array([v.x, v.y])
    
    # ========== CONTROL LOOP ==========
    
    def control_loop(self):
        """Main control loop: state machine + potential field navigation."""
        
        # Check if mission complete
        if self.goal_idx >= len(self.waypoints):
            if self.state != 'DONE':
                self.get_logger().info('All waypoints reached! Mission complete.')
                self.state = 'DONE'
            self.stop_robot()
            return
        
        # Current goal
        gx, gy = self.waypoints[self.goal_idx]
        dx = gx - self.globalPos.x
        dy = gy - self.globalPos.y
        dist = math.hypot(dx, dy)
        
        # Goal tolerance for current waypoint
        tol = self.goal_radii[min(self.goal_idx, len(self.goal_radii)-1)]
        
        # ===== STATE MACHINE =====
        
        if self.state == 'DRIVE':
            # Check if reached goal
            if dist <= tol:
                self.get_logger().info(
                    f'Reached waypoint {self.goal_idx} at ({self.globalPos.x:.2f}, {self.globalPos.y:.2f}). '
                    f'Dwelling for {self.dwell_time_sec}s...'
                )
                self.state = 'DWELL'
                self.dwell_start = time.time()
                self.stop_robot()
                return
            
            # ===== POTENTIAL FIELD NAVIGATION =====
            
            # 1. Attractive force (toward goal)
            att_vec = np.array([dx, dy])
            att_norm = np.linalg.norm(att_vec)
            
            if att_norm > 1e-6:
                att_dir = att_vec / att_norm  # unit vector toward goal
            else:
                att_dir = np.array([1.0, 0.0])  # default forward
            
            # 2. Repulsive force (away from obstacle)
            obs_dist = np.linalg.norm(self.obs_vec_robot)
            rep_vec = np.zeros(2)
            
            if obs_dist > 1e-6 and obs_dist < self.rep_cutoff:
                # Transform obstacle from robot frame to global frame
                yaw = self.globalAng
                R = np.array([
                    [math.cos(yaw), -math.sin(yaw)],
                    [math.sin(yaw),  math.cos(yaw)]
                ])
                obs_global = R @ self.obs_vec_robot
                
                # Repulsive magnitude: inverse square with cutoff
                rep_mag = self.k_rep * (1.0/obs_dist - 1.0/self.rep_cutoff)
                
                # Repulsive direction: away from obstacle
                obs_dir = obs_global / np.linalg.norm(obs_global)
                rep_vec = -rep_mag * obs_dir
            
            # 3. Combined desired velocity vector (global frame)
            desired_vec = self.k_lin * att_dir + rep_vec
            desired_norm = np.linalg.norm(desired_vec)
            
            # 4. Convert to robot commands (v, w)
            
            if desired_norm > 1e-6:
                # Desired heading in global frame
                desired_heading = math.atan2(desired_vec[1], desired_vec[0])
                
                # Heading error
                heading_err = self.angle_difference(desired_heading, self.globalAng)
                
                # Angular velocity: proportional to heading error
                w = self.k_ang * heading_err
                
                # NEW FIX 1: Reduce angular gain when obstacle is close (prevent oscillation)
                if obs_dist > 0.01 and obs_dist < self.obstacle_proximity_threshold:
                    angular_damping = self.angular_damping_near_obstacle
                    w *= angular_damping
                    self.get_logger().info(
                        f'Obstacle close ({obs_dist:.2f}m) - damping angular velocity',
                        throttle_duration_sec=1.0
                    )
                
                # NEW FIX 2: Low-pass filter on angular velocity (smooth out oscillations)
                w = (self.angular_smoothing_alpha * w + 
                     (1.0 - self.angular_smoothing_alpha) * self.prev_angular_vel)
                self.prev_angular_vel = w
                
                # Linear velocity: approach behavior for precision
                # Slow down as we approach goal (improves exponential scoring)
                if dist < self.approach_slowdown_radius:
                    # Proportional slowdown within approach radius
                    approach_factor = max(
                        dist / self.approach_slowdown_radius,
                        self.min_approach_speed / self.max_v
                    )
                    base_speed = self.max_v * approach_factor
                else:
                    base_speed = self.max_v
                
                # Scale by alignment with desired direction
                alignment_factor = max(0.0, math.cos(heading_err))
                magnitude_factor = min(1.0, desired_norm)
                
                v = base_speed * alignment_factor * magnitude_factor
                
                # Reduce speed when turning sharply
                if abs(heading_err) > math.radians(45):
                    v *= 0.5
                
                # NEW FIX 3: Ensure minimum forward progress when obstacle nearby
                # (prevents getting stuck oscillating in place)
                if obs_dist > 0.01 and obs_dist < self.obstacle_proximity_threshold:
                    if abs(v) < self.min_forward_progress:
                        v = self.min_forward_progress
            else:
                # No desired motion
                v = 0.0
                w = 0.0
            
            # 5. Enforce speed limits (CRITICAL LAB REQUIREMENT)
            v = max(-self.max_v, min(self.max_v, v))
            w = max(-self.max_w, min(self.max_w, w))
            
            # 6. Publish command
            self.publish_cmd(v, w)
        
        elif self.state == 'DWELL':
            # Hold position for dwell_time_sec
            elapsed = time.time() - self.dwell_start
            
            if elapsed >= self.dwell_time_sec:
                self.get_logger().info(f'Dwell complete. Moving to next waypoint.')
                self.goal_idx += 1
                self.state = 'DRIVE'
            else:
                # Keep publishing zero velocity to ensure stop
                self.stop_robot()
    
    # ========== COMMAND PUBLISHING ==========
    
    def publish_cmd(self, v, w):
        """Publish velocity command to /cmd_vel."""
        msg = Twist()
        msg.linear.x = float(v)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(w)
        self.pub_cmd.publish(msg)
    
    def stop_robot(self):
        """Publish zero velocity."""
        self.publish_cmd(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
