#!/usr/bin/env python3
# get_object_range.py
# Lab 4: Obstacle Detection Node
# Subscribes: /scan (LaserScan)
# Publishes: /obstacle_vector (Vector3) - nearest obstacle in robot frame

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class GetObjectRange(Node):
    def __init__(self):
        super().__init__('get_object_range')
        
        # ===== TUNABLE PARAMETERS =====
        self.front_half_angle_deg = 60.0   # scan ±60° forward arc (wider for better side detection)
        self.min_valid_range = 0.05        # ignore readings < 5cm (sensor noise)
        self.max_considered_range = 2.5    # ignore obstacles beyond 2.5m
        
        # Clustering parameters (to distinguish obstacles from walls/chairs)
        self.cluster_max_gap = 0.15        # points >15cm apart = different clusters
        self.min_cluster_size = 5          # need ≥5 points to be valid obstacle
        self.max_cluster_width = 1.0       # reject clusters >1m wide (likely walls)
        # ==============================
        
        # Use BEST_EFFORT QoS to match sensor topics
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sensor_qos
        )
        self.pub = self.create_publisher(Vector3, '/obstacle_vector', 10)
        
        self.get_logger().info(
            f'get_object_range started: scanning ±{self.front_half_angle_deg}° '
            f'for obstacles in [{self.min_valid_range}, {self.max_considered_range}]m'
        )

    def scan_callback(self, msg: LaserScan):
        """Find nearest valid obstacle in forward arc, publish as robot-frame vector."""
        
        if len(msg.ranges) == 0:
            self.publish_zero_vector()
            return
        
        num_readings = len(msg.ranges)
        angle_increment = msg.angle_increment
        
        # Convert front sector angle to radians
        half_angle_rad = math.radians(self.front_half_angle_deg)
        half_angle_indices = int(half_angle_rad / angle_increment)
        
        # CRITICAL FIX: Handle LIDAR indexing correctly for wraparound
        # LIDAR convention: index 0 = front (0°), increases CCW
        # Right side: negative angles wrap to end of array
        # Left side: positive angles at start of array
        
        # Collect valid points in forward sector with (x, y, index, range)
        valid_points = []
        
        # RIGHT side of front arc (indices wrap from end of array)
        # angles: [0, -half_angle] → indices: [num-half_angle_indices, num-1]
        for i in range(num_readings - half_angle_indices, num_readings):
            r = msg.ranges[i]
            
            # Filter invalid/out-of-bounds ranges
            if not math.isfinite(r):
                continue
            if r < self.min_valid_range or r > self.max_considered_range:
                continue
            
            # Angle for this index (wraps negative for right side)
            angle = (i - num_readings) * angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            valid_points.append((x, y, i, r))
        
        # FRONT (index 0)
        r = msg.ranges[0]
        if math.isfinite(r) and self.min_valid_range <= r <= self.max_considered_range:
            x = r
            y = 0.0
            valid_points.append((x, y, 0, r))
        
        # LEFT side of front arc
        # angles: [0, +half_angle] → indices: [1, half_angle_indices]
        for i in range(1, min(half_angle_indices + 1, num_readings)):
            r = msg.ranges[i]
            
            # Filter invalid/out-of-bounds ranges
            if not math.isfinite(r):
                continue
            if r < self.min_valid_range or r > self.max_considered_range:
                continue
            
            # Convert to Cartesian coordinates in robot frame
            angle = i * angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            valid_points.append((x, y, i, r))
        
        if not valid_points:
            # No obstacles detected
            self.publish_zero_vector()
            return
        
        # Cluster points to identify discrete obstacles vs walls
        clusters = self._cluster_points(valid_points)
        
        # Filter clusters: reject walls (too wide), keep compact obstacles
        valid_clusters = [
            c for c in clusters 
            if len(c) >= self.min_cluster_size and self._cluster_width(c) <= self.max_cluster_width
        ]
        
        if not valid_clusters:
            # All clusters rejected (likely walls/furniture) - no threat
            self.publish_zero_vector()
            return
        
        # Find nearest cluster centroid
        nearest_cluster = min(valid_clusters, key=lambda c: self._cluster_distance(c))
        cx, cy = self._cluster_centroid(nearest_cluster)
        
        # Publish obstacle vector in robot frame
        v = Vector3()
        v.x = cx
        v.y = cy
        v.z = 0.0
        self.pub.publish(v)
        
        # Log proximity warnings for debugging/safety
        distance = math.sqrt(cx**2 + cy**2)
        if distance < 0.20:
            self.get_logger().warn(
                f'⚠️  PROXIMITY ALERT: Obstacle at {distance:.3f}m!',
                throttle_duration_sec=0.5
            )
        elif distance < 0.35:
            self.get_logger().info(
                f'Obstacle detected at {distance:.3f}m',
                throttle_duration_sec=2.0
            )
    
    def publish_zero_vector(self):
        """Publish zero vector when no valid data."""
        v = Vector3()
        v.x = 0.0
        v.y = 0.0
        v.z = 0.0
        self.pub.publish(v)
    
    def _cluster_points(self, points):
        """Cluster nearby LIDAR points into discrete obstacles."""
        if not points:
            return []
        
        # Sort by index for sequential clustering
        sorted_points = sorted(points, key=lambda p: p[2])
        
        clusters = []
        current_cluster = [sorted_points[0]]
        
        for i in range(1, len(sorted_points)):
            prev = sorted_points[i-1]
            curr = sorted_points[i]
            
            # Euclidean distance between consecutive points
            dx = curr[0] - prev[0]
            dy = curr[1] - prev[1]
            gap = math.sqrt(dx**2 + dy**2)
            
            if gap <= self.cluster_max_gap:
                # Same cluster
                current_cluster.append(curr)
            else:
                # Gap too large - start new cluster
                clusters.append(current_cluster)
                current_cluster = [curr]
        
        # Don't forget last cluster
        clusters.append(current_cluster)
        return clusters
    
    def _cluster_centroid(self, cluster):
        """Compute centroid (x, y) of cluster."""
        cx = sum(p[0] for p in cluster) / len(cluster)
        cy = sum(p[1] for p in cluster) / len(cluster)
        return cx, cy
    
    def _cluster_distance(self, cluster):
        """Distance from robot to cluster centroid."""
        cx, cy = self._cluster_centroid(cluster)
        return math.sqrt(cx**2 + cy**2)
    
    def _cluster_width(self, cluster):
        """Maximum width of cluster (for wall rejection)."""
        if len(cluster) < 2:
            return 0.0
        
        x_vals = [p[0] for p in cluster]
        y_vals = [p[1] for p in cluster]
        
        width_x = max(x_vals) - min(x_vals)
        width_y = max(y_vals) - min(y_vals)
        
        return math.sqrt(width_x**2 + width_y**2)

def main(args=None):
    rclpy.init(args=args)
    node = GetObjectRange()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
