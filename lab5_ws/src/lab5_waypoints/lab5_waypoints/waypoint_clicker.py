#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class WaypointClickerNode(Node):
    def __init__(self):
        super().__init__('waypoint_clicker')
        
        # Action client to Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscribe to clicked points from RViz
        self.click_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )
        
        self.navigating = False
        self.get_logger().info('Waypoint clicker ready. Click points in RViz to navigate.')
        self.get_logger().info('Use "Publish Point" tool in RViz toolbar.')
        
        # Wait for Nav2 action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Nav2 action server not available yet')
        else:
            self.get_logger().info('Connected to NavigateToPose action server')
    
    def clicked_point_callback(self, msg: PointStamped):
        """Send goal immediately when user clicks in RViz"""
        if self.navigating:
            self.get_logger().warn('Already navigating, ignoring click')
            return
        
        x, y = msg.point.x, msg.point.y
        self.get_logger().info(f'New waypoint clicked: ({x:.2f}, {y:.2f})')
        
        # Create and send goal immediately
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        # Face forward (no specific orientation)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.navigating = True
        
        # Send goal
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2!')
            self.navigating = False
            return
        
        self.get_logger().info('Goal accepted, navigating...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Handle navigation completion"""
        result = future.result()
        status = result.status
        
        self.navigating = False
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Waypoint reached! Click next point.')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Navigation aborted by Nav2')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation canceled')
        else:
            self.get_logger().warn(f'Navigation ended with status: {status}')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointClickerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
