#!/usr/bin/env python3
"""Test sending a large FollowPath goal to Nav2 controller."""
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros


class TestFollowPath(Node):
    def __init__(self):
        super().__init__('test_follow_path')
        self._client = ActionClient(self, FollowPath, 'follow_path')
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    def get_robot_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_link', Time(seconds=0),
                timeout=rclpy.duration.Duration(seconds=5))
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception as e:
            self.get_logger().error(f'TF error: {e}')
            return None

    def run(self):
        self.get_logger().info('Waiting for follow_path action server...')
        self._client.wait_for_server()

        # Wait for TF to be ready
        import time
        time.sleep(2)

        pose = self.get_robot_pose()
        if not pose:
            self.get_logger().error('Cannot get robot pose')
            return
        rx, ry = pose
        self.get_logger().info(f'Robot at ({rx:.1f}, {ry:.1f}) in map frame')

        # Build a path of 500 waypoints: straight south for 500m
        n_points = 500
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = Time(seconds=0).to_msg()

        for i in range(n_points):
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.stamp = Time(seconds=0).to_msg()
            p.pose.position.x = rx
            p.pose.position.y = ry - (i * 1.0)  # 1m spacing, heading south
            p.pose.orientation.z = -0.7071  # facing south
            p.pose.orientation.w = 0.7071
            path.poses.append(p)

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'general_goal_checker'

        self.get_logger().info(f'Sending {len(path.poses)} poses to FollowPath...')
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal REJECTED!')
            return
        self.get_logger().info('Goal accepted, waiting for result...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60)
        result = result_future.result()
        self.get_logger().info(
            f'FollowPath finished with status={result.status}'
        )


def main():
    rclpy.init()
    node = TestFollowPath()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
