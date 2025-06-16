import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import time

class MoveToGoal(Node):
    def __init__(self):
        super().__init__('move_to_goal_node')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.package_path = get_package_share_directory('pystarter')
        self.config_dir = os.path.join(self.package_path, 'config')

    def load_waypoint(self, index):
        path = os.path.join(self.config_dir, f'waypoint{index}.yaml')
        if not os.path.exists(path):
            return None

        with open(path, 'r') as f:
            data = yaml.safe_load(f)
            return data['pose']

    def send_goal(self, pose_dict):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = pose_dict['x']
        goal_msg.pose.pose.position.y = pose_dict['y']

        import tf_transformations
        import math
        q = tf_transformations.quaternion_from_euler(0, 0, pose_dict['theta'])

        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self.client.wait_for_server()
        self.get_logger().info('Sending goal...')
        return self.client.send_goal_async(goal_msg)

    def run(self):
        index = 1
        while rclpy.ok():
            pose = self.load_waypoint(index)
            if pose is None:
                self.get_logger().info(f"No more waypoint{index}.yaml, finished.")
                break

            future = self.send_goal(pose)
            rclpy.spin_until_future_complete(self, future)

            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f"Goal {index} was rejected!")
                break

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()

            if result.status != 4:
                self.get_logger().info(f"Goal {index} succeeded.")
            else:
                self.get_logger().warn(f"Goal {index} aborted.")
                break

            index += 1
            time.sleep(2.0)  # 다음 waypoint 전 잠시 대기

