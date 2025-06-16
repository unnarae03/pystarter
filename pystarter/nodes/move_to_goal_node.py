import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import tf_transformations
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.index = 1
        self.package_path = get_package_share_directory('pystarter')
        self.send_next_goal()

    def send_next_goal(self):
        file_path = os.path.join(self.package_path, 'config', f'waypoint{self.index}.yaml')
        if not os.path.exists(file_path):
            self.get_logger().info(f'✅ All waypoints complete (last index = {self.index - 1})')
            rclpy.shutdown()
            return

        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
        x = data['pose']['x']
        y = data['pose']['y']
        theta = data['pose']['theta']

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        q = tf_transformations.quaternion_from_euler(0, 0, theta)
        goal.pose.pose.orientation.x = q[0]
        goal.pose.pose.orientation.y = q[1]
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]

        self.client.wait_for_server()
        self.get_logger().info(f'🚩 Sending goal {self.index}: x={x}, y={y}, θ={theta} rad')
        self.client.send_goal_async(goal).add_done_callback(self.goal_response)

    def goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'❌ Goal {self.index} rejected')
            rclpy.shutdown()
            return
        self.get_logger().info(f'✅ Goal {self.index} accepted')
        goal_handle.get_result_async().add_done_callback(self.goal_result)

    def goal_result(self, future):
        result = future.result().result
        self.get_logger().info(f'🏁 Goal {self.index} result received.')
        self.index += 1
        self.send_next_goal()

def main():
    rclpy.init()
    node = WaypointNavigator()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
