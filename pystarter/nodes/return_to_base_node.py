import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import math

def return_to_base(pose_dict, node):
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    node.get_logger().info("Waiting for NavigateToPose action server...")
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error('NavigateToPose server not available')
        return

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = pose_dict['x']
    goal_msg.pose.pose.position.y = pose_dict['y']
    goal_msg.pose.pose.position.z = 0.0

    theta = pose_dict['theta']
    goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
    goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

    node.get_logger().info("Sending return-to-base goal...")
    future = client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future)
    goal_handle = future.result()

    if not goal_handle.accepted:
        node.get_logger().error("Return goal rejected.")
        return

    node.get_logger().info("Return goal accepted. Waiting for result...")
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)

    result = result_future.result()
    if result.status == 4:
        node.get_logger().warn("Return goal aborted!")
    else:
        node.get_logger().info("Returned to base.")
