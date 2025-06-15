import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import tf_transformations
import tf2_ros
import py_trees

class SetAngleNode(py_trees.behaviour.Behaviour):
    def __init__(self, name="SetAngle", index=0):
        super().__init__(name)
        self.index = index
        self.node = rclpy.create_node("set_angle_node_bt")
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.threshold = 0.03  # rad 기준 회전 오차 허용치

    def update(self):
        try:
            # ✅ YAML 로드
            filename = f"waypoint{self.index + 1}.yaml"
            config_path = os.path.join(
                get_package_share_directory("pystarter"),
                "config",
                filename
            )
            with open(config_path, 'r') as file:
                data = yaml.safe_load(file)
                target_theta = data["pose"]["theta"]

            # ✅ 현재 yaw 획득
            trans = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
            q = trans.transform.rotation
            _, _, current_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

            error = target_theta - current_yaw
            if abs(error) > self.threshold:
                twist = Twist()
                twist.angular.z = 0.4 * error
                self.publisher.publish(twist)
                self.node.get_logger().info(f"[SetAngle] 회전 중: error={error:.3f}")
                return py_trees.common.Status.RUNNING
            else:
                self.node.get_logger().info("[SetAngle] 각도 정렬 완료")
                return py_trees.common.Status.SUCCESS

        except Exception as e:
            self.node.get_logger().error(f"[SetAngle] 실패: {e}")
            return py_trees.common.Status.FAILURE
