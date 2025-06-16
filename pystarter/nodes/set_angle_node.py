import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros
import tf_transformations
import os
import yaml
from ament_index_python.packages import get_package_share_directory
import py_trees

class SetAngleNode(py_trees.behaviour.Behaviour):
    def __init__(self, index=0):
        super().__init__(name=f"SetAngle_{index}")
        self.index = index
        self.node = rclpy.create_node(f"set_angle_node_{index}")
        self.publisher = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.done = False

    def initialise(self):
        self.done = False

    def update(self):
        if self.done:
            return py_trees.common.Status.SUCCESS

        try:
            # 현재 yaw 추출
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            quat = transform.transform.rotation
            _, _, current_yaw = tf_transformations.euler_from_quaternion(
                [quat.x, quat.y, quat.z, quat.w]
            )

            # 목표 yaw 추출
            target_theta = self.load_theta_from_yaml(self.index)
            angle_diff = (target_theta - current_yaw + 3.14) % (2 * 3.14) - 3.14

            if abs(angle_diff) < 0.01:
                print(f"[SetAngle {self.index}] ✅ 이미 목표 각도 도달")
                self.done = True
                return py_trees.common.Status.SUCCESS

            twist = Twist()
            twist.angular.z = 0.3 if angle_diff > 0 else -0.3
            self.publisher.publish(twist)
            print(f"[SetAngle {self.index}] 🔄 회전 중... (차이: {angle_diff:.3f})")
            return py_trees.common.Status.RUNNING

        except Exception as e:
            self.node.get_logger().warn(f"[SetAngle {self.index}] ⚠ TF 오류: {str(e)}")
            return py_trees.common.Status.RUNNING

    def load_theta_from_yaml(self, index):
        filename = f"waypoint{index + 1}.yaml"
        config_path = os.path.join(
            get_package_share_directory("pystarter"),
            "config",
            filename
        )
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)
        return float(data['pose']['theta'])
