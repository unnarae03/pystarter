import py_trees
import rclpy
from geometry_msgs.msg import Twist
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import tf_transformations
import tf2_ros

class SetAngle(py_trees.behaviour.Behaviour):
    def __init__(self, index=0):
        super().__init__(name="SetAngle")  # ✅ 노드 이름 고정
        self.index = index
        self.node = rclpy.create_node("set_angle_node_bt")

        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        self.threshold = 0.03  # ✅ 역치: 약 ±1.7도

    def update(self):
        try:
            # ✅ waypoint{index+1}.yaml 파일 경로 찾기
            filename = f"waypoint{self.index + 1}.yaml"
            config_path = os.path.join(
                get_package_share_directory("pystarter"),
                "config",
                filename
            )

            # ✅ pose.theta 로딩
            with open(config_path, 'r') as file:
                data = yaml.safe_load(file)
                target_theta = data["pose"]["theta"]  # ✅ 단일 pose 반영됨

            # ✅ 현재 yaw 계산
            trans = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
            q = trans.transform.rotation
            _, _, current_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

            # ✅ 회전 오차 계산 및 회전 지시
            error = target_theta - current_yaw
            if abs(error) > self.threshold:
                twist = Twist()
                twist.angular.z = 0.4 * error
                self.publisher.publish(twist)
                return py_trees.common.Status.RUNNING
            else:
                return py_trees.common.Status.SUCCESS

        except Exception as e:
            self.node.get_logger().error(f"[SetAngle] 실패: {e}")
            return py_trees.common.Status.FAILURE
