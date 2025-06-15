import rclpy
from geometry_msgs.msg import Twist
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import tf_transformations
import tf2_ros

class SetAngle:
    def __init__(self, index=0):
        self.index = index
        self.node = rclpy.create_node("set_angle_node")
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.threshold = 0.03  # 역치

    def run(self):
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
            else:
                self.node.get_logger().info("[SetAngle] 각도 정렬 완료")

        except Exception as e:
            self.node.get_logger().error(f"[SetAngle] 실패: {e}")

# ✅ 단독 실행 진짜 main()
def main():
    rclpy.init()
    sa = SetAngle(index=0)  # 여기서 원하는 인덱스 설정
    sa.run()
    rclpy.shutdown()
