import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import py_trees
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import math
import tf_transformations  # ✅ θ -> quaternion 변환용

class MoveToGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name="MoveToGoal", index=0):
        super().__init__(name)
        self.index = index
        self.blackboard = py_trees.blackboard.Blackboard()
        self.node = rclpy.create_node("move_to_goal_node")
        self.client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.initialized = False

    def initialise(self):
        self.node.get_logger().info("🔗 액션 서버 연결 대기 중...")
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("❌ 액션 서버 연결 실패")
            return py_trees.common.Status.FAILURE
        self.initialized = True
        self.node.get_logger().info("✅ 액션 서버 연결 완료")

    def update(self):
        if not self.initialized:
            return py_trees.common.Status.FAILURE

        filename = f"waypoint{self.index + 1}.yaml"
        config_path = os.path.join(
            get_package_share_directory("pystarter"),
            "config",
            filename
        )

        try:
            with open(config_path, 'r') as file:
                data = yaml.safe_load(file)
                x = data["pose"]["x"]
                y = data["pose"]["y"]
                theta = data["pose"]["theta"]

                # θ → quaternion 변환
                q = tf_transformations.quaternion_from_euler(0, 0, theta)

                goal_pose = PoseStamped()
                goal_pose.header.frame_id = "map"
                goal_pose.header.stamp = self.node.get_clock().now().to_msg()
                goal_pose.pose.position.x = x
                goal_pose.pose.position.y = y
                goal_pose.pose.orientation.x = q[0]
                goal_pose.pose.orientation.y = q[1]
                goal_pose.pose.orientation.z = q[2]
                goal_pose.pose.orientation.w = q[3]

                self.blackboard.set("goal_pose", goal_pose)
                self.node.get_logger().info(
                    f"📤 목표 전송: x={x:.2f}, y={y:.2f}, θ={theta:.2f}rad"
                )

        except Exception as e:
            self.node.get_logger().error(f"❌ {config_path} 로드 실패: {e}")
            return py_trees.common.Status.FAILURE

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.blackboard.get("goal_pose")
        self.client.send_goal_async(goal_msg)

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.node.get_logger().info("🛑 MoveToGoal 종료됨.")
