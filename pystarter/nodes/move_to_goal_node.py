import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import py_trees
from ament_index_python.packages import get_package_share_directory
import os
import yaml

class MoveToGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name="MoveToGoal", index=0):
        super().__init__(name)
        self.index = index
        self.blackboard = py_trees.blackboard.Blackboard()
        self.node = rclpy.create_node("move_to_goal_node")
        # ActionClient 객체 생성
        self.client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.initialized = False

    def initialise(self):
        # 액션 서버와 연결 대기
        self.node.get_logger().info("🔗 기다리는 중... 액션 서버와 연결 중.")
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("❌ 액션 서버 연결 실패")
            return py_trees.common.Status.FAILURE
        self.initialized = True
        self.node.get_logger().info("✅ NavigateToPose 액션 서버 연결 완료")

    def update(self):
        if not self.initialized:
            return py_trees.common.Status.FAILURE

        # ✅ waypoint{index+1}.yaml 파일 경로 찾기
        filename = f"waypoint{self.index + 1}.yaml"
        config_path = os.path.join(
            get_package_share_directory("pystarter"),
            "config",
            filename
        )

        # yaml 파일 읽기
        try:
            with open(config_path, 'r') as file:
                waypoint_data = yaml.safe_load(file)
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = "map"
                goal_pose.pose.position.x = waypoint_data['position'][0]
                goal_pose.pose.position.y = waypoint_data['position'][1]
                goal_pose.pose.position.z = waypoint_data['position'][2]

                # orientation을 처리하려면 퀘터니언으로 변환
                goal_pose.pose.orientation.x = waypoint_data['orientation'][0]
                goal_pose.pose.orientation.y = waypoint_data['orientation'][1]
                goal_pose.pose.orientation.z = waypoint_data['orientation'][2]
                goal_pose.pose.orientation.w = waypoint_data['orientation'][3]
                
                self.blackboard.set("goal_pose", goal_pose)
                self.node.get_logger().info(f"📤 목표 전송: x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}")

        except Exception as e:
            self.node.get_logger().error(f"❌ Failed to load waypoint from {config_path}: {e}")
            return py_trees.common.Status.FAILURE

        # goal 메시지 전송
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.blackboard.get("goal_pose")

        # 액션 서버로 목표 전송
        self.client.send_goal_async(goal_msg)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.node.get_logger().info("MoveToGoal 종료됨.")
