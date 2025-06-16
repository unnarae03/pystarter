import rclpy
from rclpy.node import Node
import py_trees
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import tf_transformations  # 외부에 위치시키면 속도 조금 더 빠름

class MoveToGoal(py_trees.behaviour.Behaviour):
    def __init__(self, index=0):
        super().__init__(name=f"MoveToGoal_{index}")
        self.index = index
        self.node = rclpy.create_node(f"move_to_goal_node_{index}")
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.goal_sent = False
        self.result_future = None

    def initialise(self):
        self.goal_sent = False
        self.result_future = None
        print(f"[MoveToGoal {self.index}] ▶ 서버 연결 대기 중...")

    def update(self):
        if not self.goal_sent:
            if not self.action_client.wait_for_server(timeout_sec=10.0):
                print(f"[MoveToGoal {self.index}] ❌ 액션 서버 연결 실패!")
                return py_trees.common.Status.FAILURE

            print(f"[MoveToGoal {self.index}] ✅ 액션 서버 연결 완료!")

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.load_pose_from_yaml(self.index)

            self.result_future = self.action_client.send_goal_async(goal_msg)
            self.goal_sent = True
            return py_trees.common.Status.RUNNING

        if self.result_future.done():
            result = self.result_future.result()
            print(f"[MoveToGoal {self.index}] 🎯 도착 완료! status: {result.status}")
            if result.status == 4:  # ABORTED
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def load_pose_from_yaml(self, index):
        filename = f"waypoint{index + 1}.yaml"
        config_path = os.path.join(
            get_package_share_directory("pystarter"),
            "config",
            filename
        )

        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(data['pose']['x'])
        pose.pose.position.y = float(data['pose']['y'])
        pose.pose.position.z = 0.0

        theta = float(data['pose']['theta'])
        q = tf_transformations.quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return pose
