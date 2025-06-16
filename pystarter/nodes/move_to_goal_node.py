import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import py_trees
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import tf_transformations
import tf2_ros
import time


class MoveToGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name="MoveToGoal", index=0):
        super().__init__(name)
        self.index = index
        self.node = rclpy.create_node("move_to_goal_node")
        self.client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self._sent_goal = False
        self._goal_done = False
        self._goal_result = None

        # ✅ TF listener 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        # ✅ 중복 로그 방지용
        self.available = False
        self._printed_goal_log = False
        self._printed_fail_log = False

        # ✅ 액션 서버 연결 (딱 한 번)
        if self.client.wait_for_server(timeout_sec=5.0):
            self.available = True
        else:
            print("❌ 액션 서버 연결 실패")

    def initialise(self):
        self._sent_goal = False
        self._goal_done = False
        self._goal_result = None
        self._printed_fail_log = False  # 웨이포인트마다 리셋

        if not self.available:
            return

        # ✅ TF 연결 대기 (최대 60초)
        wait_time = 0.0
        max_wait = 60.0
        print("⏳ map → base_link 연결 대기 중...")

        while wait_time < max_wait:
            if self.tf_buffer.can_transform("map", "base_link", rclpy.time.Time()):
                print(f"✅ map → base_link 연결 성공 (대기 시간: {wait_time:.1f}초)")
                break
            time.sleep(0.1)
            wait_time += 0.1

        if wait_time >= max_wait:
            print("❌ map → base_link 변환이 60초 내에 안 잡혔습니다. 실행 중단")
            self._goal_result = py_trees.common.Status.FAILURE
            self._goal_done = True
            return

        if hasattr(self, "goal_pose"):
            return  # 이미 세팅된 경우 다시 안 함

        # ✅ YAML 로드
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

                q = tf_transformations.quaternion_from_euler(0, 0, theta)

                self.goal_pose = PoseStamped()
                self.goal_pose.header.frame_id = "map"
                self.goal_pose.header.stamp = self.node.get_clock().now().to_msg()
                self.goal_pose.pose.position.x = x
                self.goal_pose.pose.position.y = y
                self.goal_pose.pose.orientation.x = q[0]
                self.goal_pose.pose.orientation.y = q[1]
                self.goal_pose.pose.orientation.z = q[2]
                self.goal_pose.pose.orientation.w = q[3]

                if not self._printed_goal_log:
                    print(f"📤 목표 전송: x={x:.2f}, y={y:.2f}, θ={theta:.2f}rad")
                    self._printed_goal_log = True

        except Exception:
            if not self._printed_fail_log:
                print(f"❌ waypoint{self.index + 1}.yaml 로드 실패")
                self._printed_fail_log = True
            self._goal_done = True
            self._goal_result = py_trees.common.Status.FAILURE

    def update(self):
        if not self.available:
            return py_trees.common.Status.FAILURE

        if self._goal_done:
            return self._goal_result

        if not self._sent_goal:
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.goal_pose

            send_future = self.client.send_goal_async(goal_msg)
            send_future.add_done_callback(self._goal_response_callback)
            self._sent_goal = True
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.RUNNING

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            if not self._printed_fail_log:
                print("❌ 목표 거부됨")
                self._printed_fail_log = True
            self._goal_result = py_trees.common.Status.FAILURE
            self._goal_done = True
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        status = future.result().status
        if status == 4:  # ABORTED
            if not self._printed_fail_log:
                print("❌ 목표 실패 (ABORTED)")
                self._printed_fail_log = True
            self._goal_result = py_trees.common.Status.FAILURE
        else:
            print("✅ 목표 도달 완료")
            self._goal_result = py_trees.common.Status.SUCCESS

        self._goal_done = True
