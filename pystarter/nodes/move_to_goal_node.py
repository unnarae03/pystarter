import rclpy
from rclpy.node import Node
import py_trees
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import yaml
import os
import time
from ament_index_python.packages import get_package_share_directory
import tf_transformations

class MoveToGoal(py_trees.behaviour.Behaviour):
    def __init__(self, index=0):
        super().__init__(name=f"MoveToGoal_{index}")
        self.index = index
        self.node = rclpy.create_node(f"move_to_goal_node_{index}")
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.goal_sent = False
        self.goal_accepted = False
        self.result_future = None
        self.result_timeout_time = None
        self.timeout_limit_sec = 10
        self.goal_done = False
        self.retry_count = 0
        self.final_status = None

    def initialise(self):
        self.goal_sent = False
        self.goal_accepted = False
        self.result_future = None
        self.result_timeout_time = None
        self.goal_done = False
        self.retry_count = 0
        self.final_status = None

    def update(self):
        if self.goal_done:
            return self.final_status

        if not self.goal_sent:
            if not self.action_client.wait_for_server(timeout_sec=3.0):
                return py_trees.common.Status.RUNNING

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.load_pose_from_yaml(self.index)

            send_goal_future = self.action_client.send_goal_async(goal_msg)

            def goal_response_callback(future):
                goal_handle = future.result()
                if goal_handle.accepted:
                    self.goal_accepted = True
                    self.result_future = goal_handle.get_result_async()
                else:
                    self.goal_accepted = False
                    self.result_future = None

            send_goal_future.add_done_callback(goal_response_callback)
            self.goal_sent = True
            return py_trees.common.Status.RUNNING

        if self.goal_sent and self.goal_accepted and self.result_future is not None:
            if self.result_timeout_time is None:
                self.result_timeout_time = time.time()

            if time.time() - self.result_timeout_time > self.timeout_limit_sec:
                self.retry_count += 1
                if self.retry_count >= 2:
                    self.final_status = py_trees.common.Status.FAILURE
                    self.goal_done = True
                    return self.final_status
                else:
                    self.goal_sent = False
                    self.goal_accepted = False
                    self.result_future = None
                    self.result_timeout_time = None
                    return py_trees.common.Status.RUNNING

            if self.result_future.done():
                result = self.result_future.result()
                self.result_timeout_time = None
                if result.status in [0, 5]:  # SUCCEEDED or CANCELED
                    self.final_status = py_trees.common.Status.SUCCESS
                    self.goal_done = True
                    return self.final_status
                else:
                    self.retry_count += 1
                    if self.retry_count >= 2:
                        self.final_status = py_trees.common.Status.FAILURE
                        self.goal_done = True
                        return self.final_status
                    else:
                        self.goal_sent = False
                        self.goal_accepted = False
                        self.result_future = None
                        self.result_timeout_time = None
                        return py_trees.common.Status.RUNNING

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

        q = tf_transformations.quaternion_from_euler(0, 0, float(data['pose']['theta']))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return pose
