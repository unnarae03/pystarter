import rclpy
from rclpy.executors import MultiThreadedExecutor
import py_trees
import os
from ament_index_python.packages import get_package_share_directory

from pystarter.nodes.move_to_goal_node import MoveToGoal
# from pystarter.nodes.set_angle_node import SetAngleNode  # ⬅ 나중에 사용할 경우 import

def waypoint_exists(index):
    filename = f"waypoint{index + 1}.yaml"
    config_path = os.path.join(
        get_package_share_directory("pystarter"),
        "config",
        filename
    )
    return os.path.exists(config_path)

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()

    index = 0
    while waypoint_exists(index):
        print(f"[INFO] Executing waypoint{index + 1}...")

        # ✅ memory 인자 명시해야 함
        root = py_trees.composites.Sequence(name=f"TreeForWaypoint{index+1}", memory=False)

        # ⬇️ 필요 시 아래 두 줄의 주석을 해제하여 각도 정렬 기능 활성화
        # set_angle = SetAngleNode(index=index)
        # root.add_child(set_angle)

        move_to_goal = MoveToGoal(index=index)
        root.add_child(move_to_goal)

        tree = py_trees.trees.BehaviourTree(root)
        tree.setup(timeout=15)

        while rclpy.ok():
            tree.tick()
            rclpy.spin_once(move_to_goal.node, timeout_sec=0.1)
            # rclpy.spin_once(set_angle.node, timeout_sec=0.1)  # ⬅ 나중에 각도 노드 추가 시 같이 돌릴 것

            if tree.root.status != py_trees.common.Status.RUNNING:
                break

        index += 1

    print("[DONE] All waypoints complete.")
    rclpy.shutdown()
