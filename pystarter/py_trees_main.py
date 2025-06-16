import rclpy
import py_trees
from pystarter.nodes.move_to_goal_node import MoveToGoal

def main():
    rclpy.init()

    index = 1
    while True:
        # waypoint 파일 존재 여부 확인
        from ament_index_python.packages import get_package_share_directory
        import os
        path = os.path.join(
            get_package_share_directory("pystarter"),
            "config",
            f"waypoint{index}.yaml"
        )
        if not os.path.exists(path):
            print(f"✅ 모든 waypoint 완료 (index={index-1})")
            break

        # PerWaypoint 트리 구성
        move_to_goal = MoveToGoal(index=index)
        seq = py_trees.composites.Sequence(name=f"PerWaypoint{index}")
        seq.add_child(move_to_goal)

        # 트리 실행
        bt = py_trees.trees.BehaviourTree(seq)
        bt.setup(timeout=15)

        # ROS2 spin
        try:
            while rclpy.ok() and bt.root.status != py_trees.common.Status.SUCCESS:
                bt.tick()
                rclpy.spin_once(move_to_goal.node, timeout_sec=0.1)
        except KeyboardInterrupt:
            break

        index += 1

    rclpy.shutdown()
