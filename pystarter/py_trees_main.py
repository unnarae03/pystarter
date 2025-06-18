import rclpy
from rclpy.executors import MultiThreadedExecutor
import py_trees
import os
import yaml
from ament_index_python.packages import get_package_share_directory

from pystarter.nodes.move_to_goal_node import MoveToGoal
from pystarter.nodes.capture_image_node import CaptureImage
from pystarter.nodes.return_to_base_node import return_to_base
from py_trees.behaviours import Success  # 실패 시 우회처리용

def waypoint_exists(index):
    filename = f"waypoint{index + 1}.yaml"
    config_path = os.path.join(
        get_package_share_directory("pystarter"),
        "config",
        filename
    )
    return os.path.exists(config_path)

def load_return_pose():
    config_path = os.path.join(
        get_package_share_directory("pystarter"),
        "return_pose",
        "base.yaml"
    )
    with open(config_path, 'r') as f:
        data = yaml.safe_load(f)
        return data['pose']

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()

    index = 0
    while waypoint_exists(index):
        print(f"[INFO] Executing waypoint{index + 1}...")

        move_to_goal = MoveToGoal(index=index)
        capture_image = CaptureImage(label=index + 1)

        # 실패해도 통과하게 구성
        move_to_goal_selector = py_trees.composites.Selector(
            name=f"RetryOrBypass_{index+1}", memory=False
        )
        move_to_goal_selector.add_child(move_to_goal)
        move_to_goal_selector.add_child(Success(name=f"BypassGoal_{index+1}"))

        # 트리 구성: MoveToGoal -> CaptureImage
        root = py_trees.composites.Sequence(
            name=f"TreeForWaypoint{index+1}", memory=False
        )
        root.add_child(move_to_goal_selector)
        root.add_child(capture_image)

        tree = py_trees.trees.BehaviourTree(root)
        tree.setup(timeout=15)

        while rclpy.ok():
            tree.tick()
            rclpy.spin_once(move_to_goal.node, timeout_sec=0.1)
            rclpy.spin_once(capture_image.node, timeout_sec=0.1)

            if tree.root.status != py_trees.common.Status.RUNNING:
                break

        index += 1

    print("[DONE] All waypoints complete.")
    print("[INFO] Returning to base...")

    base_pose = load_return_pose()
    return_node = rclpy.create_node('return_to_base_node')
    return_to_base(base_pose, return_node)
    return_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
