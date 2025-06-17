import rclpy
from rclpy.executors import MultiThreadedExecutor
import py_trees
import os
import yaml
from ament_index_python.packages import get_package_share_directory

from pystarter.nodes.move_to_goal_node import MoveToGoal
from pystarter.nodes.return_to_base_node import return_to_base
from pystarter.nodes.capture_image_node import CaptureImage #add

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

        root = py_trees.composites.Sequence(name=f"TreeForWaypoint{index+1}", memory=False)

        move_to_goal = MoveToGoal(index=index)
        capture_image = CaptureImage(label=index + 1) #add

        root.add_child(move_to_goal)
        root.add_child(capture_image)  #add

        tree = py_trees.trees.BehaviourTree(root)
        tree.setup(timeout=15)

        while rclpy.ok():
            tree.tick()
            rclpy.spin_once(move_to_goal.node, timeout_sec=0.1)
            rclpy.spin_once(capture_image.node, timeout_sec=0.1) #add

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
