import rclpy
from rclpy.executors import MultiThreadedExecutor
import py_trees
import os
from ament_index_python.packages import get_package_share_directory

from pystarter.nodes.move_to_goal_node import MoveToGoal
# from pystarter.nodes.set_angle_node import SetAngleNode  # 유지


def waypoint_exists(index):
    filename = f"waypoint{index + 1}.yaml"
    config_path = os.path.join(
        get_package_share_directory("pystarter"),
        "config",
        filename
    )
    return os.path.exists(config_path)


def create_tree(index):
    root = py_trees.composites.Sequence(name="MainSequence", memory=False)

    move_to_goal = MoveToGoal(index=index)
    # set_angle = SetAngleNode(index=index)  # 주석 유지

    root.add_children([move_to_goal])
    return root, move_to_goal


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()

    index = 0
    while rclpy.ok():
        if not waypoint_exists(index):
            print("✅ 모든 웨이포인트 완료. 종료합니다.")
            break

        tree_root, move_to_goal_node = create_tree(index)
        behaviour_tree = py_trees.trees.BehaviourTree(tree_root)
        behaviour_tree.setup(timeout=15)

        status = py_trees.common.Status.RUNNING
        last_status = None

        while status == py_trees.common.Status.RUNNING and rclpy.ok():
            behaviour_tree.tick()
            rclpy.spin_once(move_to_goal_node.node, timeout_sec=0.1)
            status = behaviour_tree.root.status  # ✅ 올바른 위치에서 상태 체크

            if status != last_status:
                last_status = status
                if status == py_trees.common.Status.FAILURE:
                    print(f"❌ waypoint{index+1} 실패. 중단합니다.")
                elif status == py_trees.common.Status.SUCCESS:
                    print(f"✅ waypoint{index+1} 성공.")

        behaviour_tree.shutdown()
        move_to_goal_node.node.destroy_node()
        rclpy.shutdown()
        rclpy.init()

        if status == py_trees.common.Status.SUCCESS:
            index += 1
        else:
            break


if __name__ == "__main__":
    main()
