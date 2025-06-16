import rclpy
from rclpy.executors import MultiThreadedExecutor
import py_trees
import os

from pystarter.nodes.move_to_goal_node import MoveToGoal

def waypoint_exists(index):
    from ament_index_python.packages import get_package_share_directory
    config_path = os.path.join(
        get_package_share_directory("pystarter"),
        "config",
        f"waypoint{index}.yaml"
    )
    return os.path.exists(config_path)

class MoveToGoalBT(py_trees.behaviour.Behaviour):
    def __init__(self, index):
        super().__init__(name=f"MoveToGoal_{index}")
        self.index = index
        self.node = MoveToGoal()

    def update(self):
        pose = self.node.load_waypoint(self.index)
        if pose is None:
            self.logger.warning(f"[BT] waypoint{self.index}.yaml 없음")
            return py_trees.common.Status.FAILURE

        future = self.node.send_goal(pose)
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.logger.error(f"[BT] Goal {self.index} rejected")
            return py_trees.common.Status.FAILURE

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result()

        if result.status != 4:
            self.logger.info(f"[BT] Goal {self.index} 성공")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.warning(f"[BT] Goal {self.index} 실패")
            return py_trees.common.Status.FAILURE


def create_tree(index):
    root = py_trees.composites.Sequence(name=f"TreeForWaypoint{index}")
    move = MoveToGoalBT(index)
    root.add_child(move)
    return root, move.node  # node는 rclpy.spin() 위해 따로 넘김


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    index = 1

    try:
        while rclpy.ok() and waypoint_exists(index):
            root, ros_node = create_tree(index)
            behaviour_tree = py_trees.trees.BehaviourTree(root)
            behaviour_tree.setup(timeout=15)

            while rclpy.ok():
                status = behaviour_tree.tick()
                rclpy.spin_once(ros_node, timeout_sec=0.1)
                if status != py_trees.common.Status.RUNNING:
                    break

            index += 1

        print("✅ 모든 waypoint 완료")

    except KeyboardInterrupt:
        print("🛑 사용자 종료")

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
