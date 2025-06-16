import rclpy
from rclpy.executors import MultiThreadedExecutor
import py_trees

from pystarter.nodes.move_to_goal_node import MoveToGoal
#from pystarter.nodes.set_angle_node import SetAngleNode


def create_tree():
    root = py_trees.composites.Sequence(name="MainSequence", memory=False)

    move_to_goal = MoveToGoal()
    #set_angle = SetAngleNode(index=0)# waypoint1.yaml 기준

    root.add_children([move_to_goal])
    return root, move_to_goal


def main():
    rclpy.init()

    # ROS 2 노드 실행기 (멀티 스레드)
    executor = MultiThreadedExecutor()

    tree, move_to_goal_node = create_tree()
    behaviour_tree = py_trees.trees.BehaviourTree(tree)
    behaviour_tree.setup(timeout=15)

    try:
        while rclpy.ok():
            behaviour_tree.tick()
            rclpy.spin_once(move_to_goal_node.node, timeout_sec=0.1)
            #rclpy.spin_once(set_angle_node.node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    behaviour_tree.shutdown()
    move_to_goal_node.node.destroy_node()
    #set_angle_node.node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
