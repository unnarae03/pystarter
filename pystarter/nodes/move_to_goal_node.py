import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import py_trees


class GoalSubscriber(Node):
    def __init__(self):
        super().__init__("goal_listener")
        self.subscription = self.create_subscription(
            PoseStamped,
            "/goal_pose",  # RViz에서 클릭한 2D Nav Goal 토픽
            self.goal_callback,
            10
        )
        self.blackboard = py_trees.blackboard.Blackboard()

    def goal_callback(self, msg):
        self.get_logger().info(f"🎯 RViz goal received: x={msg.pose.position.x}, y={msg.pose.position.y}")
        self.blackboard.set("goal_pose", msg)


class MoveToGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name="MoveToGoal"):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.node = rclpy.create_node("move_to_goal_node")
        self.client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.initialized = False

    def initialise(self):
        # 액션 서버와 연결 대기
        if self.client.wait_for_server(timeout_sec=2.0):
            self.initialized = True
            self.node.get_logger().info("✅ NavigateToPose 액션 서버 연결 완료")
        else:
            self.node.get_logger().error("❌ 액션 서버 연결 실패")
            self.initialized = False

    def update(self):
        if not self.initialized:
            return py_trees.common.Status.FAILURE

        # goal_pose가 blackboard에서 없으면 대기
        goal: PoseStamped = self.blackboard.get("goal_pose")
        if goal is None:
            self.node.get_logger().warn("⚠️ goal_pose가 blackboard에 없음")
            return py_trees.common.Status.RUNNING  # 계속 대기

        # goal 메시지 생성 후 액션 서버로 전송
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        self.node.get_logger().info(
            f"📤 목표 전송: x={goal.pose.position.x}, y={goal.pose.position.y}"
        )
        self.client.send_goal_async(goal_msg)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.node.get_logger().info("MoveToGoal 종료됨.")


def main():
    rclpy.init()

    # 멀티스레드 실행기 (여러 노드 동시 실행)
    executor = MultiThreadedExecutor()
    goal_listener = GoalSubscriber()
    executor.add_node(goal_listener)

    # MoveToGoal 노드 트리
    move_to_goal_node = MoveToGoal()
    tree = py_trees.trees.BehaviourTree(root=move_to_goal_node)
    tree.setup(timeout=15)

    try:
        while rclpy.ok():
            tree.tick()  # 트리 실행
            rclpy.spin_once(goal_listener, timeout_sec=0.1)  # goal_listener 동작
            rclpy.spin_once(move_to_goal_node.node, timeout_sec=0.1)  # MoveToGoal 노드 실행
    except KeyboardInterrupt:
        pass

    tree.shutdown()
    goal_listener.destroy_node()
    move_to_goal_node.node.destroy_node()
    rclpy.shutdown()
