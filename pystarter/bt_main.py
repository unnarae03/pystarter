# bt_main.py

import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = Node('bt_executor')
    node.get_logger().info('✅ Behavior Tree 메인 실행 시작됨')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
