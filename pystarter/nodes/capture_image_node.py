import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import cv2
import os
import yaml
import py_trees
from tf_transformations import euler_from_quaternion
from ament_index_python.packages import get_package_share_directory


class CaptureImage(py_trees.behaviour.Behaviour):
    def __init__(self, label=1):
        super().__init__(name=f"CaptureImage_{label}")
        self.label = label
        self.node = rclpy.create_node(f"capture_image_node_{label}")
        self.bridge = CvBridge()

        self.image_msg = None
        self.pose_msg = None
        self.image_received = False
        self.pose_received = False

        self._triggered = False  # 트리 진입 시에만 저장되도록 설정
        self._saved = False

        # ✅ Topic subscribe
        self.image_sub = self.node.create_subscription(
            Image, '/oakd/rgb/preview/image_raw', self.image_callback, 10)
        self.pose_sub = self.node.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        # ✅ Save path: ~/ros2_ws/src/pystarter/pystarter/logs/images/current
        base_dir = get_package_share_directory("pystarter")
        self.save_dir = os.path.join(base_dir, "logs", "images", "current")
        os.makedirs(self.save_dir, exist_ok=True)

    def initialise(self):
        self._triggered = True  # 트리 진입 시점이 waypoint 도착으로 간주
        self._saved = False

    def image_callback(self, msg):
        self.image_msg = msg
        self.image_received = True

    def pose_callback(self, msg):
        self.pose_msg = msg
        self.pose_received = True

    def update(self):
        rclpy.spin_once(self.node, timeout_sec=0.5)

        if self._saved:
            return py_trees.common.Status.SUCCESS

        if self._triggered and self.image_received and self.pose_received:
            capture_id = self.get_next_capture_id()
            self.save_image_and_pose(capture_id)
            self._saved = True
            print(f"[CaptureImage] ✅ Saved {capture_id}.jpg and {capture_id}.yaml")
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def get_next_capture_id(self):
        files = [f for f in os.listdir(self.save_dir) if f.endswith('.jpg')]
        return len(files) + 1

    def save_image_and_pose(self, capture_id):
        # ✅ 이미지 저장
        cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding='bgr8')
        img_path = os.path.join(self.save_dir, f"{capture_id}.jpg")
        cv2.imwrite(img_path, cv_image)

        # ✅ Pose → theta 변환 후 저장
        pose = self.pose_msg.pose.pose
        position = pose.position
        orientation = pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, theta = euler_from_quaternion(q)

        pose_data = {
            'pose': {
                'theta' : theta,
                'x': position.x,
                'y': position.y,
            }
        }
        yaml_path = os.path.join(self.save_dir, f"{capture_id}.yaml")
        with open(yaml_path, 'w') as f:
            yaml.dump(pose_data, f)
