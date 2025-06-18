import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import py_trees
import os
import yaml
import time
import cv2
from tf_transformations import euler_from_quaternion

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
        self._saved = False
        self._start_time = None
        self.timeout_sec = 5.0

        # ✅ 저장 경로: ~/ros2_ws/src/pystarter/logs/images/current
        package_root = os.path.join(os.path.expanduser("~"), "ros2_ws", "src", "pystarter")
        self.save_dir = os.path.join(package_root, "logs", "images", "current")
        os.makedirs(self.save_dir, exist_ok=True)

        # ✅ 토픽 구독
        self.image_sub = self.node.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            qos_profile_sensor_data  # 센서 QoS 맞춤
        )
        self.pose_sub = self.node.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10  # 기본 QoS
        )

    def initialise(self):
        self.image_received = False
        self.pose_received = False
        self._saved = False
        self._start_time = time.time()

    def image_callback(self, msg):
        self.image_msg = msg
        self.image_received = True

    def pose_callback(self, msg):
        self.pose_msg = msg
        self.pose_received = True

    def update(self):
        if self._saved:
            return py_trees.common.Status.SUCCESS

        if self.image_received and self.pose_received:
            capture_id = self.get_next_capture_id()
            self.save_image_and_pose(capture_id)
            self._saved = True
            return py_trees.common.Status.SUCCESS

        if time.time() - self._start_time > self.timeout_sec:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def get_next_capture_id(self):
        filenames = os.listdir(self.save_dir)
        nums = []
        for fname in filenames:
            if fname.endswith(".jpg"):
                try:
                    num = int(os.path.splitext(fname)[0])
                    nums.append(num)
                except:
                    continue
        return max(nums, default=0) + 1

    def save_image_and_pose(self, capture_id):
        # 이미지 저장
        cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding='bgr8')
        img_path = os.path.join(self.save_dir, f"{capture_id}.jpg")
        cv2.imwrite(img_path, cv_image)

        # pose 저장
        pose = self.pose_msg.pose.pose
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        _, _, theta = euler_from_quaternion(q)

        pose_dict = {
            "pose": {
                "x": pose.position.x,
                "y": pose.position.y,
                "theta": theta
            }
        }
        yaml_path = os.path.join(self.save_dir, f"{capture_id}.yaml")
        with open(yaml_path, 'w') as f:
            yaml.dump(pose_dict, f)
