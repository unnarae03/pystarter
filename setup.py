from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'pystarter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # 🔧 추가됨: config 폴더 안의 YAML 파일들 설치
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # 🔧 추가됨: 이미지 리소스 설치
        (os.path.join('share', package_name, 'logs/images/reference'), glob('logs/images/reference/*.jpg')),
    ],
    install_requires=[
        'setuptools',
        'py_trees',
    ],
    zip_safe=True,
    maintainer='hayeon',
    maintainer_email='shinhy9@naver.com',
    description='Python package to integrate py_trees with ROS 2 navigation for robot control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bt_main = pystarter.bt_main:main',
            'move_to_goal = pystarter.move_to_goal_node:main',  # 선택적: 단독 실행 필요할 때만 유지
            'set_angle_node = pystarter.nodes.set_angle_node:main',
        ],
    },
)
