from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'pystarter'
here = os.path.dirname(__file__)  # setup.py 위치 = ~/ros2_ws/src/pystarter

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        
        # ✅ config/*.yaml 포함 (pystarter/config 내부)
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join(here, 'pystarter', 'config', '*.yaml'))),

        # ✅ logs/images/reference/*.jpg 포함
        (os.path.join('share', package_name, 'logs/images/reference'),
            glob(os.path.join(here, 'pystarter', 'logs', 'images', 'reference', '*.jpg'))),
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
            'move_to_goal = pystarter.move_to_goal_node:main',
            #'set_angle_node = pystarter.nodes.set_angle_node:main',
            'py_trees_main = pystarter.py_trees_main:main',
        ],
    },
)
