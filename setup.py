from setuptools import find_packages, setup

package_name = 'pystarter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=[
        'setuptools',
        'py_trees',  # 추가된 필수 의존성
    ],
    zip_safe=True,
    maintainer='hayeon',
    maintainer_email='shinhy9@naver.com',
    description='Python package to integrate py_trees with ROS 2 navigation for robot control',
    license='MIT',  # 실제 사용되는 라이센스로 수정
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bt_main = pystarter.bt_main:main',
            'move_to_goal = pystarter.move_to_goal:main',  # MoveToGoal 실행 명령어
        ],
    },
)
