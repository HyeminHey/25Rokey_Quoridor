from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'quoridor_main'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyemin',
    maintainer_email='hera27seo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'game_orchestrator = quoridor_main.game_orchestrator_node:main',
            'robot_ctrl = quoridor_main.robot_ctrl_node:main',
            'robot_ctrl_client = quoridor_main.robot_ctrl_client:main',
            'speech_service = quoridor_main.speech_service_node:main',
            'speech_service_client = quoridor_main.speech_service_client:main',
            'board_ros = quoridor_main.board_ros_node:main',
            'board_ros_client = quoridor_main.test_cli:main',
        ],
    },
)
