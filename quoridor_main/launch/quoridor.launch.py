from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='quoridor_main',
            executable='game_orchestrator',
            name='game_orchestrator',
            output='screen'
        ),

        Node(
            package='quoridor_main',
            executable='robot_ctrl_node',
            name='robot_ctrl_node',
            output='screen'
        ),

        Node(
            package='quoridor_main',
            executable='speech_service_node',
            name='speech_service_node',
            output='screen'
        ),

        Node(
            package='quoridor_main',
            executable='board_ros_node',
            name='board_ros_node',
            output='screen'
        ),

        Node(
            package='quoridor_main',
            executable='detect_node',
            name='detect_node',
            output='screen'
        ),

        Node(
            package='quoridor_main',
            executable='clean_up_node',
            name='clean_up_node',
            output='screen'
        ),

        Node(
            package='quoridor_main',
            executable='wakeup_word_node_korean',
            name='wakeup_word_node_korean',
            output='screen'
        ),

        Node(
            package='quoridor_main',
            executable='rule_break_handler_node',
            name='rule_break_handler_node',
            output='screen'
        ),
    ])
