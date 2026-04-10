from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='main_task26',
            executable='bangun',
            name='bangun_node',
            output='screen'
        ),

        Node(
            package='main_task26',
            executable='jalan',
            name='jalan_node',
            output='screen'
        ),

        Node(
            package='main_task26',
            executable='button',
            name='button_node',
            output='screen'
        ),

        Node(
            package='main_task26',
            executable='maintask',
            name='main_task_node',
            output='screen'
        ),

        Node(
            package='main_task26',
            executable='headcontrol',
            name='head_node',
            output='screen'
        ),

        Node(
            package='main_task26',
            executable='vision',
            name='vision_node',
            output='screen'
        ),

        Node(
            package='main_task26',
            executable='jarak',
            name='jarak_node',
            output='screen'
        ),
    ])