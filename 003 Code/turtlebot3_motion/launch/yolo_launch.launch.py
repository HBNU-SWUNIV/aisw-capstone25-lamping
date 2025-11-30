from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_motion',
            executable='camera_bridge',
            name='camera_bridge',
            output='screen'
        ),
        #Node(
        #    package='turtlebot3_motion',
        #    executable='camera_publisher',
        #    name='camera_publisher',
        #    output='screen'
        #),
        Node(
            package='turtlebot3_motion',
            executable='yolo_node',
            name='yolo_node',
            output='screen'
        ),
        # 필요 시 주석 해제
        # Node(
        #     package='turtlebot3_motion',
        #     executable='warning_sound_node',
        #     name='warning_sound_node',
        #     output='screen'
        # ),
        Node(
            package='turtlebot3_motion',
            executable='static_obstacle_node',
            name='static_obstacle_node',
        )
    ])
