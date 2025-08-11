from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    YoloCppNode = Node(
        package='yolocpp',
        executable='yolocpp',
        name='yolocppnode',
        output='screen',
        parameters=[
            {'kp': 0.001},
            {'ki': 0.0},
            {'kd': 0.002},
            {'filter_alpha': 0.8},
            {'integral_limit': 10.0}
        ]
    )

    return LaunchDescription([
        YoloCppNode
    ])