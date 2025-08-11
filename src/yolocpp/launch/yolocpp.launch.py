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
            {'kd': 0.0},
            {'kf': 0.0},
            {'filter_alpha': 0.7},
            {'system_delay': 0.05}
        ]
    )

    return LaunchDescription([
        YoloCppNode
    ])