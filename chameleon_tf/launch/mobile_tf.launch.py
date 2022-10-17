import launch
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package="chameleon_tf",
        executable="chameleon_tf",
        name="chameleon_tf",
        output="screen",
        parameters=[
            {"source_frame": "frame_a"},
            {"target_frame": "frame_b"},
            {"initial_translation": [
                0.0,
                0.0,
                0.0
            ]},
            {"initial_rotation": [
                0.0,
                0.0,
                0.0
            ]},
        ]
    )

    transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='dummy_tf',
        arguments=["0", "1.0", "1.0", "0", "0", "0", "frame_aa", "frame_bb"]
    )

    return launch.LaunchDescription([node, transform])

