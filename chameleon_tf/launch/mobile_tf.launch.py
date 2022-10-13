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

    return launch.LaunchDescription([node])

