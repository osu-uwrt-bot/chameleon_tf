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
            {"transform_locks": [
                False,      # unlock x
                False,      # unlock y
                True,       # lock z
                True,       # lock roll
                True,       # lock pitch
                False       # unlock yaw
            ]}
        ]
    )

    transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='dummy_tf',
        arguments=[
                "--x", "0",
                "--y", "1.0", 
                "--z", "1.0", 
                "--roll", "0", 
                "--pitch", "0", 
                "--yaw", "1.5707", 
                "--frame-id", "frame_aa", 
                "--child-frame-id", "frame_bb"
            ]
    )

    # test action cli call
    # ros2 action send_goal /frame_b/model_tf chameleon_tf_msgs/action/ModelFrame -f "{monitor_parent: 'frame_aa', monitor_child: 'frame_bb'}"

    return launch.LaunchDescription([node, transform])

