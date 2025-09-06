from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot',
            executable='blade_node',
            name='blade_node',
            output='screen'
        ),
        Node(
            package='robot',
            executable='arm_node',
            name='arm_node',
            output='screen'
        ),
        Node(
            package='robot',
            executable='pump_node',
            name='pump_node',
            output='screen'
        ),
        Node(
            package='robot',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='camera_ros',
            executable='camera_node',
            name='imx219_cam',
            parameters=[
                {"camera": "/base/axi/pcie@120000/rp1/i2c@88000/imx219@10",
                "role": "viewfinder",
                "format": "BGR888",
                "width": 640, "height": 480, "fps": 15,
                "image_transport": "compressed"},
                'config/camera_compressed.yaml'    # ← 이 줄 추가
            ],
            output='screen'
        ),

    ])
