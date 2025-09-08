from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot',
            executable='blade_node',
            name='blade_node',
            output='screen',
            parameters= [{'pwm_active_high': True}]
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
        # Node(
        #     package='robot',
        #     executable='teleop_node',
        #     name='teleop_node',
        #     output='screen',
        #     emulate_tty=True
        # ),
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
                'config/camera_compressed.yaml'   
            ],
            output='screen'
        ),
        Node(
            package='robot',
            executable='auto_flip',
            name='auto_flip',
            parameters=[{
                'imu_topic': '/imu/data',  # 실제 IMU 토픽명으로 맞추세요
                'in_topic':  '/imx219_cam/image_raw/compressed',
                'out_topic': '/imx219_cam/image_flipped/compressed',
                'invert_threshold_deg': 120.0,  # 이 이상 기울면 뒤집기 ON
                'recover_threshold_deg': 60.0   # 이 이하로 돌아오면 뒤집기 OFF (히스테리시스)
            }],
            output='screen'
        ),

    ])
