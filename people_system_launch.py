from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transform: base_link -> scan (LiDAR)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_scan',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'scan']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),

        # Static transform: base_link -> camera_link (camera 1.3m above)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_camera',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
        ),

        # Your test node (manual point publishing)
        Node(
            package='people_to_costmap',
            executable='test_fixed_person_publisher',
            name='test_fixed_person_publisher',
            parameters=[{'target_frame': 'base_link'}]
        ),

        # Optional: start nav2
        # Node(
        #     package='nav2_bringup',
        #     executable='bringup_launch.py',
        #     output='screen',
        #     arguments=['slam:=True'],
        # ),
    ])

