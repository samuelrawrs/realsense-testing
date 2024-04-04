from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_camera_launch_description(serial_no, camera_namespace, camera_name, enable_gyro, enable_accel, enable_infra1, enable_infra2):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),  
                'launch',
                'rs_launch.py'  
            ])
        ]),
        launch_arguments={
            'serial_no': serial_no,
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,  # Added argument
            'enable_gyro': enable_gyro,  # Added argument
            'enable_accel': enable_accel,  # Added argument
            'enable_infra1': enable_infra1,  # Added argument
            'enable_infra2': enable_infra2  # Added argument
        }.items(),
    )


def generate_launch_description():
    # Declare arguments for serial numbers
    serial_arg_1 = DeclareLaunchArgument('serial_no_1', default_value='_151422253428')
    #_142422250867
    # Camera 1
    camera1_launch = generate_camera_launch_description(
        LaunchConfiguration('serial_no_1'),
        'camera',  # camera namespace
        'camera1',  # camera name
        'true',  # enable_gyro
        'true',  # enable_accel
        'true',  # enable_infra1
        'true'   # enable_infra2
    )

    
    camera1_rgb_info_node = Node(
        package='realsense_camera_stats',
        executable='camera_info_node',
        name='camera1_rgb_info_node',
        parameters=[{'camera_topic': '/camera/camera1/color/image_raw'}],
        remappings=[('/camera_info_stats', '/camera1/color/stats')]
    )

    camera1_infra1_info_node = Node(
        package='realsense_camera_stats',
        executable='camera_info_node',
        name='camera1_infra1_info_node',
        parameters=[{'camera_topic': '/camera/camera1/infra1/image_rect_raw/image_raw'}],
        remappings=[('/camera_info_stats', '/camera1/infra1/stats')]
    )

    camera1_infra2_info_node = Node(
        package='realsense_camera_stats',
        executable='camera_info_node',
        name='camera1_infra2_info_node',
        parameters=[{'camera_topic': '/camera/camera1/infra2/image_rect_raw/image_raw'}],
        remappings=[('/camera_info_stats', '/camera1/infra2/stats')]
    )
        
    return LaunchDescription([
        serial_arg_1,
        camera1_launch,
        camera1_rgb_info_node,
        camera1_infra1_info_node,
        camera1_infra2_info_node
    ])
