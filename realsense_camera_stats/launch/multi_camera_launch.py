from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_camera_launch_description(serial_no, camera_namespace, enable_color, enable_gyro, enable_accel, enable_infra1, enable_infra2,
                                       camera_name, rgb_camera_profile, depth_module_profile,
                                       depth_module_enable_auto_exposure, depth_module_emitter_enabled):
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
            'enable_color': enable_color,
            'enable_gyro': enable_gyro,
            'enable_accel': enable_accel,
            'enable_infra1': enable_infra1,
            'enable_infra2': enable_infra2,
            'camera_name': camera_name,
            'rgb_camera.profile': rgb_camera_profile,
            'depth_module.profile': depth_module_profile,
            'depth_module.enable_auto_exposure': depth_module_enable_auto_exposure,
            'depth_module.emitter_enabled': depth_module_emitter_enabled,
        }.items(),
    )

def generate_camera_info_nodes(camera_namespace):
    color_topics = ['color']
    infra_topics = ['infra1', 'infra2']
    nodes = []

    for topic in color_topics:
        node = Node(
            package='realsense_camera_stats',
            executable='camera_info_node',
            name=f'{camera_namespace}_{topic}_info_node',
            parameters=[{'camera_topic': f'/{camera_namespace}/camera/{topic}/image_raw'}],
            remappings=[('/camera_info_stats', f'/{camera_namespace}/{topic}/stats')]
        )
        nodes.append(node)

    for topic in infra_topics:
        node = Node(
            package='realsense_camera_stats',
            executable='camera_info_node',
            name=f'{camera_namespace}_{topic}_info_node',
            parameters=[{'camera_topic': f'/{camera_namespace}/camera/{topic}/image_rect_raw'}],
            remappings=[('/camera_info_stats', f'/{camera_namespace}/{topic}/stats')]
        )
        nodes.append(node)
    return nodes

def generate_launch_description():
    camera_configs = [
        {"serial_no": "_151422253428", "namespace": "camera1"},
        {"serial_no": "_142422250867", "namespace": "camera2"},
        {"serial_no": "_231622300684", "namespace": "camera3"},
    ]
    
    launch_actions = []
    for idx, config in enumerate(camera_configs, start=1):
        serial_arg = DeclareLaunchArgument(f'serial_no_{idx}', default_value=config["serial_no"])
        camera_launch = generate_camera_launch_description(
            LaunchConfiguration(f'serial_no_{idx}'),
            config["namespace"],
            'false',  # enable_color
            'true',  # enable_gyro
            'true',  # enable_accel
            'true',  # enable_infra1
            'true',  # enable_infra2
            config.get("camera_name", "camera"),  # camera_name
            config.get("rgb_camera_profile", "424x240x60"),  # rgb_camera.profile
            config.get("depth_module_profile", "480x270x60"),  # depth_module.profile
            config.get("depth_module_enable_auto_exposure", "false"),  # depth_module.enable_auto_exposure : if depth_module.exposure is set, this doesn't work
            config.get("depth_module_emitter_enabled", "0"),  # depth_module.emitter_enabled : 0 - disables emitter, 1 - enables emitter, 2- emitter auto
        )
        info_nodes = generate_camera_info_nodes(config["namespace"])

        launch_actions.append(serial_arg)
        launch_actions.append(camera_launch)
        launch_actions.extend(info_nodes)

    return LaunchDescription(launch_actions)
