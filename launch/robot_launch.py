import os
# import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('gijitaiken_webots')
    
    # 1. Konfigurasi Dunia
    world = LaunchConfiguration('world')
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='Hiro.wbt',
        description='Pilih world yang sudah ada robotnya'
    )
    
    # 2. Webots Simulator
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    # 3. Driver Robot Hiro (Webots Controller)
    # Controller Python akan di-load berdasarkan definisi di URDF
    hiro_driver = WebotsController(
        robot_name='Hiro',
        parameters=[
            {
                'robot_description': os.path.join(package_dir, 'resource', 'hiro.urdf'),
            }
        ],
        remappings=[
            ('/camera/image/image_color', '/camera/image'),
        ]
    )

    # 4. Utilities
    web_video_server_node = Node(
        package='web_video_server', executable='web_video_server',
        name='web_video_server',
        parameters=[{'port': 8080, 'address': '0.0.0.0'}]
    )

    rosbridge_node = Node(
        package='rosbridge_server', executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'port': 9090}] 
    )

    # 5. Hybrid Nodes (Sesuai CMakeLists.txt & Structure Baru)
    unbuffered_env = {'PYTHONUNBUFFERED': '1'}
    # IMU Node (Python Script)
    imu_adapter = Node(
        package='gijitaiken_webots', 
        executable='imu_main.py',       # Sesuai nama file di folder root package
        name='imu_adapter_node',
        output='screen',
        additional_env=unbuffered_env
    )

    # Logic Bridge / Tachimawari (Python Script)
    tachimawari_node = Node(
        package='gijitaiken_webots', 
        executable='logic_main.py',     # Sesuai nama file di folder root package
        name='tachimawari_node', 
        output='screen',
        additional_env=unbuffered_env
    )

    # Vision Node (C++ Executable) -> BARU
    vision_node = Node(
        package='gijitaiken_webots',
        executable='vision_node_exec',  # Sesuai target add_executable di CMakeLists
        name='vision_controller',
        output='screen',
        additional_env=unbuffered_env
    )

    return LaunchDescription([
        declare_world,
        webots,
        hiro_driver,
        web_video_server_node,
        rosbridge_node,
        imu_adapter,
        tachimawari_node,
        vision_node, # Jangan lupa daftarkan vision node

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        )
    ])