from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Ambil path ke file config yang akan kita *overwrite*
    aruku_node_pkg = get_package_share_directory('aruku_node')
    config_file = os.path.join(aruku_node_pkg, 'config', 'kinematic.json')

    return LaunchDescription([
        Node(
            package='aruku_node',
            executable='aruku_node',
            name='aruku_node',
            output='screen',
            # Pastikan 'aruku_node' memuat ulang config setiap kali di-launch
            # (Jika 'aruku_node' C++ Anda memuat config sekali saja, ini sudah cukup)
        ),
        Node(
            package='mujoco_node',
            executable='simulator_node',
            name='mujoco_node',
            output='screen'
        ),
    ])