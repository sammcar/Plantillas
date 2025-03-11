from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Argumentos usuales del launch
    use_sim_time = LaunchConfiguration('use_sim_time')
    package_name = 'paquete del launch'
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    
    # Importar nodos
    nodo_ejemplo = Node(
        package='paquete del nodo',
        executable='nodo.py',
        arguments=["Argumentos de nodo (si aplica)"],
        output='screen'
    )


    return LaunchDescription([
        nodo_ejemplo,
    ])