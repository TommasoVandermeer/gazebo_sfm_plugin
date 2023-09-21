import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_file_name = 'world_sfm.world'
    pkg_dir = get_package_share_directory('gazebo_sfm_plugin')

    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, 'models')

    world = os.path.join(pkg_dir, 'worlds', world_file_name)

    config = os.path.join(pkg_dir,'config','world_sfm_agents.yaml')
    
    agents_loader=Node(
        package = 'gazebo_sfm_plugin',
        name = 'agent_params_loader',
        executable = 'load_agent_params',
        parameters = [config]
    )

    spawn_entity = Node(package='gazebo_sfm_plugin', executable='spawn_robot.py',
                        arguments=['pioneer3at', 'demo', '-0.2', '10.0', '0.0'],
                        output='screen')

    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'], #, '--pause'
            output='screen')

    return LaunchDescription([
        spawn_entity,
        agents_loader,
        gazebo,
    ])
