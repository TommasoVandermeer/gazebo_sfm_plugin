from os import environ, path

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_dir = get_package_share_directory('gazebo_sfm_plugin')
    my_gazebo_models = path.join(pkg_dir,'models')
    print('my models:', my_gazebo_models)

    world_path = path.join(pkg_dir,'worlds','cafe3.world')

    environ["GAZEBO_MODEL_PATH"] = my_gazebo_models

    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'],
            output='screen')

    return LaunchDescription([
        gazebo,
    ])


# Add boolean commands if true
def _boolean_command(arg):
    cmd = ['"--', arg, '" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd
