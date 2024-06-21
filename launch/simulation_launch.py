import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    xml_file_name = "model/model_1.xml"
    xml_file = os.path.join(get_package_share_path("aerial_system_mujoco"), xml_file_name)
    print(xml_file)
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value=xml_file,
            description='Path to the model XML file.'
        ),
        Node(
            package='aerial_system_mujoco',
            executable='drones_simulation',
            name='simulate_mujoco',
            output='screen',
            arguments=[LaunchConfiguration('model_path')],
        )
    ])