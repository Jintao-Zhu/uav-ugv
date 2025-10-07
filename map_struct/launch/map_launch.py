from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument
)
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    map_struct_dir = get_package_share_directory('map_struct')
    gazebo_ros_share = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(map_struct_dir, 'worlds', 'yahboomcar.world'),
        description='Path to the Gazebo world file'
    )
    
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'false'
        }.items()
    )
    
    
    return LaunchDescription([
        world_arg,
        start_gazebo, 
    ])
