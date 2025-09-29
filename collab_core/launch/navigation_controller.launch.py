#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('collab_core')
    config_file = os.path.join(pkg_dir, 'config', 'navigation_params.yaml')
    
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # 导航控制节点
    navigation_controller_node = Node(
        package='collab_core',
        executable='navigation_controller',
        name='navigation_controller',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # 可以在这里添加话题重映射
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        navigation_controller_node,
    ])