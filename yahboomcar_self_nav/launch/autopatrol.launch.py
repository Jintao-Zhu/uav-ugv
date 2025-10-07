import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 获取与拼接默认路径
    autopatrol_robot_dir = get_package_share_directory(
        'yahboomcar_self_nav')
    patrol_config_path = os.path.join(
        autopatrol_robot_dir, 'config', 'patrol_config.yaml')
    
    action_node_stop_control = launch_ros.actions.Node(
        package='yahboomcar_self_nav',
        executable='patrol_node_stop',
        parameters=[patrol_config_path]
    )
    
    action_node_return_control = launch_ros.actions.Node(
        package='yahboomcar_self_nav',
        executable='patrol_node_return',
        parameters=[patrol_config_path]
    )

    return launch.LaunchDescription([
        action_node_stop_control,
        #action_node_return_control,#如果要启动patrol_node_return，需要更换注释
    ])