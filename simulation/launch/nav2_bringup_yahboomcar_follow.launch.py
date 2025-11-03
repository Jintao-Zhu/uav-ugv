#!/usr/bin/env python3
# 增强稳定性的Nav2启动文件
#zjt 9.2晚所写
# 启动 Nav2 导航栈，搭建导航所需的基础设施，为无人车提供一套 “开箱即用、稳定可靠” 的导航系统
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    生成更稳定的Nav2系统 - 针对跟随任务优化
    """
    
    # ============================================
    # 包路径配置
    # ============================================
    simulation_pkg = FindPackageShare('simulation')
    nav2_bringup_pkg = FindPackageShare('nav2_bringup')
    
    # ============================================
    # 启动参数声明
    # ============================================
    launch_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='使用仿真时间'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                simulation_pkg, 'config', 'nav2_params.yaml'
            ]),
            description='Nav2参数文件'
        ),
        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=PathJoinSubstitution([
                nav2_bringup_pkg, 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'
            ]),
            description='行为树XML文件路径'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='自动启动导航栈'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',  # 默认关闭RViz减少资源消耗
            description='启动RViz2可视化'
        )
    ]
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # ============================================
    # 环境变量 - 优化性能和日志
    # ============================================
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )
    
    # 减少DDS的开销
    rmw_fastrtps_envvar = SetEnvironmentVariable(
        'RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'
    )
    
    # 优化内存使用
    malloc_trim_envvar = SetEnvironmentVariable(
        'MALLOC_TRIM_THRESHOLD_', '100000'
    )
    
    # ============================================
    # TF变换发布器 - 分阶段启动
    # ============================================
    map_odom_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_publisher', 
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 注意：不要发布 odom->base_footprint 的静态TF，移动底盘应由里程计/驱动发布动态TF
    
    # ============================================
    # Nav2节点 - 增强稳定性配置
    # ============================================
    
    # 控制器服务器 - 增加重启策略
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            # 增加稳定性参数
            'controller_frequency': 10.0,  # 降低频率减少CPU负载
            'min_x_velocity_threshold': 0.001,
            'min_y_velocity_threshold': 0.001,
            'min_theta_velocity_threshold': 0.001,
        }],
        remappings=[
            ('cmd_vel', '/yahboomcar/cmd_vel'),
            ('odom', '/yahboomcar/odom'),
            ('scan', '/yahboomcar/scan')
        ],
        # 增加重启策略
        respawn=True,
        respawn_delay=2.0
    )
    
    # 路径规划器 - 优化内存使用
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'planner_frequency': 1.0,  # 跟随模式下可以降低规划频率
            'expected_planner_frequency': 1.0
        }],
        remappings=[
            ('odom', '/yahboomcar/odom'),
            ('scan', '/yahboomcar/scan')
        ],
        respawn=True,
        respawn_delay=2.0
    )
    
    # 行为树导航器 - 简化配置
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'default_bt_xml_filename': default_bt_xml_filename,
            'bt_loop_duration': 10,  # 降低BT循环频率
            'default_server_timeout': 20
        }],
        remappings=[('odom', '/yahboomcar/odom')],
        respawn=True,
        respawn_delay=2.0
    )
    
    # 恢复行为服务器
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'costmap_topic': 'local_costmap/costmap_raw',
            'footprint_topic': 'local_costmap/published_footprint',
            'cycle_frequency': 5.0  # 降低恢复行为频率
        }],
        respawn=True,
        respawn_delay=2.0
    )
    
    # 速度平滑器 - 针对跟随优化
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'smoothing_frequency': 20.0,
            'scale_velocities': False,  # 跟随模式下关闭速度缩放
            'feedback': 'OPEN_LOOP',
            'max_velocity': [1.5, 0.0, 1.5],  # 增加最大速度
            'min_velocity': [-1.5, 0.0, -1.5],
            'deadband_velocity': [0.0, 0.0, 0.0],
            'velocity_timeout': 1.0,
            'acceleration': [2.0, 0.0, 2.0],  # 增加加速度
            'deceleration': [-2.0, 0.0, -2.0]
        }],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', '/yahboomcar/cmd_vel'),
            ('odom', '/yahboomcar/odom')
        ],
        respawn=True,
        respawn_delay=2.0
    )
    
    # ============================================
    # 增强的生命周期管理器
    # ============================================
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                    'controller_server',
                    'planner_server', 
                    'behavior_server',
                    'bt_navigator',
                    'velocity_smoother'
                ],
                'bond_timeout': 4.0,  # 增加bond超时时间
                'attempt_respawn_reconnection': True  # 启用重连尝试
            }
        ],
        respawn=True,
        respawn_delay=2.0
    )
    
    # ============================================
    # 可选的RViz
    # ============================================
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            nav2_bringup_pkg, 'rviz', 'nav2_default_view.rviz'
        ])],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # ============================================
    # 系统监控节点 - 自定义
    # ============================================
    system_monitor = Node(
        package='simulation',  # 假设在您的包中
        executable='nav2_monitor.py',  # 需要创建这个脚本
        name='nav2_system_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        respawn=True,
        respawn_delay=5.0
    )
    
    # ============================================
    # 构建Launch描述 - 分阶段启动
    # ============================================
    ld = LaunchDescription()
    
    # 添加环境变量
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(rmw_fastrtps_envvar)
    ld.add_action(malloc_trim_envvar)
    
    # 添加启动参数
    for arg in launch_args:
        ld.add_action(arg)
    
    # 第1阶段：TF变换（立即启动）
    ld.add_action(map_odom_publisher)
    # ld.add_action(odom_base_publisher)
    
    # 第2阶段：核心导航节点（延迟1秒启动）
    ld.add_action(TimerAction(
        period=1.0,
        actions=[controller_server, planner_server]
    ))
    
    # 第3阶段：辅助节点（延迟2秒启动）
    ld.add_action(TimerAction(
        period=2.0,
        actions=[behavior_server, velocity_smoother]
    ))
    
    # 第4阶段：高级功能（延迟3秒启动）
    ld.add_action(TimerAction(
        period=3.0,
        actions=[bt_navigator]
    ))
    
    # 第5阶段：生命周期管理器（延迟4秒启动）
    ld.add_action(TimerAction(
        period=4.0,
        actions=[lifecycle_manager]
    ))
    
    # 可选：RViz（延迟5秒启动）
    ld.add_action(TimerAction(
        period=5.0,
        actions=[rviz_node]
    ))
    
    # 系统监控（延迟6秒启动）
    # ld.add_action(TimerAction(
    #     period=6.0,
    #     actions=[system_monitor]
    # ))
    
    return ld