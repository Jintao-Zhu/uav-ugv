from launch import LaunchDescription  # 这是旧launch，现尝试把另一个launch的mavros改成MAVLink 
from launch.actions import (
    SetEnvironmentVariable, ExecuteProcess, OpaqueFunction, Shutdown,
    RegisterEventHandler, EmitEvent, TimerAction
)
from launch.event_handlers import OnProcessStart
from launch.events import Shutdown as ShutdownEvent
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node
import os
import time


def kill_existing_gazebo(context, *args, **kwargs):
    """关闭已运行的Gazebo、PX4和MicroXRCEAgent进程"""
    os.system("pkill -TERM gzserver")
    os.system("pkill -TERM gzclient")
    os.system("pkill -TERM gazebo")
    os.system("pkill -TERM px4")
    os.system("pkill -TERM MicroXRCEAgent")
    time.sleep(1)
    os.system("pkill -9 gzserver")
    os.system("pkill -9 gzclient")
    os.system("pkill -9 gazebo")
    os.system("pkill -9 px4")
    os.system("pkill -9 MicroXRCEAgent")
    time.sleep(1)
    return []


def generate_launch_description():
    px4_path = os.path.expanduser('~/PX4-Autopilot')

    # 环境变量
    env_vars = [
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[
                EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
                ':/home/suda/drone_ugv_ws/src/simulation/models',
                f':{px4_path}/Tools/sitl_gazebo/models',
                ':/home/suda/.gazebo/models'
            ]
        ),
        SetEnvironmentVariable(
            name='GAZEBO_PLUGIN_PATH',
            value=[
                EnvironmentVariable('GAZEBO_PLUGIN_PATH', default_value=''),
                ':/usr/lib/x86_64-linux-gnu/gazebo-11/plugins'
            ]
        ),
        SetEnvironmentVariable(
            name='LD_LIBRARY_PATH',
            value=[
                EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
                ':/usr/lib/x86_64-linux-gnu/gazebo-11/plugins'
            ]
        ),
        SetEnvironmentVariable(name='PX4_SIM_MODEL', value='iris_depth_camera'),
        SetEnvironmentVariable(name='ROS_PARAMETER_OVERRIDES', value='use_sim_time:=true')
    ]

    gazebo = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'sleep 2 && gazebo /home/suda/drone_ugv_ws/src/simulation/worlds/coordination_world.world -s libgazebo_ros_init.so -s libgazebo_ros_factory.so'
        ],
        output='screen',
        on_exit=Shutdown()
    )

    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'none_iris'],
        cwd=px4_path,
        output='screen',
        shell=True,
        on_exit=Shutdown(),
        name='px4_sitl'
    )

    microxrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
        name='microxrce_agent',
        on_exit=EmitEvent(event=ShutdownEvent(reason='MicroXRCEAgent exited'))
    )

    # MAVROS 配置文件路径
    mavros_config_path = '/home/suda/drone_ugv_ws/src/simulation/config/mavros_config.yaml'

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace='mavros',
        output='screen',
        parameters=[mavros_config_path],
        remappings=[
            ('local_position/pose', '/drone/pose'),
            ('local_position/odom', '/drone/odom')
        ]
    )

    drone_model_path = '/home/suda/drone_ugv_ws/src/simulation/models/iris_depth_camera/iris_depth_camera.sdf'
    ugv_model_path = '/home/suda/drone_ugv_ws/src/simulation/models/yahboomcar_X3/model.sdf'

    spawn_drone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_drone',
        output='screen',
        arguments=['-file', drone_model_path, '-entity', 'iris_depth_camera',
                   '-x', '0', '-y', '0', '-z', '0.3']
    )

    spawn_ugv = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_ugv',
        output='screen',
        arguments=['-file', ugv_model_path, '-entity', 'yahboomcar_X3',
                   '-x', '2', '-y', '0', '-z', '0.05']
    )

    # Pose to TF 节点
    pose_to_tf_node = Node(
        package='simulation',
        executable='pose_to_tf',
        name='pose_to_tf',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # UGV TF（仅保留与底盘固定的传感器TF；不要发布 map->ugv 的静态TF）

    static_tf_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0.0435", "--y", "0.000053", "--z", "0.1915",
                   "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
                   "--frame-id", "ugv_base_link", "--child-frame-id", "laser_link"],
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

    static_tf_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0.057105", "--y", "0.000018", "--z", "0.11905",
                   "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
                   "--frame-id", "ugv_base_link", "--child-frame-id", "camera_link"],
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

    # 可选：如果Nav2的robot_base_frame使用的是base_footprint，这里提供单位静态TF保证一致性
    static_tf_base_footprint = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
            "--frame-id", "base_footprint", "--child-frame-id", "ugv_base_link"
        ],
        parameters=[{'use_sim_time': True}],
        output="screen"
    )


    # Drone TF (固定相机)
    static_tf_drone_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0.1", "--y", "0.0", "--z", "-0.05",
            "--yaw", "0.0", "--pitch", "0.785", "--roll", "0.0",
            "--frame-id", "drone_base_link", "--child-frame-id", "drone_camera_link"
        ],
        parameters=[{'use_sim_time': True}],
        output="screen",
        name="static_tf_drone_camera_publisher"
    )

    # Drone TF (相机本体 -> 光学坐标系)
    static_tf_drone_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--roll", "-1.57079632679", "--pitch", "0.0", "--yaw", "1.57079632679",
            "--frame-id", "drone_camera_link", "--child-frame-id", "drone_camera_optical_frame"
        ],
        parameters=[{'use_sim_time': True}],
        output="screen",
        name="static_tf_drone_optical_publisher"
    )

    # 事件监听
    start_agent_after_px4 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=px4_sitl,
            on_start=[microxrce_agent]
        )
    )

    start_mavros_after_agent = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=microxrce_agent,
            on_start=[TimerAction(period=8.0, actions=[mavros_node])]
        )
    )

    start_tf_after_ugv = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_ugv,
            on_start=[static_tf_laser, static_tf_camera, static_tf_base_footprint]
        )
    )

    start_drone_tf_after_spawn = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_drone,
            on_start=[TimerAction(period=3.0, actions=[static_tf_drone_camera,
                                                      static_tf_drone_optical])]
        )
    )

    # 在MAVROS启动后启动pose_to_tf节点
    start_pose_to_tf_after_mavros = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=mavros_node,
            on_start=[TimerAction(period=2.0, actions=[pose_to_tf_node])]
        )
    )

    return LaunchDescription([
        OpaqueFunction(function=kill_existing_gazebo),
        *env_vars,
        gazebo,
        px4_sitl,
        start_agent_after_px4,
        start_mavros_after_agent,
        spawn_drone,
        spawn_ugv,
        start_tf_after_ugv,
        start_drone_tf_after_spawn,
        start_pose_to_tf_after_mavros,
        
    ])
