from launch import LaunchDescription
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

    # PX4 odometry bridge (发布 map->drone_base_link)
    px4_bridge_node = Node(
        package='simulation',
        executable='px4_odometry_bridge',
        name='px4_odometry_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Pose to TF 节点
    pose_to_tf_node = Node(
        package='simulation',
        executable='pose_to_tf',
        name='pose_to_tf',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 静态 TF: map -> odom (单位变换)
    static_tf_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{'use_sim_time': True}],
        output="screen",
        name="static_tf_map_to_odom"
    )

    # UGV TF（传感器）
    static_tf_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0435", "0.000053", "0.1915",
                   "0.0", "0.0", "0.0",
                   "ugv_base_link", "laser_link"],
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

    static_tf_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.057105", "0.000018", "0.11905",
                   "0.0", "0.0", "0.0",
                   "ugv_base_link", "camera_link"],
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

    static_tf_base_footprint = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0",
                   "0", "0", "0",
                   "base_footprint", "ugv_base_link"],
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

    # Drone TF (固定相机，挂在 drone_base_link 下面)
    static_tf_drone_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.1", "0.0", "-0.05",
                   "0.0", "0.785", "0.0",
                   "drone_base_link", "drone_camera_link"],
        parameters=[{'use_sim_time': True}],
        output="screen",
        name="static_tf_drone_camera_publisher"
    )

    static_tf_drone_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.0", "0.0",
                   "-1.57079632679", "0.0", "1.57079632679",
                   "drone_camera_link", "drone_camera_optical_frame"],
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

    start_bridge_after_agent = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=microxrce_agent,
            on_start=[TimerAction(period=3.0, actions=[px4_bridge_node,
                                                      pose_to_tf_node,
                                                      static_tf_map_odom])]
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

    return LaunchDescription([
        OpaqueFunction(function=kill_existing_gazebo),
        *env_vars,
        gazebo,
        px4_sitl,
        start_agent_after_px4,
        start_bridge_after_agent,
        spawn_drone,
        spawn_ugv,
        start_tf_after_ugv,
        start_drone_tf_after_spawn,
    ])
