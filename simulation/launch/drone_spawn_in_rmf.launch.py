from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node
import os
import time

def kill_existing_drone_processes(context, *args, **kwargs):
    """关闭残留进程，但减少不必要的等待"""
    # 使用 -9 强制秒杀，无需温和等待
    os.system("pkill -9 px4")
    os.system("pkill -9 MicroXRCEAgent")
    # 0.1秒足够系统回收资源了，不需要睡1秒
    time.sleep(0.1) 
    return []

def generate_launch_description():
    px4_path = os.path.expanduser('~/PX4-Autopilot')
    
    # === 关键优化：直接定位到编译好的二进制文件 ===
    # 跳过 'make' 命令的编译检查，直接运行程序，启动速度极快
    px4_build_dir = os.path.join(px4_path, 'build/px4_sitl_default')
    px4_binary = os.path.join(px4_build_dir, 'bin/px4')
    px4_startup_script = os.path.join(px4_build_dir, 'etc/init.d-posix/rcS')
    px4_rootfs = os.path.join(px4_build_dir, 'tmp/rootfs')

    # 确保运行目录存在（防止第一次运行没编译过）
    if not os.path.exists(px4_rootfs):
        print("警告: 未找到编译目录，回退到使用 make 命令...")
        cmd_px4 = ['make', 'px4_sitl', 'none_iris']
        cwd_px4 = px4_path
    else:
        # 使用极速启动命令
        cmd_px4 = [px4_binary, px4_startup_script]
        cwd_px4 = px4_rootfs

    # 路径设置（保持你之前修复好的逻辑）
    px4_gazebo_models = os.path.join(px4_path, 'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models')
    my_model_path = '/home/suda/drone_ugv_ws/src/simulation/models'
    
    combined_model_path = os.path.join(
        os.getenv('GAZEBO_MODEL_PATH', ''),
        my_model_path,
        px4_gazebo_models,
        os.path.expanduser('~/.gazebo/models')
    ).replace(':', os.pathsep)

    env_vars = [
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=combined_model_path),
        SetEnvironmentVariable(name='PX4_SIM_MODEL', value='iris_depth_camera'),
        SetEnvironmentVariable(name='PX4_ESTIMATOR', value='ekf2'), # 显式指定估计器
        SetEnvironmentVariable(name='ROS_PARAMETER_OVERRIDES', value='use_sim_time:=true')
    ]

    # 启动 PX4 SITL (优化后)
    px4_sitl = ExecuteProcess(
        cmd=cmd_px4,
        cwd=cwd_px4,
        output='screen',
        shell=False, # 直接运行二进制不需要 shell=True
        name='px4_sitl'
    )

    # 启动 MicroXRCEAgent
    microxrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
        name='microxrce_agent'
    )

    # 指定无人机 SDF 文件路径
    drone_model_path = os.path.join(my_model_path, 'iris_depth_camera', 'iris_depth_camera.sdf')

    # Spawn 无人机
    spawn_drone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_drone',
        output='screen',
        arguments=[
            '-file', drone_model_path,
            '-entity', 'iris_depth_camera',
            '-x', '40.0', '-y', '-40.0', '-z', '0.2',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0'
        ]
    )

    # TF 变换
    static_tf_drone_camera = Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=["0.1", "0.0", "-0.05", "0.0", "0.785", "0.0", "drone_base_link", "drone_camera_link"],
        parameters=[{'use_sim_time': True}], output="screen"
    )

    static_tf_drone_optical = Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=["0.0", "0.0", "0.0", "-1.57079632679", "0.0", "1.57079632679", "drone_camera_link", "drone_camera_optical_frame"],
        parameters=[{'use_sim_time': True}], output="screen"
    )

    px4_bridge_node = Node(
        package='simulation', executable='px4_odometry_bridge',
        name='px4_odometry_bridge', output='screen', parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        OpaqueFunction(function=kill_existing_drone_processes),
        *env_vars,
        px4_sitl,
        # 调整时间轴：
        # Agent 1秒后启动 (无需等太久)
        TimerAction(period=1.0, actions=[microxrce_agent]),
        # Spawn 改为 6秒 (给 PX4 足够的初始化时间，防止 Gazebo 还在锁步暂停时就请求生成)
        TimerAction(period=6.0, actions=[spawn_drone]),
        # TF 和 Bridge 延后启动
        TimerAction(period=8.0, actions=[static_tf_drone_camera, static_tf_drone_optical, px4_bridge_node])
        #TimerAction(period=8.0, actions=[static_tf_drone_camera, static_tf_drone_optical])
    ])