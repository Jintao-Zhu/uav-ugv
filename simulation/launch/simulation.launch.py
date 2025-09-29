from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable, ExecuteProcess, OpaqueFunction, Shutdown,
    RegisterEventHandler, EmitEvent
)
from launch.event_handlers import OnProcessStart
from launch.events import Shutdown as ShutdownEvent
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node
import os

def kill_existing_gazebo(context, *args, **kwargs):
    """关闭已运行的Gazebo、PX4和MicroXRCEAgent进程"""
    import time
    # 先尝试优雅关闭
    os.system("pkill -TERM gzserver")
    os.system("pkill -TERM gzclient") 
    os.system("pkill -TERM gazebo")
    os.system("pkill -TERM px4")
    os.system("pkill -TERM MicroXRCEAgent")
    
    # 等待一秒钟让进程正常退出
    time.sleep(1)
    
    # 强制杀死残留进程
    os.system("pkill -9 gzserver")
    os.system("pkill -9 gzclient")
    os.system("pkill -9 gazebo")
    os.system("pkill -9 px4")
    os.system("pkill -9 MicroXRCEAgent")
    
    # 再等待一秒确保清理完成
    time.sleep(1)
    return []

def generate_launch_description():
    # 设置PX4路径
    px4_path = os.path.expanduser('~/PX4-Autopilot')
    
    # 设置环境变量：添加.gazebo/models路径，确保Gazebo能找到模型
    env_vars = [
        # 已有的GAZEBO_MODEL_PATH设置
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[
                EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
                ':/home/suda/drone_ugv_ws/src/simulation/models',
                f':{px4_path}/Tools/sitl_gazebo/models',
                ':/home/suda/.gazebo/models'
            ]
        ),
        # 添加以下环境变量
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
        # 设置PX4仿真模型
        SetEnvironmentVariable(
            name='PX4_SIM_MODEL',
            value='iris'
        )
    ]
    
    # 启动Gazebo仿真环境（添加延迟确保端口释放）
    gazebo = ExecuteProcess(
        cmd=[
            'bash', '-c', 
            'sleep 2 && gazebo /home/suda/drone_ugv_ws/src/simulation/worlds/coordination_world.world -s libgazebo_ros_init.so -s libgazebo_ros_factory.so'
        ],
        output='screen',
        on_exit=Shutdown()  # 如果Gazebo关闭，整个launch也关闭
    )
    
    # 启动PX4 SITL (软件在环仿真)
    px4_sitl = ExecuteProcess(
        cmd=[
            'make', 
            'px4_sitl', 
            'none_iris'
        ],
        cwd=px4_path,  # 在PX4-Autopilot目录下执行命令
        output='screen',
        shell=True,  # 需要shell来执行make命令
        on_exit=Shutdown(),  # 如果PX4关闭，整个launch也关闭
        name='px4_sitl'  # 命名进程，便于后续监听
    )
    
    # 启动MicroXRCEAgent（UDPv4，端口8888）
    microxrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
        name='microxrce_agent',
        on_exit=EmitEvent(event=ShutdownEvent(reason='MicroXRCEAgent exited'))
    )
    
    # 无人机模型路径
    drone_model_path = '/home/suda/drone_ugv_ws/src/simulation/models/iris/iris.sdf'

    # 无人车模型路径
    ugv_model_path = '/home/suda/drone_ugv_ws/src/simulation/models/yahboomcar_X3/model.sdf'

    # 加载无人机模型 (iris)
    spawn_drone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_drone',
        output='screen',
        arguments=[
            '-file', drone_model_path,
            '-entity', 'iris',
            '-x', '0', '-y', '0', '-z', '0.1'
        ]
    )
    
    # 加载无人车模型
    spawn_ugv = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_ugv',
        output='screen',
        arguments=[
            '-file', ugv_model_path,
            '-entity', 'yahboomcar_X3',
            '-x', '2', '-y', '0', '-z', '0.05'  # 初始位置
        ]
    )

    # ##############################
    # 新增：发布无人车的静态TF变换
    # ##############################
    
    # 1. 发布 base_footprint → laser_link (激光雷达TF)
    static_tf_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        # 参数：x y z yaw pitch roll 父帧 子帧
        arguments=[
            "0.0435", "0.000053", "0.1915",  # 从SDF提取的激光雷达位置
            "0.0", "0.0", "0.0",             # 无旋转
            "base_footprint", "laser_link"   # 父帧→子帧
        ],
        parameters=[{'use_sim_time': True}],
        output="screen",
        name="static_tf_laser_publisher"
    )
    
    # 2. 发布 base_footprint → camera_link (相机TF)
    static_tf_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.057105", "0.000018", "0.11905",  # 从SDF提取的相机位置
            "0.0", "0.0", "0.0",                # 无旋转
            "base_footprint", "camera_link"     # 父帧→子帧
        ],
        parameters=[{'use_sim_time': True}],
        output="screen",
        name="static_tf_camera_publisher"
    )

    # 3. 可选：如果需要base_link帧，发布 base_footprint → base_link
    static_tf_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.0", "0.0", "0.08",  # base_link在base_footprint上方8cm
            "0.0", "0.0", "0.0",
            "base_footprint", "base_link"
        ],
        parameters=[{'use_sim_time': True}],
        output="screen",
        name="static_tf_base_link_publisher"
    )
    
    # 监听PX4启动后再启动MicroXRCEAgent
    start_agent_after_px4 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=px4_sitl,  # 监听PX4进程启动
            on_start=[microxrce_agent]  # PX4启动后启动Agent
        )
    )

    # 监听无人车加载后再发布TF（确保模型加载完成）
    start_tf_after_ugv = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_ugv,  # 监听无人车加载完成
            on_start=[
                static_tf_laser, 
                static_tf_camera,
                static_tf_base_link  # 若启用base_link则添加
            ]
        )
    )
    
    return LaunchDescription([
        # 先关闭已存在的相关进程
        OpaqueFunction(function=kill_existing_gazebo),
        # 设置环境变量
        *env_vars,
        # 启动Gazebo
        gazebo,
        # 启动PX4 SITL飞控
        px4_sitl,
        # 注册进程监听事件，用于启动Agent
        start_agent_after_px4,
        # 加载无人机模型
        spawn_drone,
        # 加载无人车模型
        spawn_ugv,
        # 注册TF发布事件（无人车加载后启动）
        start_tf_after_ugv
    ])
