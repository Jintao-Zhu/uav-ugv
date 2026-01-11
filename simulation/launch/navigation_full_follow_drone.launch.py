from launch import LaunchDescription  # 启动一系列的无人车避障跟随无人机节点
from launch.actions import (
    ExecuteProcess, RegisterEventHandler, LogInfo, TimerAction,
    OpaqueFunction, EmitEvent
)
from launch.event_handlers import OnProcessIO, OnProcessStart
from launch.events import Shutdown as ShutdownEvent
from launch_ros.actions import Node
import os

def kill_existing_control_nodes(context, *args, **kwargs):
    """关闭残留控制节点，避免多实例冲突"""
    os.system("pkill -f 'ros2 run drone_control circle_node'")
    os.system("pkill -f 'ros2 run collab_core navigation_follow_goal'")
    os.system("pkill -f 'ros2 launch simulation nav2_bringup_yahboomcar_follow.launch.py'")
    return []

def generate_launch_description():
    # --------------------------
    # 1. 终端1：启动原仿真（调用已有simulation.launch.py）
    # --------------------------
    simulation_launch = ExecuteProcess(
        cmd=["ros2", "launch", "simulation", "simulation.launch.py"],
        output="screen",
        emulate_tty=True,  # 模拟终端输出，保留原日志格式
        shell=True,
        name="terminal1_simulation",
        # 原仿真退出时，终止所有后续节点
        on_exit=EmitEvent(event=ShutdownEvent(reason="原仿真进程退出，终止控制节点"))
    )

    # --------------------------
    # 2. 终端2：无人机环绕节点（Node节点支持parameters）
    # --------------------------
    circle_node = Node(
        package="drone_control",
        executable="circle_node",
        name="terminal2_circle_node",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": True,"circle_radius_":10.0,"angular_velocity_":0.05,
                    "center_x_":0.0,"center_y_":0.0}]  # Node节点可正常传参
    )

    # 3. 终端3：Nav2导航服务
    # --------------------------
    nav2_launch = ExecuteProcess(
        cmd=["ros2", "launch", "simulation", "nav2_bringup_yahboomcar_follow.launch.py"],
        output="screen",
        emulate_tty=True,
        shell=True,
        name="terminal4_nav2_service"
        # 注意：若Nav2需要use_sim_time，需在nav2_bringup_yahboomcar_follow.launch.py内部配置，而非此处传参
    )

    # --------------------------
    # 4. 终端4：跟随无人机节点
    # --------------------------
    nav_controller = Node(
        package="collab_core",
        executable="navigation_follow_goal",
        name="terminal5_nav_controller",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": True}]
    )

    # --------------------------
    # 5. 终端5：最终目标节点
    # --------------------------
    nav_target = Node(
        package="collab_core",
        executable="navigation_target_goal",
        name="terminal6_nav_target",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": True}]
    )

    # --------------------------
    # 事件1：监听原仿真的"Ready for takeoff!"，启动终端2
    # --------------------------
    start_circle_after_takeoff = RegisterEventHandler(
        OnProcessIO(
            target_action=simulation_launch,
            on_stdout=lambda event: [
                LogInfo(msg="="*50),
                LogInfo(msg="检测到无人机就绪（Ready for takeoff!），启动环绕节点（终端2）"),
                LogInfo(msg="="*50),
                circle_node
            ] if "Ready for takeoff!" in event.text.decode() else []
        )
    )


    # 事件2：终端2启动后，延迟3秒直接启动Nav2
    start_nav2_after_circle = RegisterEventHandler(
        OnProcessStart(
            target_action=circle_node,
            on_start=[
                LogInfo(msg="\n" + "="*50),
                LogInfo(msg="环绕节点启动完成，3秒后启动Nav2导航服务"),
                LogInfo(msg="="*50),
                TimerAction(period=3.0, actions=[nav2_launch])
            ]
        )
    )

    # --------------------------
    # 事件3：终端3启动后，延迟3秒启动终端4
    # --------------------------
    start_controller_after_nav2 = RegisterEventHandler(
        OnProcessStart(
            target_action=nav2_launch,
            on_start=[
                LogInfo(msg="\n" + "="*50),
                LogInfo(msg="Nav2导航服务启动完成，3秒后启动跟随follow节点（终端4）"),
                LogInfo(msg="="*50),
                TimerAction(period=3.0, actions=[nav_controller])
            ]
        )
    )

    # --------------------------
    # 事件5：终端4启动后，延迟3秒启动终端5（保持不变）
    # --------------------------
    start_target_after_follow = RegisterEventHandler(
        OnProcessStart(
            target_action=nav_controller,
            on_start=[
                LogInfo(msg="\n" + "="*50),
                LogInfo(msg="跟随follow节点启动完成,3秒后启动最终目标target节点（终端5）"),
                LogInfo(msg="="*50),
                TimerAction(period=3.0, actions=[nav_target])
            ]
        )
    )

    return LaunchDescription([
        # 先清理残留节点
        OpaqueFunction(function=kill_existing_control_nodes),
        # 启动原仿真（终端1）
        simulation_launch,
        # 注册所有事件（删除原事件2、3，新增事件start_nav2_after_circle）
        start_circle_after_takeoff,
        start_nav2_after_circle,
        start_controller_after_nav2,
        start_target_after_follow
    ])
