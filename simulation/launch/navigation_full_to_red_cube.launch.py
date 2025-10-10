from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, RegisterEventHandler, LogInfo, TimerAction,
    OpaqueFunction, EmitEvent, Shutdown
)
from launch.event_handlers import OnProcessStart
from launch.events import Shutdown as ShutdownEvent
from launch_ros.actions import Node
import os

def kill_existing_nodes(context, *args, **kwargs):
    """清理残留的目标节点，避免多实例冲突"""
    os.system("pkill -f 'ros2 launch simulation simulation.launch.py'")
    os.system("pkill -f 'ros2 launch simulation nav2_bringup_yahboomcar_follow.launch.py'")
    os.system("pkill -f 'ros2 run collab_core navigation_cube_goal'")
    os.system("pkill -f 'ros2 run image_processing red_cube_tracker_node'")
    os.system("pkill -f 'ros2 run image_processing red_cube_detector_node'")
    return []

def generate_launch_description():
    # --------------------------
    # 1. 终端1：启动仿真环境（基础依赖）
    # --------------------------
    simulation_launch = ExecuteProcess(
        cmd=["ros2", "launch", "simulation", "simulation.launch.py"],
        output="screen",
        emulate_tty=True,
        shell=True,
        name="terminal1_simulation",
        # 仿真退出时终止所有节点，避免残留
        on_exit=EmitEvent(event=ShutdownEvent(reason="仿真进程退出，终止所有控制节点"))
    )

    # --------------------------
    # 2. 终端2：启动 Nav2 导航栈
    # --------------------------
    nav2_launch = ExecuteProcess(
        cmd=["ros2", "launch", "simulation", "nav2_bringup_yahboomcar_follow.launch.py"],
        output="screen",
        emulate_tty=True,
        shell=True,
        name="terminal2_nav2_stack"
    )

    # --------------------------
    # 3. 终端3：导航转换节点（订阅target→转Nav2动作）
    # --------------------------
    nav_cube_goal_node = Node(
        package="collab_core",
        executable="navigation_cube_goal",
        name="terminal3_nav_cube_goal",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": True}]  # 与仿真时间同步
    )

    # --------------------------
    # 4. 终端4：Tracker节点（清理缓存+接收detector消息）
    # --------------------------
    tracker_node = ExecuteProcess(
        cmd=["ros2", "run", "image_processing", "red_cube_tracker_node", "clear"],
        output="screen",
        emulate_tty=True,
        shell=True,
        name="terminal4_red_cube_tracker"
    )

    # --------------------------
    # 5. 终端5：Detector节点（扫描文件+发布目标话题）
    # --------------------------
    detector_node = Node(
        package="image_processing",
        executable="red_cube_detector_node",
        name="terminal5_red_cube_detector",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": True}]  # 与仿真时间同步
    )

    # --------------------------
    # 事件1：仿真启动后，延迟10秒启动 Nav2 导航栈（核心修改）
    # --------------------------
    start_nav2_after_simulation = RegisterEventHandler(
        OnProcessStart(
            target_action=simulation_launch,
            on_start=[
                LogInfo(msg="\n" + "="*60),
                LogInfo(msg="仿真进程已启动，将在 10 秒后启动 Nav2 导航栈（终端2）"),
                LogInfo(msg="="*60),
                TimerAction(period=10.0, actions=[nav2_launch])  # 固定延迟3秒
            ]
        )
    )

    # --------------------------
    # 事件2：Nav2启动后→延迟5秒启动导航转换节点
    # --------------------------
    start_nav_goal_after_nav2 = RegisterEventHandler(
        OnProcessStart(
            target_action=nav2_launch,
            on_start=[
                LogInfo(msg="\n" + "="*60),
                LogInfo(msg="Nav2 导航栈启动完成，2秒后启动导航转换节点（终端3）"),
                LogInfo(msg="="*60),
                TimerAction(period=5.0, actions=[nav_cube_goal_node])
            ]
        )
    )

    # --------------------------
    # 事件3：导航转换节点启动后→延迟2秒启动Tracker
    # --------------------------
    start_tracker_after_nav_goal = RegisterEventHandler(
        OnProcessStart(
            target_action=nav_cube_goal_node,
            on_start=[
                LogInfo(msg="\n" + "="*60),
                LogInfo(msg="导航转换节点启动完成，1秒后启动 Tracker 节点（终端4）"),
                LogInfo(msg="="*60),
                TimerAction(period=2.0, actions=[tracker_node])
            ]
        )
    )

    # --------------------------
    # 事件4：Tracker启动后→延迟2秒启动Detector
    # --------------------------
    start_detector_after_tracker = RegisterEventHandler(
        OnProcessStart(
            target_action=tracker_node,
            on_start=[
                LogInfo(msg="\n" + "="*60),
                LogInfo(msg="Tracker 节点启动完成，2秒后启动 Detector 节点（终端5）"),
                LogInfo(msg="="*60),
                TimerAction(period=2.0, actions=[detector_node])
            ]
        )
    )

    return LaunchDescription([
        # 第一步：清理残留节点，避免冲突
        OpaqueFunction(function=kill_existing_nodes),
        # 第二步：启动仿真（后续节点由事件触发）
        simulation_launch,
        # 注册所有事件，按顺序触发
        start_nav2_after_simulation,
        start_nav_goal_after_nav2,
        start_tracker_after_nav_goal,
        start_detector_after_tracker
    ])
