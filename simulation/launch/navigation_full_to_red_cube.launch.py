from launch import LaunchDescription # 1.7启动全跟随链路后，再启动这个节点
from launch.actions import (
    ExecuteProcess, RegisterEventHandler, LogInfo, TimerAction,
    OpaqueFunction, EmitEvent
)
from launch.event_handlers import OnProcessStart, OnProcessIO
from launch.events import Shutdown as ShutdownEvent
from launch_ros.actions import Node
import os

def kill_existing_nodes(context, *args, **kwargs):
    """清理残留的目标节点，避免多实例冲突"""
    os.system("pkill -f 'ros2 launch simulation simulation.launch.py'")
    os.system("pkill -f 'ros2 launch simulation nav2_bringup_yahboomcar_follow.launch.py'")
    os.system("pkill -f 'ros2 run collab_core navigation_cube_goal'")
    os.system("pkill -f 'ros2 run collab_core navigation_target_goal'")   
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
    # 3.导航目标节点（terminal6_nav_target）
    # --------------------------
    nav_target = Node(
        package="collab_core",
        executable="navigation_target_goal",
        name="terminal6_nav_target",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": True}]  # 与仿真时间同步
    )

    # --------------------------
    # 4. 导航转换节点（订阅target→转Nav2动作）
    # --------------------------
    nav_cube_goal_node = Node(
        package="collab_core",
        executable="navigation_cube_goal",
        name="terminal3_nav_cube_goal",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": True}]
    )

    # --------------------------
    # 5. Tracker节点（清理缓存+接收detector消息）
    # --------------------------
    tracker_node = ExecuteProcess(
        cmd=["ros2", "run", "image_processing", "red_cube_tracker_node", "clear"],
        output="screen",
        emulate_tty=True,
        shell=True,
        name="terminal4_red_cube_tracker"
    )

    # --------------------------
    # 6. Detector节点（扫描文件+发布目标话题）
    # --------------------------
    detector_node = Node(
        package="image_processing",
        executable="red_cube_detector_node_xml",
        name="terminal5_red_cube_detector",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": True}]
    )

    # --------------------------
    # 事件1：仿真启动后→延迟10秒启动终端2（Nav2）
    # --------------------------
    start_nav2_after_simulation = RegisterEventHandler(
        OnProcessStart(
            target_action=simulation_launch,
            on_start=[
                LogInfo(msg="\n" + "="*60),
                LogInfo(msg="仿真进程已启动，将在 10 秒后启动 Nav2 导航栈（终端2）"),
                LogInfo(msg="="*60),
                TimerAction(period=10.0, actions=[nav2_launch])
            ]
        )
    )

    # --------------------------
    # 新增事件2：Nav2启动后→延迟3秒nav_target
    # --------------------------
    start_nav_target_after_nav2 = RegisterEventHandler(
        OnProcessStart(
            target_action=nav2_launch,  # 触发源：终端2（Nav2）启动
            on_start=[
                LogInfo(msg="\n" + "="*60),
                LogInfo(msg="Nav2 导航栈启动完成，3秒后启动导航目标节点（终端6）"),
                LogInfo(msg="="*60),
                TimerAction(period=3.0, actions=[nav_target])  # 延迟启动终端6
            ]
        )
    )

    # --------------------------
    # 事件3：nav_target启动后→延迟5秒启动导航转换
    # --------------------------
    start_nav_goal_after_nav_target = RegisterEventHandler(
        OnProcessStart(
            target_action=nav_target,  
            on_start=[
                LogInfo(msg="\n" + "="*60),
                LogInfo(msg="导航目标节点（终端6）启动完成，5秒后启动导航转换节点（终端3）"),
                LogInfo(msg="="*60),
                TimerAction(period=5.0, actions=[nav_cube_goal_node])  
            ]
        )
    )

    # --------------------------
    # 事件4：终端3（导航转换）启动后→延迟2秒启动终端4（Tracker）
    # --------------------------
    start_tracker_after_nav_goal = RegisterEventHandler(
        OnProcessStart(
            target_action=nav_cube_goal_node,
            on_start=[
                LogInfo(msg="\n" + "="*60),
                LogInfo(msg="导航转换节点启动完成，2秒后启动 Tracker 节点（终端4）"),
                LogInfo(msg="="*60),
                TimerAction(period=2.0, actions=[tracker_node])
            ]
        )
    )

    # --------------------------
    # 事件5：终端4（Tracker）启动后→延迟2秒启动终端5（Detector）
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
        # 第一步：清理残留节点（含新增的终端6节点）
        OpaqueFunction(function=kill_existing_nodes),
        # 第二步：启动仿真
        simulation_launch,
        # 注册所有事件（按触发顺序排列）
        start_nav2_after_simulation,
        start_nav_target_after_nav2, 
        start_nav_goal_after_nav_target, 
        start_tracker_after_nav_goal,
        start_detector_after_tracker
    ])
