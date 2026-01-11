from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, RegisterEventHandler, LogInfo, TimerAction,
    OpaqueFunction, EmitEvent
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown as ShutdownEvent
from launch_ros.actions import Node
import os
import sys

def kill_existing_rmf_nodes(context, *args, **kwargs):
    """清理残留的RMF相关节点，避免端口占用/多实例冲突"""
    rmf_processes = [
        "ros2 launch rmf_demos_gz_classic airport_terminal.launch.xml",
        "ros2 launch simulation spawn_red_cube_in_rmf.launch.py",
        "ros2 run robot_dispose_adapter task_monitor_node",
        "ros2 run rmf_custom_tasks single_point_task_publisher",
        "ros2 run image_processing auto_send_waypoints"
    ]
    for proc in rmf_processes:
        os.system(f"pkill -f '{proc}'")
    # 额外清理GZ仿真残留
    os.system("pkill -f 'gz sim'")
    os.system("pkill -f 'rmf_fleet_adapter'")
    return []

def generate_launch_description():
    # --------------------------
    # RMF工作空间路径（保留，供其他组件使用）
    # --------------------------
    rmf_ws_path = os.path.expanduser('~/rmf_ws_2')

    # --------------------------
    # 【已删除】1. 启动RMF航站楼场景的代码块
    # --------------------------

    # --------------------------
    # 2. 加载红色立方体模型（保留）
    # --------------------------
    spawn_red_cube = ExecuteProcess(
        cmd=["/bin/bash", "-c", "ros2 launch simulation spawn_red_cube_in_rmf.launch.py"],
        output="screen",
        emulate_tty=True,
        shell=False,
        name="spawn_red_cube"
    )

    # --------------------------
    # 3. 启动任务监控节点（保留）
    # --------------------------
    task_monitor_node = Node(
        package="robot_dispose_adapter",
        executable="task_monitor_node",
        name="task_monitor_node",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": True}]
    )

    # --------------------------
    # 4. 启动自定义任务发布节点（保留）
    # --------------------------
    custom_task_publisher = Node(
        package="rmf_custom_tasks",
        executable="single_point_task_publisher",
        name="single_point_task_publisher",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": True}]
    )

    # --------------------------
    # 5. 启动模拟任务发布节点（保留）
    # --------------------------
    auto_send_waypoints = Node(
        package="image_processing",
        executable="auto_send_waypoints",
        name="auto_send_waypoints",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": True}]
    )

    # --------------------------
    # 事件1：【修改】原监听RMF启动的逻辑改为直接启动后续节点
    # （因为删除了RMF航站楼，无需延迟等待，直接按顺序启动所有节点）
    # --------------------------
    start_all_nodes = [
        LogInfo(msg="="*60),
        LogInfo(msg="RMF航站楼未启动，直接加载红色立方体"),
        LogInfo(msg="="*60),
        spawn_red_cube,
        # 立方体启动后延迟2秒启动任务监控节点
        TimerAction(period=2.0, actions=[
            LogInfo(msg="红色立方体加载中，启动任务监控节点"),
            task_monitor_node
        ]),
        # 延迟5秒启动自定义任务节点
        TimerAction(period=5.0, actions=[
            LogInfo(msg="启动自定义任务发布节点"),
            custom_task_publisher
        ]),
        # 延迟8秒启动无人机目标模拟节点
        TimerAction(period=8.0, actions=[
            LogInfo(msg="启动无人机目标模拟节点"),
            auto_send_waypoints
        ])
    ]

    # --------------------------
    # 事件2：任务监控节点启动提示（保留）
    # --------------------------
    task_monitor_start_log = RegisterEventHandler(
        OnProcessStart(
            target_action=task_monitor_node,
            on_start=[
                LogInfo(msg="\n" + "="*60),
                LogInfo(msg="✅ 任务监控节点启动完成！"),
                LogInfo(msg="✅ RMF多无人车任务调度系统已就绪！"),
                LogInfo(msg="="*60)
            ]
        )
    )

    # --------------------------
    # 组装LaunchDescription（【修改】删除rmf_airport_launch，新增直接启动节点的逻辑）
    # --------------------------
    return LaunchDescription([
        # 清理残留进程
        OpaqueFunction(function=kill_existing_rmf_nodes),
        # 【新增】直接启动所有节点（替代原RMF启动后的延迟逻辑）
        *start_all_nodes,
        # 注册事件
        task_monitor_start_log
    ])
