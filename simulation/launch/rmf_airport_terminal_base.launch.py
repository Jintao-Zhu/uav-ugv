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


def generate_launch_description():
    # --------------------------
    # 1. 加载红色立方体模型（ExecuteProcess执行launch文件）
    # --------------------------
    spawn_red_cube = ExecuteProcess(
        cmd=["/bin/bash", "-c", "ros2 launch simulation spawn_red_cube_in_rmf.launch.py"],
        output="screen",
        emulate_tty=True,
        shell=False,
        name="spawn_red_cube"
    )

    # --------------------------
    # 2. 启动任务监控节点（ROS2 Node）
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
    # 3. 启动自定义任务发布节点（ROS2 Node）
    # --------------------------
    custom_task_publisher = Node(
        package="rmf_custom_tasks_self",
        executable="single_point_task_publisher",
        name="single_point_task_publisher",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": True}]
    )

    # --------------------------
    # 4. 启动强化学习调度节点（推理模式）
    # --------------------------
    rl_dispatcher_node = Node(
        package="rl_dispatcher",
        executable="rl_dispatcher_node",
        name="rl_dispatcher_node",
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "-p", "mode:=infer"],  # 指定推理模式
        parameters=[{"use_sim_time": True}]
    )

    # --------------------------
    # 5. 启动无人机目标模拟节点
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
    # 节点启动顺序：按依赖关系延迟启动，避免初始化冲突
    # --------------------------
    start_sequence = [
        # 第一步：清理残留进程后，启动红色立方体
        LogInfo(msg="="*60),
        LogInfo(msg="🚀 开始启动RMF强化学习调度系统（推理模式）"),
        LogInfo(msg="="*60),
        spawn_red_cube,
        
        # 立方体启动后延迟2秒：启动任务监控节点
        TimerAction(period=2.0, actions=[
            LogInfo(msg="📦 红色立方体加载中，启动任务监控节点..."),
            task_monitor_node
        ]),
        
        # 延迟5秒：启动自定义任务发布节点
        TimerAction(period=5.0, actions=[
            LogInfo(msg="📤 启动自定义任务发布节点..."),
            custom_task_publisher
        ]),
        
        # 延迟7秒：启动强化学习调度节点（核心）
        TimerAction(period=7.0, actions=[
            LogInfo(msg="🧠 启动强化学习调度节点（推理模式）..."),
            rl_dispatcher_node
        ]),
        
        # 延迟10秒：启动无人机目标模拟节点（最后启动，确保依赖节点就绪）
        TimerAction(period=10.0, actions=[
            LogInfo(msg="✈️  启动无人机目标模拟节点..."),
            auto_send_waypoints
        ])
    ]

    # --------------------------
    # 事件监听：强化学习节点启动完成提示
    # --------------------------
    rl_node_start_log = RegisterEventHandler(
        OnProcessStart(
            target_action=rl_dispatcher_node,
            on_start=[
                LogInfo(msg="\n" + "="*60),
                LogInfo(msg="✅ 强化学习调度节点（推理模式）启动完成！"),
                LogInfo(msg="✅ 所有节点启动完毕，系统进入任务调度状态！"),
                LogInfo(msg="="*60)
            ]
        )
    )

    # --------------------------
    # 事件监听：任意节点退出则关闭整个系统（可选，保证鲁棒性）
    # --------------------------
    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rl_dispatcher_node,  # 核心节点退出则关机
            on_exit=[
                LogInfo(msg="⚠️  强化学习调度节点已退出，关闭整个系统..."),
                EmitEvent(event=ShutdownEvent(reason="RL dispatcher node exited"))
            ]
        )
    )

    # --------------------------
    # 组装LaunchDescription
    # --------------------------
    return LaunchDescription([
        # 第二步：按顺序启动所有节点
        *start_sequence,
        # 第三步：注册事件监听
        rl_node_start_log,
        shutdown_on_exit  # 可选：核心节点退出则关机，可根据需求注释
    ])
