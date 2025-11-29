from launch import LaunchDescription
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
    os.system("pkill -f 'ros2 launch simulation simulation_ugv_uav.launch.py'")
    os.system("pkill -f 'ros2 launch yahboomcar_nav display_nav_launch.py'")
    os.system("pkill -f 'ros2 launch yahboomcar_nav navigation_default_launch.py'")
    os.system("pkill -f 'ros2 run gazebo_sync_node'")
    os.system("pkill -f 'ros2 launch yahboomcar_self_nav autopatrol.launch.py'")
    # os.system("pkill -f 'ros2 run drone_control lower_circle_node'")
    return []

def generate_launch_description():
    # --------------------------
    # 1. 终端1：启动仿真环境
    # --------------------------
    simulation_launch = ExecuteProcess(
        cmd=["ros2", "launch", "simulation", "simulation_ugv_uav.launch.py"],
        output="screen",
        emulate_tty=True,
        shell=True,
        name="terminal1_simulation",
        # 仿真退出时终止所有节点
        on_exit=EmitEvent(event=ShutdownEvent(reason="仿真进程退出，终止所有控制节点"))
    )

    # --------------------------
    # 2. 终端2：启动显示导航
    # --------------------------
    display_nav_launch = ExecuteProcess(
        cmd=["ros2", "launch", "yahboomcar_nav", "display_nav_launch.py",">","/dev/null","2>&1"],
        output="screen",
        emulate_tty=True,
        shell=True,
        name="terminal2_display_nav"
    )

    # --------------------------
    # 3. 终端3：启动默认导航
    # --------------------------
    navigation_default_launch = ExecuteProcess(
        cmd=["ros2", "launch", "yahboomcar_nav", "navigation_default_launch.py",">","/dev/null","2>&1"],
        output="screen",
        emulate_tty=True,
        shell=True,
        name="terminal3_navigation_default"
    )

    # --------------------------
    # 4. 终端4：启动Gazebo同步节点
    # --------------------------
    gazebo_sync_node = ExecuteProcess(
        cmd=["ros2", "run", "yahboomcar_self_nav", "gazebo_sync_node",">","/dev/null","2>&1"],
        output="screen",
        emulate_tty=True,
        shell=True,
        name="terminal4_gazebo_sync"
    )

    # --------------------------
    # 5. 终端5：启动巡逻节点
    # --------------------------
    patrol_node_stop = ExecuteProcess(
        cmd=["ros2", "launch", "yahboomcar_self_nav", "autopatrol.launch.py"],
        output="screen",
        emulate_tty=True,
        shell=True,
        name="terminal5_patrol_stop"
    )

    # --------------------------
    # 6. 终端6：启动无人机环绕节点
    # --------------------------
    drone_circle_node = ExecuteProcess(
        cmd=["ros2", "run", "drone_control", "lower_circle_node"],
        output="screen",
        emulate_tty=True,
        shell=True,
        name="terminal6_drone_circle"
    )

    # --------------------------
    # 事件1：仿真启动后，延迟2秒启动显示导航
    # --------------------------
    start_display_nav_after_simulation = RegisterEventHandler(
        OnProcessStart(
            target_action=simulation_launch,
            on_start=[
                LogInfo(msg="\n" + "="*60),
                LogInfo(msg="仿真进程已启动，将在 2秒后启动显示导航（终端2）"),
                LogInfo(msg="="*60),
                TimerAction(period=2.0, actions=[display_nav_launch])
            ]
        )
    )

    # --------------------------
    # 事件2：显示导航启动后，延迟2秒启动默认导航
    # --------------------------
    start_navigation_default_after_display = RegisterEventHandler(
        OnProcessStart(
            target_action=display_nav_launch,
            on_start=[
                LogInfo(msg="\n" + "="*60),
                LogInfo(msg="显示导航启动完成，2秒后启动默认导航（终端3）"),
                LogInfo(msg="="*60),
                TimerAction(period=2.0, actions=[navigation_default_launch])
            ]
        )
    )

    # --------------------------
    # 事件3：默认导航启动后，延迟2秒启动Gazebo同步节点
    # --------------------------
    start_gazebo_sync_after_navigation = RegisterEventHandler(
        OnProcessStart(
            target_action=navigation_default_launch,
            # target_action=simulation_launch,
            on_start=[
                LogInfo(msg="\n" + "="*60),
                LogInfo(msg="默认导航启动完成，2秒后启动Gazebo同步节点（终端4）"),
                LogInfo(msg="="*60),
                TimerAction(period=2.0, actions=[gazebo_sync_node])
            ]
        )
    )

    # --------------------------
    # 事件4：Gazebo同步节点启动后，延迟2秒启动巡逻停止节点
    # --------------------------
    start_patrol_after_gazebo_sync = RegisterEventHandler(
        OnProcessStart(
            target_action=gazebo_sync_node,
            on_start=[
                LogInfo(msg="\n" + "="*60),
                LogInfo(msg="Gazebo同步节点启动完成，5秒后启动巡逻停止节点（终端5）"),
                LogInfo(msg="="*60),
                TimerAction(period=2.0, actions=[patrol_node_stop])
            ]
        )
    )

    # --------------------------
    # 事件5：巡逻停止节点启动后，延迟2秒启动无人机环绕节点
    # --------------------------
    start_drone_circle_after_patrol = RegisterEventHandler(
        OnProcessStart(
            target_action=patrol_node_stop,
            on_start=[
                LogInfo(msg="\n" + "="*60),
                LogInfo(msg="巡逻停止节点启动完成，2秒后启动无人机环绕节点（终端6）"),
                LogInfo(msg="="*60),
                TimerAction(period=2.0, actions=[drone_circle_node])
            ]
        )
    )

    return LaunchDescription([
        # 第一步：清理残留节点，避免冲突
        OpaqueFunction(function=kill_existing_nodes),
        # 第二步：启动仿真（后续节点由事件触发）
        simulation_launch,
        # 注册所有事件，按顺序触发
        start_display_nav_after_simulation,
        start_navigation_default_after_display,
        start_gazebo_sync_after_navigation,
        # start_patrol_after_gazebo_sync,
        # start_drone_circle_after_patrol
    ])