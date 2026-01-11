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
    """
    功能：清理系统中残留的相关ROS2节点进程
    目的：避免节点多实例运行导致的端口占用、通信冲突等问题
    """
    # 清理仿真环境启动进程
    os.system("pkill -f 'ros2 launch simulation simulation.launch.py'")
    # 清理导航启动进程
    os.system("pkill -f 'ros2 launch simulation nav2_bringup_yahboomcar_follow.launch.py'")
    # 清理导航目标转换节点进程
    os.system("pkill -f 'ros2 run collab_core navigation_cube_goal'")
    # 清理导航目标节点进程
    os.system("pkill -f 'ros2 run collab_core navigation_target_goal'")   
    # 清理红色立方体追踪节点进程
    os.system("pkill -f 'ros2 run image_processing red_cube_tracker_node'")
    # 清理红色立方体检测节点进程
    os.system("pkill -f 'ros2 run image_processing red_cube_detector_node'")
    # 清理红色立方体采集节点进程（新增）
    os.system("pkill -f 'ros2 run image_processing red_cube_capture_node'")
    return []

def generate_launch_description():
    """生成ROS2启动描述，按顺序启动采集→导航转换→追踪→检测节点"""

    # ========================== 节点定义 ==========================
    # 0. 红色立方体采集节点（新增）
    # 功能：订阅相机彩色图/深度图话题，进行目标检测并保存图像+位姿数据
    # 所属包：image_processing，可执行文件：red_cube_capture_node
    capture_node = Node(
        package="image_processing",
        executable="red_cube_capture_node",
        name="terminal2_red_cube_capture",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": True},          # 使用仿真时间
            {"confidence_threshold": 0.5},   # 检测置信度阈值
            {"nms_threshold": 0.45},         # NMS非极大值抑制阈值
            {"target_class_id": 0},          # 目标类别ID
            {"image_save_path": "image_save"},# 图像保存路径
            {"color_topic": "/camera/image_raw"},  # 彩色图话题
            {"depth_topic": "/camera/depth/image_raw"} # 深度图话题
        ]
    )

    # 1. 导航目标转换节点
    # 功能：处理导航目标坐标转换，为后续追踪提供目标信息
    # 所属包：collab_core，可执行文件：navigation_cube_goal
    nav_cube_goal_node = Node(
        package="collab_core",
        executable="navigation_cube_goal",
        name="terminal3_nav_cube_goal",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": True}]  # 使用仿真时间
    )

    # 2. 红色立方体追踪节点
    # 功能：订阅 /red_cube/detections 话题，对检测结果进行去重和聚合处理，最终发布更稳定的目标位置信息到新话题
    # 启动方式：通过命令行执行（带clear参数清理缓存）
    tracker_node = ExecuteProcess(
        cmd=["ros2", "run", "image_processing", "red_cube_tracker_node", "clear"],
        output="screen",
        emulate_tty=True,
        shell=True,
        name="terminal4_red_cube_tracker"
    )

    # 3. 红色立方体检测节点
    # 功能：扫描配置文件，检测红色立方体并发布目标话题
    # 所属包：image_processing，可执行文件：red_cube_detector_node_xml
    detector_node = Node(
        package="image_processing",
        executable="red_cube_detector_node_xml",
        name="terminal5_red_cube_detector",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": True}]  # 使用仿真时间
    )

    # ========================== 事件处理 ==========================
    # 事件0：采集节点启动完成后，延迟3秒启动导航转换节点（新增）
    start_nav_after_capture = RegisterEventHandler(
        OnProcessStart(
            target_action=capture_node,
            on_start=[
                LogInfo(msg="\n" + "="*60),
                LogInfo(msg="【节点启动】采集节点已启动完成，将在3秒后启动导航转换节点（terminal3）"),
                LogInfo(msg="="*60),
                TimerAction(period=3.0, actions=[nav_cube_goal_node])
            ]
        )
    )

    # 事件1：导航转换节点启动完成后，延迟2秒启动追踪节点
    start_tracker_after_nav_goal = RegisterEventHandler(
        OnProcessStart(
            target_action=nav_cube_goal_node,
            on_start=[
                LogInfo(msg="\n" + "="*60),
                LogInfo(msg="【节点启动】导航转换节点已启动完成，将在2秒后启动追踪节点（terminal4）"),
                LogInfo(msg="="*60),
                TimerAction(period=2.0, actions=[tracker_node])
            ]
        )
    )

    # 事件2：追踪节点启动完成后，延迟2秒启动检测节点
    start_detector_after_tracker = RegisterEventHandler(
        OnProcessStart(
            target_action=tracker_node,
            on_start=[
                LogInfo(msg="\n" + "="*60),
                LogInfo(msg="【节点启动】追踪节点已启动完成，将在2秒后启动检测节点（terminal5）"),
                LogInfo(msg="="*60),
                TimerAction(period=2.0, actions=[detector_node])
            ]
        )
    )

    # ========================== 启动流程 ==========================
    return LaunchDescription([
        # 第一步：清理残留节点进程（含新增的采集节点）
        OpaqueFunction(function=kill_existing_nodes),
        
        # 第二步：启动采集节点（作为整个流程的起点）
        capture_node,
        
        # 注册节点启动的依赖事件（按执行顺序注册）
        start_nav_after_capture,
        start_tracker_after_nav_goal,
        start_detector_after_tracker
    ])
