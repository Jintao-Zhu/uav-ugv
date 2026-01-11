from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    # 红色立方体模型的真实路径（确保文件夹名是 RedCube，若为小写 redcube 需对应修改）
    cube_sdf_path = os.path.expanduser("~/.gazebo/models/RedCube/model.sdf")

    # 调试信息：验证模型路径（启动时终端打印，方便排查）
    print(f"=== 红色立方体 SDF 路径：{cube_sdf_path} ===")
    if os.path.exists(cube_sdf_path):
        print("✅ 路径存在，可正常加载")
    else:
        print("❌ 路径不存在！请检查：1. 文件夹名是否为 RedCube；2. 是否放在 ~/.gazebo/models/ 下")

    # 核心：10个不重复的真实航点（名称+坐标均来自 /nav_graphs）
    cubes = [
        #("red_cube_junction_n01", "1.57", "-45.93", "0.5"),    # 西北区域 已修改 tiny3
        #("red_cube_n08", "59.61", "-7.42", "0.5"),            # 北部区域  已修改 tiny2
        ("red_cube_n14", "80.84", "-28.52", "0.5"),           # 中部区域  会被junction_south_west挡住 deliver0
        ("red_cube_n13", "84.44", "-4.94", "0.5"),            # 北部区域  已修改 deliver0
        ("red_cube_n23", "182.80", "-42.30", "0.5"),          # 东北区域  已修改 deliver1
        ("red_cube_west_koi_pond", "34.32", "-10.13", "0.5"),  # 很神奇，调用的不是TinyRobot,是delivery小车
        ("red_cube_s08", "96.61", "-50.50", "0.5"),           # 西南区域 被delivery0搬走了，，，
        ("red_cube_s10", "122.10", "-46.68", "0.5"),          # 中偏西区域  delivery1
        ("red_cube_s11", "152.73", "-43.00", "0.5"),          # 东部区域 就在delivery1旁边
        ("red_cube_junction_south_west", "84.56", "-38.81", "0.5")  # 中南部区域 已修改 delivery0
    ]

    # 第一步：创建空列表，存放所有启动节点
    launch_actions = []

    # 第二步：添加Z坐标命令行参数（适配1m边长，默认0.5，可通过--z调整）
    launch_actions.append(
        DeclareLaunchArgument(
            'z',
            default_value='0.5',  # 关键调整：1m边长→Z=0.5（一半贴地，不嵌入、不悬空）
            description='所有立方体的Z坐标（1m边长建议0.5，刚好一半在地面）'
        )
    )

    # 第三步：循环生成每个立方体的spawn节点（名称唯一，无冲突）
    for cube_name, x, y, z_default in cubes:
        launch_actions.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name=f'spawn_{cube_name}',  # 节点名称唯一（如 spawn_red_cube_junction_n01）
                output='screen',
                arguments=[
                    '-file', cube_sdf_path,  # 立方体SDF文件路径
                    '-entity', cube_name,    # 实体名称（=red_cube_真实航点名，方便后续删除）
                    '-x', x,                 # 真实航点X坐标（从/nav_graphs提取，精准匹配）
                    '-y', y,                 # 真实航点Y坐标
                    '-z', LaunchConfiguration('z'),  # Z坐标（支持命令行覆盖，如 --z 0.6）
                    '-R', '0.0', '-P', '0.0', '-Y', '0.0'  # 无旋转（保持立方体正立）
                ],
                parameters=[{'use_sim_time': True}]  # 启用仿真时间（与RMF同步，避免时间错乱）
            )
        )

    # 第四步：返回启动描述
    return LaunchDescription(launch_actions)
