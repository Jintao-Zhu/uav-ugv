from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    # 红色立方体模型的真实路径
    cube_sdf_path = os.path.expanduser("~/.gazebo/models/RedCube/model.sdf")

    # 调试信息：验证模型路径
    print(f"=== 红色立方体 SDF 路径：{cube_sdf_path} ===")
    if os.path.exists(cube_sdf_path):
        print("✅ 路径存在，可正常加载")
    else:
        print("❌ 路径不存在！请检查文件夹名是否为 RedCube")

    # 定义16个立方体（实体名称+坐标，名称唯一）
    cubes = [
        ("red_cube_1", "30.0", "-40.0", "0.0"),  
        ("red_cube_2", "50.0", "-40.0", "0.0"),  
        ("red_cube_3", "25.0", "-30.0", "0.0"),  
        ("red_cube_4", "22.0", "-21.0", "0.0"),
        ("red_cube_5", "60.0", "-26.0", "0.0"),
        ("red_cube_6", "70.0", "-10.0", "0.0"),
        ("red_cube_7", "66.9", "-49.9", "0.0"),
        ("red_cube_8", "102.0", "-13.4", "0.0"),
        ("red_cube_9", "101.2", "-39.8", "0.0"),
        ("red_cube_10", "137.1", "-11.7", "0.0"),
        ("red_cube_11", "177.7", "-21.4", "0.0"),
        ("red_cube_12", "190.3", "-43.0", "0.0"),
        ("red_cube_13", "237.4", "-21.0", "0.0"),
        ("red_cube_14", "192.1", "-20.8", "0.0"),
        ("red_cube_15", "221.6", "-50.1", "0.0"),
        ("red_cube_16", "272.3", "-34.0", "0.0")
    ]

    # 第一步：创建一个空列表，用于存放所有节点
    launch_actions = []

    # 第二步：添加Z坐标命令行参数到列表
    launch_actions.append(
        DeclareLaunchArgument('z', default_value='1.0', description='所有立方体的Z坐标（默认1.0）')
    )

    # 第三步：循环生成每个立方体节点，添加到列表中
    for cube_name, x, y, z in cubes:
        launch_actions.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name=f'spawn_{cube_name}',  # 每个节点名称唯一
                output='screen',
                arguments=[
                    '-file', cube_sdf_path,
                    '-entity', cube_name,    # 实体名称唯一
                    '-x', x,
                    '-y', y,
                    '-z', LaunchConfiguration('z'),
                    '-R', '0.0', '-P', '0.0', '-Y', '0.0'
                ],
                parameters=[{'use_sim_time': True}]
            )
        )

    # 第四步：将列表传入 LaunchDescription
    return LaunchDescription(launch_actions)
