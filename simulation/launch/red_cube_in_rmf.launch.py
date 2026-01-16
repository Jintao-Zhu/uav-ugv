from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
import os
import sys

def generate_launch_description():
    # ========================================================
    # 1. 模型路径配置 (支持环境变量)
    # ========================================================
    cube_sdf_path = os.getenv(
        'RED_CUBE_MODEL_PATH',
        os.path.expanduser("~/.gazebo/models/RedCube/model.sdf")
    )

    # 路径验证 (关键改进: 路径不存在时终止启动)
    print(f"=== 红色立方体 SDF 路径: {cube_sdf_path} ===")
    if not os.path.exists(cube_sdf_path):
        print("❌ 错误: 模型文件不存在!")
        print("   请检查:")
        print("   1. 文件夹名是否为 'RedCube' (注意大小写)")
        print("   2. 路径是否为 ~/.gazebo/models/RedCube/model.sdf")
        print("   3. 或设置环境变量: export RED_CUBE_MODEL_PATH=/path/to/model.sdf")
        sys.exit(1)  # ← 关键改进: 终止启动
    print("✅ 模型路径有效")

    # ========================================================
    # 2. 航点配置 (清晰分组)
    # ========================================================
    # 活跃的立方体 (将被生成)
    active_cubes = [
        ("red_cube_n14", "80.84", "-28.52", "0.5"),
        ("red_cube_n13", "84.44", "-4.94", "0.5"),
        ("red_cube_west_koi_pond", "34.32", "-10.13", "0.5"),
        ("red_cube_s08", "96.61", "-50.50", "0.5"),
        ("red_cube_s10", "122.10", "-46.68", "0.5"),
        ("red_cube_junction_south_west", "84.56", "-38.81", "0.5")
    ]

    # 禁用的立方体 (不会生成, 便于管理)
    disabled_cubes = [
        # ("red_cube_junction_n01", "1.57", "-45.93", "0.5"),
        # ("red_cube_n08", "59.61", "-7.42", "0.5"),
        # ("red_cube_n23", "182.80", "-42.30", "0.5"),
        # ("red_cube_s11", "152.73", "-43.00", "0.5"),
    ]

    print(f"📦 将生成 {len(active_cubes)} 个红色立方体")
    if disabled_cubes:
        print(f"⏸️  已禁用 {len(disabled_cubes)} 个立方体")

    # ========================================================
    # 3. 可选功能: 清理已存在的实体
    # ========================================================
    cleanup_existing = os.getenv('CLEANUP_EXISTING_CUBES', 'false').lower() == 'true'
    
    launch_actions = []

    if cleanup_existing:
        print("🧹 启用清理模式: 将删除已存在的立方体")
        for cube_name, _, _, _ in active_cubes:
            launch_actions.append(
                ExecuteProcess(
                    cmd=['gz', 'model', '-m', cube_name, '-d'],
                    output='log',
                    on_exit=lambda event, context: None  # 忽略删除失败
                )
            )
        # 等待删除完成
        print("⏳ 等待 2 秒清理旧实体...")

    # ========================================================
    # 4. 参数声明
    # ========================================================
    launch_actions.append(
        DeclareLaunchArgument(
            'z',
            default_value='0.5',
            description='立方体Z坐标 (1m边长建议0.5)'
        )
    )

    launch_actions.append(
        DeclareLaunchArgument(
            'spawn_delay',
            default_value='0.5',
            description='每个立方体生成的延迟间隔(秒)'
        )
    )

    # ========================================================
    # 5. 生成立方体节点 (关键改进: 添加延迟)
    # ========================================================
    spawn_delay = 0.0
    delay_increment = 0.5  # 每个立方体间隔0.5秒

    for i, (cube_name, x, y, z_default) in enumerate(active_cubes):
        spawn_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_{cube_name}',
            output='screen',
            arguments=[
                '-file', cube_sdf_path,
                '-entity', cube_name,
                '-x', x,
                '-y', y,
                '-z', LaunchConfiguration('z'),
                '-R', '0.0', '-P', '0.0', '-Y', '0.0'
            ],
            parameters=[{'use_sim_time': True}]
        )

        # 关键改进: 使用 TimerAction 添加延迟
        if i == 0:
            # 第一个立方体: 立即生成 (或等待清理完成)
            delay = 2.0 if cleanup_existing else 0.0
        else:
            # 后续立方体: 依次延迟
            delay = (2.0 if cleanup_existing else 0.0) + i * delay_increment

        launch_actions.append(
            TimerAction(
                period=delay,
                actions=[spawn_node]
            )
        )
        
        print(f"  [{i+1}/{len(active_cubes)}] {cube_name} 将在 {delay:.1f}秒后生成")

    print("=" * 60)
    print("🚀 启动配置完成")
    print("=" * 60)

    return LaunchDescription(launch_actions)