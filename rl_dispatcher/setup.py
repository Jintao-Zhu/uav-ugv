from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rl_dispatcher'

setup(
    name=package_name,
    version='0.0.0',
    # 保留自动查找包，排除测试目录
    packages=find_packages(exclude=['test']),
    data_files=[
        # 1. 保留ROS2包索引配置
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 2. 新增：将scripts目录下的Python文件部署到功能包目录
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    # 3. 核心依赖：补充ROS2和RL所需库
    install_requires=[
        'setuptools',
        'rclpy',                # ROS2 Python核心库
        'numpy',                # RL状态向量计算
        'stable-baselines3[extra]',  # RL算法库（PPO）
        'gym',                  # RL环境基础库
        'json5'                 # 可选：JSON解析（兼容RMF任务消息）
    ],
    zip_safe=True,
    maintainer='suda',
    maintainer_email='2327406014@stu.suda.edu.cn',
    # 4. 完善描述：贴合RL调度场景
    description='Reinforcement Learning dispatcher for RMF, bypass bidding to assign specific delivery robots',
    # 5. 补充许可证（和package.xml一致）
    license='Apache-2.0',
    tests_require=['pytest'],
    # 6. 关键：配置节点入口（ROS2运行节点的命令）
    entry_points={
        'console_scripts': [
            # 格式："终端命令名 = 包名.文件名:主函数名"
            'rl_dispatcher_node = rl_dispatcher.rl_dispatcher_node:main',
        ],
    },
)
