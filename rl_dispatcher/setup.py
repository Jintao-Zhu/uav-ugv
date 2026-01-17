from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rl_dispatcher'

setup(
    name=package_name,
    version='0.0.0',
    # 修正：find_packages() 会自动查找包含__init__.py的目录，确保模块目录名=包名
    packages=find_packages(exclude=['test']),
    data_files=[
        # 1. ROS2包索引（必须保留）
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 修正：如果你的Python节点文件在 "rl_dispatcher/" 模块目录下，无需单独安装scripts
        # （删除scripts相关行，避免目录结构混乱）
        # (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    # 修正：install_requires只声明Python pip依赖，ROS2依赖在package.xml中声明
    install_requires=[
        'setuptools',
        'numpy>=1.21.0',                # 指定版本，避免兼容性问题
        'stable-baselines3[extra]>=2.0.0',  # RL算法库
        'gym>=0.26.0',                  # RL环境库
        # 移除rclpy/json5：rclpy是ROS2依赖，在package.xml中声明；json5非必需
    ],
    zip_safe=True,
    maintainer='suda',
    maintainer_email='2327406014@stu.suda.edu.cn',
    description='Reinforcement Learning dispatcher for RMF, bypass bidding to assign specific delivery robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    # 核心：入口脚本配置（必须匹配「包名.模块内文件名:主函数名」）
    entry_points={
        'console_scripts': [
            # 格式："终端命令名 = 包名.节点文件名:主函数名"
            # 前提：rl_dispatcher_node.py 必须在 rl_dispatcher/ 模块目录下
            'rl_dispatcher_node = rl_dispatcher.rl_dispatcher_node:main',
        ],
    },
)
