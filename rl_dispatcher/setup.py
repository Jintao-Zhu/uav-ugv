from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rl_dispatcher'

setup(
    name=package_name,
    version='0.0.0',
    # 保留：自动查找模块（排除test）
    packages=find_packages(exclude=['test']),
    data_files=[
        # 1. ROS2包索引（必须保留）
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 可选：如果有launch文件，添加这行（后续可放对比测试的launch）
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    # 保留：Python pip依赖（仅声明PyPI包，ROS2依赖在package.xml）
    install_requires=[
        'setuptools',
        'numpy>=1.21.0',                # 指定版本，避免兼容性问题
        'stable-baselines3[extra]>=2.0.0',  # RL算法库
        'gymnasium>=0.26.0',             # 修正：原gym改为gymnasium（匹配你的RL节点）
    ],
    zip_safe=True,
    maintainer='suda',
    maintainer_email='2327406014@stu.suda.edu.cn',
    # 修正描述：补充原生调度对比节点说明
    description='Reinforcement Learning dispatcher for RMF (with RMF native greedy dispatcher for comparison), bypass bidding to assign specific delivery robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    # 核心修改：新增RMF原生调度节点的入口
    entry_points={
        'console_scripts': [
            # 原有RL调度节点（保留）
            'rl_dispatcher_node = rl_dispatcher.rl_dispatcher_node:main',
            'rl_dispatcher_node_version_2 = rl_dispatcher.rl_dispatcher_node_version_2:main',
            'rl_dispatcher_node_version_3 = rl_dispatcher.rl_dispatcher_node_version_3:main',
            # 新增：RMF原生贪心调度对比节点
            'rmf_native_dispatcher_node = rl_dispatcher.rmf_native_dispatcher_node:main',
            'tan_xin = rl_dispatcher.tan_xin:main',
            'round_robin = rl_dispatcher.round_robin:main',
        ],
    },
)
