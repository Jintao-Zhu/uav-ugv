from setuptools import setup
import os
from glob import glob

package_name = 'data_collect'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suda',
    maintainer_email='suda@example.com',
    description='RMF任务数据采集节点',
    license='Apache-2.0',
    tests_require=['pytest'],
    # 恢复entry_points（ROS2标准方式）
    entry_points={
        'console_scripts': [
            # 格式：可执行名 = 包名.模块名:主函数名（如果脚本在scripts，用下面的方式）
            'data_recorder_node = data_collect.data_recorder_node:main',
            # 如果你想保留scripts目录，也可以用以下方式（需安装importlib-metadata）
            # 'data_recorder_node = scripts.data_recorder_node:main'
        ],
    },
)
