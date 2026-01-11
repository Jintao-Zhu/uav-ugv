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
        # 关键：将scripts下的脚本安装到系统可执行路径
        (os.path.join('lib', package_name), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suda',
    maintainer_email='suda@example.com',
    description='RMF任务数据采集节点',
    license='Apache-2.0',
    tests_require=['pytest'],
    # 完全删除entry_points，避免模块导入错误
    entry_points={}
)

