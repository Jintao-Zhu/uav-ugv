from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yahboomcar_self_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suda',
    maintainer_email='2327406014@stu.suda.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'set_car_initial_pose=yahboomcar_self_nav.set_car_initial_pose:main',
            'get_car_pose=yahboomcar_self_nav.get_car_pose:main',
            'nav_to_pose=yahboomcar_self_nav.nav_to_pose:main',
            'nav_to_waypoints=yahboomcar_self_nav.nav_to_waypoints:main',
            'patrol_node=yahboomcar_self_nav.patrol_node:main',
        ],
    },
)
