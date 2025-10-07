from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'map_struct'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'), glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.*'))),
        (os.path.join('share', package_name, 'scripts'), glob(os.path.join('scripts', '*.*'))),
        (os.path.join('share', package_name, 'models', 'auto_walls'), glob(os.path.join('models','auto_walls','*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lxc',
    maintainer_email='2832981069@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
