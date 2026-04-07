from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'swarm_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament 索引
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # 配置文件
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cookie',
    maintainer_email='cookie@todo.todo',
    description='分布式无人机蜂群协同控制系统',
    license='MIT',
    entry_points={
        'console_scripts': [
            'drone_controller_node = swarm_control.drone_controller:main',
            'formation_commander_node = swarm_control.formation_commander:main',
            'swarm_ground_station = swarm_control.swarm_ground_station:main',
            'search_mission_demo = swarm_control.search_mission_demo:main',
        ],
    },
)
