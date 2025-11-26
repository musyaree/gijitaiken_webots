from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mujoco_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),

        (os.path.join('share', package_name, 'model'), 
            glob('model/*.xml')),
        
        (os.path.join('share', package_name, 'model/assets'), 
            glob('model/assets/*.stl') + glob('model/assets/*.part')),
            
        (os.path.join('share', package_name, 'model/assets/merged'), 
            glob('model/assets/merged/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='musyaree',
    maintainer_email='email@anda.com',
    description='MuJoCo Simulator Node for ROS 2',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator_node = mujoco_node.simulator:main',
        ],
    },
)