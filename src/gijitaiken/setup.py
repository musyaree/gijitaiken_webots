from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gijitaiken'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        
        (os.path.join('share', package_name, 'model'), glob('model/*.xml')),
        
        (os.path.join('share', package_name, 'model/assets'), 
            glob('model/assets/*.stl') + glob('model/assets/*.part')),
        
        (os.path.join('share', package_name, 'model/assets/merged'), glob('model/assets/merged/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='musyaree',
    maintainer_email='faeyzarahnaf07@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'start = gijitaiken.gijitaiken_node:main',
            'teleop = gijitaiken.teleop_node:main'
        ],
    },
)
