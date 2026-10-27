import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'auv_bringup'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'missions'),
            glob('config/missions/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lemuel Ossai Ifuwe',
    maintainer_email='lemuelossaiifuwe@gmail.com',
    description='Launch files and configuration for the Pi-based AUV.',
    license='MIT',
    entry_points={'console_scripts': []},
)
