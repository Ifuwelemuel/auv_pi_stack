from setuptools import find_packages, setup

package_name = 'auv_drivers'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lemuel ossai Ifuwe',
    maintainer_email='ifuwelemuel123@gmail.com',
    description='Hardware drivers: PCA9685 thrusters, Bar02 depth, serial GPS.',
    license='MIT',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'thruster_driver = auv_drivers.thruster_driver:main',
            'depth_sensor = auv_drivers.depth_sensor:main',
            'gps_node = auv_drivers.gps_node:main',
        ],
    },
)
