from setuptools import find_packages, setup

package_name = 'auv_control'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lemuel Ossai Ifuwe',
    maintainer_email='lemuelossaiifuwe@gmail.com',
    description='Safety, arbitration, teleop, autopilot and navigation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_supervisor = auv_control.safety_supervisor:main',
            'actuator_mixer = auv_control.actuator_mixer:main',
            'teleop_node = auv_control.teleop_node:main',
            'autopilot = auv_control.autopilot:main',
            'navigator = auv_control.navigator:main',
        ],
    },
)
