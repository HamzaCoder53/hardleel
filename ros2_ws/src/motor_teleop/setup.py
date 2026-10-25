from glob import glob
from setuptools import setup

package_name = 'motor_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='cmd_vel to Pico serial motor bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motor_bridge = motor_teleop.motor_bridge:main',
            'deadband_calibrate = motor_teleop.deadband_calibrate:main',
            'motor_calibrator = motor_teleop.motor_calibrator:main',
            'fit_speed_from_bag = motor_teleop.fit_speed_from_bag:main',
        ],
    },
)
