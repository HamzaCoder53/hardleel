from glob import glob
from setuptools import setup

package_name = 'hardleel_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hardleel',
    maintainer_email='hardleel@example.com',
    description='Behaviour tree mission for the hardleel waste-sorting robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission = hardleel_mission.mission_node:main',
            'where = hardleel_mission.mission_node:where',
        ],
    },
)
