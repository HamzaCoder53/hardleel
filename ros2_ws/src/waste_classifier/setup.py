from glob import glob
from setuptools import setup

package_name = 'waste_classifier'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob('models/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hardleel',
    maintainer_email='you@example.com',
    description='On-demand waste classification service',
    license='MIT',
    entry_points={
        'console_scripts': [
            'classify_service = waste_classifier.classify_service:main',
            'test_photos = waste_classifier.test_photos:main',
            'clip_classifier = waste_classifier.clip_classifier_node:main',
            'clip_test_images = waste_classifier.clip_test_images:main',
            'clip_monitor_test = waste_classifier.clip_monitor_test:main',
        ],
    },
)
