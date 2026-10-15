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
        ],
    },
)
