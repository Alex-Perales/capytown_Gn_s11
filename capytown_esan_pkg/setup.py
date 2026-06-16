from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'capytown_esan_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CapyTown Grupo 4',
    maintainer_email='aalexperm@gmail.com',
    description='CapyTown G4 - lane_detector y lane_controller (RC-2, Semana 11)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_detector = capytown_esan.lane_detector:main',
            'lane_controller = capytown_esan.lane_controller:main',
            'lane_node = capytown_esan.lane_node:main',
        ],
    },
)
