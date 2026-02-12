from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'nautilus_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='devs',
    maintainer_email='dosg0801@usherbrooke.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'test_mission = nautilus_mission.launch.test_mission:main',
        ],
    },
)
