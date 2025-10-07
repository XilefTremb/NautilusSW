from setuptools import setup, find_packages

package_name = 'mavlink_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages= find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mavlink_bridge']),
        ('share/mavlink_bridge', ['package.xml']),
    ],
    install_requires=['rclpy', 'pymavlink'],
    entry_points={
        'console_scripts': [
            'mavlink_telemetry_bridge = telemetry_bridge.mavlink_telemetry_bridge:main',
        ],
    },
)
