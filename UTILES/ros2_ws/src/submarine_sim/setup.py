from setuptools import setup

package_name = 'submarine_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'sensor_msgs'],
    zip_safe=True,
    maintainer='Xavier Lefebvre',
    maintainer_email='xavier@example.com',
    description='Simulation de capteurs fictifs',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_publisher = submarine_sim.sensor_publisher:main',
            'gui_sensor_display = submarine_sim.gui_sensor_display:main',
        ],
    },
)
