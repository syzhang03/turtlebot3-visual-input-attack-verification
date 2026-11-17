from setuptools import setup

setup(
    name='tb3_sensor_bringup', version='0.1.0',
    packages=['tb3_sensor_bringup'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/tb3_sensor_bringup']),
        ('share/tb3_sensor_bringup', ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='DASE7508 team', maintainer_email='maintainer@example.com',
    description='Wheel-speed conversion and four-sensor synchronised recording',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'wheel_speed = tb3_sensor_bringup.wheel_speed:main',
        'sensor_sync = tb3_sensor_bringup.sensor_sync:main',
        'check_sensors = tb3_sensor_bringup.check_sensors:main',
    ]},
)
