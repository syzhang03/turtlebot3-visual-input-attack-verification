from setuptools import setup

setup(
    name='tb3_lidar_attack', version='0.1.0', packages=['tb3_lidar_attack'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/tb3_lidar_attack']),
        ('share/tb3_lidar_attack', ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='DASE7508 team', maintainer_email='maintainer@example.com',
    description='LiDAR range-bias relay for controlled simulation experiments',
    license='Apache-2.0',
    entry_points={'console_scripts': ['lidar_bias = tb3_lidar_attack.node:main']},
)
