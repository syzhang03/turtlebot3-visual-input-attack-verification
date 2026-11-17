from setuptools import setup

setup(
    name='tb3_lidar_baseline', version='0.1.0',
    packages=['tb3_lidar_baseline'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/tb3_lidar_baseline']),
        ('share/tb3_lidar_baseline', ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='DASE7508 team', maintainer_email='maintainer@example.com',
    description='LiDAR-only forward navigation baseline', license='Apache-2.0',
    entry_points={'console_scripts': ['lidar_baseline = tb3_lidar_baseline.node:main']},
)
