from setuptools import setup


setup(
    name='tb3_camera_attack',
    version='0.1.0',
    packages=['tb3_camera_attack'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/tb3_camera_attack']),
        ('share/tb3_camera_attack', ['package.xml']),
        ('share/tb3_camera_attack/config', ['config/camera_occlusion.yaml']),
        ('share/tb3_camera_attack/launch', ['launch/camera_occlusion.launch.py']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='DASE7508 team',
    maintainer_email='maintainer@example.com',
    description='Clean camera relay and configurable black image occlusion',
    license='Apache-2.0',
    entry_points={
        'console_scripts': ['camera_occlusion = tb3_camera_attack.node:main'],
    },
)
