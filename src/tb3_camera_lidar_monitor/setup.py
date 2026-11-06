from setuptools import setup
setup(name='tb3_camera_lidar_monitor', version='0.1.0', packages=['tb3_camera_lidar_monitor'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/tb3_camera_lidar_monitor']),
                  ('share/tb3_camera_lidar_monitor', ['package.xml'])],
      install_requires=['setuptools'], license='Apache-2.0',
      entry_points={'console_scripts': ['camera_lidar_monitor = tb3_camera_lidar_monitor.node:main']})
