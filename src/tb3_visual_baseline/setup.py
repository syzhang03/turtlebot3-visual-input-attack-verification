from setuptools import setup

setup(
    name='tb3_visual_baseline', version='0.1.0',
    packages=['tb3_visual_baseline'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/tb3_visual_baseline']),
        ('share/tb3_visual_baseline', ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='DASE7508 team', maintainer_email='maintainer@example.com',
    description='Camera-only forward navigation baseline', license='Apache-2.0',
    entry_points={'console_scripts': ['baseline = tb3_visual_baseline.node:main']},
)
