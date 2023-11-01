import os
from glob import glob
from setuptools import setup

package_name = 'stair_detection_ros'

setup(
    name=package_name,
    version='0.1.0',
    # Packages to export
    packages=[package_name],
    # Files we want to install
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz2'), 
            glob(os.path.join('rviz2', '*.rviz'))),
        (os.path.join('share', package_name, 'models'), 
            glob(os.path.join('models', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nimrod',
    maintainer_email='nimicu21@gmail.com',
    classifiers=[
        'Intended Audience :: Developers',
        'License :: Apache License 2.0',
        'Programming Language :: Python3',
        'Topic :: Software Development',
    ],
    description='Stair detection package using yolov8 for exo-skeleton system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'stair_detection = stair_detection_ros.stair_detection:main',
            'stair_detection_fused = stair_detection_ros.stair_detection_fused:main'
        ],
    },
)
