from setuptools import setup
import os
from glob import glob

package_name = 'stair_detection_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'rviz2'), glob('rviz2/*.rviz')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nimrod',
    maintainer_email='nimicu21@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stair_detection_node = stair_detection_ros.stair_detection_node:main',
            'stair_detection_fused_node = stair_detection_ros.stair_detection_fused_node:main'
        ],
    },
)
