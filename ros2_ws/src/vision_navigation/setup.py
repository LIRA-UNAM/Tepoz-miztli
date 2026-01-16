from setuptools import find_packages, setup
import os
from glob  import glob

package_name = 'vision_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'weights'), glob('weights/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='quique',
    maintainer_email='enriquemedranorabak@hotmail.com',
    description='Paquete de navegación visual para dron con YOLO y RealSense',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detector = vision_navigation.yolo_node:main',
        ],
    },
)
