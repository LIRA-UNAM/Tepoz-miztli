from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolo_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='humubuntu',
    maintainer_email='dragonoidhor@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_node = yolo_vision.yolo_detector:main',
            'auto_pilot = yolo_vision.vision_navigation:main',
            'gates = yolo_vision.yolo_gates:main',
        ],
    },
)
