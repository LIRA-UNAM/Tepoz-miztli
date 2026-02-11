from setuptools import setup
import os
from glob import glob

package_name = 'mission1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Índice del paquete
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # Launch files (por si luego agregas)
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tu_nombre',
    maintainer_email='tu_email@correo.com',
    description='Mission manager con máquina de estados para dron PX4 en offboard',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blue_mission = mission1.blue_mission:main',
        ],
    },
)

