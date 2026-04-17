from setuptools import find_packages, setup
import os
from setuptools import setup
from glob import glob

package_name = 'TMR'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='humubuntu',
    maintainer_email='dragonoidhor@gmail.com',
    description='Lanzador de misisones para el TMR 2026',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            #Misiones (Paquete de misiones)
            'mision1 = TMR.mision1:main',
            'mision2 = TMR.mision2:main',
            'mision3 = TMR.mision3:main',
            'mision4 = TMR.mision4:main',
            'mision1LA = TMR.mision1_lazo_abierto:main',
            'mision4LA = TMR.mision4_lazo_abierto:main',
            'version2 = TMR.mision1_v2:main',
            'blue_abierto = TMR.lazo_abierto_blue:main',
            'mision5LA = TMR.mision5_lazo_abierto:main',
            # 'mision5 = TMR.mision5:main',
            # 'mision6 = TMR.mision6:main',
            #
        ],
    },
)
