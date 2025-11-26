import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gazebo_envs'

def get_model_files():
    model_files = []
    for root, dirs, files in os.walk('models'):
        install_path = os.path.join('share', package_name, root)
        model_files.append((install_path, [os.path.join(root, f) for f in files]))
    
    return model_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ] + get_model_files(),
    
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
        ],
    },
)
