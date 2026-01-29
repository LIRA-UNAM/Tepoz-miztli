from setuptools import find_packages, setup

package_name = 'visual_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        'realsense_processor_node = visual_odometry.realsense_processor:main',
        'visual_odometry_node = visual_odometry.VO:main'
        ],
    },
)
