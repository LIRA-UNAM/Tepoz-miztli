from setuptools import setup

package_name = 'px4_takeoff'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yehousa',
    maintainer_email='yehousa@todo.todo',
    description='PX4 takeoff using MAVROS OFFBOARD',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'takeoff = px4_takeoff.takeoff_node:main',
            # 'OpticalFLow_takeoff = px4_takeoff.OpticalFlow_takeOff:main', #Quique
            'Hover_OpticalFLow = px4_takeoff.Hover_OpticalFlow:main', #Quique 2
            # 'Trajectory = px4_takeoff.Trajectory_Test:main', #Quique
            'sensor_position = px4_takeoff.sensor_position:main',
        ],
    },
)

