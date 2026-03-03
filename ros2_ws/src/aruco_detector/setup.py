from setuptools import find_packages, setup

package_name = 'aruco_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/aruco_detector']),
        ('share/aruco_detector', ['package.xml']),
        ('share/aruco_detector/launch', ['launch/aruco.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yehousa',
    maintainer_email='yesbalonori@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'detect_aruco_node = aruco_detector.detect_aruco_node:main',
        ],
    },
)

