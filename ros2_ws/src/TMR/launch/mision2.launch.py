"""
Launch Mision 2: Circuito completo Green

Despegue, Cruce de Gates Green, Esquivar columnas, Encontrar Aruco
Dibujar en el pizarrón, Identiica zona de aterizaje, Landing.

Levanta: 
vision.launch.py
Nodo mision2

Comando de uso en la terminal:
ros2 launch TMR mision2.launch.py
""" 

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_source import PythonLaunchDescriptSource
from launch_ros.actions import Node

def generate_launch_description():
    #Paquetes
    vision_share = get_package_share_directory('vision')
    
    #Subnodos
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptSource(
            os.path.join(vision_share, 'launch',  'vision_launch.py')
        )
    )

    mision2_node = Node(
        package='TMR',
        executable='mision2',
        name='mision2',
        output='screen',
        emulate_tty=True,
    )
 
    return LaunchDescription([
        vision_launch,
        mision2_node,
    ])
