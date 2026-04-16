"""
Launch Mision 4: Pizarron - Aterrizaje

Despegue, Esquivar columnas, Dibujar pizarrón e Identiica zona de aterizaje, Landing.

Levanta: 
vision.launch.py
Nodo mision4

Comando de uso en la terminal:
ros2 launch TMR mision4.launch.py
""" 

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_source import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    #Paquetes
    vision_share = get_package_share_directory('vision')
    
    #Subnodos
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vision_share, 'launch',  'vision_launch.py')
        )
    )

    mision4_node = Node(
        package='TMR',
        executable='mision4',
        name='mision4',
        output='screen',
        emulate_tty=True,
    )
 
    return LaunchDescription([
        vision_launch,
        mision4_node,
    ])
