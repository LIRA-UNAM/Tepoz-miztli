# Drone-Software
Software repository for Pumas Drone Team, for the TMR 2026

# Instalar MicroXRDE
Debe de ir en el directorio raíz.

git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git

cd Micro-XRCE-DDS-Agent

mkdir build

cd build

cmake ..

make

sudo make install

sudo ldconfig /usr/local/lib/

# Instalación de QGroundControl
Antes de instalar asegurarse de tener esto actualizado e instalado

sudo usermod -a -G dialout $USER

sudo apt-get remove modemmanager -y

sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y

sudo apt install libfuse2 -y

sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y

Reiniciar o cerrar la terminar para aplicar cambios o en su defecto
source install/setup.bash

Instalar .AppImage de QGroundControl
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage

Instala y ejecuta
chmod +x ./QGroundControl-x86_64.AppImage

./QGroundControl-x86_64.AppImage  (or double click)

# Dentro de drone_ws y de px4_ros_com_ws
colcon build

# En la raíz o en Tepoz-miztli sourcea esto
source ~/Tepoz-miztli/px4_ros_com_ws/install/setup.bash

source ~/Tepoz-miztli/drone_ws/install/setup.bash

# Ejecutar el launch pero con start_simulation_2.py
ros2 launch simulation_launch_pkg start_simulation_2.py qgc_path:="/home/$USER$/Desktop/QGroundControl-x86_64.AppImage"

# En otra terminal activa la camara o el rtq
ros2 run rqt_image_view rqt_image_view 

# En otra terminal ejecuta el topico o nodo para moverlo manual
ros2 run pix_commander_pkg manual 0.0 -2.5 1.2 90.0 [X Y Z Angulo]
