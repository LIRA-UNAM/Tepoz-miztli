#!/bin/bash

# --- CONFIGURACIÓN ---
# Si necesitas una versión específica de PX4, cámbiala aquí (ej. v1.14.0)
PX4_VERSION="main" 
DIR_PX4="PX4-Autopilot"

echo "=========================================="
echo "   INSTALADOR DE ENTORNO TEPOZ-MIZTLI"
echo "=========================================="

# 1. Verificar si ya existe PX4
if [ -d "$DIR_PX4" ]; then
    echo "[INFO] La carpeta $DIR_PX4 ya existe."
    echo "¿Deseas borrarla y reinstalarla limpia? (s/n)"
    read -r respuesta
    if [ "$respuesta" = "s" ]; then
        rm -rf "$DIR_PX4"
        echo "[OK] Carpeta anterior eliminada."
    else
        echo "[INFO] Omitiendo descarga de PX4."
    fi
fi

# 2. Descargar PX4 (Solo si no existe)
if [ ! -d "$DIR_PX4" ]; then
    echo "[INFO] Clonando PX4-Autopilot (Esto puede tardar)..."
    # Clonamos recursivo para traer todos los drivers y librerías
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    
    cd $DIR_PX4
    # Si necesitas una versión especifica, descomenta la siguiente línea:
    # git checkout $PX4_VERSION
    cd ..
    echo "[OK] PX4 Descargado correctamente."
fi

# 3. Instalar dependencias de Python necesarias
echo "[INFO] Verificando dependencias de Python..."
pip3 install --user kconfiglib jsonschema jinja2 pyros-genmsg packaging toml numpy future

# 4. Instalar MicroXRCEAgent (Necesario para ROS2)
if ! command -v MicroXRCEAgent &> /dev/null
then
    echo "[INFO] MicroXRCEAgent no encontrado. Instalando..."
    cd ~
    if [ -d "Micro-XRCE-DDS-Agent" ]; then rm -rf Micro-XRCE-DDS-Agent; fi
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent
    mkdir build && cd build
    cmake ..
    make
    sudo make install
    sudo ldconfig /usr/local/lib/
    echo "[OK] MicroXRCEAgent instalado."
else
    echo "[OK] MicroXRCEAgent ya está instalado."
fi

echo "=========================================="
echo "   INSTALACIÓN COMPLETADA EXITOSAMENTE"
echo "=========================================="
echo "Para correr la simulación:"
echo "1. cd Tepoz-miztli"
echo "2. Colcon build en drone_ws y px4_ros_com_ws"
echo "3. source drone_ws/install/setup.bash"
echo "4. source ~/Tepoz-miztli/px4_ros_com_ws/install/setup.bash"
echo "5. ros2 launch simulation_launch_pkg start_simulation_2.py qgc_path:=\"Ruta/A/QGC\""