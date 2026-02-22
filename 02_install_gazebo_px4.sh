#!/bin/bash
###############################################################################
# 02_install_gazebo_px4.sh
# Installation de Gazebo Harmonic + PX4 Autopilot SITL
# Pour simulation multi-drones dans un environnement Warehouse
###############################################################################
set -e

export HOME_DIR="$HOME"
export WORKDIR="$HOME/drone_simulation"
mkdir -p "$WORKDIR"
cd "$WORKDIR"

echo "=============================================="
echo " ÉTAPE 1: Installation de ROS 2 Humble"
echo "=============================================="

# Ajouter le repo ROS 2
if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
fi

sudo apt-get update
sudo apt-get install -y \
    ros-humble-desktop \
    ros-humble-ros-base \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep

# Initialiser rosdep (ignorer si déjà fait)
sudo rosdep init 2>/dev/null || true
rosdep update

# Source ROS 2 automatiquement
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi
source /opt/ros/humble/setup.bash

echo "[OK] ROS 2 Humble installé"

echo "=============================================="
echo " ÉTAPE 2: Installation de Gazebo Harmonic"
echo "=============================================="

# Installer Gazebo Harmonic (GZ Harmonic = compatible ROS 2 Humble)
sudo apt-get install -y ros-humble-ros-gz

# Vérifier l'installation
echo "[INFO] Test de Gazebo..."
if gz sim --version > /dev/null 2>&1; then
    echo "[OK] Gazebo Harmonic installé: $(gz sim --version)"
else
    # Installation alternative directe
    echo "[INFO] Installation directe de Gazebo Harmonic..."
    sudo wget https://packages.osrfoundation.org/gazebo.gpg \
        -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
        sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    sudo apt-get update
    sudo apt-get install -y gz-harmonic
    echo "[OK] Gazebo Harmonic installé"
fi

echo "=============================================="
echo " ÉTAPE 3: Installation de PX4 Autopilot"
echo "=============================================="

cd "$WORKDIR"

if [ ! -d "PX4-Autopilot" ]; then
    echo "[INFO] Clonage de PX4-Autopilot..."
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
else
    echo "[INFO] PX4-Autopilot existe déjà, mise à jour..."
    cd PX4-Autopilot
    git pull
    git submodule update --init --recursive
    cd ..
fi

cd PX4-Autopilot

# Installer les dépendances PX4 (script officiel)
echo "[INFO] Installation des dépendances PX4..."
bash ./Tools/setup/ubuntu.sh --no-sim-tools

# Installer le setup pour Gazebo (gz)
echo "[INFO] Configuration PX4 + Gazebo..."
pip3 install --user pyros-genpymsg

echo "[OK] PX4 Autopilot installé"

echo "=============================================="
echo " ÉTAPE 4: Compiler PX4 SITL avec Gazebo"
echo "=============================================="

cd "$WORKDIR/PX4-Autopilot"

# Build PX4 SITL pour Gazebo
echo "[INFO] Compilation de PX4 SITL (première compilation, ~10-15 min)..."
make px4_sitl gz_x500

# Arrêter la simulation de test (Ctrl+C automatique)
sleep 10
echo "[INFO] Compilation terminée. Arrêt de la simulation de test..."
pkill -f px4 2>/dev/null || true
pkill -f gz 2>/dev/null || true
sleep 3

echo "[OK] PX4 SITL compilé et testé"

echo "=============================================="
echo " ÉTAPE 5: Installation MAVROS + Micro XRCE-DDS"
echo "=============================================="

# MAVROS pour communication ROS 2 ↔ PX4
sudo apt-get install -y \
    ros-humble-mavros \
    ros-humble-mavros-extras

# Installer les données géographiques pour MAVROS
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Micro XRCE-DDS Agent (communication PX4 ↔ ROS 2)
cd "$WORKDIR"
if [ ! -d "Micro-XRCE-DDS-Agent" ]; then
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    sudo make install
    sudo ldconfig
else
    echo "[INFO] Micro-XRCE-DDS-Agent déjà installé"
fi

echo "[OK] MAVROS + Micro XRCE-DDS installés"

echo "=============================================="
echo " ÉTAPE 6: Télécharger le monde Warehouse"
echo "=============================================="

cd "$WORKDIR"
mkdir -p gazebo_worlds

# Créer un monde Warehouse pour Gazebo
cat > gazebo_worlds/warehouse.sdf << 'WAREHOUSE_EOF'
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="warehouse">
    
    <!-- Physics -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    
    <!-- Lumière -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Sol -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Murs de l'entrepôt -->
    <!-- Mur Nord -->
    <model name="wall_north">
      <static>true</static>
      <pose>0 15 2.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>30 0.3 5</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>30 0.3 5</size></box></geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Mur Sud -->
    <model name="wall_south">
      <static>true</static>
      <pose>0 -15 2.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>30 0.3 5</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>30 0.3 5</size></box></geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Mur Est -->
    <model name="wall_east">
      <static>true</static>
      <pose>15 0 2.5 0 0 1.5708</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>30 0.3 5</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>30 0.3 5</size></box></geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Mur Ouest -->
    <model name="wall_west">
      <static>true</static>
      <pose>-15 0 2.5 0 0 1.5708</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>30 0.3 5</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>30 0.3 5</size></box></geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Étagères dans l'entrepôt -->
    <!-- Rangée 1 -->
    <model name="shelf_1">
      <static>true</static>
      <pose>-8 -5 1.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>1 8 3</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1 8 3</size></box></geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.7 0.5 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Rangée 2 -->
    <model name="shelf_2">
      <static>true</static>
      <pose>-3 -5 1.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>1 8 3</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1 8 3</size></box></geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.7 0.5 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Rangée 3 -->
    <model name="shelf_3">
      <static>true</static>
      <pose>3 -5 1.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>1 8 3</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1 8 3</size></box></geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.7 0.5 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Rangée 4 -->
    <model name="shelf_4">
      <static>true</static>
      <pose>8 -5 1.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>1 8 3</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1 8 3</size></box></geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.7 0.5 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Rangées du haut -->
    <model name="shelf_5">
      <static>true</static>
      <pose>-8 5 1.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>1 8 3</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1 8 3</size></box></geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.7 0.5 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <model name="shelf_6">
      <static>true</static>
      <pose>3 5 1.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>1 8 3</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1 8 3</size></box></geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.7 0.5 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Boîtes/palettes -->
    <model name="pallet_1">
      <static>true</static>
      <pose>-10 10 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>1.2 1.2 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1.2 1.2 1</size></box></geometry>
          <material>
            <ambient>0.8 0.6 0.2 1</ambient>
            <diffuse>0.9 0.7 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <model name="pallet_2">
      <static>true</static>
      <pose>10 10 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>1.2 1.2 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1.2 1.2 1</size></box></geometry>
          <material>
            <ambient>0.8 0.6 0.2 1</ambient>
            <diffuse>0.9 0.7 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
  </world>
</sdf>
WAREHOUSE_EOF

echo "[OK] Monde Warehouse créé: $WORKDIR/gazebo_worlds/warehouse.sdf"

echo ""
echo "=============================================="
echo " [DONE] Gazebo + PX4 SITL installés!"
echo "=============================================="
echo ""
echo "Pour tester un drone seul:"
echo "  cd ~/drone_simulation/PX4-Autopilot"
echo "  make px4_sitl gz_x500"
echo ""
echo "Pour tester dans le Warehouse (headless, sans rendering):"
echo "  export HEADLESS=1"
echo "  make px4_sitl gz_x500"
echo ""
echo "Prochaine étape: ./03_install_ns3_sionna.sh"
