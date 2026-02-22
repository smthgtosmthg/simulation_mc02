#!/bin/bash
###############################################################################
# 06_verify_installation.sh
# Vérifie que tout est correctement installé
###############################################################################

echo "=============================================="
echo " VÉRIFICATION DE L'INSTALLATION"
echo "=============================================="
echo ""

PASS=0
FAIL=0

check() {
    local name="$1"
    local cmd="$2"
    
    if eval "$cmd" > /dev/null 2>&1; then
        echo "  [✓] $name"
        PASS=$((PASS + 1))
    else
        echo "  [✗] $name"
        FAIL=$((FAIL + 1))
    fi
}

echo "--- Système ---"
check "Ubuntu 22.04"          "lsb_release -d | grep -q '22.04'"
check "GCC"                    "gcc --version"
check "CMake"                  "cmake --version"
check "Python3"                "python3 --version"
check "pip3"                   "pip3 --version"
check "Git"                    "git --version"
check "tmux"                   "tmux -V"

echo ""
echo "--- GPU NVIDIA ---"
check "nvidia-smi"             "nvidia-smi"
check "CUDA (nvcc)"            "nvcc --version"

echo ""
echo "--- ROS 2 ---"
check "ROS 2 Humble"           "source /opt/ros/humble/setup.bash && ros2 --help"
check "MAVROS"                 "dpkg -l | grep -q ros-humble-mavros"
check "ros-gz bridge"          "dpkg -l | grep -q ros-humble-ros-gz"

echo ""
echo "--- Gazebo ---"
check "Gazebo (gz sim)"        "gz sim --version"
check "Warehouse world"        "test -f $HOME/drone_simulation/gazebo_worlds/warehouse.sdf"

echo ""
echo "--- PX4 ---"
check "PX4-Autopilot cloné"   "test -d $HOME/drone_simulation/PX4-Autopilot"
check "PX4 SITL build"        "test -f $HOME/drone_simulation/PX4-Autopilot/build/px4_sitl_default/bin/px4"
check "Micro XRCE-DDS Agent"  "which MicroXRCEAgent"

echo ""
echo "--- NS-3 (ns3sionna) ---"
check "NS-3.40 téléchargé"    "test -d $HOME/drone_simulation/ns-allinone-3.40"
check "ns3sionna module"      "test -d $HOME/drone_simulation/ns-allinone-3.40/ns-3.40/contrib/sionna"
NS3_DIR="$HOME/drone_simulation/ns-allinone-3.40/ns-3.40"
check "NS-3 compilé"          "test -d $NS3_DIR/build"

echo ""
echo "--- NS-3-RT (Alternative) ---"
check "ns3-rt cloné"          "test -d $HOME/drone_simulation/ns3-rt"
check "ns3-rt compilé"        "test -d $HOME/drone_simulation/ns3-rt/build"

echo ""
echo "--- Python / Sionna ---"
SIONNA_VENV="$HOME/drone_simulation/ns-allinone-3.40/ns-3.40/contrib/sionna/model/ns3sionna/sionna-venv"
if [ -d "$SIONNA_VENV" ]; then
    source "$SIONNA_VENV/bin/activate"
    check "TensorFlow"        "python3 -c 'import tensorflow'"
    check "Sionna RT"         "python3 -c 'import sionna'"
    
    if nvidia-smi > /dev/null 2>&1; then
        GPU_COUNT=$(python3 -c "import tensorflow as tf; print(len(tf.config.list_physical_devices('GPU')))" 2>/dev/null || echo "0")
        if [ "$GPU_COUNT" -gt "0" ]; then
            echo "  [✓] TensorFlow GPU ($GPU_COUNT GPU(s) détectée(s))"
            PASS=$((PASS + 1))
        else
            echo "  [✗] TensorFlow GPU (GPU détectée mais pas utilisée par TF)"
            FAIL=$((FAIL + 1))
        fi
    fi
    deactivate
else
    echo "  [✗] Environnement Python Sionna non trouvé"
    FAIL=$((FAIL + 1))
fi

echo ""
echo "=============================================="
echo " RÉSULTAT: $PASS réussi(s), $FAIL échoué(s)"
echo "=============================================="

if [ $FAIL -eq 0 ]; then
    echo " Tout est installé correctement!"
else
    echo " Certains composants manquent. Relancer les scripts correspondants."
fi
