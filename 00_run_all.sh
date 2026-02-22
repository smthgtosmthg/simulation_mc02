#!/bin/bash
###############################################################################
# 00_run_all.sh
# Script maître - exécute tout dans l'ordre
# Usage: ./00_run_all.sh [étape]
#   ./00_run_all.sh        → tout installer
#   ./00_run_all.sh 3      → reprendre à l'étape 3
###############################################################################
set -e

START_STEP=${1:-1}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=============================================="
echo "  INSTALLATION COMPLÈTE - Simulation Drones"
echo "  Digital Twin: Gazebo + PX4 + NS-3 + Sionna"
echo "=============================================="
echo ""
echo "  Démarrage à l'étape: $START_STEP"
echo ""

run_step() {
    local step=$1
    local script=$2
    local desc=$3
    
    if [ $step -ge $START_STEP ]; then
        echo ""
        echo "######################################################"
        echo "  ÉTAPE $step: $desc"
        echo "######################################################"
        echo ""
        
        chmod +x "$SCRIPT_DIR/$script"
        bash "$SCRIPT_DIR/$script"
        
        echo ""
        echo "[OK] Étape $step terminée."
        echo ""
        read -p "Continuer avec l'étape suivante? (O/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Nn]$ ]]; then
            echo "Arrêté. Relancer avec: ./00_run_all.sh $((step + 1))"
            exit 0
        fi
    fi
}

run_step 1 "01_system_setup.sh"           "Dépendances système + NVIDIA"
run_step 2 "02_install_gazebo_px4.sh"     "Gazebo + PX4 SITL"
run_step 3 "03_install_ns3_sionna.sh"     "NS-3 + ns3sionna"
# run_step 4 "04_install_ns3rt.sh"        "ns3-rt (ALTERNATIVE - décommenter si besoin)"
run_step 5 "06_verify_installation.sh"    "Vérification installation"
run_step 6 "07_ns3_rssi_latency_example.sh" "Exemple RSSI + Latence"

echo ""
echo "=============================================="
echo "  INSTALLATION TERMINÉE!"
echo "=============================================="
echo ""
echo "  Pour lancer la simulation multi-drones:"
echo "    ./05_launch_multi_drone.sh 3"
echo ""
echo "  Pour le mode sans GPU (headless):"
echo "    HEADLESS=1 ./05_launch_multi_drone.sh 3"
echo ""
