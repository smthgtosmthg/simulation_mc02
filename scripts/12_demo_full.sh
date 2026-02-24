#!/bin/bash
###############################################################################
# Script 12 : Démo complète — Multi-drones + Communication (RSSI + Latence)
#
# Ce script orchestre TOUT :
#   Terminal 1 : Gazebo + N drones ArduPilot SITL
#   Terminal 2 : Bridge ArduPilot ↔ NS-3 (RSSI + Latence)
#   Terminal 3 : Vol automatique multi-drones
#
# Usage :
#   ./12_demo_full.sh           → 3 drones, mode standalone
#   ./12_demo_full.sh 5         → 5 drones, mode standalone
#   ./12_demo_full.sh 3 ns3     → 3 drones, avec NS-3
#
# Pour arrêter : Ctrl+C (nettoyage automatique)
###############################################################################
set -e

N_DRONES=${1:-3}
MODE=${2:-standalone}    # "standalone" ou "ns3"

WORKSPACE="$HOME/simulation_mc02"
SCRIPTS="$WORKSPACE/scripts"

echo "=============================================="
echo "  DÉMO COMPLÈTE — $N_DRONES drones"
echo "  Mode communication : $MODE"
echo "=============================================="
echo ""

# ============================================================
# Nettoyage
# ============================================================
echo "Nettoyage des anciens processus..."
pkill -f arducopter 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f sim_vehicle 2>/dev/null || true
pkill -f mavproxy 2>/dev/null || true
pkill -f ns3_bridge 2>/dev/null || true
sleep 2

PIDS=()

cleanup() {
    echo ""
    echo "========================================"
    echo " Arrêt de la démo..."
    echo "========================================"
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    pkill -f arducopter 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f sim_vehicle 2>/dev/null || true
    sleep 1
    echo ""
    echo "Résultats sauvegardés dans :"
    echo "  $WORKSPACE/comm_metrics.csv"
    echo ""
    echo "Démo terminée."
    exit 0
}

trap cleanup SIGINT SIGTERM

# ============================================================
# Étape 1 : Setup Warehouse (si pas déjà fait)
# ============================================================
if [ ! -f "$WORKSPACE/worlds/warehouse.sdf" ]; then
    echo "[1/4] Setup Warehouse..."
    bash "$SCRIPTS/05_setup_warehouse.sh"
else
    echo "[1/4] Warehouse déjà configuré."
fi

# ============================================================
# Étape 2 : Lancer les drones
# ============================================================
echo ""
echo "[2/4] Lancement de $N_DRONES drones dans le warehouse..."
echo "      (Gazebo headless + ArduPilot SITL)"
echo ""

# On lance le script multi-drones en forçant headless (mode 2)
# On le fait manuellement ici pour contrôler les processus
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$WORKSPACE/models:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$WORKSPACE/worlds:$GZ_SIM_RESOURCE_PATH

# Créer les modèles iris
ARDUPILOT_DIR="$HOME/ardupilot"
PLUGIN_DIR="$HOME/ardupilot_gazebo"
IRIS_MODEL_SRC="$PLUGIN_DIR/models/iris_with_ardupilot"
MODELS_DIR="$WORKSPACE/models"
WORLDS_DIR="$WORKSPACE/worlds"
GENERATED_WORLD="$WORLDS_DIR/warehouse_drones.sdf"
DRONE_SPACING=3

# Vérification
if [ ! -d "$ARDUPILOT_DIR" ]; then
    echo "ERREUR : ArduPilot non trouvé. Lance d'abord 03_install_px4_sitl.sh"
    exit 1
fi

mkdir -p "$MODELS_DIR"

echo "Création des modèles iris..."
for i in $(seq 0 $((N_DRONES - 1))); do
    MODEL_NAME="iris_instance_$i"
    MODEL_DEST="$MODELS_DIR/$MODEL_NAME"
    PORT_IN=$((9002 + i * 10))
    PORT_OUT=$((9003 + i * 10))
    rm -rf "$MODEL_DEST"
    cp -r "$IRIS_MODEL_SRC" "$MODEL_DEST"
    sed -i "s|<fdm_port_in>[0-9]\+</fdm_port_in>|<fdm_port_in>$PORT_IN</fdm_port_in>|g" "$MODEL_DEST/model.sdf"
    sed -i "s|<fdm_port_out>[0-9]\+</fdm_port_out>|<fdm_port_out>$PORT_OUT</fdm_port_out>|g" "$MODEL_DEST/model.sdf" 2>/dev/null || true
    echo "  Drone $i : fdm_in=$PORT_IN"
done

# Générer le monde SDF (utiliser le script existant si dispo, sinon en ligne)
bash "$SCRIPTS/06_launch_multi_drones.sh" "$N_DRONES" << 'INPUT'
2
INPUT
&
LAUNCH_PID=$!
PIDS+=($LAUNCH_PID)

# Attendre que les drones soient prêts
echo ""
echo "Attente du démarrage complet (60s)..."
sleep 60

# ============================================================
# Étape 3 : Lancer le bridge communication
# ============================================================
echo ""
echo "[3/4] Lancement du bridge communication (RSSI + Latence)..."

if [ "$MODE" = "ns3" ]; then
    python3 "$SCRIPTS/11_ns3_bridge.py" --drones "$N_DRONES" --duration 120 &
else
    python3 "$SCRIPTS/11_ns3_bridge.py" --drones "$N_DRONES" --no-ns3 --duration 120 &
fi
BRIDGE_PID=$!
PIDS+=($BRIDGE_PID)

# Laisser le bridge se connecter
sleep 10

# ============================================================
# Étape 4 : Vol automatique
# ============================================================
echo ""
echo "[4/4] Vol automatique multi-drones..."
echo ""

python3 "$SCRIPTS/08_multi_drone_flight.py" --drones "$N_DRONES" --hover 30 &
FLIGHT_PID=$!
PIDS+=($FLIGHT_PID)

# Attendre la fin du vol
wait $FLIGHT_PID 2>/dev/null || true

echo ""
echo "Vol terminé. Le bridge continue la collecte..."
echo "Ctrl+C pour arrêter."

# Attendre
wait
