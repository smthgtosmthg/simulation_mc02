#!/bin/bash
###############################################################################
# 05_launch_multi_drone.sh
# Lance une simulation multi-drones (3 drones) dans le Warehouse
# PX4 SITL + Gazebo + communication inter-drones
###############################################################################
set -e

export WORKDIR="$HOME/drone_simulation"
export PX4_DIR="$WORKDIR/PX4-Autopilot"

# Nombre de drones
NUM_DRONES=${1:-3}

# Mode headless (sans rendering) - mettre 0 pour afficher Gazebo
HEADLESS=${HEADLESS:-0}

echo "=============================================="
echo " Lancement simulation: $NUM_DRONES drones"
echo " Mode headless: $HEADLESS"
echo "=============================================="

# Sourcer ROS 2
source /opt/ros/humble/setup.bash

# -- Nettoyer les anciennes instances --
echo "[INFO] Nettoyage des anciennes instances..."
pkill -f "px4" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f "MicroXRCEAgent" 2>/dev/null || true
sleep 2

# -- Lancer Micro XRCE-DDS Agent --
echo "[INFO] Démarrage Micro XRCE-DDS Agent..."
MicroXRCEAgent udp4 -p 8888 &
XRCE_PID=$!
echo "[OK] XRCE-DDS Agent PID: $XRCE_PID"
sleep 2

# -- Lancer Gazebo avec le monde Warehouse --
echo "[INFO] Démarrage Gazebo Warehouse..."

if [ "$HEADLESS" = "1" ]; then
    gz sim -s -r "$WORKDIR/gazebo_worlds/warehouse.sdf" &
else
    gz sim -r "$WORKDIR/gazebo_worlds/warehouse.sdf" &
fi
GZ_PID=$!
echo "[OK] Gazebo PID: $GZ_PID"
sleep 5

# -- Lancer les instances PX4 SITL --
DRONE_PIDS=()

for i in $(seq 1 $NUM_DRONES); do
    echo ""
    echo "[INFO] Démarrage Drone $i / $NUM_DRONES ..."
    
    INSTANCE_ID=$((i - 1))
    
    # Positions de spawn (espacées dans le warehouse)
    case $i in
        1) SPAWN_X="-5"; SPAWN_Y="0"; SPAWN_Z="0.5" ;;
        2) SPAWN_X="0";  SPAWN_Y="0"; SPAWN_Z="0.5" ;;
        3) SPAWN_X="5";  SPAWN_Y="0"; SPAWN_Z="0.5" ;;
        4) SPAWN_X="-5"; SPAWN_Y="5"; SPAWN_Z="0.5" ;;
        5) SPAWN_X="0";  SPAWN_Y="5"; SPAWN_Z="0.5" ;;
        *) SPAWN_X="$((i * 3 - 10))"; SPAWN_Y="8"; SPAWN_Z="0.5" ;;
    esac
    
    # Variables d'environnement PX4 pour multi-instance
    export PX4_SYS_AUTOSTART=4001
    export PX4_GZ_MODEL=x500
    export PX4_GZ_MODEL_POSE="${SPAWN_X},${SPAWN_Y},${SPAWN_Z},0,0,0"
    export PX4_GZ_WORLD=warehouse
    export PX4_INSTANCE=$INSTANCE_ID
    
    # Lancer PX4 SITL pour ce drone
    cd "$PX4_DIR"
    
    # Chaque instance PX4 utilise des ports différents
    # MAVLink: 14540 + instance, SITL: 14560 + instance
    $PX4_DIR/build/px4_sitl_default/bin/px4 \
        -i $INSTANCE_ID \
        -d "$PX4_DIR/build/px4_sitl_default/etc" \
        >"/tmp/px4_drone_${i}.log" 2>&1 &
    
    DRONE_PIDS+=($!)
    echo "[OK] Drone $i lancé (instance $INSTANCE_ID) à position ($SPAWN_X, $SPAWN_Y, $SPAWN_Z)"
    echo "     MAVLink port: $((14540 + INSTANCE_ID))"
    echo "     Log: /tmp/px4_drone_${i}.log"
    
    sleep 3
done

echo ""
echo "=============================================="
echo " SIMULATION ACTIVE"
echo "=============================================="
echo ""
echo " Drones actifs: $NUM_DRONES"
echo " Gazebo PID: $GZ_PID"
echo " XRCE-DDS PID: $XRCE_PID"
echo ""
echo " Ports MAVLink par drone:"
for i in $(seq 1 $NUM_DRONES); do
    echo "   Drone $i: localhost:$((14540 + i - 1))"
done
echo ""
echo " Logs:"
for i in $(seq 1 $NUM_DRONES); do
    echo "   Drone $i: /tmp/px4_drone_${i}.log"
done
echo ""
echo " Pour se connecter via MAVROS:"
echo "   ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557"
echo ""
echo " Pour lister les topics ROS 2:"
echo "   ros2 topic list"
echo ""
echo " Pour voir la position d'un drone:"
echo "   ros2 topic echo /fmu/out/vehicle_local_position"
echo ""
echo "=============================================="
echo " Appuie Ctrl+C pour arrêter tout"
echo "=============================================="

# Fonction de nettoyage
cleanup() {
    echo ""
    echo "[INFO] Arrêt de la simulation..."
    for pid in "${DRONE_PIDS[@]}"; do
        kill $pid 2>/dev/null || true
    done
    kill $GZ_PID 2>/dev/null || true
    kill $XRCE_PID 2>/dev/null || true
    pkill -f "px4" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    echo "[OK] Simulation arrêtée."
}

trap cleanup SIGINT SIGTERM

# Attendre
wait
