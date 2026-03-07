#!/bin/bash
###############################################################################
# Script 4 : Test d'un seul drone — ArduPilot SITL + Gazebo Harmonic
#
# Ce script lance un drone (iris) dans Gazebo, le connecte à ArduPilot SITL,
# puis donne accès à MAVProxy pour piloter.
#
# Deux modes :
#   A) Manuel   : tu tapes les commandes toi-même dans MAVProxy
#   B) Autonome : le script fait arm → takeoff → hover 15s → land
###############################################################################
set -e

ARDUPILOT_DIR="$HOME/ardupilot"
PLUGIN_DIR="$HOME/ardupilot_gazebo"

echo "=============================================="
echo " [4/4] Test : un drone dans Gazebo"
echo "=============================================="

# --- Vérifier qu'ArduPilot est installé ---
if [ ! -d "$ARDUPILOT_DIR" ]; then
    echo "ERREUR : ArduPilot non trouvé dans $ARDUPILOT_DIR"
    echo "Lance d'abord le script 03_install_px4_sitl.sh"
    exit 1
fi

if [ ! -d "$PLUGIN_DIR" ]; then
    echo "ERREUR : ardupilot_gazebo non trouvé dans $PLUGIN_DIR"
    echo "Lance d'abord le script 03_install_px4_sitl.sh"
    exit 1
fi

# --- Nettoyer les anciens processus ---
echo "Nettoyage des anciens processus..."
pkill -f arducopter 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f mavproxy 2>/dev/null || true
sleep 2

# --- Charger les variables d'environnement ---
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

# --- Mode rendering ---
echo ""
echo "Options de rendering :"
echo "  1) Avec rendering (fenêtre Gazebo visible) — nécessite GPU"
echo "  2) Sans rendering (headless) — moins gourmand"
echo ""
read -p "Choix [1/2, défaut=1] : " RENDER_CHOICE
RENDER_CHOICE=${RENDER_CHOICE:-1}

if [ "$RENDER_CHOICE" = "2" ]; then
    GZ_HEADLESS="-s"
    echo "Mode headless activé."
else
    GZ_HEADLESS=""
fi

# --- Mode de vol ---
echo ""
echo "Mode de vol :"
echo "  1) Manuel    — tu contrôles via le prompt MAVProxy"
echo "  2) Autonome  — arm, takeoff 10m, hover 15s, land automatiquement"
echo ""
read -p "Choix [1/2, défaut=1] : " FLIGHT_MODE
FLIGHT_MODE=${FLIGHT_MODE:-1}

echo ""
echo "==> Lancement de Gazebo avec le monde iris_runway..."
echo ""

# Lancer Gazebo en arrière-plan
gz sim $GZ_HEADLESS -r iris_runway.sdf &
GZ_PID=$!

# Attendre que Gazebo soit prêt
echo "Attente du démarrage de Gazebo (10s)..."
sleep 10

cd "$ARDUPILOT_DIR"

if [ "$FLIGHT_MODE" = "2" ]; then
    # ============================================================
    # MODE AUTONOME : vol automatique
    # ============================================================
    echo ""
    echo "==> MODE AUTONOME : vol automatique"
    echo "==> Lancement d'ArduPilot SITL..."
    echo ""

    # Lancer sim_vehicle.py en arrière-plan (sans --console --map pour éviter les crashs)
    sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-mavproxy &
    SIM_PID=$!

    echo "Attente du démarrage d'ArduPilot SITL (20s pour calibration)..."
    sleep 20

    echo ""
    echo "==> Connexion MAVProxy et vol automatique..."
    echo ""

    # Lancer MAVProxy avec des commandes automatiques
    # On attend que le drone soit prêt, puis on exécute la séquence
    mavproxy.py --master tcp:127.0.0.1:5760 --out 127.0.0.1:14550 \
        --cmd "set streamrate 10;" \
        --cmd "mode guided; arm throttle; takeoff 10;" &
    MAVPROXY_PID=$!

    echo "-----------------------------------------------"
    echo " Vol automatique en cours..."
    echo " Le drone va :"
    echo "   1. Passer en GUIDED"
    echo "   2. Armer les moteurs"
    echo "   3. Décoller à 10m"
    echo "-----------------------------------------------"
    echo ""

    # Attendre le décollage
    sleep 30

    echo "==> Hover pendant 15 secondes..."
    sleep 15

    echo "==> Atterrissage..."
    # Envoyer la commande land via mavproxy
    echo "mode land" | mavproxy.py --master tcp:127.0.0.1:5762 2>/dev/null || true

    echo "Attente de l'atterrissage (20s)..."
    sleep 20

    echo ""
    echo "=============================================="
    echo " Vol autonome terminé !"
    echo "=============================================="

    # Nettoyage
    kill $MAVPROXY_PID 2>/dev/null || true
    kill $SIM_PID 2>/dev/null || true
    kill $GZ_PID 2>/dev/null || true

else
    # ============================================================
    # MODE MANUEL : prompt MAVProxy interactif
    # ============================================================
    echo ""
    echo "==> MODE MANUEL : prompt MAVProxy interactif"
    echo ""
    echo "======================================================="
    echo " SÉQUENCE POUR FAIRE VOLER LE DRONE :"
    echo "======================================================="
    echo ""
    echo "  ⚠ ATTENDS que ces messages apparaissent :"
    echo "     - 'ArduPilot Ready'"
    echo "     - 'EKF3 IMU0 is using GPS'"
    echo "     - 'Flight battery 100 percent'"
    echo ""
    echo "  PUIS tape dans cet ordre :"
    echo "     1) mode guided"
    echo "     2) arm throttle"
    echo "     3) takeoff 10"
    echo ""
    echo "  Pour atterrir :"
    echo "     4) mode land"
    echo ""
    echo "  ⚠ Si 'arm throttle' affiche 'FAILED', attends 10s"
    echo "     et réessaie. L'EKF a besoin de temps pour calibrer."
    echo ""
    echo "======================================================="
    echo ""
    echo "Appuie sur Entrée pour lancer ArduPilot SITL..."
    read

    # Lancer sim_vehicle.py SANS --console et --map (évite les crashs matplotlib)
    # Le prompt MAVProxy apparaît directement dans ce terminal
    sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON

    # Quand l'utilisateur quitte, tuer Gazebo
    kill $GZ_PID 2>/dev/null || true
fi
