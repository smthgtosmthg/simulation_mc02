#!/bin/bash
###############################################################################
# Script 4 : Test d'un seul drone — PX4 SITL + Gazebo Garden
#
# Ce script lance un drone x500 dans Gazebo, le connecte à PX4 SITL,
# puis exécute un vol automatique simple (arm, takeoff, hover, land)
# via MAVLink pour vérifier que tout fonctionne.
###############################################################################
set -e

PX4_DIR="$HOME/PX4-Autopilot"

echo "=============================================="
echo " [4/4] Test : un drone dans Gazebo"
echo "=============================================="

# --- Vérifier que PX4 est installé ---
if [ ! -d "$PX4_DIR" ]; then
    echo "ERREUR : PX4-Autopilot non trouvé dans $PX4_DIR"
    echo "Lance d'abord le script 03_install_px4_sitl.sh"
    exit 1
fi

# --- Mode de lancement ---
echo ""
echo "Options de lancement :"
echo "  1) Avec rendering (fenêtre Gazebo visible) — nécessite GPU"
echo "  2) Sans rendering (headless) — moins gourmand"
echo ""
read -p "Choix [1/2, défaut=1] : " CHOICE
CHOICE=${CHOICE:-1}

cd "$PX4_DIR"

if [ "$CHOICE" = "2" ]; then
    echo ""
    echo "Lancement en mode headless..."
    export HEADLESS=1
fi

echo ""
echo "==> Lancement de PX4 SITL + Gazebo avec le drone x500..."
echo "==> Une fois le drone prêt, tu verras 'pxh>' dans le terminal."
echo ""
echo "-----------------------------------------------"
echo " Commandes utiles dans le shell PX4 (pxh>) :~/PX4-Autopilot$ make px4_sitl gz_x500"
echo "-----------------------------------------------"
echo "  commander takeoff        → décollage"
echo "  commander land            → atterrissage"
echo "  commander arm             → armer les moteurs"
echo "  commander disarm          → désarmer les moteurs"
echo "  listener sensor_combined  → voir les données capteurs"
echo "  listener vehicle_local_position → position du drone"
echo "-----------------------------------------------"
echo ""
echo "Pour connecter QGroundControl ou MAVProxy en externe :"
echo "  UDP : localhost:14540 (MAVLink)"
echo "  TCP : localhost:4560 (simulateur)"
echo ""
echo "Appuie sur Entrée pour lancer la simulation..."
read

# Lancer PX4 SITL avec Gazebo Garden et le modèle x500
make px4_sitl gz_x500
