#!/bin/bash
###############################################################################
# Script 3 : Installation de PX4-Autopilot (SITL = Software In The Loop)
# PX4 est le flight controller open-source pour drones.
# SITL permet de simuler le firmware PX4 sans matériel réel.
###############################################################################
set -e

echo "=============================================="
echo " [3/4] Installation de PX4-Autopilot SITL"
echo "=============================================="

# Répertoire d'installation
PX4_DIR="$HOME/PX4-Autopilot"

if [ -d "$PX4_DIR" ]; then
    echo "PX4-Autopilot existe déjà dans $PX4_DIR"
    echo "Pour réinstaller, supprime le dossier : rm -rf $PX4_DIR"
    echo "Mise à jour du repo existant..."
    cd "$PX4_DIR"
    git pull
    git submodule update --init --recursive
else
    echo "Clonage de PX4-Autopilot (branche main — compatible Gazebo Harmonic)..."
    cd "$HOME"
    git clone --recursive https://github.com/PX4/PX4-Autopilot.git
    cd "$PX4_DIR"
fi

# --- Lancer le script d'installation officiel de PX4 ---
# Ce script installe toutes les dépendances manquantes automatiquement
echo ""
echo "Exécution du script de setup PX4 (ubuntu.sh)..."
bash ./Tools/setup/ubuntu.sh --no-nuttx

# --- Compiler PX4 pour SITL avec Gazebo Garden ---
echo ""
echo "Compilation de PX4 SITL pour Gazebo (première compilation, peut prendre ~10min)..."
cd "$PX4_DIR"
make px4_sitl gz_x500

echo ""
echo "=============================================="
echo " PX4 SITL installé et compilé avec succès !"
echo "=============================================="
echo ""
echo "NOTE : La commande 'make px4_sitl gz_x500' a lancé une simulation."
echo "       Tu peux la fermer avec Ctrl+C."
echo "       Le drone x500 est un quadcopter standard."
echo ""
echo "==> Passe au script suivant : 04_test_single_drone.sh"
