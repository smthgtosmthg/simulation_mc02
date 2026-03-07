#!/bin/bash
###############################################################################
# Script 2 : Installation de Gazebo Harmonic (GZ Harmonic / gz-harmonic)
# Gazebo Harmonic est la dernière version LTS de Gazebo (sortie 2023).
# Compatible Ubuntu 22.04.
###############################################################################
set -e

echo "=============================================="
echo " [2/4] Installation de Gazebo Harmonic"
echo "=============================================="

# --- Ajouter le dépôt OSRF Gazebo ---
sudo apt-get install -y lsb-release gnupg

# Clé GPG
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

# Dépôt
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update

# --- Installer Gazebo Harmonic ---
sudo apt-get install -y gz-harmonic

echo ""
echo "==> Vérification de l'installation de Gazebo :"
gz sim --version 2>/dev/null && echo "Gazebo Harmonic installé avec succès !" || echo "ATTENTION : gz sim non trouvé, vérifier l'installation."

echo ""
echo "==> Pour tester Gazebo avec le rendering (GPU activé) :"
echo "    gz sim -v 4 shapes.sdf"
echo ""
echo "==> Pour tester Gazebo SANS rendering (headless) :"
echo "    gz sim -v 4 -s shapes.sdf"
echo ""
echo "==> Passe au script suivant : 03_install_px4_sitl.sh"
