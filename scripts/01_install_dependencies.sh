#!/bin/bash
###############################################################################
# Script 1 : Installation des dépendances système
# Cible    : Ubuntu 22.04 LTS
# GPU      : NVIDIA RTX 2060 SUPER (drivers déjà installés)
###############################################################################
set -e

echo "=============================================="
echo " [1/4] Installation des dépendances système"
echo "=============================================="

sudo apt-get update

# Outils de build essentiels
sudo apt-get install -y \
    git \
    wget \
    curl \
    cmake \
    build-essential \
    ninja-build \
    pkg-config \
    astyle \
    dmidecode \
    libxml2-utils

# Python 3 et bibliothèques nécessaires pour PX4
sudo apt-get install -y \
    python3-pip \
    python3-jinja2 \
    python3-jsonschema \
    python3-numpy \
    python3-empy \
    python3-toml \
    python3-packaging \
    python3-setuptools

# GStreamer (pour le streaming vidéo des drones)
sudo apt-get install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly

# Outils Python supplémentaires via pip
pip3 install --user pyserial pymavlink mavproxy pyros-genmsg

echo ""
echo "==> Dépendances système installées avec succès !"
echo "==> Passe au script suivant : 02_install_gazebo.sh"
