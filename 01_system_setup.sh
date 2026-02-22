#!/bin/bash
###############################################################################
# 01_system_setup.sh
# Installation des dépendances système, drivers NVIDIA, CUDA
# Ubuntu 22.04 LTS recommandé
###############################################################################
set -e

echo "=============================================="
echo " ÉTAPE 1: Mise à jour système + dépendances"
echo "=============================================="

sudo apt-get update && sudo apt-get upgrade -y

# Dépendances de base
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  wget \
  curl \
  unzip \
  pkg-config \
  software-properties-common \
  python3 \
  python3-pip \
  python3-venv \
  python3-dev \
  python3-setuptools \
  python3-wheel \
  gcc \
  g++ \
  ninja-build \
  lsb-release \
  gnupg2 \
  net-tools \
  htop \
  tmux \
  vim

echo "=============================================="
echo " ÉTAPE 2: Drivers NVIDIA + CUDA"
echo "=============================================="

# Vérifier si GPU NVIDIA détecté
if lspci | grep -i nvidia > /dev/null 2>&1; then
    echo "[OK] GPU NVIDIA détectée"
    
    # Installer les drivers NVIDIA (si pas déjà installés)
    if ! nvidia-smi > /dev/null 2>&1; then
        echo "[INFO] Installation des drivers NVIDIA..."
        sudo apt-get install -y nvidia-driver-535
        echo "[WARN] Un REBOOT sera nécessaire après l'installation des drivers."
        echo "       Exécute: sudo reboot"
        echo "       Puis relance ce script pour vérifier."
    else
        echo "[OK] Drivers NVIDIA déjà installés:"
        nvidia-smi
    fi
    
    # CUDA Toolkit (nécessaire pour Sionna RT / TensorFlow GPU)
    if ! nvcc --version > /dev/null 2>&1; then
        echo "[INFO] Installation de CUDA Toolkit 12.2..."
        wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
        sudo dpkg -i cuda-keyring_1.1-1_all.deb
        sudo apt-get update
        sudo apt-get install -y cuda-toolkit-12-2
        rm -f cuda-keyring_1.1-1_all.deb
        
        # Ajouter CUDA au PATH
        echo '' >> ~/.bashrc
        echo '# CUDA' >> ~/.bashrc
        echo 'export PATH=/usr/local/cuda-12.2/bin:$PATH' >> ~/.bashrc
        echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.2/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
        source ~/.bashrc
        echo "[OK] CUDA installé. Fais 'source ~/.bashrc' ou ouvre un nouveau terminal."
    else
        echo "[OK] CUDA déjà installé:"
        nvcc --version
    fi
else
    echo "[WARN] Aucune GPU NVIDIA détectée. La simulation fonctionnera en CPU only."
    echo "       Gazebo rendering sera plus lent."
fi

echo "=============================================="
echo " ÉTAPE 3: Dépendances additionnelles"
echo "=============================================="

# Dépendances pour NS-3
sudo apt-get install -y \
  libzmq5 \
  libzmq3-dev \
  libprotobuf-dev \
  protobuf-compiler \
  libsqlite3-dev \
  libxml2-dev \
  libgtk-3-dev \
  gdb \
  valgrind \
  doxygen \
  graphviz \
  imagemagick \
  texlive \
  texlive-extra-utils \
  texlive-latex-extra \
  texlive-font-utils \
  dvipng \
  latexmk \
  gsl-bin \
  libgsl-dev \
  tcpdump \
  sqlite3 \
  qtbase5-dev \
  qtchooser \
  qt5-qmake \
  qtbase5-dev-tools

# Dépendances pour Gazebo
sudo apt-get install -y \
  libgl1-mesa-dev \
  libglu1-mesa-dev \
  xorg-dev \
  libxcb-xinerama0

# LLVM (nécessaire pour Sionna RT / Mitsuba)
sudo apt-get install -y llvm

echo ""
echo "=============================================="
echo " [DONE] Dépendances système installées!"
echo "=============================================="
echo ""
echo "Si les drivers NVIDIA viennent d'être installés:"
echo "  → sudo reboot"
echo "  → Après reboot: nvidia-smi pour vérifier"
echo ""
echo "Prochaine étape: ./02_install_gazebo_px4.sh"
