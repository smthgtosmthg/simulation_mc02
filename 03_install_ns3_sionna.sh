#!/bin/bash
###############################################################################
# 03_install_ns3_sionna.sh
# Installation de NS-3.40 + module ns3sionna (TU Berlin)
# Ray tracing réaliste avec NVIDIA Sionna RT
# Permet d'obtenir RSSI et latence entre drones
###############################################################################
set -e

export WORKDIR="$HOME/drone_simulation"
mkdir -p "$WORKDIR"
cd "$WORKDIR"

echo "=============================================="
echo " OPTION A: NS-3 + ns3sionna (TU Berlin)"
echo " https://github.com/tkn-tub/ns3sionna"
echo "=============================================="

echo ""
echo "--- Étape 1: Télécharger NS-3.40 ---"

cd "$WORKDIR"
if [ ! -f "ns-allinone-3.40.tar.bz2" ]; then
    wget https://www.nsnam.org/releases/ns-allinone-3.40.tar.bz2
fi

if [ ! -d "ns-allinone-3.40" ]; then
    tar xf ns-allinone-3.40.tar.bz2
fi

echo "[OK] NS-3.40 téléchargé"

echo ""
echo "--- Étape 2: Cloner ns3sionna dans contrib ---"

cd "$WORKDIR/ns-allinone-3.40/ns-3.40/contrib"

if [ ! -d "sionna" ]; then
    git clone https://github.com/tkn-tub/ns3sionna.git ./sionna
    echo "[OK] ns3sionna cloné"
else
    echo "[INFO] ns3sionna déjà cloné, mise à jour..."
    cd sionna && git pull && cd ..
fi

echo ""
echo "--- Étape 3: Compiler NS-3 avec ns3sionna ---"

cd "$WORKDIR/ns-allinone-3.40/ns-3.40"

./ns3 configure --enable-examples
./ns3 build -j$(nproc)

echo "[OK] NS-3 + ns3sionna compilés"

echo ""
echo "--- Étape 4: Installer le serveur Python Sionna ---"

cd "$WORKDIR/ns-allinone-3.40/ns-3.40/contrib/sionna/model/ns3sionna/"

# Créer un environnement virtuel Python
if [ ! -d "sionna-venv" ]; then
    python3 -m venv sionna-venv
fi

source sionna-venv/bin/activate

# Installer les dépendances Python
pip install --upgrade pip
pip install -r requirements.txt

# Vérifier l'installation
echo "[INFO] Test des imports Python..."
python3 tests/test_imports.py

echo "[OK] Serveur Python ns3sionna installé"

echo ""
echo "--- Étape 5: Installer TensorFlow GPU (pour Sionna RT) ---"

# Si GPU NVIDIA disponible
if nvidia-smi > /dev/null 2>&1; then
    echo "[INFO] GPU détectée - Installation de TensorFlow GPU..."
    pip install 'tensorflow[and-cuda]'
    echo "[INFO] Vérification GPU TensorFlow..."
    python3 -c "import tensorflow as tf; print('GPUs:', tf.config.list_physical_devices('GPU'))"
else
    echo "[INFO] Pas de GPU - Installation de TensorFlow CPU..."
    pip install tensorflow
    sudo apt-get install -y llvm
    python3 -c "import tensorflow as tf; print(tf.reduce_sum(tf.random.normal([1000, 1000])))"
fi

# Installer Sionna RT
pip install sionna-rt

echo "[OK] Sionna RT installé"

deactivate

echo ""
echo "=============================================="
echo " [DONE] NS-3 + ns3sionna installés!"
echo "=============================================="
echo ""
echo "Pour lancer une simulation:"
echo "  Terminal 1 (Serveur Sionna):"
echo "    cd $WORKDIR/ns-allinone-3.40/ns-3.40/contrib/sionna/model/ns3sionna"
echo "    source sionna-venv/bin/activate"
echo "    ./run.sh"
echo ""
echo "  Terminal 2 (NS-3):"
echo "    cd $WORKDIR/ns-allinone-3.40/ns-3.40"
echo "    ./ns3 run example-sionna-sensing-mobile"
echo ""
echo "Prochaine étape: ./04_install_ns3rt.sh (ALTERNATIVE)"
echo "  OU directement: ./05_launch_multi_drone.sh"
