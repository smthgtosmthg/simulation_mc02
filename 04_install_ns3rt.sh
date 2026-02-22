#!/bin/bash
###############################################################################
# 04_install_ns3rt.sh
# ALTERNATIVE: Installation de ns3-rt (Roberto Pegurri)
# NS-3 avec NVIDIA Sionna RT intégré via socket UDP
# https://github.com/robpegurri/ns3-rt
###############################################################################
set -e

export WORKDIR="$HOME/drone_simulation"
mkdir -p "$WORKDIR"
cd "$WORKDIR"

echo "=============================================="
echo " OPTION B: ns3-rt (Pegurri/NVIDIA)"
echo " https://github.com/robpegurri/ns3-rt"
echo "=============================================="
echo ""
echo "[NOTE] Ceci est une ALTERNATIVE à 03_install_ns3_sionna.sh"
echo "       Utilise l'un OU l'autre, pas les deux."
echo ""

echo "--- Étape 1: Cloner ns3-rt ---"

cd "$WORKDIR"
if [ ! -d "ns3-rt" ]; then
    git clone https://github.com/robpegurri/ns3-rt.git
    echo "[OK] ns3-rt cloné"
else
    echo "[INFO] ns3-rt existe déjà, mise à jour..."
    cd ns3-rt && git pull && cd ..
fi

echo ""
echo "--- Étape 2: Compiler ns3-rt ---"

cd "$WORKDIR/ns3-rt"
./ns3 configure --disable-python --enable-examples
./ns3 build -j$(nproc)

echo "[OK] ns3-rt compilé"

echo ""
echo "--- Étape 3: Installer Sionna RT (Python) ---"

cd "$WORKDIR/ns3-rt"

if [ ! -d "sionna-rt-venv" ]; then
    python3 -m venv sionna-rt-venv
fi

source sionna-rt-venv/bin/activate

pip install --upgrade pip

# TensorFlow + Sionna RT
if nvidia-smi > /dev/null 2>&1; then
    echo "[INFO] GPU détectée - Installation TensorFlow GPU..."
    pip install 'tensorflow[and-cuda]'
    python3 -c "import tensorflow as tf; print('GPUs:', tf.config.list_physical_devices('GPU'))"
else
    echo "[INFO] Pas de GPU - Installation TensorFlow CPU..."
    sudo apt-get install -y llvm
    pip install tensorflow
fi

pip install sionna-rt

echo "[OK] Sionna RT installé"

echo ""
echo "--- Étape 4: Test de l'exemple ---"

echo ""
echo "Pour tester ns3-rt:"
echo ""
echo "  Terminal 1 (Serveur Sionna RT):"
echo "    cd $WORKDIR/ns3-rt"
echo "    source sionna-rt-venv/bin/activate"
echo "    cd src/sionna"
echo "    python3 sionna_server_v1_script.py --local-machine --frequency=2.1e9 \\"
echo "      --path-to-xml-scenario=scenarios/SionnaExampleScenario/scene.xml"
echo ""
echo "  Terminal 2 (NS-3 simulation):"
echo "    cd $WORKDIR/ns3-rt"
echo "    ./ns3 run simple-sionna-example"
echo ""

deactivate

echo "=============================================="
echo " [DONE] ns3-rt installé!"
echo "=============================================="
echo ""
echo "ns3-rt communique avec Sionna via socket UDP (port 8103)."
echo "Il peut tourner sur la même machine ou sur 2 machines différentes."
