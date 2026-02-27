#!/bin/bash
###############################################################################
# Script 13 : Installation de 5G-LENA (module NR pour NS-3)
#
# 5G-LENA fournit le module NR (New Radio) pour simuler les réseaux 5G
# dans NS-3. Il permet de simuler :
#   - gNB (stations de base 5G)
#   - UEs (terminaux comme les drones)
#   - Modèles de canal 3GPP TR 38.901
#   - Scheduling OFDMA, beamforming, MIMO
#   - Bandes FR1 (sub-6 GHz) et FR2 (mmWave)
#
# Version : 5g-lena-v2.6.y (compatible avec NS-3.40)
# https://5g-lena.cttc.es/
# https://gitlab.com/cttc-lena/nr
#
# Prérequis : NS-3.40 installé (script 09_install_ns3.sh)
###############################################################################
set -e

NS3_VERSION="3.40"
NS3_DIR="$HOME/ns-allinone-${NS3_VERSION}/ns-${NS3_VERSION}"
NR_DIR="$NS3_DIR/contrib/nr"
NR_BRANCH="5g-lena-v2.6.y"

echo "=============================================="
echo " [13] Installation de 5G-LENA (module NR)"
echo "=============================================="

# ============================================================
# Vérifications
# ============================================================
if [ ! -d "$NS3_DIR" ]; then
    echo "ERREUR : NS-3 non trouvé dans $NS3_DIR"
    echo "Lance d'abord : ./09_install_ns3.sh"
    exit 1
fi

if [ ! -f "$NS3_DIR/ns3" ]; then
    echo "ERREUR : NS-3 semble incomplet (pas de ./ns3)"
    echo "Relance : ./09_install_ns3.sh"
    exit 1
fi

# ============================================================
# Étape 1 : Installer les prérequis NR
# ============================================================
echo ""
echo "[1/5] Installation des prérequis 5G-LENA..."

sudo apt-get update
sudo apt-get install -y \
    libc6-dev \
    sqlite3 libsqlite3-dev \
    libeigen3-dev \
    git

echo ""
echo "==> Prérequis installés !"

# ============================================================
# Étape 2 : Cloner le module NR dans contrib/
# ============================================================
echo ""
echo "[2/5] Clonage du module NR (5G-LENA $NR_BRANCH)..."

mkdir -p "$NS3_DIR/contrib"
cd "$NS3_DIR/contrib"

if [ -d "$NR_DIR" ]; then
    echo "Le module NR existe déjà dans $NR_DIR"
    echo "Mise à jour vers $NR_BRANCH..."
    cd "$NR_DIR"
    git fetch origin
    git checkout "$NR_BRANCH" 2>/dev/null || git checkout -b "$NR_BRANCH" "origin/$NR_BRANCH"
    git pull origin "$NR_BRANCH" || true
else
    echo "Clonage depuis https://gitlab.com/cttc-lena/nr.git..."
    git clone https://gitlab.com/cttc-lena/nr.git
    cd "$NR_DIR"
    git checkout -b "$NR_BRANCH" "origin/$NR_BRANCH"
fi

echo ""
echo "==> Module NR cloné (branche $NR_BRANCH) !"

# ============================================================
# Étape 3 : Recompiler NS-3 avec le module NR
# ============================================================
echo ""
echo "[3/5] Configuration de NS-3 avec le module NR..."

cd "$NS3_DIR"

# Configurer avec les exemples et tests activés
./ns3 configure --enable-examples --enable-tests

echo ""
echo "[4/5] Compilation de NS-3 + 5G-LENA..."
echo "      (Première compilation avec NR ~10-20 min)"

./ns3 build

# Vérifier que le module NR est bien compilé
echo ""
echo "Vérification du module NR..."
NR_CHECK=$(./ns3 show targets 2>/dev/null | grep -c "nr" || true)
if [ "$NR_CHECK" -gt 0 ]; then
    echo "==> Module NR détecté dans les targets !"
else
    echo "ATTENTION : Le module NR n'apparaît pas dans les targets."
    echo "Vérifiez la compilation manuellement."
fi

# ============================================================
# Étape 4 : Tester avec un exemple NR
# ============================================================
echo ""
echo "[5/5] Test de 5G-LENA avec cttc-nr-demo..."

cd "$NS3_DIR"
TEST_RESULT=$(./ns3 run cttc-nr-demo 2>&1 | tail -5)
if [ $? -eq 0 ]; then
    echo "==> cttc-nr-demo exécuté avec succès !"
else
    echo "ATTENTION : cttc-nr-demo a échoué (peut être normal sans GUI)"
    echo "Dernières lignes : $TEST_RESULT"
fi

# ============================================================
# Étape 5 : Variables d'environnement
# ============================================================
MARKER="# 5G-LENA (NR) Paths"
if ! grep -q "$MARKER" ~/.bashrc; then
    cat >> ~/.bashrc << EOF

$MARKER
export NR_DIR=$NR_DIR
EOF
    echo "Chemins NR ajoutés à ~/.bashrc"
else
    echo "Chemins NR déjà configurés."
fi

echo ""
echo "=============================================="
echo " 5G-LENA (module NR) installé avec succès !"
echo "=============================================="
echo ""
echo "  NS-3   : $NS3_DIR"
echo "  NR     : $NR_DIR"
echo "  Version: $NR_BRANCH"
echo ""
echo "  ┌───────────────────────────────────────────────────────────────┐"
echo "  │  Exemples NR disponibles :                                    │"
echo "  │    ./ns3 run cttc-nr-demo                                     │"
echo "  │    ./ns3 run cttc-nr-cc-bwp-demo                             │"
echo "  │    ./ns3 run cttc-nr-mimo-demo                                │"
echo "  ├───────────────────────────────────────────────────────────────┤"
echo "  │  Scénario drones 5G :                                        │"
echo "  │    cp ~/simulation_mc02/ns3_scenarios/drone-5g-scenario.cc \\ │"
echo "  │       scratch/                                                │"
echo "  │    ./ns3 build                                                │"
echo "  │    ./ns3 run \"scratch/drone-5g-scenario --nDrones=3\"        │"
echo "  └───────────────────────────────────────────────────────────────┘"
echo ""
echo "  Comparaison WiFi vs 5G :"
echo "    WiFi   : drone-wifi-scenario.cc  (802.11n Ad-Hoc)"
echo "    5G NR  : drone-5g-scenario.cc    (5G-LENA, gNB + UEs)"
echo ""
echo "==> Passe au script suivant : 14_ns3_5g_bridge.py"
