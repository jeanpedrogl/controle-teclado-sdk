#!/usr/bin/env bash
# Cria o venv Python deste projeto. O ROS2 Humble e o rosbridge-suite precisam estar instalados
# (apt: ros-humble-desktop e ros-humble-rosbridge-suite); este script não instala nada com sudo.
#
# O venv usa --system-site-packages porque o rclpy vem do apt, não do pip.
set -euo pipefail
cd "$(dirname "$0")"

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ROS2 Humble não encontrado em /opt/ros/humble. Instale-o (veja o setup_ambiente.sh de visao_aplicativo_celular)." >&2
    exit 1
fi
if ! dpkg -s ros-humble-rosbridge-suite &>/dev/null; then
    echo "Falta o rosbridge: sudo apt install ros-humble-rosbridge-suite" >&2
    exit 1
fi

python3 -m venv --system-site-packages .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -r requirements.txt

cat <<'MSG'

Pronto. Para compilar e rodar, a partir desta pasta:

    source /opt/ros/humble/setup.bash
    source .venv/bin/activate
    python3 -m colcon build --symlink-install
    source install/setup.bash
    ros2 launch ponte_teclado_drone sistema.launch.py

(use "python3 -m colcon", não só "colcon": o executável do apt ignora o venv ativado.)
MSG
