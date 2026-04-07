#!/bin/bash
# ==============================================================================
# start_swarm_controller.sh — 单独启动 swarm_formation 控制节点
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "[ERROR] 未找到 gnome-terminal，无法自动打开控制器终端。"
    exit 1
fi

if [ -z "$DISPLAY" ]; then
    echo "[ERROR] 当前没有图形桌面 DISPLAY，无法自动启动控制器终端。"
    exit 1
fi

source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

if ros2 node list 2>/dev/null | grep -qx '/swarm_formation'; then
    echo "swarm_formation 已在运行，无需重复启动。"
    exit 0
fi

gnome-terminal --tab --title="swarm-formation" -- bash -c "source /opt/ros/humble/setup.bash && source '$HOME/ros2_ws/install/setup.bash' && ros2 run uav_bridge swarm_formation; exec bash" &

echo "已启动 swarm_formation 控制器终端。"
echo "若要切换队形，可执行: cd $SCRIPT_DIR && ./formation_switch.sh grid"