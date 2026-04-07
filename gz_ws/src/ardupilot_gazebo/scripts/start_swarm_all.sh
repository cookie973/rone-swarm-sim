#!/bin/bash
# ==============================================================================
# start_swarm_all.sh — 一键拉起 6-UAV 蜂群仿真基础设施
# 启动内容:
#   1. Gazebo Harmonic
#   2. 6 个 ArduPilot SITL
#   3. 6 个 uav_bridge
#   4. 可选 swarm_formation 控制节点
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTO_CONTROLLER=true
GAZEBO_WAIT=${GAZEBO_WAIT:-8}
SITL_WAIT=${SITL_WAIT:-25}
BRIDGE_WAIT=${BRIDGE_WAIT:-5}

for arg in "$@"; do
    if [ "$arg" = "--no-controller" ]; then
        AUTO_CONTROLLER=false
    fi
done

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "[ERROR] 未找到 gnome-terminal，无法自动开多标签页。"
    exit 1
fi

if [ -z "$DISPLAY" ]; then
    echo "[ERROR] 当前没有图形桌面 DISPLAY，无法自动启动 Gazebo/终端标签页。"
    exit 1
fi

source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

echo "========================================="
echo " 一键启动 6-UAV 蜂群仿真"
echo " Gazebo 等待: ${GAZEBO_WAIT}s"
echo " SITL 等待:   ${SITL_WAIT}s"
echo " Bridge 等待: ${BRIDGE_WAIT}s"
echo " 控制节点:    ${AUTO_CONTROLLER}"
echo "========================================="

gnome-terminal --tab --title="Gazebo-6UAV" -- bash -c "cd '$SCRIPT_DIR' && ./start_gazebo.sh; exec bash" &
sleep "$GAZEBO_WAIT"

"$SCRIPT_DIR/start_sitl_all.sh"
sleep "$SITL_WAIT"

"$SCRIPT_DIR/start_bridge_all.sh"
sleep "$BRIDGE_WAIT"

if [ "$AUTO_CONTROLLER" = true ]; then
    "$SCRIPT_DIR/start_swarm_controller.sh"
fi

echo ""
echo "基础设施已启动。推荐下一步："
echo "  1. cd $SCRIPT_DIR && ./takeoff_all.sh"
echo "  2. cd $SCRIPT_DIR && ./formation_switch.sh leader_follower grid"
echo "  3. 实验结束后执行: cd $SCRIPT_DIR && ./land_all.sh"
