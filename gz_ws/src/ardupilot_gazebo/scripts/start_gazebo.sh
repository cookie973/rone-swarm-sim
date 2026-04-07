#!/bin/bash
# ==============================================================================
# start_gazebo.sh — 启动 Gazebo Harmonic 载入 6-UAV 蜂群世界
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

# 设置 Gazebo 资源路径（模型 + 世界）
export GZ_SIM_RESOURCE_PATH="${REPO_DIR}/models:${REPO_DIR}/worlds:${GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${REPO_DIR}/../../build/ardupilot_gazebo:${GZ_SIM_SYSTEM_PLUGIN_PATH}"

echo "========================================="
echo " 启动 6-UAV 蜂群 Gazebo 仿真"
echo " World: gazebo_6uav.sdf"
echo " 渲染引擎: ogre"
echo " 模型路径: ${REPO_DIR}/models"
echo "========================================="

echo "清理旧的 Gazebo 进程..."
pkill -f "gz sim -v4 -r --render-engine ogre gazebo_6uav.sdf" 2>/dev/null || true
pkill -f "gz sim server" 2>/dev/null || true
pkill -f "gz sim gui" 2>/dev/null || true
sleep 2

gz sim -v4 -r --render-engine ogre gazebo_6uav.sdf
