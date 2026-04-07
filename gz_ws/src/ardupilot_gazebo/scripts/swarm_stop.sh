#!/bin/bash
# ==============================================================================
# swarm_stop.sh — 优雅停止全部 6-UAV 蜂群仿真进程
#
# 会按反序关停: 编队控制 → bridge → SITL → Gazebo
# 并释放所有端口，清理残留进程
#
# 用法:
#   ./swarm_stop.sh
# ==============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
PID_FILE="${REPO_DIR}/logs/.swarm_pids"

echo "========================================="
echo " 停止 6-UAV 蜂群仿真"
echo "========================================="

# ---------- 根据 PID 文件优雅关闭 ----------
if [ -f "$PID_FILE" ]; then
    echo "[1] 根据 PID 文件关闭已知进程..."

    # 反序关闭: controller → qgc_bridge → bridge → sitl → gazebo
    for prefix in swarm_formation qgc_bridge bridge_uav sitl_uav gazebo; do
        while IFS='=' read -r key pid; do
            [[ "$key" == \#* ]] && continue
            [[ "$key" != ${prefix}* ]] && continue
            pid=$(echo "$pid" | tr -d ' \r')
            if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                echo "  关闭 $key (PID=$pid)..."
                kill -SIGTERM "$pid" 2>/dev/null || true
            fi
        done < "$PID_FILE"
    done

    sleep 3

    # 强制清理未退出的
    while IFS='=' read -r key pid; do
        [[ "$key" == \#* ]] && continue
        pid=$(echo "$pid" | tr -d ' \r')
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            echo "  强制终止 $key (PID=$pid)..."
            kill -SIGKILL "$pid" 2>/dev/null || true
        fi
    done < "$PID_FILE"

    rm -f "$PID_FILE"
fi

# ---------- 清理可能残留的进程 ----------
echo ""
echo "[2] 清理可能的残留进程..."

# swarm_formation
pkill -f "uav_bridge/swarm_formation" 2>/dev/null || true
pkill -f "ros2 run uav_bridge swarm_formation" 2>/dev/null || true

# qgc_bridge
pkill -f "uav_bridge/qgc_bridge" 2>/dev/null || true
pkill -f "ros2 run uav_bridge qgc_bridge" 2>/dev/null || true

# ROS2 bridge
pkill -f "uav_bridge/lib/uav_bridge/mavlink_tx" 2>/dev/null || true
pkill -f "ros2 launch uav_bridge camera_mavlink" 2>/dev/null || true

# SITL
pkill -f "sim_vehicle.py.*-v ArduCopter" 2>/dev/null || true
pkill -f "arducopter.*--model JSON" 2>/dev/null || true
pkill -f "mavproxy.py.*sitl_instances" 2>/dev/null || true

# Gazebo
pkill -f "gz sim" 2>/dev/null || true

sleep 2

# ---------- 验证清理结果 ----------
echo ""
echo "[3] 验证..."

remaining=0
for pattern in "gz sim" "arducopter.*JSON" "mavproxy.py.*sitl" "mavlink_tx" "qgc_bridge" "swarm_formation"; do
    count=$(pgrep -f "$pattern" 2>/dev/null | wc -l)
    if [ "$count" -gt 0 ]; then
        echo "  [WARN] 仍有 $count 个 '$pattern' 进程残留"
        remaining=$((remaining + count))
    fi
done

if [ "$remaining" -eq 0 ]; then
    echo "  所有进程已关闭"
else
    echo ""
    echo "  如果仍有残留进程，可用以下命令强制清理:"
    echo "    pkill -9 -f 'gz sim'; pkill -9 -f arducopter; pkill -9 -f mavproxy; pkill -9 -f mavlink_tx"
fi

echo ""
echo "仿真已停止。"
