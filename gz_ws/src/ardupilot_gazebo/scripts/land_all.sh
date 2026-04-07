#!/bin/bash
# ==============================================================================
# land_all.sh — 6 架无人机批量降落
#
# 策略：
#   1. 默认先暂停 swarm_formation，避免编队控制继续发速度指令
#   2. 对未完成的无人机并发补发 LAND 模式和 land 命令
#   3. 以 armed=false 或高度接近地面作为完成判据
# ==============================================================================
set -euo pipefail

NUM_UAV="${NUM_UAV:-6}"
LAND_TIMEOUT="${LAND_TIMEOUT:-60}"
LAND_PERIOD="${LAND_PERIOD:-2}"
GROUND_ALTITUDE="${GROUND_ALTITUDE:-0.2}"
LAND_STOP_CONTROLLER="${LAND_STOP_CONTROLLER:-1}"

if [ -z "${ROS_DISTRO:-}" ]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
fi

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    set +u
    source "$HOME/ros2_ws/install/setup.bash"
    set -u
fi

get_altitude() {
    local uav_id="$1"
    timeout 3s ros2 topic echo --once "/uav${uav_id}/uav/odom" 2>/dev/null | awk '/z:/ {print $2; exit}'
}

get_armed_state() {
    local uav_id="$1"
    timeout 3s ros2 topic echo --once "/uav${uav_id}/uav/armed" 2>/dev/null | awk '/data:/ {print $2; exit}'
}

is_grounded() {
    local altitude="$1"
    awk -v alt="$altitude" -v threshold="$GROUND_ALTITUDE" 'BEGIN { exit !(alt <= threshold) }'
}

if [ "$LAND_STOP_CONTROLLER" = "1" ]; then
    if ros2 node list 2>/dev/null | grep -qx '/swarm_formation'; then
        echo "[land_all] 检测到 /swarm_formation，先暂停编队控制器以避免降落冲突"
        pkill -f 'ros2 run uav_bridge swarm_formation|install/uav_bridge/lib/uav_bridge/swarm_formation' 2>/dev/null || true
        sleep 1
    fi
fi

echo "========================================="
echo " 批量降落 ${NUM_UAV} 架无人机"
echo "========================================="

declare -A landed
start_ts="$(date +%s)"

while true; do
    landed_count=0

    for ((i=1; i<=NUM_UAV; i++)); do
        armed_state="$(get_armed_state "$i")"
        altitude="$(get_altitude "$i")"

        if [ "$armed_state" = "false" ]; then
            landed[$i]=1
        elif [ -n "$altitude" ] && is_grounded "$altitude"; then
            landed[$i]=1
        fi

        if [ "${landed[$i]:-0}" = "1" ]; then
            landed_count=$((landed_count + 1))
            echo "[land_all] uav${i}: landed (armed=${armed_state:-unknown}, alt=${altitude:-unknown}m)"
            continue
        fi

        echo "[land_all] uav${i}: 发送 LAND (armed=${armed_state:-unknown}, alt=${altitude:-unknown}m)"
        ros2 topic pub --once "/uav${i}/uav/cmd/mode" std_msgs/msg/String '{data: LAND}' >/dev/null 2>&1 &
        ros2 topic pub --once "/uav${i}/uav/cmd/land" std_msgs/msg/Empty '{}' >/dev/null 2>&1 &
    done
    wait || true

    echo "[land_all] progress ${landed_count}/${NUM_UAV}"
    if [ "$landed_count" -eq "$NUM_UAV" ]; then
        echo "[land_all] ${NUM_UAV}/${NUM_UAV} 架无人机已完成降落"
        exit 0
    fi

    if [ $(( $(date +%s) - start_ts )) -ge "$LAND_TIMEOUT" ]; then
        echo "[land_all] timeout: 仍有无人机未完成降落" >&2
        for ((i=1; i<=NUM_UAV; i++)); do
            armed_state="$(get_armed_state "$i")"
            altitude="$(get_altitude "$i")"
            echo "[land_all] uav${i}: armed=${armed_state:-unknown} alt=${altitude:-unknown}m" >&2
        done
        exit 1
    fi

    sleep "$LAND_PERIOD"
done
