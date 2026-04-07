#!/bin/bash
# ==============================================================================
# takeoff_all_v2.sh — 6 架无人机 2.0 起飞脚本
#
# 策略：
#   1. 全部无人机并发进入 GUIDED
#   2. 全部无人机在低油门 RC override 下并发解锁
#   3. 等全部 armed=true 后，统一切高油门 RC override
#   4. 并发持续发送 TAKEOFF，直到全部达到目标高度附近
#
# 说明：
#   这个版本保留已验证成功的两阶段 RC override 逻辑，但把“逐架串行”改成
#   “并发解锁 + 同步起飞”，用于在不覆盖稳定版 takeoff_all.sh 的前提下压缩起飞时间。
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ALTITUDE="${1:-5.0}"
NUM_UAV="${NUM_UAV:-6}"
SUCCESS_ALTITUDE="${SUCCESS_ALTITUDE:-0.7}"
ARM_TIMEOUT="${ARM_TIMEOUT:-20}"
TAKEOFF_TIMEOUT="${TAKEOFF_TIMEOUT:-25}"
TAKEOFF_STOP_CONTROLLER="${TAKEOFF_STOP_CONTROLLER:-1}"
RC_OVERRIDE_RATE="${RC_OVERRIDE_RATE:-5}"
RC_OVERRIDE_THROTTLE="${RC_OVERRIDE_THROTTLE:-1600}"
RC_OVERRIDE_IDLE_THROTTLE="${RC_OVERRIDE_IDLE_THROTTLE:-1000}"
TAKEOFF_STREAM_DURATION="${TAKEOFF_STREAM_DURATION:-10}"
TAKEOFF_STREAM_RATE="${TAKEOFF_STREAM_RATE:-3}"

LOW_RC_PIDS=()
HIGH_RC_PIDS=()
TAKEOFF_PIDS=()

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

if [ "$TAKEOFF_STOP_CONTROLLER" = "1" ]; then
    if ros2 node list 2>/dev/null | grep -qx '/swarm_formation'; then
        echo "[takeoff_all_v2] 检测到 /swarm_formation，先暂停编队控制器以避免起飞冲突"
        pkill -f 'ros2 run uav_bridge swarm_formation|install/uav_bridge/lib/uav_bridge/swarm_formation' 2>/dev/null || true
        sleep 1
    fi
fi

cleanup_pid_array() {
    local -n pid_array_ref="$1"
    local pid
    for pid in "${pid_array_ref[@]:-}"; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        fi
    done
}

release_rc_override() {
    local uav_id="$1"
    ros2 topic pub --once "/uav${uav_id}/uav/cmd/rc_override" std_msgs/msg/UInt16MultiArray \
        '{data: [65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535]}' \
        >/dev/null 2>&1 || true
}

release_all_rc_override() {
    local i
    for ((i=1; i<=NUM_UAV; i++)); do
        release_rc_override "$i"
    done
}

cleanup() {
    cleanup_pid_array LOW_RC_PIDS
    cleanup_pid_array HIGH_RC_PIDS
    cleanup_pid_array TAKEOFF_PIDS
    release_all_rc_override
    pkill -f '/uav[0-9]+/uav/cmd/rc_override' 2>/dev/null || true
}

trap cleanup EXIT INT TERM

get_altitude() {
    local uav_id="$1"
    timeout 3s ros2 topic echo --once "/uav${uav_id}/uav/odom" 2>/dev/null | awk '/z:/ {print $2; exit}'
}

get_armed_state() {
    local uav_id="$1"
    timeout 3s ros2 topic echo --once "/uav${uav_id}/uav/armed" 2>/dev/null | awk '/data:/ {print $2; exit}'
}

start_low_throttle_override() {
    local uav_id="$1"
    ros2 topic pub -r "$RC_OVERRIDE_RATE" "/uav${uav_id}/uav/cmd/rc_override" std_msgs/msg/UInt16MultiArray \
        "{data: [1500, 1500, ${RC_OVERRIDE_IDLE_THROTTLE}, 1500, 65535, 65535, 65535, 65535]}" \
        >/dev/null 2>&1 &
    LOW_RC_PIDS+=("$!")
}

start_high_throttle_override() {
    local uav_id="$1"
    ros2 topic pub -r "$RC_OVERRIDE_RATE" "/uav${uav_id}/uav/cmd/rc_override" std_msgs/msg/UInt16MultiArray \
        "{data: [1500, 1500, ${RC_OVERRIDE_THROTTLE}, 1500, 65535, 65535, 65535, 65535]}" \
        >/dev/null 2>&1 &
    HIGH_RC_PIDS+=("$!")
}

start_takeoff_stream() {
    local uav_id="$1"
    timeout "$TAKEOFF_STREAM_DURATION" ros2 topic pub -r "$TAKEOFF_STREAM_RATE" "/uav${uav_id}/uav/cmd/takeoff" std_msgs/msg/Float32 \
        "{data: ${ALTITUDE}}" >/dev/null 2>&1 &
    TAKEOFF_PIDS+=("$!")
}

echo "[takeoff_all_v2] 并发设置 GUIDED 模式"
for ((i=1; i<=NUM_UAV; i++)); do
    ros2 topic pub --once "/uav${i}/uav/cmd/mode" std_msgs/msg/String '{data: GUIDED}' >/dev/null 2>&1 &
done
wait || true

echo "[takeoff_all_v2] 启动低油门 RC override，并发解锁 ${NUM_UAV} 架无人机"
for ((i=1; i<=NUM_UAV; i++)); do
    start_low_throttle_override "$i"
done

arm_start_ts="$(date +%s)"
while true; do
    armed_count=0
    for ((i=1; i<=NUM_UAV; i++)); do
        armed_state="$(get_armed_state "$i")"
        if [ "$armed_state" = "true" ]; then
            armed_count=$((armed_count + 1))
        else
            ros2 topic pub --once "/uav${i}/uav/cmd/arm" std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 &
        fi
    done
    wait || true

    echo "[takeoff_all_v2] armed ${armed_count}/${NUM_UAV}"
    if [ "$armed_count" -eq "$NUM_UAV" ]; then
        break
    fi

    if [ $(( $(date +%s) - arm_start_ts )) -ge "$ARM_TIMEOUT" ]; then
        echo "[takeoff_all_v2] arm timeout: 仅 ${armed_count}/${NUM_UAV} 架完成解锁" >&2
        exit 1
    fi

    sleep 1
done

echo "[takeoff_all_v2] 全部解锁完成，切换到高油门 RC override"
cleanup_pid_array LOW_RC_PIDS
LOW_RC_PIDS=()

for ((i=1; i<=NUM_UAV; i++)); do
    start_high_throttle_override "$i"
done

sleep 1

echo "[takeoff_all_v2] 并发发送 TAKEOFF"
for ((i=1; i<=NUM_UAV; i++)); do
    start_takeoff_stream "$i"
done

takeoff_start_ts="$(date +%s)"
declare -A reached
while true; do
    reached_count=0
    for ((i=1; i<=NUM_UAV; i++)); do
        altitude="$(get_altitude "$i")"
        if [ -n "$altitude" ] && awk -v alt="$altitude" -v success="$SUCCESS_ALTITUDE" 'BEGIN { exit !(alt >= success) }'; then
            reached[$i]=1
        fi

        if [ "${reached[$i]:-0}" = "1" ]; then
            reached_count=$((reached_count + 1))
        fi
    done

    echo "[takeoff_all_v2] airborne ${reached_count}/${NUM_UAV}"
    if [ "$reached_count" -eq "$NUM_UAV" ]; then
        break
    fi

    if [ $(( $(date +%s) - takeoff_start_ts )) -ge "$TAKEOFF_TIMEOUT" ]; then
        echo "[takeoff_all_v2] takeoff timeout，未全部达到成功高度" >&2
        for ((i=1; i<=NUM_UAV; i++)); do
            altitude="$(get_altitude "$i")"
            echo "[takeoff_all_v2] uav${i}: alt=${altitude:-unknown}m" >&2
        done
        exit 1
    fi

    sleep 1
done

cleanup_pid_array TAKEOFF_PIDS
TAKEOFF_PIDS=()
cleanup_pid_array HIGH_RC_PIDS
HIGH_RC_PIDS=()
release_all_rc_override

echo "[takeoff_all_v2] ${NUM_UAV}/${NUM_UAV} 架无人机已完成并发起飞"
for ((i=1; i<=NUM_UAV; i++)); do
    altitude="$(get_altitude "$i")"
    echo "[takeoff_all_v2] uav${i}: alt=${altitude:-unknown}m"
done

exit 0