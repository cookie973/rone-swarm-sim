#!/bin/bash
# ==============================================================================
# takeoff_all.sh — 6 架无人机批量起飞包装脚本
#
# 说明：
#   默认使用 CLI 引擎，并支持“先串行解锁再并行起飞”的混合流程，
#   以降低 arm timeout 并保持接近同时起飞。
#   如需使用 Python 协调器，可设置 TAKEOFF_ENGINE=python。
#   默认会在起飞前暂停 swarm_formation，避免编队速度控制与起飞控制互相打架。
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ALTITUDE="${1:-5.0}"
NUM_UAV="${NUM_UAV:-6}"
SUCCESS_ALTITUDE="${SUCCESS_ALTITUDE:-0.7}"
TAKEOFF_TIMEOUT="${TAKEOFF_TIMEOUT:-45.0}"
CLIMB_SPEED="${CLIMB_SPEED:-0.6}"
CLIMB_ASSIST_DELAY="${CLIMB_ASSIST_DELAY:-1.0}"
TARGET_ALTITUDE_TOLERANCE="${TARGET_ALTITUDE_TOLERANCE:-0.5}"
TAKEOFF_STOP_CONTROLLER="${TAKEOFF_STOP_CONTROLLER:-1}"
TAKEOFF_ENGINE="${TAKEOFF_ENGINE:-cli}"
TAKEOFF_RC_OVERRIDE="${TAKEOFF_RC_OVERRIDE:-0}"
TAKEOFF_PARALLEL="${TAKEOFF_PARALLEL:-1}"
TAKEOFF_ARM_SERIAL_FIRST="${TAKEOFF_ARM_SERIAL_FIRST:-1}"
RC_OVERRIDE_RATE="${RC_OVERRIDE_RATE:-5}"
RC_OVERRIDE_THROTTLE="${RC_OVERRIDE_THROTTLE:-1600}"
RC_OVERRIDE_IDLE_THROTTLE="${RC_OVERRIDE_IDLE_THROTTLE:-1000}"
PER_UAV_TIMEOUT="${PER_UAV_TIMEOUT:-16}"
TAKEOFF_STREAM_DURATION="${TAKEOFF_STREAM_DURATION:-5}"
TAKEOFF_RETRY_ROUNDS="${TAKEOFF_RETRY_ROUNDS:-3}"
ARM_RETRY_DELAY="${ARM_RETRY_DELAY:-0.4}"
TAKEOFF_RETRY_DELAY="${TAKEOFF_RETRY_DELAY:-0.4}"
PRE_TAKEOFF_DELAY="${PRE_TAKEOFF_DELAY:-0.4}"

RC_OVERRIDE_PIDS=()

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

if [ -x "$HOME/.venv/bin/python" ]; then
    PYTHON_BIN="$HOME/.venv/bin/python"
else
    PYTHON_BIN="python3"
fi

if [ "$TAKEOFF_STOP_CONTROLLER" = "1" ]; then
    if ros2 node list 2>/dev/null | grep -qx '/swarm_formation'; then
        echo "[takeoff_all] 检测到 /swarm_formation，先暂停编队控制器以避免起飞冲突"
        pkill -f 'ros2 run uav_bridge swarm_formation|install/uav_bridge/lib/uav_bridge/swarm_formation' 2>/dev/null || true
        sleep 0.5
    fi
    if ros2 node list 2>/dev/null | grep -qx '/formation_commander'; then
        echo "[takeoff_all] 检测到 /formation_commander，先暂停队形指挥器以避免起飞冲突"
        pkill -f 'formation_commander' 2>/dev/null || true
        sleep 0.5
    fi
fi

cleanup() {
    for pid in "${RC_OVERRIDE_PIDS[@]:-}"; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    pkill -f '/uav[0-9]+/uav/cmd/rc_override' 2>/dev/null || true
}

trap cleanup EXIT INT TERM

get_altitude() {
    local uav_id="$1"
    timeout 3s ros2 topic echo --once "/uav${uav_id}/uav/odom" 2>/dev/null | awk '/z:/ {print $2; exit}'
}

is_airborne() {
    local uav_id="$1"
    local altitude
    altitude="$(get_altitude "$uav_id")"
    if [ -z "$altitude" ]; then
        return 1
    fi
    awk -v alt="$altitude" -v success="$SUCCESS_ALTITUDE" 'BEGIN { exit !(alt >= success) }'
}

release_rc_override() {
    local uav_id="$1"
    ros2 topic pub --once "/uav${uav_id}/uav/cmd/rc_override" std_msgs/msg/UInt16MultiArray \
        '{data: [65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535]}' \
        >/dev/null 2>&1 || true
}

takeoff_one_cli() {
    local uav_id="$1"
    local skip_arm="${2:-0}"
    local arming_rc_pid=""
    local rc_pid=""
    local takeoff_pid=""
    local altitude=""
    local armed_state=""
    local start_ts
    start_ts="$(date +%s)"

    echo "[takeoff_all] uav${uav_id}: GUIDED + ARM + TAKEOFF"

    if [ "$TAKEOFF_RC_OVERRIDE" = "1" ]; then
        ros2 topic pub -r "$RC_OVERRIDE_RATE" "/uav${uav_id}/uav/cmd/rc_override" std_msgs/msg/UInt16MultiArray \
            "{data: [1500, 1500, ${RC_OVERRIDE_IDLE_THROTTLE}, 1500, 65535, 65535, 65535, 65535]}" \
            >/dev/null 2>&1 &
        arming_rc_pid="$!"
        RC_OVERRIDE_PIDS+=("$arming_rc_pid")
    fi

    ros2 topic pub --once "/uav${uav_id}/uav/cmd/mode" std_msgs/msg/String '{data: GUIDED}' >/dev/null 2>&1

    if [ "$skip_arm" != "1" ]; then
        while true; do
            ros2 topic pub --once "/uav${uav_id}/uav/cmd/arm" std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1
            armed_state="$(timeout 3s ros2 topic echo --once "/uav${uav_id}/uav/armed" 2>/dev/null | awk '/data:/ {print $2; exit}')"
            if [ "$armed_state" = "true" ]; then
                break
            fi
            if [ $(( $(date +%s) - start_ts )) -ge "$PER_UAV_TIMEOUT" ]; then
                echo "[takeoff_all] uav${uav_id}: arm timeout" >&2
                if [ -n "$arming_rc_pid" ] && kill -0 "$arming_rc_pid" 2>/dev/null; then
                    kill "$arming_rc_pid" 2>/dev/null || true
                    wait "$arming_rc_pid" 2>/dev/null || true
                fi
                release_rc_override "$uav_id"
                return 1
            fi
            sleep "$ARM_RETRY_DELAY"
        done
    fi

    if [ -n "$arming_rc_pid" ] && kill -0 "$arming_rc_pid" 2>/dev/null; then
        kill "$arming_rc_pid" 2>/dev/null || true
        wait "$arming_rc_pid" 2>/dev/null || true
    fi

    if [ "$TAKEOFF_RC_OVERRIDE" = "1" ]; then
        ros2 topic pub -r "$RC_OVERRIDE_RATE" "/uav${uav_id}/uav/cmd/rc_override" std_msgs/msg/UInt16MultiArray \
            "{data: [1500, 1500, ${RC_OVERRIDE_THROTTLE}, 1500, 65535, 65535, 65535, 65535]}" \
            >/dev/null 2>&1 &
        rc_pid="$!"
        RC_OVERRIDE_PIDS+=("$rc_pid")
    fi

    sleep "$PRE_TAKEOFF_DELAY"
    timeout "$TAKEOFF_STREAM_DURATION" ros2 topic pub -r 3 "/uav${uav_id}/uav/cmd/takeoff" std_msgs/msg/Float32 "{data: ${ALTITUDE}}" >/dev/null 2>&1 &
    takeoff_pid="$!"

    while true; do
        altitude="$(get_altitude "$uav_id")"
        if [ -n "$altitude" ] && awk -v alt="$altitude" -v success="$SUCCESS_ALTITUDE" 'BEGIN { exit !(alt >= success) }'; then
            echo "[takeoff_all] uav${uav_id}: altitude=${altitude}m"
            if [ -n "$takeoff_pid" ] && kill -0 "$takeoff_pid" 2>/dev/null; then
                kill "$takeoff_pid" 2>/dev/null || true
                wait "$takeoff_pid" 2>/dev/null || true
            fi
            if [ -n "$arming_rc_pid" ] && kill -0 "$arming_rc_pid" 2>/dev/null; then
                kill "$arming_rc_pid" 2>/dev/null || true
                wait "$arming_rc_pid" 2>/dev/null || true
            fi
            if [ -n "$rc_pid" ] && kill -0 "$rc_pid" 2>/dev/null; then
                kill "$rc_pid" 2>/dev/null || true
                wait "$rc_pid" 2>/dev/null || true
            fi
            release_rc_override "$uav_id"
            return 0
        fi

        if [ $(( $(date +%s) - start_ts )) -ge "$PER_UAV_TIMEOUT" ]; then
            echo "[takeoff_all] uav${uav_id}: timeout, altitude=${altitude:-unknown}m" >&2
            if [ -n "$takeoff_pid" ] && kill -0 "$takeoff_pid" 2>/dev/null; then
                kill "$takeoff_pid" 2>/dev/null || true
                wait "$takeoff_pid" 2>/dev/null || true
            fi
            if [ -n "$arming_rc_pid" ] && kill -0 "$arming_rc_pid" 2>/dev/null; then
                kill "$arming_rc_pid" 2>/dev/null || true
                wait "$arming_rc_pid" 2>/dev/null || true
            fi
            if [ -n "$rc_pid" ] && kill -0 "$rc_pid" 2>/dev/null; then
                kill "$rc_pid" 2>/dev/null || true
                wait "$rc_pid" 2>/dev/null || true
            fi
            release_rc_override "$uav_id"
            return 1
        fi

        sleep "$TAKEOFF_RETRY_DELAY"
    done
}

arm_one_cli() {
    local uav_id="$1"
    local start_ts
    local armed_state=""
    start_ts="$(date +%s)"

    ros2 topic pub --once "/uav${uav_id}/uav/cmd/mode" std_msgs/msg/String '{data: GUIDED}' >/dev/null 2>&1
    while true; do
        ros2 topic pub --once "/uav${uav_id}/uav/cmd/arm" std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1
        armed_state="$(timeout 3s ros2 topic echo --once "/uav${uav_id}/uav/armed" 2>/dev/null | awk '/data:/ {print $2; exit}')"
        if [ "$armed_state" = "true" ]; then
            echo "[takeoff_all] uav${uav_id}: arm ok"
            return 0
        fi
        if [ $(( $(date +%s) - start_ts )) -ge "$PER_UAV_TIMEOUT" ]; then
            echo "[takeoff_all] uav${uav_id}: arm timeout" >&2
            return 1
        fi
        sleep "$ARM_RETRY_DELAY"
    done
}

if [ "$TAKEOFF_ENGINE" = "cli" ]; then
    failures=0
    if [ "$TAKEOFF_PARALLEL" = "1" ]; then
        launch_ids=()
        if [ "$TAKEOFF_ARM_SERIAL_FIRST" = "1" ]; then
            for ((i=1; i<=NUM_UAV; i++)); do
                if arm_one_cli "$i"; then
                    launch_ids+=("$i")
                else
                    failures=$((failures + 1))
                fi
            done
        else
            for ((i=1; i<=NUM_UAV; i++)); do
                launch_ids+=("$i")
            done
        fi

        pids=()
        for id in "${launch_ids[@]}"; do
            if [ "$TAKEOFF_ARM_SERIAL_FIRST" = "1" ]; then
                takeoff_one_cli "$id" 1 &
            else
                takeoff_one_cli "$id" &
            fi
            pids+=("$!")
        done
        for pid in "${pids[@]}"; do
            if ! wait "$pid"; then
                failures=$((failures + 1))
            fi
        done
    else
        for ((i=1; i<=NUM_UAV; i++)); do
            if ! takeoff_one_cli "$i"; then
                failures=$((failures + 1))
            fi
        done
    fi

    # Retry only failed UAVs to improve reliability under transient command loss.
    if [ "$TAKEOFF_RETRY_ROUNDS" -gt 0 ]; then
        for ((round=1; round<=TAKEOFF_RETRY_ROUNDS; round++)); do
            failed_ids=()
            for ((i=1; i<=NUM_UAV; i++)); do
                if ! is_airborne "$i"; then
                    failed_ids+=("$i")
                fi
            done

            if [ "${#failed_ids[@]}" -eq 0 ]; then
                break
            fi

            echo "[takeoff_all] retry round ${round}: failed UAVs=${failed_ids[*]}"
            for id in "${failed_ids[@]}"; do
                if ! takeoff_one_cli "$id"; then
                    true
                fi
            done
        done
    fi

    failures=0
    for ((i=1; i<=NUM_UAV; i++)); do
        if ! is_airborne "$i"; then
            failures=$((failures + 1))
        fi
    done

    if [ "$failures" -ne 0 ]; then
        echo "[takeoff_all] ${failures}/${NUM_UAV} 架无人机起飞失败" >&2
        exit 1
    fi

    echo "[takeoff_all] ${NUM_UAV}/${NUM_UAV} 架无人机已完成起飞"
    exit 0
fi

"$PYTHON_BIN" "$SCRIPT_DIR/takeoff_all.py" \
    "$ALTITUDE" \
    --num-uav "$NUM_UAV" \
    --success-altitude "$SUCCESS_ALTITUDE" \
    --timeout "$TAKEOFF_TIMEOUT" \
    --climb-speed "$CLIMB_SPEED" \
    --climb-assist-delay "$CLIMB_ASSIST_DELAY" \
    --target-altitude-tolerance "$TARGET_ALTITUDE_TOLERANCE" \
    $( [ "$TAKEOFF_PARALLEL" = "1" ] && echo "--parallel-takeoff" )

exit $?
