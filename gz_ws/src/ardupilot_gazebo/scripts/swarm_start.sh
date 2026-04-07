#!/bin/bash
# ==============================================================================
# swarm_start.sh — 一键后台启动 6-UAV 蜂群仿真全栈
#
# 启动内容:
#   1. Gazebo Harmonic (gazebo_6uav.sdf)
#   2. 6 个 ArduPilot SITL 实例 (独立 SYSID / 端口)
#   3. 6 个 ROS2 uav_bridge 节点
#   4. 1 个 qgc_bridge QGC 中继节点
#   5. 1 个 swarm_formation 编队控制节点
#
# 所有进程后台运行，日志保存到 $LOG_DIR/{gazebo,uav1..uav6,bridge_uav1..uav6,qgc_bridge,swarm_formation}.log
#
# 用法:
#   ./swarm_start.sh                    # 默认全部启动
#   ./swarm_start.sh --no-controller    # 不启动编队控制器
#   GAZEBO_WAIT=12 ./swarm_start.sh     # 自定义 Gazebo 启动等待时间
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
NUM_UAV=6
AUTO_CONTROLLER=true
GAZEBO_WAIT=${GAZEBO_WAIT:-6}
SITL_WAIT=${SITL_WAIT:-1}
SITL_INIT_WAIT=${SITL_INIT_WAIT:-10}
BRIDGE_WAIT=${BRIDGE_WAIT:-0.5}
BRIDGE_INIT_WAIT=${BRIDGE_INIT_WAIT:-2}
QGC_WAIT=${QGC_WAIT:-1}
CONTROLLER_WAIT=${CONTROLLER_WAIT:-1}
LOG_DIR="${REPO_DIR}/logs"
PID_FILE="${REPO_DIR}/logs/.swarm_pids"

# ArduPilot 路径
ARDUPILOT_DIR="$HOME/ardupilot"
VEHICLE=ArduCopter
FRAME=gazebo-iris
MODEL=JSON
PARM_FILE="${REPO_DIR}/config/gazebo-iris-gimbal.parm"
SITL_INSTANCE_DIR="${REPO_DIR}/sitl_instances"

# ---------- 参数解析 ----------
for arg in "$@"; do
    case "$arg" in
        --no-controller) AUTO_CONTROLLER=false ;;
        --help|-h)
            echo "用法: $0 [--no-controller] [--help]"
            echo "  --no-controller  不自动启动 swarm_formation 编队控制节点"
            echo ""
            echo "环境变量:"
            echo "  GAZEBO_WAIT   Gazebo 启动后等待秒数 (默认 10)"
            echo "  SITL_WAIT     每个 SITL 启动间隔秒数 (默认 3)"
            echo "  BRIDGE_WAIT   每个 bridge 启动间隔秒数 (默认 2)"
            echo "  QGC_WAIT      qgc_bridge 启动间隔秒数 (默认 2)"
            echo "  LOG_DIR       日志存储目录 (默认 ${REPO_DIR}/logs)"
            exit 0
            ;;
    esac
done

# ---------- 前置校验 ----------
errors=0

if [ ! -f "${REPO_DIR}/worlds/gazebo_6uav.sdf" ]; then
    echo "[ERROR] 世界文件不存在: ${REPO_DIR}/worlds/gazebo_6uav.sdf"
    errors=1
fi

if [ ! -f "$PARM_FILE" ]; then
    echo "[ERROR] 参数文件不存在: $PARM_FILE"
    errors=1
fi

if [ ! -f "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" ]; then
    echo "[ERROR] sim_vehicle.py 不存在: $ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py"
    errors=1
fi

if ! command -v gz >/dev/null 2>&1; then
    echo "[ERROR] 未找到 gz (Gazebo Harmonic) 命令"
    errors=1
fi

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "[ERROR] 未找到 ROS2 Humble: /opt/ros/humble/setup.bash"
    errors=1
fi

# 端口占用检查
check_port() {
    local port=$1
    local desc=$2
    if ss -lntu 2>/dev/null | grep -q ":${port}\b"; then
        echo "[WARN] 端口 ${port} 已被占用 (${desc})，可能有残留进程"
    fi
}

for ((i=0; i<NUM_UAV; i++)); do
    check_port $((9002 + 10*i))  "Gazebo FDM uav$((i+1))"
    check_port $((14501 + i))    "Bridge RX uav$((i+1))"
    check_port $((14601 + i))    "Bridge TX uav$((i+1))"
    check_port $((14540 + i))    "QGC external uav$((i+1))"
    check_port $((14740 + i))    "QGC relay uav$((i+1))"
    check_port $((5760 + 10*i))  "MAVProxy TCP uav$((i+1))"
done

if [ "$errors" -ne 0 ]; then
    echo "[ERROR] 前置校验失败，请先修复以上问题"
    exit 1
fi

# ---------- 清理旧进程 ----------
echo "========================================="
echo "  清理旧进程..."
echo "========================================="
"$SCRIPT_DIR/swarm_stop.sh" 2>/dev/null || true
sleep 1

# ---------- 准备日志目录 ----------
mkdir -p "$LOG_DIR"
mkdir -p "$SITL_INSTANCE_DIR"
> "$PID_FILE"

# 记录启动时间
echo "# swarm_start.sh 启动于 $(date '+%Y-%m-%d %H:%M:%S')" >> "$PID_FILE"

# ---------- 启动 Gazebo ----------
echo ""
echo "========================================="
echo " [1/5] 启动 Gazebo Harmonic"
echo "========================================="

export GZ_SIM_RESOURCE_PATH="${REPO_DIR}/models:${REPO_DIR}/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${REPO_DIR}/../../build/ardupilot_gazebo:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

gz sim -v4 -r --render-engine ogre gazebo_6uav.sdf \
    > "$LOG_DIR/gazebo.log" 2>&1 &
GZ_PID=$!
echo "gazebo=$GZ_PID" >> "$PID_FILE"
echo "[Gazebo] PID=$GZ_PID  日志: $LOG_DIR/gazebo.log"
echo "[Gazebo] 等待 ${GAZEBO_WAIT}s 初始化..."
sleep "$GAZEBO_WAIT"

if ! kill -0 $GZ_PID 2>/dev/null; then
    echo "[ERROR] Gazebo 启动失败，请查看 $LOG_DIR/gazebo.log"
    exit 1
fi

# ---------- 启动 6 个 SITL ----------
echo ""
echo "========================================="
echo " [2/5] 启动 ${NUM_UAV} 个 ArduPilot SITL"
echo "========================================="

set +u
source /opt/ros/humble/setup.bash
set -u

SITL_PIDS=()
for ((i=0; i<NUM_UAV; i++)); do
    UAV_NUM=$((i+1))
    RX_PORT=$((14501+i))
    TX_PORT=$((14601+i))
    QGC_PORT=$((14540+i))
    QGC_RELAY_PORT=$((14740+i))
    INSTANCE_DIR="${SITL_INSTANCE_DIR}/uav${UAV_NUM}"
    mkdir -p "$INSTANCE_DIR"

    echo "[uav${UAV_NUM}] Instance=$i  FDM=$((9002+10*i))  RX=$RX_PORT  TX=$TX_PORT  QGC=$QGC_PORT  Relay=$QGC_RELAY_PORT"

    (
        cd "$ARDUPILOT_DIR"
        Tools/autotest/sim_vehicle.py \
            -v "$VEHICLE" \
            -f "$FRAME" \
            --model "$MODEL" \
            -I "$i" \
            -w \
            --auto-sysid \
            --mavproxy-args="--non-interactive --streamrate=20" \
            --add-param-file="$PARM_FILE" \
            --out="127.0.0.1:${RX_PORT}" \
            --out="127.0.0.1:${TX_PORT}" \
            --out="127.0.0.1:${QGC_RELAY_PORT}" \
            --aircraft="$INSTANCE_DIR" \
            --no-rebuild \
            --no-extra-ports
    ) > "$LOG_DIR/uav${UAV_NUM}.log" 2>&1 &
    SITL_PIDS+=($!)
    echo "sitl_uav${UAV_NUM}=$!" >> "$PID_FILE"

    sleep "$SITL_WAIT"
done

echo "[SITL] 等待初始化 (约 ${SITL_INIT_WAIT}s)..."
sleep "$SITL_INIT_WAIT"

for ((i=0; i<NUM_UAV; i++)); do
    UAV_NUM=$((i+1))
    if ! kill -0 "${SITL_PIDS[$i]}" 2>/dev/null; then
        echo "[WARN] uav${UAV_NUM} SITL 进程可能已退出，请检查 $LOG_DIR/uav${UAV_NUM}.log"
    fi
done

# ---------- 启动 6 个 ROS2 bridge ----------
echo ""
echo "========================================="
echo " [3/5] 启动 ${NUM_UAV} 个 ROS2 uav_bridge"
echo "========================================="

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    set +u
    source "$HOME/ros2_ws/install/setup.bash"
    set -u
fi

BRIDGE_PIDS=()
for ((i=0; i<NUM_UAV; i++)); do
    UAV_NUM=$((i+1))
    NS="uav${UAV_NUM}"
    TX_PORT=$((14601+i))
    TX_URL="udp:127.0.0.1:${TX_PORT}"
    TARGET_SYSID=${UAV_NUM}

    echo "[bridge-${NS}] TX=${TX_URL}  SYSID=${TARGET_SYSID}"

    ros2 launch uav_bridge camera_mavlink.launch.py \
        namespace:="${NS}" \
        mavlink_url_tx:="${TX_URL}" \
        target_system:="${TARGET_SYSID}" \
        target_component:=1 \
        enable_rx:=false \
        enable_rqt:=false \
        enable_image:=false \
    > "$LOG_DIR/bridge_uav${UAV_NUM}.log" 2>&1 &
    BRIDGE_PIDS+=($!)
    echo "bridge_uav${UAV_NUM}=$!" >> "$PID_FILE"

    sleep "$BRIDGE_WAIT"
done

echo "[Bridge] 等待 ${BRIDGE_INIT_WAIT}s 初始化..."
sleep "$BRIDGE_INIT_WAIT"

# ---------- 启动 QGC 中继 ----------
echo ""
echo "========================================="
echo " [4/5] 启动 qgc_bridge QGC 中继节点"
echo "========================================="

ros2 run uav_bridge qgc_bridge \
> "$LOG_DIR/qgc_bridge.log" 2>&1 &
QGC_PID=$!
echo "qgc_bridge=$QGC_PID" >> "$PID_FILE"
echo "[QGC Bridge] PID=$QGC_PID  日志: $LOG_DIR/qgc_bridge.log"
sleep "$QGC_WAIT"

# ---------- 启动编队控制器 ----------
if [ "$AUTO_CONTROLLER" = true ]; then
    echo ""
    echo "========================================="
    echo " [5/5] 启动 swarm_formation 编队控制节点"
    echo "========================================="

    ros2 run uav_bridge swarm_formation \
    > "$LOG_DIR/swarm_formation.log" 2>&1 &
    CTRL_PID=$!
    echo "swarm_formation=$CTRL_PID" >> "$PID_FILE"
    echo "[Controller] PID=$CTRL_PID  日志: $LOG_DIR/swarm_formation.log"
    sleep "$CONTROLLER_WAIT"
fi

# ---------- 汇总 ----------
echo ""
echo "========================================="
echo " 6-UAV 蜂群仿真启动完成"
echo "========================================="
echo ""
echo "日志目录: $LOG_DIR/"
echo "  gazebo.log              Gazebo 日志"
for ((i=1; i<=NUM_UAV; i++)); do
    echo "  uav${i}.log               SITL uav${i} 日志"
done
for ((i=1; i<=NUM_UAV; i++)); do
    echo "  bridge_uav${i}.log        Bridge uav${i} 日志"
done
if [ "$AUTO_CONTROLLER" = true ]; then
    echo "  swarm_formation.log     编队控制器日志"
fi
echo "  qgc_bridge.log          QGC 中继日志"
echo ""
echo "实时查看日志:"
echo "  tail -f $LOG_DIR/uav1.log"
echo "  tail -f $LOG_DIR/gazebo.log"
echo "  $SCRIPT_DIR/swarm_logs.sh uav1"
echo "  $SCRIPT_DIR/swarm_logs.sh all"
echo ""
echo "下一步:"
echo "  cd $SCRIPT_DIR && ./takeoff_all.sh"
echo "  cd $SCRIPT_DIR && ./formation_switch.sh grid"
echo ""
echo "停止所有进程:"
echo "  cd $SCRIPT_DIR && ./swarm_stop.sh"
echo ""
echo "QGC 连接端口:"
for ((i=0; i<NUM_UAV; i++)); do
    echo "  uav$((i+1)): UDP 127.0.0.1:$((14540+i))"
done
