#!/bin/bash
# ==============================================================================
# start_sitl_all.sh — 在后台启动 6 个 ArduPilot SITL 实例
# 端口约定:
#   Instance I → fdm_port = 9002 + 10*I
#   Instance I → MAVProxy master TCP = 5760 + 10*I
#   Instance I → MAVProxy auto-output UDP = 14550 + 10*I
#   额外 --out: udp:127.0.0.1:(14501+I) 用于 bridge RX
#               udp:127.0.0.1:(14601+I) 用于 bridge TX
# ==============================================================================
set -e

NUM_UAV=6
VEHICLE=ArduCopter
FRAME=gazebo-iris
MODEL=JSON
PARM_FILE="$HOME/gz_ws/src/ardupilot_gazebo/config/gazebo-iris-gimbal.parm"
SITL_BASE_DIR="$HOME/gz_ws/src/ardupilot_gazebo/sitl_instances"

echo "========================================="
echo " 启动 ${NUM_UAV} 个 ArduPilot SITL 实例"
echo "========================================="

echo "清理旧的 SITL / MAVProxy 残留进程..."
pkill -f "$HOME/ardupilot/Tools/autotest/sim_vehicle.py.*-v ${VEHICLE}" 2>/dev/null || true
pkill -f "$HOME/ardupilot/build/sitl/bin/arducopter .*gazebo-iris-gimbal.parm" 2>/dev/null || true
pkill -f "xterm .*${HOME}/ardupilot/build/sitl/bin/arducopter .*gazebo-iris-gimbal.parm" 2>/dev/null || true
pkill -f "mavproxy.py .*${SITL_BASE_DIR}/uav[1-6]" 2>/dev/null || true
sleep 2

mkdir -p "$SITL_BASE_DIR"

for ((i=0; i<NUM_UAV; i++)); do
    UAV_NUM=$((i+1))
    RX_PORT=$((14501+i))
    TX_PORT=$((14601+i))
    INSTANCE_DIR="${SITL_BASE_DIR}/uav${UAV_NUM}"

    mkdir -p "$INSTANCE_DIR"

    echo "[uav${UAV_NUM}] Instance=$i  FDM=9002+$((10*i))=$((9002+10*i))  RX=$RX_PORT  TX=$TX_PORT  AIRCRAFT_DIR=$INSTANCE_DIR"

    # 每个 SITL 在独立 gnome-terminal 标签页中启动
    gnome-terminal --tab --title="SITL-uav${UAV_NUM}" -- bash -c "
        cd $HOME/ardupilot
        Tools/autotest/sim_vehicle.py \
            -v ${VEHICLE} \
            -f ${FRAME} \
            --model ${MODEL} \
            -I ${i} \
            -w \
            --auto-sysid \
            --mavproxy-args='--non-interactive --streamrate=20' \
            --add-param-file=${PARM_FILE} \
            --out=127.0.0.1:${RX_PORT} \
            --out=127.0.0.1:${TX_PORT} \
            --console \
            --aircraft='${INSTANCE_DIR}' \
            --no-rebuild
        exec bash
    " &
    
    # 错开启动，避免同时编译冲突
    sleep 3
done

echo ""
echo "所有 ${NUM_UAV} 个 SITL 实例已在独立终端标签页中启动。"
echo "请等待所有实例完成初始化（出现 'APM: EKF3 IMU0 is using GPS' 字样）。"
echo ""
echo "端口映射:"
for ((i=0; i<NUM_UAV; i++)); do
    echo "  uav$((i+1)): SITL Instance=$i  FDM=$((9002+10*i))  RX=$((14501+i))  TX=$((14601+i))  AIRCRAFT_DIR=${SITL_BASE_DIR}/uav$((i+1))"
done
