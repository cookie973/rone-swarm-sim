#!/bin/bash
# ==============================================================================
# start_bridge_all.sh — 启动 6 个 uav_bridge ROS2 节点（带命名空间）
# 当前脚本只启动 mavlink_tx，由它同时承担 telemetry RX + command TX
# ==============================================================================
set -e

NUM_UAV=6
WORLD_NAME="iris_runway"

echo "========================================="
echo " 启动 ${NUM_UAV} 个 uav_bridge 节点"
echo "========================================="

echo "清理旧的 bridge 残留进程..."
pkill -f "/home/cookie/ros2_ws/install/uav_bridge/lib/uav_bridge/mavlink_tx" 2>/dev/null || true
pkill -f "ros2 launch uav_bridge camera_mavlink.launch.py.*namespace:=uav" 2>/dev/null || true
sleep 1

# 确保 ROS2 环境已 source
if [ -z "$ROS_DISTRO" ]; then
    echo "[ERROR] 请先 source ROS2 环境: source /opt/ros/humble/setup.bash"
    exit 1
fi

# source 工作空间
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

for ((i=0; i<NUM_UAV; i++)); do
    UAV_NUM=$((i+1))
    NS="uav${UAV_NUM}"
    RX_PORT=$((14501+i))
    TX_PORT=$((14601+i))
    TARGET_SYSID=${UAV_NUM}
    RX_URL="udp:127.0.0.1:${RX_PORT}"
    TX_URL="udp:127.0.0.1:${TX_PORT}"
    GZ_IMAGE_TOPIC="/world/${WORLD_NAME}/model/${NS}/model/gimbal/link/pitch_link/sensor/camera/image"

    echo "[${NS}] RX=${RX_URL}  TX=${TX_URL}"

    gnome-terminal --tab --title="bridge-${NS}" -- bash -c "
        source /opt/ros/humble/setup.bash
        source $HOME/ros2_ws/install/setup.bash
        ros2 launch uav_bridge camera_mavlink.launch.py \
            namespace:=${NS} \
            mavlink_url_rx:='${RX_URL}' \
            mavlink_url_tx:='${TX_URL}' \
            target_system:=${TARGET_SYSID} \
            target_component:=1 \
            gz_image_topic:='${GZ_IMAGE_TOPIC}' \
            ros_image_topic:='/${NS}/camera/image' \
            rqt_image_topic:='/${NS}/camera/image' \
            enable_rx:=false \
            enable_rqt:=false \
            enable_image:=false
        exec bash
    " &

    sleep 1
done

echo ""
echo "所有 ${NUM_UAV} 个 bridge 节点已在独立终端标签页中启动。"
echo ""
echo "ROS2 话题（以 uav1 为例）:"
echo "  状态:  /uav1/uav/pose, /uav1/uav/mode, /uav1/uav/armed"
echo "  控制:  /uav1/uav/cmd/arm, /uav1/uav/cmd/mode, /uav1/uav/cmd/takeoff"
echo "         /uav1/uav/cmd/land, /uav1/uav/cmd/velocity, /uav1/uav/cmd/waypoint"
