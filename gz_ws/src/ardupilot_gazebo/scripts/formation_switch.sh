#!/bin/bash
# ==============================================================================
# formation_switch.sh — 切换蜂群控制模式与队形
# 用法:
#   ./formation_switch.sh grid
#   ./formation_switch.sh leader_follower grid
#   ./formation_switch.sh anchored circle
# ==============================================================================
set -e

VALID_MODES="anchored leader_follower"
VALID_FORMATIONS="circle line v_shape grid"
MODE=""
FORMATION=""

if [ $# -eq 1 ]; then
    FORMATION="$1"
elif [ $# -eq 2 ]; then
    MODE="$1"
    FORMATION="$2"
else
    echo "用法: $0 [anchored|leader_follower] <circle|line|v_shape|grid>"
    exit 1
fi

if [ -n "$MODE" ] && [[ " $VALID_MODES " != *" $MODE "* ]]; then
    echo "[ERROR] 无效控制模式: $MODE"
    echo "可选模式: $VALID_MODES"
    exit 1
fi

if [[ " $VALID_FORMATIONS " != *" $FORMATION "* ]]; then
    echo "[ERROR] 无效队形: $FORMATION"
    echo "可选队形: $VALID_FORMATIONS"
    exit 1
fi

if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

if ! ros2 node list 2>/dev/null | grep -qx '/swarm_formation'; then
    echo "[ERROR] 未检测到 /swarm_formation 控制节点。"
    echo "请先运行: $HOME/gz_ws/src/ardupilot_gazebo/scripts/start_swarm_controller.sh"
    echo "或者手动运行: ros2 run uav_bridge swarm_formation"
    exit 1
fi

if [ -n "$MODE" ]; then
    echo "切换控制模式 -> $MODE"
    ros2 topic pub --once /swarm/mode std_msgs/msg/String "{data: '$MODE'}"
    sleep 1
fi

echo "切换队形 -> $FORMATION"
ros2 topic pub --once /swarm/formation std_msgs/msg/String "{data: '$FORMATION'}"
