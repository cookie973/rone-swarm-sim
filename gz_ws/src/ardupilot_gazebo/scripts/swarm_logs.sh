#!/bin/bash
# ==============================================================================
# swarm_logs.sh — 查看蜂群日志
#
# 用法:
#   ./swarm_logs.sh              # 列出所有日志文件
#   ./swarm_logs.sh all          # 汇总全部日志实时输出
#   ./swarm_logs.sh gazebo       # 查看 Gazebo 日志
#   ./swarm_logs.sh sitl 3       # 查看 SITL UAV3 日志
#   ./swarm_logs.sh bridge 1     # 查看 Bridge UAV1 日志
#   ./swarm_logs.sh formation    # 查看编队控制日志
#   ./swarm_logs.sh qgc          # 查看 QGC 中继日志
#   ./swarm_logs.sh -f bridge 2  # 实时跟踪 Bridge UAV2 日志 (tail -f)
# ==============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
LOG_DIR="${REPO_DIR}/logs"

if [ ! -d "$LOG_DIR" ]; then
    echo "日志目录不存在: $LOG_DIR"
    exit 1
fi

# 检查是否要求 tail -f
FOLLOW=false
if [ "${1:-}" = "-f" ]; then
    FOLLOW=true
    shift
fi

COMPONENT="${1:-}"
UAV_NUM="${2:-}"

show_usage() {
    echo "用法: $0 [-f] [component] [uav_number]"
    echo ""
    echo "组件:"
    echo "  all         汇总全部日志实时输出"
    echo "  gazebo      Gazebo 日志"
    echo "  sitl N      SITL UAV N 日志 (N=1..6)"
    echo "  bridge N    Bridge UAV N 日志 (N=1..6)"
    echo "  formation   编队控制日志"
    echo "  qgc         QGC 中继日志"
    echo ""
    echo "选项:"
    echo "  -f          实时跟踪日志 (tail -f)"
    echo ""
    echo "不带参数则列出所有日志文件及大小。"
}

list_logs() {
    echo "============ 日志文件 ============"
    if ls "$LOG_DIR"/*.log 1>/dev/null 2>&1; then
        ls -lhS "$LOG_DIR"/*.log
    else
        echo "没有日志文件"
    fi
}

view_log() {
    local logfile="$1"
    if [ ! -f "$logfile" ]; then
        echo "日志文件不存在: $logfile"
        exit 1
    fi
    if $FOLLOW; then
        tail -f "$logfile"
    else
        tail -n 100 "$logfile"
    fi
}

case "$COMPONENT" in
    "")
        list_logs
        ;;
    help|-h|--help)
        show_usage
        ;;
    all)
        echo "汇总全部日志实时输出 (Ctrl+C 退出)..."
        tail -f "$LOG_DIR"/*.log 2>/dev/null || echo "没有日志文件"
        ;;
    gazebo)
        view_log "$LOG_DIR/gazebo.log"
        ;;
    sitl)
        if [ -z "$UAV_NUM" ]; then
            echo "请指定 UAV 编号 (1-6)，例如: $0 sitl 3"
            exit 1
        fi
        view_log "$LOG_DIR/sitl_uav${UAV_NUM}.log"
        ;;
    bridge)
        if [ -z "$UAV_NUM" ]; then
            echo "请指定 UAV 编号 (1-6)，例如: $0 bridge 1"
            exit 1
        fi
        view_log "$LOG_DIR/bridge_uav${UAV_NUM}.log"
        ;;
    formation)
        view_log "$LOG_DIR/swarm_formation.log"
        ;;
    qgc)
        view_log "$LOG_DIR/qgc_bridge.log"
        ;;
    *)
        echo "未知组件: $COMPONENT"
        show_usage
        exit 1
        ;;
esac
