#!/usr/bin/env bash
# rl_real_JXG 开机自启动脚本
# 参考: /home/kickpi/rl/autostart/sdog_deploy_autostart.sh
# 注意: 不能用 -u，ROS2 setup.bash 会引用未定义变量(AMENT_TRACE_SETUP_FILES)
set -eo pipefail

# ROS2 Jazzy 环境（服务环境为非交互 shell，必须显式 source）
source /opt/ros/jazzy/setup.bash

# FastDDS 网络接口白名单：让 DDS 在本机回环 + Insight 9 USB 网卡上发现
# （默认 FastDDS 只在 wlan0 组播，看不到通过 USB 以太网接入的相机话题）
export FASTRTPS_DEFAULT_PROFILES_FILE="/home/kickpi/rl_sar-w2_recovered/fastdds_insight.xml"

BIN_DIR="/home/kickpi/rl_sar-w2_recovered/cmake_build/bin"
LOG_DIR="${BIN_DIR}/logs"

# 创建日志目录
mkdir -p "${LOG_DIR}"
# 日志文件名，带时间戳
LOG_FILE="${LOG_DIR}/rl_real_JXG_$(date +%Y%m%d_%H%M%S).log"

cd "$BIN_DIR"

# 单实例锁：防止重复启动多个 rl_real_JXG 进程抢占同一 CAN 总线
# （2026-09-06 曾发生 6 秒内双实例抢总线，enableAllMotors 交错执行导致
#   电机多圈零点参考丢失 ±2π，自检报 too far from zero，无法起立）
LOCK_FILE="/tmp/rl_real_JXG_${USER}.lock"
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
    echo "[autostart] 检测到已有 rl_real_JXG 实例在运行 (锁: ${LOCK_FILE})，本次启动跳过。" | tee -a "${LOG_FILE}"
    echo "[autostart] 如需强制重启，先执行: pkill -f rl_real_JXG" | tee -a "${LOG_FILE}"
    exit 0
fi
echo "[autostart] 单实例锁获取成功: ${LOCK_FILE}" | tee -a "${LOG_FILE}"

echo "[autostart] Starting rl_real_JXG (JXGw) as $(whoami)..." | tee -a "${LOG_FILE}"

# stdbuf -oL: 行缓冲，保证程序日志实时写入 journal/log文件
# tee 实现：屏幕/journal + 日志文件双输出
exec stdbuf -oL ./rl_real_JXG 2>&1 | tee -a "${LOG_FILE}"
