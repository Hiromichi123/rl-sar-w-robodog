#!/bin/bash
# jxg-web.service 专用启动脚本:独立 Web 上位机进程(web_server)。
# 与 rl_real_JXG 分离部署 —— 重启控制程序时浏览器/SSH 会话不中断。
#
# 职责:
#   1. 兜底加载 /opt/ros/<distro>/setup.bash(web_server 链接 rclcpp/FastDDS);
#   2. FastDDS 仅用 UDP(与 start.sh 同一 profile,跨用户收发可靠);
#   3. exec 启动 web_server(默认 0.0.0.0:8088)。
#
# 环境变量:
#   JXG_WEB_PORT    监听端口(默认 8088)
#   WEB_ROOT        静态页根目录(默认编译期宏 .../web)
#   WEB_ADMIN_TOKEN WebSSH/WebFTP/电源操作鉴权令牌(空=局域网放行)
set -u

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "${SCRIPT_DIR}/.." && pwd )"
BIN="${REPO_ROOT}/local_build/bin/web_server"

# ── ROS2 运行时环境兜底 ──
if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
    for _ros_setup in /opt/ros/lyrical/setup.bash /opt/ros/jazzy/setup.bash /opt/ros/humble/setup.bash; do
        if [ -f "$_ros_setup" ]; then
            set +u
            # shellcheck disable=SC1090
            . "$_ros_setup"
            set -u
            echo "[web] sourced $_ros_setup (ROS_DISTRO=${ROS_DISTRO:-?})"
            break
        fi
    done
fi

# ── FastDDS UDP-only(与主程序一致,避免 root SHM 段跨用户不可用) ──
FASTDDS_PROFILE="${REPO_ROOT}/fastdds_udp_only.xml"
if [ -f "$FASTDDS_PROFILE" ]; then
    export FASTDDS_DEFAULT_PROFILES_FILE="$FASTDDS_PROFILE"
    export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_PROFILE"
fi

if [ ! -x "$BIN" ]; then
    echo "[web] ERROR: web_server 不存在或不可执行: $BIN" >&2
    exit 1
fi

# ── 服务环境配置(管理令牌/端口等) ──
ENV_FILE="${SCRIPT_DIR}/web_env.conf"
if [ -f "$ENV_FILE" ]; then
    # set -a:文件内赋值自动 export,否则 exec 的子进程读不到这些变量
    set -a
    # shellcheck disable=SC1090
    . "$ENV_FILE"
    set +a
fi

export WEB_ROOT=/home/jinjiao/rl_sar-w2_recovered_v7.8/web
cd "$REPO_ROOT"
# v7.2: 导出构建库路径(web_bridge/rl_sdk 等) + ROS 库路径兜底
# (部分环境 AMENT_PREFIX_PATH 已设置但 LD_LIBRARY_PATH 缺 ROS lib,source 分支被跳过)
ROS_LIB_DIR=""
for _d in /opt/ros/lyrical /opt/ros/jazzy /opt/ros/humble; do
    [ -f "$_d/lib/librclcpp.so" ] && { ROS_LIB_DIR="$_d/lib"; break; }
done
# lyrical 等发行版把 fastdds/rmw 依赖拆到 lib/aarch64-linux-gnu,两个目录都要
ROS_LIB_ARCH_DIR=""
[ -n "$ROS_LIB_DIR" ] && [ -d "${ROS_LIB_DIR}/aarch64-linux-gnu" ] && ROS_LIB_ARCH_DIR="${ROS_LIB_DIR}/aarch64-linux-gnu"
export LD_LIBRARY_PATH="${REPO_ROOT}/local_build/lib${ROS_LIB_DIR:+:${ROS_LIB_DIR}}${ROS_LIB_ARCH_DIR:+:${ROS_LIB_ARCH_DIR}}:${LD_LIBRARY_PATH:-}"

echo "[web] 启动 $BIN (port ${JXG_WEB_PORT:-8088})"
# 锁定小核 CPU0-3(A53):大核 4-7 预留给 rl_real_JXG 的实时控制环,
# 避免 web 的 FastDDS/HTTP 线程在大核上抢占,增加控制环调度抖动。
exec taskset -c 0-3 "$BIN"
