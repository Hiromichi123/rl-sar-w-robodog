#!/bin/bash
# rl_real_JXG (v4) 本机启动脚本(裸机直跑,非容器; v4 起 ROS2 话题与 WebSocket 可共存)。
#
# 职责:
#   1. 单实例锁(flock):防止双实例抢 CAN 总线导致电机多圈零点丢失(2026-09-06 事故);
#   2. 自动检测 CAN 模式并等待设备就绪:
#      - /dev/ttycan1 存在且为字符设备 → USB2CAN 模式,等待 ttycan1~ttycan4(udev 软链到 /dev/ttyACM*);
#      - 否则 → SocketCAN 模式,调用 can_setup.sh 配置并等待 can0~can3(candleLight gs_usb)UP;
#   3. root 启动时迁入根 cgroup,为 SCHED_FIFO 实时线程分配 RT runtime;
#   4. v4: 兜底加载 /opt/ros/<distro>/setup.bash, 为共存二进制提供 rclcpp/FastDDS 运行时;
#   5. exec 启动主程序 rl_real_JXG(v5 固定 JXGw,无启动参数)。
#
# 环境变量:
#   CAN_UP_WAIT   等待 CAN 设备就绪的超时秒数(默认 30,超时仅告警不阻塞)
#   CAN_IFACES    参与等待的设备节点/接口列表(默认按检测到的模式自动选取)
#
# 用法:
#   sudo bash scripts/start.sh [extra args]   # 推荐(root,自动迁根 cgroup)
set -u

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "${SCRIPT_DIR}/.." && pwd )"
BIN="${REPO_ROOT}/local_build/bin/rl_real_JXG"

# ── v4: ROS2 运行时环境兜底(共存构建的二进制链接了 /opt/ros 下的 rclcpp/FastDDS) ──
# 已在 source 过 ROS 的终端中启动则不重复加载。
if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
    for _ros_setup in /opt/ros/lyrical/setup.bash /opt/ros/jazzy/setup.bash /opt/ros/humble/setup.bash; do
        if [ -f "$_ros_setup" ]; then
            # ROS setup 脚本引用了大量未定义变量, 与本脚本的 set -u 冲突, 临时关闭
            set +u
            # shellcheck disable=SC1090
            . "$_ros_setup"
            set -u
            echo "[start] sourced $_ros_setup (ROS_DISTRO=${ROS_DISTRO:-?})"
            break
        fi
    done
fi

# ── v4: FastDDS 仅用 UDP(关闭 SHM) ──────────────────────────────
# 服务以 root 运行, 本机 ros2 CLI 多为普通用户: root 属主的 SHM 段跨用户
# 不可用会导致"能 list 到话题但 echo 收不到数据"。节点侧 UDP-only 后,
# 任意用户/网络上位机均可直接收发,无需各自配置。
FASTDDS_PROFILE="${REPO_ROOT}/fastdds_udp_only.xml"
if [ -f "$FASTDDS_PROFILE" ]; then
    export FASTDDS_DEFAULT_PROFILES_FILE="$FASTDDS_PROFILE"
    # 兼容读取旧变量名的 rmw 版本
    export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_PROFILE"
fi

CAN_UP_WAIT="${CAN_UP_WAIT:-30}"

# ── CAN 模式选择: 以 base.yaml 的 can_transport 为准 ─────────────
# v7.2 双硬件可共存(ttycan1~4 与 can0~4 同时在线),硬件探测会误判,
# 因此优先读 policy/JXGw/base.yaml 的 can_transport(usb2can|socketcan);
# 读不到配置时再回退硬件探测:/dev/ttycan1 字符设备存在 → usb2can,否则 socketcan。
YAML_CFG="${REPO_ROOT}/policy/JXGw/base.yaml"
yaml_transport=""
if [ -f "$YAML_CFG" ]; then
    yaml_transport="$(grep -E '^[[:space:]]*can_transport:' "$YAML_CFG" | tail -1 \
        | sed -E 's/.*can_transport:[[:space:]]*//; s/[#].*$//; s/["'"'"']//g' \
        | tr -d '[:space:]')"
fi
case "$yaml_transport" in
    usb2can|socketcan) CAN_MODE="$yaml_transport" ;;
    *)
        if [ -c /dev/ttycan1 ]; then CAN_MODE="usb2can"; else CAN_MODE="socketcan"; fi
        ;;
esac
if [ "$CAN_MODE" = "usb2can" ]; then
    CAN_IFACES="${CAN_IFACES:-/dev/ttycan1 /dev/ttycan2 /dev/ttycan3 /dev/ttycan4}"
else
    CAN_IFACES="${CAN_IFACES:-can0 can1 can2 can3}"
fi
echo "[start] CAN 模式: $CAN_MODE (等待节点:$CAN_IFACES)"

# v7.7: SocketCAN 接口配置已内嵌 C++(motor_manager.cpp → CanSetup::Run)
# 保留 can_setup.sh 作为独立调试工具，启动时不再自动调用
# run_can_setup() 已废弃，由主程序在 init() 时自动完成接口配置

if [ ! -x "$BIN" ]; then
    echo "[start] ERROR: 主程序不存在或不可执行: $BIN" >&2
    echo "[start] 请先在仓库根目录执行: cmake src/rl_sar/ -B cmake_build -DUSE_CMAKE=ON && cmake --build cmake_build -j8" >&2
    exit 1
fi

# ── 单实例锁(持有到进程退出,exec 后继承) ─────────────────────────
RUN_USER="${USER:-$(id -un 2>/dev/null || echo unknown)}"
LOCK_FILE="/tmp/rl_real_JXG_${RUN_USER}.lock"
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
    echo "[start] ERROR: 已有 rl_real_JXG 实例在运行(锁: ${LOCK_FILE}),拒绝双开。" >&2
    echo "[start] 如需重启,先: pkill -f rl_real_JXG" >&2
    exit 1
fi
echo "[start] 单实例锁获取成功: ${LOCK_FILE}"

# ── 等待 CAN 设备就绪 ────────────────────────────────────────────
# USB2CAN 模式: USB-CDC 串口(cdc_acm),udev 规则按物理端口建软链 /dev/ttycan1~4,
#               检查字符设备存在性;
# SocketCAN 模式: 网络接口(candleLight gs_usb),udev 规则 99-candlelight-can.rules
#               按 USB 序列号固定命名 can0~can4,can_setup.sh 配置 bitrate 并 up,
#               检查接口处于 "UP,LOWER_UP"。
if [ "$CAN_MODE" = "usb2can" ]; then
    can_is_up() {
        [ -c "$1" ] || { [ -e "$1" ] && [ -c "$(readlink -f "$1")" ]; }
    }
else
    can_is_up() {
        ip link show "$1" 2>/dev/null | grep -q "UP,LOWER_UP"
    }
fi

wait_can_up() {
    local missing elapsed=0
    while true; do
        missing=""
        for c in $CAN_IFACES; do
            can_is_up "$c" || missing="$missing $c"
        done
        if [ -z "$missing" ]; then
            echo "[start] CAN 设备全部就绪($CAN_MODE):$CAN_IFACES"
            return 0
        fi
        if [ "$elapsed" -ge "$CAN_UP_WAIT" ]; then
            echo "[start] WARN: 等待 CAN 设备超时(${CAN_UP_WAIT}s),以下节点仍未就绪:$missing"
            echo "[start] 继续启动主程序(相关电机可能不可用)"
            return 1
        fi
        # SocketCAN: 接口配置已内嵌 C++(CanSetup::Run),此处仅等待设备节点枚举
        if [ "$CAN_MODE" = "socketcan" ] && [ $((elapsed % 3)) -eq 0 ]; then
            true  # 主程序启动时由 CanSetup::Run 完成配置,无需 shell 侧操作
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
}

echo "[start] 等待 CAN 设备节点就绪(最多 ${CAN_UP_WAIT}s): $CAN_IFACES"
wait_can_up || true

# ── 实时调度环境准备 ─────────────────────────────────────────────
# 厂商内核开启 CONFIG_RT_GROUP_SCHED 且纯 cgroup v2(无 cpu.rt.max 接口),
# 子 cgroup 的 RT runtime 均为 0 —— 需迁入根 cgroup 才能用 SCHED_FIFO。
SELF_CG="$(awk -F: '$1=="0"{print $3}' /proc/$$/cgroup)"
if [ "$(id -u)" = "0" ]; then
    if [ "$SELF_CG" != "/" ] && [ -w /sys/fs/cgroup/cgroup.procs ]; then
        if echo $$ > /sys/fs/cgroup/cgroup.procs 2>/dev/null; then
            echo "[start] 已迁入根 cgroup(为 SCHED_FIFO 实时线程分配 RT runtime)"
        else
            echo "[start] WARN: 迁入根 cgroup 失败,实时线程可能无法启用"
        fi
    fi
else
    cap_eff_hex="$(awk '/^CapEff:/{print $2}' /proc/self/status)"
    if [ -n "$cap_eff_hex" ] && [ $(( 0x${cap_eff_hex} & (1 << 23) )) -eq 0 ]; then
        echo "[start] WARN: 非 root 且无 CAP_SYS_NICE,控制环将以普通调度运行(实时性下降)。"
        echo "[start]       推荐启动方式: sudo bash $(readlink -f "$0")"
    fi
fi

cd "$REPO_ROOT"
# HR logger 等诊断 CSV 写入相对路径 logs/，需预先创建
mkdir -p "${REPO_ROOT}/logs"

# v7.2: 导出构建库路径(rl_sdk/observation_buffer/web_bridge/inference_runtime 等)
# + ROS 库路径兜底(部分环境 AMENT_PREFIX_PATH 已设置但 LD_LIBRARY_PATH 缺 ROS lib)
ROS_LIB_DIR=""
for _d in /opt/ros/lyrical /opt/ros/jazzy /opt/ros/humble; do
    [ -f "$_d/lib/librclcpp.so" ] && { ROS_LIB_DIR="$_d/lib"; break; }
done
# lyrical 等发行版把 fastdds/rmw 依赖拆到 lib/aarch64-linux-gnu,两个目录都要
ROS_LIB_ARCH_DIR=""
[ -n "$ROS_LIB_DIR" ] && [ -d "${ROS_LIB_DIR}/aarch64-linux-gnu" ] && ROS_LIB_ARCH_DIR="${ROS_LIB_DIR}/aarch64-linux-gnu"
export LD_LIBRARY_PATH="${REPO_ROOT}/local_build/lib${ROS_LIB_DIR:+:${ROS_LIB_DIR}}${ROS_LIB_ARCH_DIR:+:${ROS_LIB_ARCH_DIR}}:${LD_LIBRARY_PATH:-}"

echo "[start] 启动 $BIN $*"
exec "$BIN" "$@"
