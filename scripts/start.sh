#!/bin/bash
# rl_real_JXG 主程序启动脚本(裸机直跑,非容器)。
#
# 职责:
#   1. 等待 4 路 USB-CAN 串口设备(ttycan1~ttycan4,udev 软链到 /dev/ttyACM*)就绪;
#   2. exec 启动主程序 rl_real_JXG(内置 WebSocket 服务器,同端口托管 Web 调试台 :8088)。
#
# 环境变量:
#   CAN_UP_WAIT   等待 CAN 设备就绪的超时秒数(默认 30,超时仅告警不阻塞)
#   CAN_IFACES    参与等待的设备节点列表(默认 "/dev/ttycan1 /dev/ttycan2 /dev/ttycan3 /dev/ttycan4")
#
# 用法:
#   scripts/start.sh                 # 默认 wheel 模式
#   scripts/start.sh extra args      # 额外参数原样透传给 rl_real_JXG(如 --web-port 8088)
set -u

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "${SCRIPT_DIR}/.." && pwd )"
BIN="${REPO_ROOT}/cmake_build/bin/rl_real_JXG"
WEB_ROOT="${REPO_ROOT}/web"

CAN_UP_WAIT="${CAN_UP_WAIT:-30}"
CAN_IFACES="${CAN_IFACES:-/dev/ttycan1 /dev/ttycan2 /dev/ttycan3 /dev/ttycan4}"

# USB-CAN 为 USB-CDC 串口(cdc_acm),udev 规则 99-usb-can.rules 按物理端口建软链。
# 设备节点为字符设备(或指向它的软链)即视为就绪。
can_is_up() {
    [ -c "$1" ] || { [ -e "$1" ] && [ -c "$(readlink -f "$1")" ]; }
}

wait_can_up() {
    local missing elapsed=0
    while true; do
        missing=""
        for c in $CAN_IFACES; do
            can_is_up "$c" || missing="$missing $c"
        done
        if [ -z "$missing" ]; then
            echo "[start] CAN 设备全部就绪:$CAN_IFACES"
            return 0
        fi
        if [ "$elapsed" -ge "$CAN_UP_WAIT" ]; then
            echo "[start] WARN: 等待 CAN 设备超时(${CAN_UP_WAIT}s),以下节点仍未就绪:$missing"
            echo "[start] 继续启动主程序(相关电机可能不可用)"
            return 1
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
}

if [ ! -x "$BIN" ]; then
    echo "[start] ERROR: 主程序不存在或不可执行: $BIN" >&2
    echo "[start] 请先在仓库根目录执行: bash build.sh" >&2
    exit 1
fi

# ── 实时调度环境准备 ─────────────────────────────────────────────
# 本机厂商内核开启了 CONFIG_RT_GROUP_SCHED 且为纯 cgroup v2(无 cpu.rt.max 接口),
# 除根 cgroup 外的所有子 cgroup 的 RT runtime 均为 0 —— 即使是 root + CAP_SYS_NICE,
# 在 user.slice / system.slice 等子 cgroup 中 sched_setscheduler(SCHED_FIFO) 也会 EPERM。
# 解决办法:以 root 启动时,把本 shell 迁入根 cgroup(之后 exec 的主程序及全部线程继承)。
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
    # CapEff 第 23 位 = CAP_SYS_NICE(见 capabilities(7))
    cap_eff_hex="$(awk '/^CapEff:/{print $2}' /proc/self/status)"
    if [ -n "$cap_eff_hex" ] && [ $(( 0x${cap_eff_hex} & (1 << 23) )) -eq 0 ]; then
        echo "[start] WARN: 非 root 且无 CAP_SYS_NICE,控制环将以普通调度运行(实时性下降)。"
        echo "[start]       本机确保实时优先级的启动方式(root,会自动迁入根 cgroup):"
        echo "[start]       sudo bash $(readlink -f "$0")"
    fi
fi

echo "[start] 等待 CAN 设备节点就绪(最多 ${CAN_UP_WAIT}s): $CAN_IFACES"
wait_can_up || true

cd "$REPO_ROOT"
echo "[start] 启动 $BIN wheel --web-root $WEB_ROOT $*"
exec "$BIN" wheel --web-root "$WEB_ROOT" "$@"
