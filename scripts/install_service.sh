#!/bin/bash
# rl-real-jxg 开机自启服务安装脚本(裸机直跑,非容器)。
#
# 完成动作:
#   1. 环境检查(root、主程序二进制、systemd);
#   2. 优雅停止手动启动的旧实例(等待 :8088 释放);
#   3. 安装 rl-real-jxg.service 到 /etc/systemd/system/;
#   4. daemon-reload + enable --now(开机自启并立即拉起);
#   5. 验证服务已激活,且 11 个关键线程确实运行在 SCHED_FIFO。
#
# 用法(需 root):
#   sudo bash scripts/install_service.sh
set -u

SERVICE_NAME="rl-real-jxg"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "${SCRIPT_DIR}/.." && pwd )"
SRC_UNIT="${REPO_ROOT}/${SERVICE_NAME}.service"
DST_UNIT="/etc/systemd/system/${SERVICE_NAME}.service"
BIN="${REPO_ROOT}/cmake_build/bin/rl_real_JXG"
WEB_PORT=8088
# 期望实时线程数: imu(1) + CAN 收发(8) + 控制环(1) + 推理(1) = 11
# (手柄线程为非硬实时输入,使用 SCHED_OTHER,不计入)
EXPECTED_FIFO=11

log()  { echo "[install] $*"; }
warn() { echo "[install] WARN: $*" >&2; }
die()  { echo "[install] ERROR: $*" >&2; exit 1; }

# ── 1. 环境检查 ─────────────────────────────────────────────────
[ "$(id -u)" = "0" ] || die "需要 root 权限,请用: sudo bash $0"
[ -f "$SRC_UNIT" ] || die "找不到 service 文件: $SRC_UNIT"
[ -x "$BIN" ]      || die "主程序不存在或不可执行: $BIN
        请先在仓库根目录执行: bash build.sh"
command -v systemctl >/dev/null || die "系统未使用 systemd,无法安装服务"

# ── 2. 停止旧实例(手动启动或旧服务),等待端口释放 ───────────────
stop_old_instance() {
    local pids
    pids="$(pgrep -x rl_real_JXG 2>/dev/null || true)"
    if systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
        log "停止已安装的旧服务 $SERVICE_NAME ..."
        systemctl stop "$SERVICE_NAME"
    fi
    if [ -n "$pids" ]; then
        log "发现手动启动的旧实例: $(echo $pids | tr '\n' ' '),发送 SIGTERM ..."
        for p in $pids; do kill -TERM "$p" 2>/dev/null || true; done
    fi
    local i
    for i in $(seq 1 10); do
        if ! ss -tln 2>/dev/null | grep -q ":${WEB_PORT} "; then
            log ":${WEB_PORT} 已释放"
            return 0
        fi
        sleep 1
    done
    # 兜底:程序在收尾时可能卡在 CAN 线程 join(passive 模式下控制环已停,不驱动电机,
    # SIGKILL 安全;串口 fd 由内核关闭)
    pids="$(pgrep -x rl_real_JXG 2>/dev/null || true)"
    if [ -n "$pids" ]; then
        warn "SIGTERM 后进程未退出(疑似卡在 CAN 线程 join): $(echo $pids | tr '\n' ' '),SIGKILL 强制结束"
        for p in $pids; do kill -KILL "$p" 2>/dev/null || true; done
        sleep 2
    fi
    if ss -tln 2>/dev/null | grep -q ":${WEB_PORT} "; then
        warn ":${WEB_PORT} 仍被占用,服务启动可能因端口冲突失败。占用进程:"
        ss -tlnp 2>/dev/null | grep ":${WEB_PORT} " || true
    else
        log ":${WEB_PORT} 已释放(SIGKILL)"
    fi
}

# ── 3/4. 安装并启动服务 ─────────────────────────────────────────
stop_old_instance

# 清除二进制上可能残留的旧版文件能力:本服务以 root + AmbientCapabilities 运行,
# 带 fE 的文件在 systemd 能力授予上下文中 execve 会 EACCES(服务退出码 126)。
if command -v setcap >/dev/null && getcap "$BIN" 2>/dev/null | grep -q cap_sys_nice; then
    log "清除二进制残留文件能力: $(getcap "$BIN")"
    setcap -r "$BIN"
fi

log "安装 $SRC_UNIT -> $DST_UNIT"
install -m 0644 "$SRC_UNIT" "$DST_UNIT" || die "写入 $DST_UNIT 失败"

log "systemctl daemon-reload"
systemctl daemon-reload
systemctl reset-failed "$SERVICE_NAME" 2>/dev/null || true

log "启用开机自启并立即启动 $SERVICE_NAME"
systemctl enable --now "$SERVICE_NAME" || die "启动服务失败,查看日志: journalctl -u $SERVICE_NAME -n 50"

# ── 5. 验证:服务 active + 11 个 FIFO 线程 ───────────────────────
fifo_count_of() {
    local pid="$1" n=0 t p
    for t in /proc/$pid/task/*; do
        p="$(awk '/^policy[[:space:]]*:/{print $3; exit}' "$t/sched" 2>/dev/null)"
        [ "$p" = "1" ] && n=$((n + 1))
    done
    echo "$n"
}

PID=""
for i in $(seq 1 20); do
    PID="$(pgrep -x rl_real_JXG | head -1 || true)"
    [ -n "$PID" ] && break
    sleep 1
done
[ -n "$PID" ] || die "服务已启动但未找到主进程,查看日志: journalctl -u $SERVICE_NAME -n 50"

# 等待全部工作线程拉起
FIFO=0
for i in $(seq 1 15); do
    FIFO="$(fifo_count_of "$PID")"
    [ "$FIFO" -ge "$EXPECTED_FIFO" ] && break
    sleep 1
done

CGROUP="$(awk -F: '$1=="0"{print $3}' /proc/$PID/cgroup 2>/dev/null)"
ACTIVE="$(systemctl is-active "$SERVICE_NAME")"

echo "--------------------------------------------------------------------------------"
log "服务状态: $ACTIVE    主进程 PID=$PID    cgroup=$CGROUP"
log "SCHED_FIFO 实时线程: $FIFO / 期望 $EXPECTED_FIFO"
if [ "$FIFO" -ge "$EXPECTED_FIFO" ] && [ "$CGROUP" = "/" ]; then
    log "安装成功,分级实时调度已生效(imu 90 / CAN 85 / control 80 / rl 70 / gamepad 55)。"
else
    warn "实时调度未完全生效。请检查:"
    warn "  journalctl -u $SERVICE_NAME -n 50"
    warn "  scripts/rtop.sh"
    exit 1
fi
echo "--------------------------------------------------------------------------------"
echo "常用命令:"
echo "  服务维护(启停/开机自启/日志): sudo bash ${REPO_ROOT}/scripts/service.sh status"
echo "  查看实时线程:                 ${REPO_ROOT}/scripts/rtop.sh"
