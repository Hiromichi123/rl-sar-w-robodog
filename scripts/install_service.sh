#!/bin/bash
# 开机自启服务安装脚本(裸机直跑,非容器)。
#
# 安装两个 systemd 服务:
#   1. rl-real-jxg   —— 主程序 rl_real_JXG(CAN/控制环/推理/ROS2 话题)
#   2. jxg-web       —— 独立 web_server(WS/HTTP/WebSSH 桥,重启主程序时会话不中断)
#
# 完成动作:
#   - 环境检查(root、二进制、systemd);
#   - 优雅停止主程序/web 旧实例(等待进程退出);
#   - 安装 udev 规则 99-candlelight-can.rules(SocketCAN can0~can4 固定命名);
#   - 禁用与本项目冲突的旧服务(rl-sar-can-setup / rl-sar-sockettocan);
#   - 立即执行 can_setup.sh 配置 SocketCAN 接口(主程序已停止,操作安全);
#   - 渲染 @REPO_ROOT@ 模板(按本脚本所在目录自动适配 vN 路径);
#   - 安装两个 unit -> /etc/systemd/system/,daemon-reload + enable --now;
#   - 验证主程序 11 个 SCHED_FIFO 实时线程 + web 进程存活。
#
# 用法(需 root):
#   sudo bash scripts/install_service.sh
set -u

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "${SCRIPT_DIR}/.." && pwd )"

# 主程序服务
MAIN_SVC="rl-real-jxg"
MAIN_BIN="${REPO_ROOT}/cmake_build/bin/rl_real_JXG"
MAIN_UNIT_SRC="${REPO_ROOT}/${MAIN_SVC}.service"
MAIN_EXPECTED_FIFO=11   # imu(1)+CAN收发(8)+控制环(1)+推理(1)=11

# Web 服务
WEB_SVC="jxg-web"
WEB_BIN="${REPO_ROOT}/cmake_build/bin/web_server"
WEB_UNIT_SRC="${REPO_ROOT}/${WEB_SVC}.service"

log()  { echo "[install] $*"; }
warn() { echo "[install] WARN: $*" >&2; }
die()  { echo "[install] ERROR: $*" >&2; exit 1; }

# ── 1. 环境检查 ─────────────────────────────────────────────────
[ "$(id -u)" = "0" ] || die "需要 root 权限,请用: sudo bash $0"
command -v systemctl >/dev/null || die "系统未使用 systemd,无法安装服务"
[ -f "$MAIN_UNIT_SRC" ] || die "找不到主程序 service 模板: $MAIN_UNIT_SRC"
[ -f "$WEB_UNIT_SRC" ]  || die "找不到 web service 模板: $WEB_UNIT_SRC(请确认 v7 已含 jxg-web.service)"
[ -x "$MAIN_BIN" ] || die "主程序不存在或不可执行: $MAIN_BIN
        请先执行: cmake src/rl_sar/ -B cmake_build -DUSE_CMAKE=ON && cmake --build cmake_build -j8"
[ -x "$WEB_BIN" ]  || die "web_server 不存在或不可执行: $WEB_BIN
        请先执行: cmake src/rl_sar/ -B cmake_build -DUSE_CMAKE=ON && cmake --build cmake_build -j8"

# ── 2. 停止旧实例 ───────────────────────────────────────────────
stop_by_pid() {
    # $1=进程名, $2=服务名(可为空)
    local proc_name="$1" svc_name="${2:-}" pids
    if [ -n "$svc_name" ] && systemctl is-active --quiet "$svc_name" 2>/dev/null; then
        log "停止已安装的旧服务 $svc_name ..."
        systemctl stop "$svc_name"
    fi
    pids="$(pgrep -x "$proc_name" 2>/dev/null || true)"
    if [ -n "$pids" ]; then
        log "发现手动启动的 $proc_name 旧实例: $(echo $pids | tr '\n' ' '),发送 SIGTERM ..."
        for p in $pids; do kill -TERM "$p" 2>/dev/null || true; done
        local i
        for i in $(seq 1 10); do
            pgrep -x "$proc_name" >/dev/null 2>&1 || { log "$proc_name 已退出"; return 0; }
            sleep 1
        done
        # 兜底:rl_real_JXG 可能卡在 CAN 线程 join(passive 不驱动电机,SIGKILL 安全)
        pids="$(pgrep -x "$proc_name" 2>/dev/null || true)"
        [ -n "$pids" ] && { warn "$proc_name 未退出,SIGKILL 强制结束"; for p in $pids; do kill -KILL "$p" 2>/dev/null || true; done; sleep 2; }
        pgrep -x "$proc_name" >/dev/null 2>&1 && warn "$proc_name 仍残留,后续启动可能因单实例锁跳过" || log "$proc_name 已全部清理"
    fi
}

stop_by_pid rl_real_JXG "$MAIN_SVC"
stop_by_pid web_server   "$WEB_SVC"

# ── 3. 清除主程序二进制残留文件能力 ─────────────────────────────
# 本服务以 root 运行,带 fE 的文件在 systemd 能力上下文 execve 会 EACCES(退出码 126)。
if command -v setcap >/dev/null && getcap "$MAIN_BIN" 2>/dev/null | grep -q cap_sys_nice; then
    log "清除主程序二进制残留文件能力: $(getcap "$MAIN_BIN")"
    setcap -r "$MAIN_BIN"
fi

# ── 3.5 安装 udev 规则(SocketCAN 接口命名) ──────────────────────
# 按 candleLight 通道序列号固定 can0~can4 命名(can0=FL...can3=RR can4=备用),
# 避免换 USB 口后接口名漂移。USB2CAN(ttycan)方案不依赖本规则,缺失时仅告警。
UDEV_RULES_SRC="${SCRIPT_DIR}/99-candlelight-can.rules"
UDEV_RULES_DST="/etc/udev/rules.d/99-candlelight-can.rules"
if [ -f "$UDEV_RULES_SRC" ]; then
    if ! cmp -s "$UDEV_RULES_SRC" "$UDEV_RULES_DST" 2>/dev/null; then
        install -m 0644 "$UDEV_RULES_SRC" "$UDEV_RULES_DST" || die "安装 udev 规则失败"
        log "安装 udev 规则: $UDEV_RULES_DST"
        udevadm control --reload-rules || true
    else
        log "udev 规则已是最新: $UDEV_RULES_DST"
    fi
else
    warn "未找到 udev 规则: $UDEV_RULES_SRC(如使用默认 can0~3 命名可忽略)"
fi

# ── 3.6 清理与本项目冲突的旧服务 ────────────────────────────────
# 旧 sockettocan 容器栈的残留服务指向已废弃路径,留着会在开机时反复失败,
# 且 rl-sar-can-setup 与本项目 start.sh 内置的 can_setup 调用职责重叠。
for legacy_svc in rl-sar-can-setup rl-sar-sockettocan; do
    if systemctl list-unit-files "${legacy_svc}.service" 2>/dev/null | grep -q "${legacy_svc}"; then
        log "禁用冲突旧服务: ${legacy_svc}"
        systemctl disable --now "${legacy_svc}.service" 2>/dev/null || true
    fi
done

# ── 3.7 立即配置 SocketCAN 接口 ─────────────────────────────────
# v7.7: 主程序启动时由 C++ CanSetup::Run() 自动完成接口配置
# 此处保留一次 shell 预配置（兼容旧 udev 命名混乱场景），失败不阻塞安装
CAN_SETUP="${SCRIPT_DIR}/can_setup.sh"
if [ -x "$CAN_SETUP" ] || [ -f "$CAN_SETUP" ]; then
    log "预配置 CAN 接口（C++ 将在启动时复核）..."
    bash "$CAN_SETUP" || warn "can_setup.sh 预配置有告警，启动时将自动重试"
else
    log "can_setup.sh 未找到，跳过预配置（启动时由 C++ 自动完成）"
fi

# ── 4. 渲染并安装 unit(通用函数) ───────────────────────────────
install_unit() {
    # $1=服务名, $2=模板路径, $3=期望二进制(仅用于日志)
    local svc="$1" src="$2" bin="$3"
    local dst="/etc/systemd/system/${svc}.service"
    local rendered escaped
    rendered="$(mktemp "/tmp/${svc}.service.XXXXXX")" || die "创建临时文件失败"
    escaped="$(printf '%s' "$REPO_ROOT" | sed 's/[&/\]/\\&/g')"
    sed "s/@REPO_ROOT@/${escaped}/g" "$src" > "$rendered" || die "渲染 unit 模板失败: $src"
    grep -q '@REPO_ROOT@' "$rendered" && die "unit 模板存在未替换的 @REPO_ROOT@: $src"
    log "安装 $svc(渲染自 $src, REPO_ROOT=$REPO_ROOT, bin=$(basename "$bin")) -> $dst"
    install -m 0644 "$rendered" "$dst" || die "写入 $dst 失败"
    rm -f "$rendered"
}

install_unit "$MAIN_SVC" "$MAIN_UNIT_SRC" "$MAIN_BIN"
install_unit "$WEB_SVC"  "$WEB_UNIT_SRC"  "$WEB_BIN"

log "systemctl daemon-reload"
systemctl daemon-reload
systemctl reset-failed "$MAIN_SVC" "$WEB_SVC" 2>/dev/null || true

# ── 5. 启用并启动两个服务 ───────────────────────────────────────
# 先起 web(不依赖主程序),再起主程序;两者开机均自启。
log "启用并启动 $WEB_SVC"
systemctl enable --now "$WEB_SVC" || die "启动 web 服务失败: journalctl -u $WEB_SVC -n 50"

log "启用并启动 $MAIN_SVC"
systemctl enable --now "$MAIN_SVC" || die "启动主程序服务失败: journalctl -u $MAIN_SVC -n 50"

# ── 6. 验证 ─────────────────────────────────────────────────────
fifo_count_of() {
    local pid="$1" n=0 t p
    for t in /proc/"$pid"/task/*; do
        p="$(awk '/^policy[[:space:]]*:/{print $3; exit}' "$t/sched" 2>/dev/null)"
        [ "$p" = "1" ] && n=$((n + 1))
    done
    echo "$n"
}

# 主程序:等待进程拉起 + 11 个 FIFO 线程
MAIN_PID=""
for i in $(seq 1 20); do
    MAIN_PID="$(pgrep -x rl_real_JXG | head -1 || true)"
    [ -n "$MAIN_PID" ] && break
    sleep 1
done
[ -n "$MAIN_PID" ] || die "主程序服务已启动但未找到进程: journalctl -u $MAIN_SVC -n 50"

FIFO=0
for i in $(seq 1 15); do
    FIFO="$(fifo_count_of "$MAIN_PID")"
    [ "$FIFO" -ge "$MAIN_EXPECTED_FIFO" ] && break
    sleep 1
done
MAIN_CGROUP="$(awk -F: '$1=="0"{print $3}' /proc/$MAIN_PID/cgroup 2>/dev/null)"
MAIN_ACTIVE="$(systemctl is-active "$MAIN_SVC")"

# web:等待进程存活
WEB_PID=""
for i in $(seq 1 15); do
    WEB_PID="$(pgrep -x web_server | head -1 || true)"
    [ -n "$WEB_PID" ] && break
    sleep 1
done
WEB_ACTIVE="$(systemctl is-active "$WEB_SVC")"

echo "--------------------------------------------------------------------------------"
log "主程序[$MAIN_SVC]  active=$MAIN_ACTIVE  PID=$MAIN_PID  FIFO=$FIFO/$MAIN_EXPECTED_FIFO  cgroup=$MAIN_CGROUP"
log "Web[$WEB_SVC]       active=$WEB_ACTIVE   PID=$WEB_PID"
echo "--------------------------------------------------------------------------------"

OK=1
if [ "$FIFO" -ge "$MAIN_EXPECTED_FIFO" ] && [ "$MAIN_CGROUP" = "/" ]; then
    log "主程序实时调度已生效(imu 90 / CAN 85 / control 80 / rl 70)。"
else
    warn "主程序实时调度未完全生效,请检查: journalctl -u $MAIN_SVC -n 50 ; scripts/rtop.sh"
    OK=0
fi
[ -n "$WEB_PID" ] || { warn "web_server 未拉起,请检查: journalctl -u $WEB_SVC -n 50"; OK=0; }

[ "$OK" = "1" ] && log "安装成功:主程序 + Web 均已运行并开机自启。" || exit 1

echo "常用命令:"
echo "  主程序维护: sudo bash ${REPO_ROOT}/scripts/service.sh status"
echo "  Web 维护:   sudo bash ${REPO_ROOT}/scripts/service.sh web status"
echo "  实时线程:   ${REPO_ROOT}/scripts/rtop.sh"
