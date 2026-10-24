#!/bin/bash
# rl-real-jxg / jxg-web 机器狗服务统一维护脚本。
#
# 用法(需 root):
#   sudo bash scripts/service.sh [web] <命令>
#
# 前缀(可选):
#   (无前缀)  操作主程序服务 rl-real-jxg
#   web       操作独立 Web 服务 jxg-web
#
# 命令:
#   install    安装/更新 systemd 服务并立即启动(主程序+web 一起装)
#   start      启动服务(服务必须已 install)
#   stop       停止服务
#   restart    重启服务
#   status     查看运行状态
#   enable     开启开机自启(不改变当前运行状态)
#   disable    关闭开机自启(不改变当前运行状态)
#   logs [N]   查看最近 N 行日志并持续跟踪(默认 100 行,Ctrl+C 退出)
#
# 相关工具: scripts/rtop.sh  实时查看各线程调度优先级/绑核/CPU 占用
set -u

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 默认操作主程序;若首参数为 "web",则切到 web 服务。
SERVICE_NAME="rl-real-jxg"
PROC_NAME="rl_real_JXG"
EXPECTED_FIFO=11
TARGET_LABEL="主程序"

if [ "${1:-}" = "web" ]; then
    SERVICE_NAME="jxg-web"
    PROC_NAME="web_server"
    EXPECTED_FIFO=0
    TARGET_LABEL="Web"
    shift
fi

DST_UNIT="/etc/systemd/system/${SERVICE_NAME}.service"

# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

need_root() {
    if [ "$(id -u)" != "0" ]; then
        print_error "需要 root 权限,请在 root 终端执行:"
        echo "    sudo bash $0 ${SERVICE_NAME#rl-real-} ${1:-<命令>}"
        exit 1
    fi
}

need_systemd() {
    command -v systemctl >/dev/null || { print_error "系统未使用 systemd,无法管理服务"; exit 1; }
}

need_installed() {
    if [ ! -f "$DST_UNIT" ]; then
        print_error "服务尚未安装: $DST_UNIT 不存在"
        echo "    请先执行: sudo bash $0 install"
        exit 1
    fi
}

# 统计主进程的 SCHED_FIFO 线程数(仅主程序用)
fifo_count_of() {
    local pid="$1" n=0 t p
    for t in /proc/"$pid"/task/*; do
        p="$(awk '/^policy[[:space:]]*:/{print $3; exit}' "$t/sched" 2>/dev/null)"
        [ "$p" = "1" ] && n=$((n + 1))
    done
    echo "$n"
}

cmd_install() {
    # install 永远装主程序+web 两个服务(无论前缀是什么)
    local installer="${SCRIPT_DIR}/install_service.sh"
    [ -f "$installer" ] || { print_error "找不到安装脚本: $installer"; exit 1; }
    exec bash "$installer"
}

cmd_start() {
    need_installed
    if systemctl is-active --quiet "$SERVICE_NAME"; then
        print_warning "服务已在运行中(如需重新加载新编译的二进制请用 restart)"
        return 0
    fi
    print_info "启动 $SERVICE_NAME ..."
    systemctl start "$SERVICE_NAME" || { print_error "启动失败,请查看日志: sudo bash $0 logs 100"; exit 1; }
    sleep 2
    show_brief_status
}

cmd_stop() {
    need_installed
    if ! systemctl is-active --quiet "$SERVICE_NAME"; then
        print_warning "服务当前未运行"
        return 0
    fi
    print_info "停止 $SERVICE_NAME ..."
    systemctl stop "$SERVICE_NAME"
    # systemd 单元配置 TimeoutStopSec=10;确认进程已退出
    if pgrep -x "$PROC_NAME" >/dev/null 2>&1; then
        print_warning "进程仍未退出,等待 systemd 超时 SIGKILL ..."
        sleep 3
    fi
    if pgrep -x "$PROC_NAME" >/dev/null 2>&1; then
        print_error "进程残留: $(pgrep -x "$PROC_NAME" | tr '\n' ' '),可手动: sudo kill -9 <PID>"
        exit 1
    fi
    print_success "服务已停止"
}

cmd_restart() {
    need_installed
    print_info "重启 $SERVICE_NAME ..."
    systemctl restart "$SERVICE_NAME" || { print_error "重启失败,请查看日志: sudo bash $0 logs 100"; exit 1; }
    sleep 2
    show_brief_status
}

cmd_enable() {
    need_installed
    systemctl enable "$SERVICE_NAME" && print_success "已开启开机自启(当前运行状态不变)"
}

cmd_disable() {
    need_installed
    systemctl disable "$SERVICE_NAME" && print_success "已关闭开机自启(当前运行状态不变;停止当前服务请用 stop)"
}

cmd_status() {
    need_installed
    systemctl status "$SERVICE_NAME" --no-pager -l | sed -n '1,12p'
    echo
    show_brief_status
}

# 一行式关键健康指标
show_brief_status() {
    local active enabled pid
    active="$(systemctl is-active "$SERVICE_NAME" 2>/dev/null || echo unknown)"
    enabled="$(systemctl is-enabled "$SERVICE_NAME" 2>/dev/null || echo disabled)"
    pid="$(pgrep -x "$PROC_NAME" | head -1 || true)"
    echo -n "[$TARGET_LABEL] 运行状态: ${active}    开机自启: ${enabled}"
    if [ -n "$pid" ]; then
        if [ "$SERVICE_NAME" = "rl-real-jxg" ]; then
            local fifo cgroup
            fifo="$(fifo_count_of "$pid")"
            cgroup="$(awk -F: '$1=="0"{print $3}' "/proc/$pid/cgroup" 2>/dev/null)"
            echo "    PID=$pid    FIFO 实时线程=$fifo/$EXPECTED_FIFO    cgroup=$cgroup"
            [ "$fifo" -lt "$EXPECTED_FIFO" ] && print_warning "FIFO 线程数不足 $EXPECTED_FIFO,实时调度可能未生效,可用 scripts/rtop.sh 排查"
            [ "$cgroup" != "/" ] && print_warning "进程不在根 cgroup($cgroup),RT 调度预算可能为 0"
        else
            echo "    PID=$pid"
        fi
    else
        echo
        [ "$active" = "active" ] && print_warning "systemd 显示 active 但未找到进程,可能正在启动或已崩溃"
    fi
}

cmd_logs() {
    need_installed
    local n="${1:-100}"
    case "$n" in
        ''|*[!0-9]*) print_error "日志行数必须是正整数: $n"; exit 1;;
    esac
    print_info "跟踪 $SERVICE_NAME 最近 $n 行日志(Ctrl+C 退出) ..."
    journalctl -u "$SERVICE_NAME" -n "$n" -f
}

usage() {
    sed -n '2,24p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

# ── 主入口 ──────────────────────────────────────────────────────
need_systemd
CMD="${1:-}"
[ "$#" -gt 0 ] && shift || true
case "$CMD" in
    install)   need_root install;   cmd_install ;;
    start)     need_root start;     cmd_start ;;
    stop)      need_root stop;      cmd_stop ;;
    restart)   need_root restart;   cmd_restart ;;
    status)    need_root status;    cmd_status ;;
    enable)    need_root enable;    cmd_enable ;;
    disable)   need_root disable;   cmd_disable ;;
    logs)      need_root logs;      cmd_logs "$@" ;;
    ""|-h|--help|help) usage ;;
    *) print_error "未知命令: $CMD"; echo; usage; exit 1 ;;
esac
