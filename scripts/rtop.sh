#!/bin/bash
# rl_real_JXG 线程优先级实时查看工具。
#
# 显示每个线程的:调度策略(OTHER/FIFO/RR)、RT 优先级、nice、CPU 亲和核、
# 最近运行核、线程状态,并按实时调度模型推断角色(imu/CAN/control/rl/gamepad/...)。
#
# 用法:
#   scripts/rtop.sh [刷新间隔秒] [PID]
#   scripts/rtop.sh          # 默认 1s 刷新,自动查找 rl_real_JXG
#   scripts/rtop.sh 0.5      # 0.5s 刷新
#
# 退出: Ctrl+C
#   策略列: FIFO=实时 SCHED_FIFO(看 RTPRIO)  OTHER=普通调度(实时性未启用时全是它)
set -u

INTERVAL="${1:-1}"
PID_ARG="${2:-}"

find_pid() {
    if [ -n "$PID_ARG" ]; then echo "$PID_ARG"; return; fi
    pgrep -x rl_real_JXG | head -1
}

# /proc/<pid>/stat 字段(跳过含空格的 "(comm)" 字段后,字段号整体减 2):
# 原字段 18=priority 19=nice 39=processor 40=policy(0 OTHER/1 FIFO/2 RR)
# cut 后位置: 16=priority 17=nice 37=processor 38=policy
print_frame() {
    local pid="$1"
    local capeff
    capeff=$(awk '/^CapEff:/{print $2}' /proc/$pid/status 2>/dev/null)
    local has_cap="-"
    if [ -n "$capeff" ] && [ $(( 0x$capeff & (1 << 23) )) -ne 0 ]; then has_cap="YES"; fi

    printf "\033[H\033[2J"  # 清屏
    echo "rl_real_JXG 线程实时调度监控  $(date '+%F %T')  (刷新间隔 ${INTERVAL}s, Ctrl+C 退出)"
    echo "PID=$pid  用户=$(ps -o user= -p $pid 2>/dev/null | tr -d ' ')  CAP_SYS_NICE=$has_cap  cgroup=$(awk -F: '$1=="0"{print $3}' /proc/$pid/cgroup 2>/dev/null)"
    echo
    printf "%-8s %-22s %-6s %-6s %-5s %-6s %-4s %s\n" \
        TID 角色 策略 RTPRIO NICE 亲和核 当前核 状态
    printf '%s\n' "--------------------------------------------------------------------------------"

    local n_fifo=0 n_other=0 n_rr=0
    local main_tid="$pid"
    for t in /proc/$pid/task/*; do
        local tid cpus comm pol prio nice cpu state rest rt
        tid=$(basename $t)
        cpus=$(awk '/Cpus_allowed_list/{print $2}' $t/status 2>/dev/null)
        comm=$(cat $t/comm 2>/dev/null)
        # 调度策略/内部优先级取自 /proc/<tid>/sched(各厂商内核 /proc/stat 字段排列不一致,不可靠):
        #   policy: 0=OTHER 1=FIFO 2=RR;prio 为内核内部值,RT 线程 rt_priority = 99 - prio
        pol=$(awk '/^policy[[:space:]]*:/{print $3; exit}' $t/sched 2>/dev/null)
        local kprio
        kprio=$(awk '/^prio[[:space:]]*:/{print $3; exit}' $t/sched 2>/dev/null)
        # nice/最近运行核/状态仍取 stat(cut 掉 "(comm)" 后:17=nice 37=processor)
        rest=$(cut -d')' -f2- $t/stat 2>/dev/null)
        # shellcheck disable=SC2086
        set -- $rest
        nice=${17}; cpu=${37}; state=$1

        local pol_name rt_show role rt
        case "${pol:-0}" in
            0) pol_name="OTHER"; rt_show="-"; rt=0; n_other=$((n_other+1));;
            1) pol_name="FIFO";  rt=$((99 - ${kprio:-0})); rt_show="$rt"; n_fifo=$((n_fifo+1));;
            2) pol_name="RR";    rt=$((99 - ${kprio:-0})); rt_show="$rt"; n_rr=$((n_rr+1));;
            *) pol_name="?${pol}"; rt_show="?"; rt=0;;
        esac

        # 角色推断(依据实时调度模型:优先级+亲和核)
        if [ "$tid" = "$main_tid" ]; then
            role="主线程"
        elif [ "$pol" = "1" ] || [ "$pol" = "2" ]; then
            case "$rt:$cpus" in
                90:4)      role="loop_imu 1kHz";;
                85:5)      role="CAN tx/rx";;
                80:4)      role="loop_control 200Hz";;
                70:6)      role="loop_rl 50Hz";;
                55:*)      role="loop_gamepad";;
                *)         role="实时(其他)";;
            esac
        else
            # 未提权时按亲和核推断(供实时调度未启用时参考)
            case "$cpus" in
                5) role="CAN tx/rx(未提权)";;
                4) role="imu/control(未提权)";;
                6) role="rl 推理(未提权)";;
                *) role="Web/音频/其他";;
            esac
        fi

        printf "%-8s %-22s %-6s %-6s %-5s %-6s %-4s %s\n" \
            "$tid" "$role" "$pol_name" "$rt_show" "$nice" "$cpus" "$cpu" "$state"
    done
    echo "--------------------------------------------------------------------------------"
    echo "汇总: FIFO 实时线程=$n_fifo  RR=$n_rr  普通 OTHER=$n_other  (设计目标 FIFO=11)"
    if [ "$n_fifo" -eq 0 ]; then
        echo "⚠ 没有 FIFO 线程:实时调度未启用。本机内核(RT_GROUP_SCHED)需 root 启动,"
        echo "  start.sh 会自动迁入根 cgroup: sudo bash scripts/start.sh"
    fi
}

PID=$(find_pid)
if [ -z "${PID:-}" ] || [ ! -e "/proc/$PID" ]; then
    echo "未找到运行中的 rl_real_JXG 进程。先启动: scripts/start.sh" >&2
    exit 1
fi

trap 'echo; exit 0' INT TERM
while true; do
    if [ ! -e "/proc/$PID" ]; then echo "进程 $PID 已退出。"; exit 0; fi
    print_frame "$PID"
    sleep "$INTERVAL"
done
