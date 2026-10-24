#!/bin/bash
# SocketCAN 接口配置脚本（sockettocan 变体，candleLight/gs_usb 五路一体盒）
# 用法: sudo ./scripts/can_setup.sh [bitrate]   默认 1000000
#
# 接口身份按【通道 MCU 序列号前缀】映射，与一体盒插在哪个 USB 口/总线无关
# （曾因上游线从 Bus3 换到 Bus1，物理端口 3-1.x→1-1.x，旧的端口硬编码映射
#   全部失配，导致真 FL 留在 can0(DOWN)、备用空通道被当 can4(RR)）。
#
# 序列号尾段 4348570220313938 为五路共用，前 8 位区分通道:
#   004B001F=can0(FL)  004B003C=can1(FR)  004C0026=can2(RL)
#   004C0029=can3(RR)  004B0041=can4(备用)
#
# 开机时内核按枚举序自动命名 can0~N，本脚本两阶段腾挪保证收敛为 can0~4。
set -u

BITRATE="${1:-1000000}"

# 目标映射: 通道序列号前缀 -> 接口名（与 99-candlelight-can.rules 一致）
declare -A SER2NAME=(
  ["004B001F"]="can0"   # FL
  ["004B003C"]="can1"   # FR
  ["004C0026"]="can2"   # RL
  ["004C0029"]="can3"   # RR
  ["004B0041"]="can4"   # 备用
)
# 固定顺序: FL FR RL RR 备用
ORDER=("004B001F" "004B003C" "004C0026" "004C0029" "004B0041")
LEGS=("FL" "FR" "RL" "RR" "备用")

# 读接口对应 USB 设备的序列号（gs_usb net 设备的 device 指向 USB 接口目录，
# serial 属性在其父设备上，逐级向上找）
serial_of() {
  local d
  d=$(readlink -f "/sys/class/net/$1/device" 2>/dev/null) || return 1
  while [ -n "$d" ] && [ "$d" != "/" ]; do
    if [ -r "$d/serial" ]; then cat "$d/serial" 2>/dev/null; return 0; fi
    d=$(dirname "$d")
  done
  return 1
}

# 读 USB 物理端口（仅用于日志核对）
port_of() {
  local dev
  dev=$(readlink -f "/sys/class/net/$1/device" 2>/dev/null) || return
  [ -n "$dev" ] && basename "$(dirname "$dev")"
}

# 序列号前缀 -> 目标接口名（前 8 位匹配）；非五路盒通道返回空
target_of_serial() {
  local ser="$1" key
  for key in "${!SER2NAME[@]}"; do
    case "$ser" in "$key"*) echo "${SER2NAME[$key]}"; return 0;; esac
  done
  return 1
}

echo "==== 阶段1: 按通道序列号腾挪错位的接口名 ===="
declare -A CURNAME   # 序列号前缀 -> 当前接口名
declare -A CURPORT   # 序列号前缀 -> USB 物理端口(日志用)
for ifname in $(ls /sys/class/net 2>/dev/null); do
  case "$ifname" in can*) ;; *) continue;; esac
  ser=$(serial_of "$ifname") || continue
  tgt=$(target_of_serial "$ser") || continue
  key="${ser:0:8}"
  CURNAME[$key]="$ifname"
  CURPORT[$key]=$(port_of "$ifname")
done

for idx in "${!ORDER[@]}"; do
  key="${ORDER[$idx]}"; leg="${LEGS[$idx]}"
  tgt="${SER2NAME[$key]}"
  cur="${CURNAME[$key]:-}"
  if [ -z "$cur" ]; then
    echo "[WARN] 通道 $key($leg) 无适配器（目标 $tgt）"
    continue
  fi
  if [ "$cur" = "$tgt" ]; then
    echo "[OK] $key($leg)@${CURPORT[$key]}: $cur 命名正确"
    continue
  fi
  ip link set "$cur" down 2>/dev/null || true
  # 中转名必须 <=15 字符(IFNAMSIZ-1): "mig_"+8位序列号 = 12
  if ip link set "$cur" name "mig_${key}" 2>/dev/null; then
    echo "[..] $key($leg)@${CURPORT[$key]}: $cur -> mig_${key}（中转）"
    CURNAME[$key]="mig_${key}"
  else
    echo "[ERR] $key($leg): $cur 中转改名失败"
  fi
done

# 阶段2: 中转名 -> 最终名（can1~5 已被腾空，无冲突）
for key in "${ORDER[@]}"; do
  cur="${CURNAME[$key]:-}"
  tgt="${SER2NAME[$key]}"
  [ -z "$cur" ] && continue
  [ "$cur" = "$tgt" ] && continue
  if ip link set "$cur" name "$tgt" 2>/dev/null; then
    echo "[OK] $key: $cur -> $tgt"
  else
    echo "[ERR] $key: $cur -> $tgt 改名失败"
  fi
done

echo "==== 阶段3: 配置 bitrate 并 UP ===="
i=0
for key in "${ORDER[@]}"; do
  leg="${LEGS[$i]}"; i=$((i+1))
  IF="${SER2NAME[$key]}"
  if ! ip link show "$IF" &>/dev/null; then
    echo "[WARN] 接口 $IF ($key/$leg) 不存在（检查适配器连接 / 重新插拔）"
    continue
  fi
  ip link set "$IF" down 2>/dev/null || true
  ip link set "$IF" type can bitrate "$BITRATE" restart-ms 100
  ip link set "$IF" txqueuelen 100
  ip link set "$IF" up
  echo "[OK] $IF ($key/$leg): bitrate=$BITRATE restart-ms=100 txqueuelen=100 up"
done

echo "---- 接口状态 ----"
for IF in can0 can1 can2 can3 can4; do
    ip -details link show "$IF" 2>/dev/null | grep -E '^[0-9]+:|can state|bitrate' || true
done
