#!/usr/bin/env python3
"""
BMS BLE 上位机
目标设备: HCYPZY26HYDA922100
应用层协议: 中国铁塔 BMS Modbus RTU over BLE (透明传输)

用法:
  python3 bms_ble_tool.py scan
  python3 bms_ble_tool.py probe
  python3 bms_ble_tool.py query --type all            # 主动查询实时/序列号/告警
  python3 bms_ble_tool.py monitor --poll              # 周期轮询实时数据
  python3 bms_ble_tool.py monitor --poll --poll-type all --poll-interval 2
  python3 bms_ble_tool.py monitor --log bms.log
  python3 bms_ble_tool.py read  --uuid <UUID>
  python3 bms_ble_tool.py write --uuid <UUID> --data 01030000001D85C3

依赖: bluez + bleak
  sudo apt install -y bluez python3-pip
  pip3 install bleak
"""
import argparse
import asyncio
import logging
import struct
import sys
from datetime import datetime

try:
    from bleak import BleakClient, BleakScanner
    from bleak.exc import BleakError
except ImportError:
    print("[-] 未安装 bleak: pip3 install bleak", file=sys.stderr)
    sys.exit(1)


# ====== 配置 ======
TARGET_NAME = "HCYPZY26HYDA922100"
DEFAULT_SLAVE = 0x01  # BMS 默认从机地址


# ====== Modbus CRC16 (移植自文档 C 代码) ======
def crc16_modbus(data: bytes) -> int:
    """Modbus RTU CRC16: 多项式 0xA001, 初值 0xFFFF, 结果高低字节交换"""
    c = 0xFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if (c & 1) else (c >> 1)
    # 文档要求 (c<<8)|(c>>8) 即交换高低字节
    return ((c << 8) | (c >> 8)) & 0xFFFF


def add_crc(frame: bytes) -> bytes:
    """在帧尾追加 CRC16（文档为大端存放，高字节在前）"""
    return frame + struct.pack(">H", crc16_modbus(frame))


def verify_crc(frame: bytes) -> bool:
    """校验整帧 CRC（大端）"""
    if len(frame) < 4:
        return False
    payload, recv = frame[:-2], struct.unpack(">H", frame[-2:])[0]
    return crc16_modbus(payload) == recv


# ====== 帧构建 ======
def build_read_holding(slave: int, start: int, count: int) -> bytes:
    """功能码 0x03: 读保持寄存器"""
    return add_crc(bytes([slave, 0x03,
                          (start >> 8) & 0xFF, start & 0xFF,
                          (count >> 8) & 0xFF, count & 0xFF]))


def build_read_coils(slave: int, start: int, count: int) -> bytes:
    """功能码 0x01: 读线圈状态"""
    return add_crc(bytes([slave, 0x01,
                          (start >> 8) & 0xFF, start & 0xFF,
                          (count >> 8) & 0xFF, count & 0xFF]))


# 预构建查询帧（按文档示例）
FRAME_REALTIME = build_read_holding(DEFAULT_SLAVE, 0x0000, 0x001D)  # 实时数据 29 寄存器
FRAME_SERIAL   = build_read_holding(DEFAULT_SLAVE, 0x03E8, 0x000D)  # 序列号 13 寄存器
FRAME_ALARM    = build_read_coils   (DEFAULT_SLAVE, 0x0000, 0x0034)  # 告警 52 线圈


# ====== 实时数据寄存器表解析 ======
def _to_signed(v: int) -> int:
    return v - 0x10000 if v & 0x8000 else v


def parse_realtime(payload: bytes) -> str:
    """29 寄存器 = 58 字节负载"""
    if len(payload) < 58:
        return f"负载长度不足({len(payload)}/58)"
    regs = [struct.unpack(">H", payload[i:i+2])[0] for i in range(0, 58, 2)]
    total_v   = regs[0] / 100.0
    idx       = regs[1]
    soc       = regs[2]
    remain    = regs[3] / 100.0
    current   = _to_signed(regs[4]) / 100.0
    avg_curr  = _to_signed(regs[5]) / 100.0
    t1, t2, t3 = _to_signed(regs[6]), _to_signed(regs[7]), _to_signed(regs[8])
    cells = [regs[9+i] / 1000.0 for i in range(20)]  # 20 串电芯
    vmin, vmax = min(cells), max(cells)
    lines = [
        f"总电压    : {total_v:7.2f} V",
        f"索引/计数 : {idx}",
        f"SOC       : {soc:4d} %",
        f"剩余容量  : {remain:7.2f} Ah",
        f"电流      : {current:+7.2f} A",
        f"平均电流  : {avg_curr:+7.2f} A",
        f"温度1/2/3 : {t1} / {t2} / {t3} °C",
        f"电芯电压(20串, 单位 V):",
    ]
    for i in range(0, 20, 5):
        row = "  ".join(f"C{j+1:02d}={cells[j]:.3f}" for j in range(i, min(i+5, 20)))
        lines.append("    " + row)
    lines.append(f"  最高 {vmax:.3f}V  最低 {vmin:.3f}V  压差 {(vmax-vmin)*1000:.0f}mV")
    return "\n".join(lines)


def parse_serial(payload: bytes) -> str:
    """13 寄存器 = 26 字节 ASCII（每寄存器 2 字符）"""
    if len(payload) < 26:
        return f"负载长度不足({len(payload)}/26)"
    try:
        sn = payload.decode("ascii", errors="replace").rstrip("\x00").strip()
        return f"BMS 序列号: {sn}\n  原始: {payload.hex()}"
    except Exception:
        return f"原始: {payload.hex()}"


# 告警位含义（基于中国铁塔常见 BMS，未在文档全部列出，需要实际校对）
ALARM_BITS = [
    "总压过高", "总压过低", "充电过流", "放电过流",
    "短路", "温度过高", "温度过低", "单体过高",
    "单体过低", "压差过大", "SOC过低", "剩余容量过低",
    "充电过温", "放电过温", "充电欠压", "放电欠压",
    "均衡失败", "通信失败", "继电器故障", "预充失败",
    "过充告警", "过放告警", "充电过流告警", "放电过流告警",
    "温度1故障", "温度2故障", "温度3故障",
    "电压采样1故障", "电压采样2故障", "电流采样故障",
    "电池反接", "断线",
    "充电Mos故障", "放电Mos故障", "预充Mos故障", "加热Mos故障",
    "风扇故障", "EEPROM故障", "RTC故障", "看门狗故障",
    "系统故障", "在线故障", "均衡1", "均衡2",
    "均衡3", "均衡4", "充电限流", "放电限流",
    "充电限压", "放电限压", "充电限温", "放电限温",
]


def parse_alarm(payload: bytes, byte_count: int) -> str:
    bits = payload[:byte_count]
    active = []
    for i in range(min(52, byte_count * 8)):
        if bits[i // 8] & (1 << (i % 8)):
            name = ALARM_BITS[i] if i < len(ALARM_BITS) else f"告警位{i+1}"
            active.append(f"[{i+1:02d}] {name}")
    return "告警状态: 正常" if not active else f"告警状态: {len(active)} 个告警\n  " + "\n  ".join(active)


def parse_modbus_response(frame: bytes) -> str:
    """完整 Modbus 响应帧解析（含 CRC 校验）"""
    if len(frame) < 5:
        return f"帧过短({len(frame)}B): {frame.hex()}"
    if not verify_crc(frame):
        return f"CRC 校验失败: {frame.hex()}"
    slave, fc = frame[0], frame[1]
    if fc & 0x80:
        return f"[Modbus 异常] 从机=0x{slave:02x} 功能码=0x{fc&0x7f:02x} 异常码=0x{frame[2]:02x}"
    if fc == 0x03:
        bc = frame[2]
        payload = frame[3:3+bc]
        if bc == 26:
            return "==== 序列号 ====\n" + parse_serial(payload)
        if bc == 58:
            return "==== 实时数据 ====\n" + parse_realtime(payload)
        return f"[读寄存器] 字节数={bc} 负载={payload.hex()}"
    if fc == 0x01:
        bc = frame[2]
        payload = frame[3:3+bc]
        if bc == 7:
            return "==== 告警状态 ====\n" + parse_alarm(payload, bc)
        return f"[读线圈] 字节数={bc} 位图={payload.hex()}"
    return f"[未识别] 从机=0x{slave:02x} 功能码=0x{fc:02x} 数据={frame.hex()}"


# ====== 通用工具 ======
def ts() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def hexdump(data: bytes, prefix: str = "  ") -> str:
    lines = []
    for i in range(0, len(data), 16):
        chunk = data[i:i+16]
        hex_part = " ".join(f"{b:02x}" for b in chunk)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        lines.append(f"{prefix}{i:08x}  {hex_part:<48}  |{ascii_part}|")
    return "\n".join(lines)


# 解析钩子（保留旧接口，给 monitor 的 notify 回调用）
def parse_frame(uuid: str, data: bytes):
    return parse_modbus_response(data)


# ====== BLE 特征自动识别（透明传输模式） ======
def find_write_chars(svcs) -> list:
    out = []
    for svc in svcs:
        for ch in svc.characteristics:
            if "write" in ch.properties or "write-without-response" in ch.properties:
                out.append(ch)
    return out


def find_notify_chars(svcs) -> list:
    out = []
    for svc in svcs:
        for ch in svc.characteristics:
            if "notify" in ch.properties or "indicate" in ch.properties:
                out.append(ch)
    return out


# ====== 地址解析 ======
async def resolve_address(args) -> str:
    if args.address:
        return args.address
    name = args.name or TARGET_NAME
    print(f"[*] 通过名称 '{name}' 查找设备（{args.scan_timeout}s）...")
    devices = await BleakScanner.discover(timeout=args.scan_timeout)
    for d in devices:
        if d.name == name:
            print(f"[+] 找到: {d.address}  RSSI={d.rssi}")
            return d.address
    for d in devices:
        if d.name and d.name.startswith(name):
            print(f"[+] 前缀匹配: {d.address}  name={d.name}")
            return d.address
    print(f"[-] 未找到 '{name}'，可用 --address <MAC> 直接指定")
    return None


# ====== 子命令 ======
async def cmd_scan(args):
    print(f"[*] 扫描 BLE 设备（{args.timeout}s）...")
    devices = await BleakScanner.discover(timeout=args.timeout)
    for d in devices:
        name = d.name or "(unknown)"
        mark = "  <=== 目标" if (name == TARGET_NAME or name.startswith(TARGET_NAME)) else ""
        print(f"  {d.address}  RSSI={d.rssi:4d}  name={name}{mark}")
    print(f"\n[*] 共 {len(devices)} 个设备")


async def cmd_probe(args):
    addr = await resolve_address(args)
    if not addr:
        return
    print(f"[*] 连接 {addr} ...")
    async with BleakClient(addr, timeout=20.0) as client:
        print(f"[+] 已连接 MTU={client.mtu_size}")
        svcs = await client.get_services()
        writes = find_write_chars(svcs)
        notifies = find_notify_chars(svcs)
        print(f"\n{'='*60}\n服务/特征清单\n{'='*60}")
        for svc in svcs:
            print(f"\n[Service] {svc.uuid}  ({svc.description or ''})")
            for ch in svc.characteristics:
                props = ",".join(sorted(ch.properties))
                tag = []
                if ch in writes: tag.append("WRITE")
                if ch in notifies: tag.append("NOTIFY")
                tag_str = f"  <{' '.join(tag)}>" if tag else ""
                print(f"  [Char] {ch.uuid}  props=[{props}]  handle={ch.handle}{tag_str}")
                if "read" in ch.properties and not args.no_read_probe:
                    try:
                        data = await client.read_gatt_char(ch.uuid)
                        if data:
                            print(f"         当前值({len(data)}B):")
                            print(hexdump(bytes(data), "           "))
                    except Exception as e:
                        print(f"         (读失败: {e})")
        print(f"\n[*] 可写特征 {len(writes)} 个（发送 Modbus 请求）")
        print(f"[*] notify 特征 {len(notifies)} 个（接收 Modbus 响应）")
        if writes and notifies:
            print(f"\n[+] 透明传输模式成立:")
            print(f"    写入特征: {writes[0].uuid}")
            print(f"    通知特征: {notifies[0].uuid}")
            print(f"    下一步: python3 bms_ble_tool.py query --type all")
            print(f"    或:     python3 bms_ble_tool.py monitor --poll")


def make_notify_handler(uuid_str, log_fp):
    def handler(_sender, data: bytearray):
        u = uuid_str or "(unknown)"
        print(f"[{ts()}] <- {u}  ({len(data)}B)")
        print(hexdump(bytes(data)))
        parsed = parse_modbus_response(bytes(data))
        if parsed:
            print(parsed)
        if log_fp:
            log_fp.write(f"{ts()}\t{u}\t{bytes(data).hex()}\n")
            log_fp.flush()
    return handler


async def cmd_monitor(args):
    addr = await resolve_address(args)
    if not addr:
        return
    log_fp = open(args.log, "a") if args.log else None
    if log_fp:
        log_fp.write(f"\n# === {ts()} monitor start target={addr} ===\n")
        print(f"[+] 日志追加写入 {args.log}")
    print(f"[*] 连接 {addr} ...")
    try:
        async with BleakClient(addr, timeout=20.0) as client:
            print(f"[+] 已连接 MTU={client.mtu_size}")
            svcs = await client.get_services()
            notifies = find_notify_chars(svcs)
            writes = find_write_chars(svcs)
            for ch in notifies:
                await client.start_notify(ch.uuid, make_notify_handler(ch.uuid, log_fp))
                print(f"[+] 订阅 {ch.uuid}")
            if not notifies:
                print("[-] 无 notify 特征")
                return
            if args.poll:
                if not writes:
                    print("[-] --poll 需要可写特征")
                    return
                wch = writes[0]
                print(f"[+] 轮询模式: 写入特征 {wch.uuid}")
                frames = []
                if args.poll_type in ("all", "realtime"):
                    frames.append(("realtime", FRAME_REALTIME))
                if args.poll_type in ("all", "alarm"):
                    frames.append(("alarm", FRAME_ALARM))
                if args.poll_type in ("all", "serial"):
                    frames.append(("serial", FRAME_SERIAL))
                print(f"[*] 轮询周期 {args.poll_interval}s，Ctrl+C 退出\n")
                idx = 0
                while True:
                    name, frame = frames[idx % len(frames)]
                    print(f"[{ts()}] -> {name}  ({frame.hex()})")
                    try:
                        await client.write_gatt_char(wch.uuid, frame, response=True)
                    except Exception as e:
                        print(f"[-] 写入失败: {e}")
                    idx += 1
                    await asyncio.sleep(args.poll_interval)
            else:
                print(f"[*] 监听中（Ctrl+C 退出）...\n")
                while True:
                    await asyncio.sleep(1.0)
    except (KeyboardInterrupt, asyncio.CancelledError):
        print("\n[*] 停止...")
    finally:
        if log_fp:
            log_fp.close()
            print(f"[+] 日志已保存 {args.log}")


async def cmd_query(args):
    addr = await resolve_address(args)
    if not addr:
        return
    print(f"[*] 连接 {addr} ...")
    async with BleakClient(addr, timeout=20.0) as client:
        print(f"[+] 已连接 MTU={client.mtu_size}")
        svcs = await client.get_services()
        writes = find_write_chars(svcs)
        notifies = find_notify_chars(svcs)
        if not writes:
            print("[-] 无可写特征，无法发送 Modbus 请求")
            return
        wch = writes[0]
        print(f"[+] 写入特征: {wch.uuid}")
        nch = notifies[0] if notifies else None
        if nch:
            print(f"[+] 通知特征: {nch.uuid}")
        else:
            print("[!] 无 notify 特征，无法接收响应")
            return

        # 订阅 notify 等响应
        resp_buf = {"data": None}

        def handler(_s, data: bytearray):
            resp_buf["data"] = bytes(data)
        await client.start_notify(nch.uuid, handler)

        queries = []
        if args.type in ("realtime", "all"):
            queries.append(("实时数据", FRAME_REALTIME))
        if args.type in ("serial", "all"):
            queries.append(("序列号", FRAME_SERIAL))
        if args.type in ("alarm", "all"):
            queries.append(("告警状态", FRAME_ALARM))

        for name, frame in queries:
            print(f"\n{'='*60}\n[{ts()}] 查询 {name}\n{'='*60}")
            print(f"请求帧: {frame.hex()}")
            resp_buf["data"] = None
            try:
                await client.write_gatt_char(wch.uuid, frame, response=True)
            except Exception as e:
                print(f"[-] 写入失败: {e}")
                continue
            for _ in range(20):  # 等 2s
                await asyncio.sleep(0.1)
                if resp_buf["data"]:
                    break
            if resp_buf["data"]:
                resp = resp_buf["data"]
                print(f"响应帧: {resp.hex()}")
                print(hexdump(resp))
                print("\n解析结果:")
                print(parse_modbus_response(resp))
            else:
                print("[-] 2s 内未收到 notify 响应")


async def cmd_read(args):
    addr = await resolve_address(args)
    if not addr:
        return
    print(f"[*] 连接 {addr} ...")
    async with BleakClient(addr, timeout=20.0) as client:
        print(f"[*] 读取 {args.uuid}")
        data = await client.read_gatt_char(args.uuid)
        print(f"[*] 收到 {len(data)} 字节:")
        print(hexdump(bytes(data)))
        print(parse_modbus_response(bytes(data)))


async def cmd_write(args):
    addr = await resolve_address(args)
    if not addr:
        return
    try:
        payload = bytes.fromhex(args.data)
    except ValueError as e:
        print(f"[-] hex 解析失败: {e}")
        return
    print(f"[*] 连接 {addr} ...")
    async with BleakClient(addr, timeout=20.0) as client:
        print(f"[*] 写入 {args.uuid}  payload={payload.hex()} ({len(payload)}B)")
        await client.write_gatt_char(args.uuid, payload, response=not args.no_response)
        print("[+] 写入完成")


# ====== CLI ======
def build_parser():
    p = argparse.ArgumentParser(
        description=f"BMS BLE 上位机（目标 {TARGET_NAME}, Modbus RTU）",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument("--name", help=f"BLE 广播名（默认 {TARGET_NAME}）")
    p.add_argument("--address", help="直接指定 MAC 地址，跳过名称解析")
    p.add_argument("--scan-timeout", type=float, default=10.0, help="扫描超时秒（默认 10）")
    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("scan", help="扫描 BLE 设备")
    sp.add_argument("--timeout", type=float, default=10.0)
    sp.set_defaults(func=cmd_scan)

    sp = sub.add_parser("probe", help="枚举服务/特征，识别透明传输模式")
    sp.add_argument("--no-read-probe", action="store_true", help="不主动读取每个可读特征")
    sp.set_defaults(func=cmd_probe)

    sp = sub.add_parser("monitor", help="监听 notify + 可选周期查询")
    sp.add_argument("--log", help="日志文件路径（TSV 追加）")
    sp.add_argument("--poll", action="store_true", help="周期发送 Modbus 查询")
    sp.add_argument("--poll-type", choices=["realtime", "serial", "alarm", "all"],
                    default="realtime", help="轮询类型（默认 realtime）")
    sp.add_argument("--poll-interval", type=float, default=1.0, help="轮询间隔秒（默认 1.0）")
    sp.set_defaults(func=cmd_monitor)

    sp = sub.add_parser("query", help="主动查询 BMS 数据")
    sp.add_argument("--type", choices=["realtime", "serial", "alarm", "all"],
                    default="all", help="查询类型（默认 all）")
    sp.set_defaults(func=cmd_query)

    sp = sub.add_parser("read", help="读取指定特征")
    sp.add_argument("--uuid", required=True)
    sp.set_defaults(func=cmd_read)

    sp = sub.add_parser("write", help="写入指定特征")
    sp.add_argument("--uuid", required=True)
    sp.add_argument("--data", required=True, help="hex 字符串，如 01030000001D85C3")
    sp.add_argument("--no-response", action="store_true", help="无响应写")
    sp.set_defaults(func=cmd_write)
    return p


def main():
    args = build_parser().parse_args()
    logging.getLogger("bleak").setLevel(logging.WARNING)
    try:
        asyncio.run(args.func(args))
    except BleakError as e:
        print(f"[-] BLE 错误: {e}")
    except KeyboardInterrupt:
        print("\n[*] 已退出")


if __name__ == "__main__":
    main()
