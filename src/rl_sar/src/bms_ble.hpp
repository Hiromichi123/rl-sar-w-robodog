// ============================================================================
// bms_ble —— 电池 BMS 蓝牙(BlueZ D-Bus)直读模块
//
// 目标设备: HCYPZY26HYDA922100(BLE 透传, 中国铁塔 BMS / Modbus 变体)
// 传输:     BlueZ 系统总线(org.bluez),GATT write-without-response + notify
// 应用层:   从机地址 0x16,功能码 0x03,CRC16-CCITT/XMODEM(低字节在前,非 Modbus CRC)
//
// 设计要点:
//   - 独立后台线程 + sd-bus(libsystemd),绝不触碰 200Hz 控制环;
//   - 后台常驻连接:进程启动即开始轮询(默认 on),断开后自动重连;
//     前端「电池」页签打开仅为查看,不做开关;如需省电可下发 "bms off" 暂停;
//   - 设备地址 Static Random 可能变化,连接前按名称扫描定位,失败按已知地址直连;
//   - 0x2005 类 HCI 命令偶发超时会导致扫描/连接偶发失败,内部做重试与重连。
// ============================================================================
#ifndef BMS_BLE_HPP
#define BMS_BLE_HPP

#include <functional>
#include <string>

namespace bms_ble
{
using Publish = std::function<void(const std::string &)>;

// BMS 快照(供 rl_real_JXG 跨进程订阅低电量判断)
struct Snapshot {
    bool   online = false;
    int    soc = -1;        // 剩余电量 %
    double voltage = 0.0;   // 总电压 V
    double current = 0.0;   // 电流 A(正=充电)
    int    alarm = 0;       // BMS 硬件报警位
    long long ts = 0;       // 数据时间戳 ms
};

// 启动后台 BLE 轮询线程(独占 BlueZ,设备缺失时降级为离线状态,不崩溃)
void Start(Publish publish);
// 停止并回收线程
void Stop();

// 处理 WS 命令族 "bms ..."(on/off/info),返回 true 表示已处理
bool HandleCommand(const std::string &text, const Publish &publish);

// 获取最新一帧 BMS 快照(线程安全;无数据时 online=false)
Snapshot GetSnapshot();
}  // namespace bms_ble

#endif  // BMS_BLE_HPP
