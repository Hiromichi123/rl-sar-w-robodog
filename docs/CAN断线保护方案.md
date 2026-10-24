# CAN 断线保护方案（L1+L2+L3 修订版 + 指令堆积治理）
-669
> 状态：**L1 / L2 / L4 已在 v6 实施并编译通过（2026-10-04）；L3（分级安全动作）未实施**
> 制定日期：2026-10-03
> 实施日期：2026-10-04（仅 L1/L2/L4，未改动任何控制环动作）
> 适用项目：rl_sar-w2_recovered_v6（NanoPi M5 / RK3576，JXGw 轮足机器人）
>
> 实施说明（与原方案的差异/细化，2026-10-04 下午修订）：
> - L1（已修订）：**不再把遥测 real_pos/real_vel/real_tau 清零**，断线时冻结显示最后真实值；
>   电机遥测帧由 9 float 扩为 **11 float**（末尾追加 `temp_c` 占位 + `feedback_age_ms`），
>   把每电机「距上次有效反馈的毫秒数」(-1=从未)透传到看板，看板新增「更新」列实时显示
>   年龄并按 ≥500ms 橙 / ≥1s 红着色；**未触碰控制环读取路径**。
> - L2：链路级静默告警（曾经收帧后再静默才报，避免上电握手误报；恢复需连续 3 拍健康；
>   全断合并一条）；**新增电机级超时告警**——链路正常但个别电机 500ms 无有效反馈时
>   Notify(warning)，与链路级去重（链路已告警则不逐电机重复）。
> - **起立门禁接入 CAN 新鲜度校验**：新增 `CheckCanFeedbackFresh(500ms)`（16 个电机含
>   轮毂逐一核对反馈年龄），① 卧姿校验 check_stand 与 ② 站姿校验 check_standing 均要求
>   CAN 全部新鲜才通过（结果 JSON 加 `can_ready`，失败 reason 带超时电机明细），冻结旧值
>   无法再骗过角度判据；③ getup 命令执行前再实时检查一次（独立最高优先，SafeHold 不豁免），
>   断线瞬间拒绝起立并 Notify(error)。只读时间戳，不改任何控制动作。
> - **实时校验（持续状态 + 全动作入口拦截）**：控制环 ~20Hz 刷新原子量
>   `can_feedback_live_`；姿态帧 posture 新增 `can_ready` 字段（8Hz 广播），前端姿态胶囊旁
>   实时显示「CAN 正常/断线」（红色闪烁，旧后端自动隐藏）；getup / locomotion(行走) /
>   getdown(趴下) 三个动作入口在执行瞬间逐一实时校验，断线即 REJECTED + Notify(error)。
>   stop/passive 不拦截（紧急停止永远可用）。运动中的控制动作不做任何改变（非 L3）。
> - L4 熔断阈值 100 连续发送失败；熔断后 tx 线程 10ms 节拍发探测帧，恢复即清空队列；
>   入队在熔断期间仍按 can_id 覆盖（队列有界不膨胀），恢复后由最新周期指令续发。
>
> 前端协议：motor_state 每电机块支持 9(旧基础)/10(旧含温度)/11(新含反馈年龄) 三种 stride，
> 前端按电机数与长度自适应；本次前端有改动（ros-bridge/dashboard/style/两个 html），
> 已 bump 版本（ros-bridge.js?v=1006、dashboard.js?v=1005、main.js?v=1155、style.css?v=1064）。

## 总体原则

1. **控制环读数据的逻辑一律不动**（清零会生成巨大力矩指令）
2. **CAN 断了指令到不了电机**——所有"安全动作"只在 TX 通路还活着时有意义
3. **默认不卸力**：冻结姿态可能是站稳的，Passive 反而是主动放倒
4. **恢复一律人工确认**，杜绝 RL 观测错配瞬间的猛动作

---

## L1 遥测真实化（让故障可见）

| 位置 | 改动 |
|---|---|
| `src/rl_sar/library/thirdparty/JXG_sdk/include/usb2can.h` | `MotorFeedback` 增加 `std::chrono::steady_clock::time_point last_rx_time{}`（默认 epoch = 从未收到） |
| `src/rl_sar/library/thirdparty/JXG_sdk/src/usb2can.cpp` `process_recv_frame` | 写入 `feedback_map_` 时打戳 `fb.last_rx_time = steady_clock::now()` |
| `motor_manager` | 新增只读接口 `getMotorLastRxTime(leg, motor)` 透传查询 `can_manager_`，**不改 `leg_feedbacks_` 结构** |
| `src/rl_sar/src/rl_real_JXG.cpp` `PublishMotorState`（L1966-2016） | 每电机检查 `now - last_rx_time > 500ms` → `realPos/realVel/realTau` 推 0 |

- 检查在 20Hz 遥测路径，不进 500Hz 控制环热路径
- 上电未握手期推 0，与现状行为一致，无回归
- 效果：看板数值归零 + 已有 1 秒标红机制（dashboard.js `is-stale`）自然触发
- 前端零改动

---

## L2 断线告警（让人知道）

- `USB2CANDevice` 增加 `std::atomic<int64_t> last_rx_ms_{0}`，rx 线程每收一合法帧即更新；暴露 `last_rx_ms()` 查询
- `PublishMotorState` 每拍（20Hz）轮询 4 路设备：
  - 某路由正常 → 静默 >500ms：推 Notify `CAN x 号链路断线`，进入告警态
  - 告警态 + 连续 3 拍（~150ms）有帧：推 `CAN x 号链路已恢复`，退出告警态
  - 全断合并为一条 `全部 CAN 链路断线`，不刷屏
- 复用现有 WebSocket Notify 通道（前端 🔔 通知栏红点），**前端零改动**

---

## L3 安全动作（修订版，分级响应）

**前提认知**：CAN 全断时指令无法到达电机，软件干预无效；L3 只在部分断连（TX 活）和恢复阶段有意义。

| 检测状态 | 动作 | 理由 |
|---|---|---|
| 全断 + IMU 角速度≈0（静止站立） | 不发任何新指令，告警 + 状态锁定 | 冻结姿态可能正站着，动了反而放倒 |
| 全断 + IMU 在运动 | 停止入队、清空待发送队列 | 摔倒不可避免，保护恢复瞬间 |
| 单路断（一腿僵、三腿可控） | 其余三腿切**阻尼位置保持**（保持当前关节角，中等 kp + 高 kd），告警 | 不卸力不放倒，三条腿撑住等人处理；**绝不自动 Passive**（避免"一腿站三腿收"别坏关节） |
| 任何恢复瞬间 | 禁止自动恢复控制，必须人工确认后重新起立 | RL 观测与实际姿态错配，恢复瞬间输出会猛动作 |

不做：自动收腿、自动 GetDown、自动 Passive。

---

## L4 指令堆积治理（tx_queue_）

**问题**：`enqueue_command` 无界 push，TX 发送失败时队列照积——CAN 拔掉后 500Hz×16 电机的指令全部堆积，内存膨胀 + 恢复瞬间几千帧过期指令洪水般补发。

**方案：队列改"每电机最新帧覆盖"语义**

电机指令是状态而非事件，旧指令毫无价值，最新帧永远正确：

1. `tx_queue_` 从 FIFO 队列改为**有界结构 + 按 can_id 去重覆盖**：
   - 入队时若队列中已有该 `can_id` 的待发送帧 → 原地覆盖为最新值
   - 队列长度天然有上界 = 该设备管理的电机数（4 个），永不膨胀
   - 实现：`std::deque` + 线性查找（≤4 项，开销忽略）或 `std::array<std::optional<MotorCommand>, N>` + 位图
2. **发送失败熔断**：`send_motor_command_frame` 连续失败 >100 次（≈0.2s）→ 标记设备 TX 异常，tx 线程退避到 10ms 一拍空转，不再从队列取帧（取出来的也发不出去，留给恢复后一次性清空）
3. **恢复清队**：`recv_fn_`/`send_fn_` 从失败恢复正常时，清空该设备全部待发送帧，从最新控制周期重新开始发——杜绝过期帧补发
4. L3 的"全断+运动中停止入队"由 `enqueue_command` 入口检查上述熔断标记实现

---

## 改动文件汇总

| 文件 | 涉及级别 |
|---|---|
| `JXG_sdk/include/usb2can.h` | L1/L2/L4：时间戳字段、设备级收帧时间、队列结构声明 |
| `JXG_sdk/src/usb2can.cpp` | L1/L2/L4：收帧打戳、队列覆盖语义、发送熔断、恢复清队 |
| `JXG_sdk/src/motor_manager.cpp`（+头文件） | L1：只读时间戳透传接口；L3：单腿位置保持指令接口 |
| `src/rl_sar/src/rl_real_JXG.cpp` | L1/L2：`PublishMotorState` 新鲜度检查 + 静默检测 + Notify；L3：分级状态机 |
| `src/rl_sar/fsm_robot/fsm_JXGw.hpp` | L3：锁定/人工确认状态位（只加状态，不改过渡逻辑） |

前端零改动。编译目标 `rl_real_JXG`（v6，静态链接 BUILD_SHARED_LIBS=OFF）：

```bash
source /opt/ros/lyrical/setup.bash
cmake --build /home/jinjiao/rl_sar-w2_recovered_v6/cmake_build --target rl_real_JXG -j8
```

重启服务：`sudo bash /home/jinjiao/rl_sar-w2_recovered_v6/scripts/service.sh install`
（L1/L2/L4 于 2026-10-04 已编译通过；web_server 不含 CAN 代码无需重编）

---

## 验证方案

1. 站立状态拔单路 USB-CAN → 3 秒内：看板该腿归零变红、🔔 断线通知、其余三腿保持位置不瘫
2. 行走中拔全部 → 看板全红、合并告警、队列不膨胀（改前后对比内存）
3. 插回 → 恢复通知、无过期帧补发引起的抽动、控制不自动恢复
4. 回归测试：正常起立/行走全流程不受影响

---

## 背景结论（问题分析依据）

- 断线后看板不红不归零的根因：`usb2can.cpp` `feedback_map_` 无时间戳缓存 → `get_feedback` 永远返回旧值+true → `motor_manager.cpp` `leg_feedbacks_` 冻结 → 控制环与遥测均用冻结数据 → 前端收包正常故不触发标红
- 现有代码无任何 CAN 断线保护；RL 推理看门狗（rl_sdk.cpp L634，100ms 无 action 强制 Passive）与 CAN 无关；电机 error_code 只写 HR 日志无消费
- 行走中全断推演：电机僵在最后一帧 MIT 指令 → 0.1~0.3s 栽倒，软件无法阻止；唯一说实话的通道是 IMU（独立通道）
