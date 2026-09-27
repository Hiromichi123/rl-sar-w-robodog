/**
 * rl_real_JXG 内置 WebSocket 桥接模块。
 *
 * 连接主控程序 rl_real_JXG 内置的 WebSocket 服务器(与 Web 静态文件同端口托管),
 * 接收电机 / IMU / 里程计遥测,下发行为命令与底盘速度指令。
 *
 * 默认连接地址: ws(s)://<当前页面 host>/ws
 * 兜底常量:     ws://127.0.0.1:8088/ws
 *
 * 线协议: {"type":"<通道>","data":<payload>}(cmd_vel 为扁平字段 vx/vy/yaw)
 *   服务器 → 页面:
 *     motor_state          data = number[](每电机 9 float: idx,tgt_pos,real_pos,tgt_vel,real_vel,tgt_tau,real_tau,kp,kd)
 *     imu_state            data = number[](19 float,见 IMU_FIELDS)
 *     pose2d               data = number[](7 float: x,y,z,yaw,vel_x,vel_y,yaw_rate)
 *     feedback             data = string(指令应答/状态反馈)
 *     notify               data = string(JSON: {id,type,content,timestamp})
 *     check_stand_result   data = string(JSON)
 *     briefing_status      data = string
 *   页面 → 服务器:
 *     command     data = string(getup/getdown/passive/locomotion/vel_stop/zero/zero_motor <id>/status)
 *     cmd_vel     扁平字段 vx/vy/yaw
 *     imu_stream  data = boolean
 *     notify_ack  data = string(JSON: {id,ok})
 *     check_stand data = 'check'
 *     briefing    data = string(场景名/stop/list)
 *
 * data 字段与旧 ROS 消息(std_msgs/Float32MultiArray 等)的 payload 形状一致,
 * 下游解析逻辑(motorTelemetry / 曲线 / 看板 / 报警)零改动。
 */
import * as THREE from 'three';
import { jointStates, currentModelId, shared } from './state.js?v=1';
import { setJointPose } from './joints.js?v=1';

// 真机地址: 与页面同源(rl_real_JXG 内置服务器)
// 本地调试兜底: ws://127.0.0.1:8088/ws
// 可通过页面顶部的连接地址输入框动态修改,下面常量仅作为默认兜底 fallback
const PAGE_WS_URL = (typeof location !== 'undefined' && location.host)
  ? `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`
  : '';
export const ROS_URL_DEFAULT = PAGE_WS_URL || 'ws://127.0.0.1:8088/ws';
// 兼容旧代码:仍导出 ROS_URL(只读)指向 DEFAULT
export const ROS_URL = ROS_URL_DEFAULT;
export const MOTOR_STATE_TOPIC = 'motor_state';        // 电机遥测通道(原 /rl_real/motor_state)
export const IMU_STATE_TOPIC   = 'imu_state';          // IMU 遥测通道(原 /rl_real/imu_state)
export const COMMAND_TOPIC     = 'command';            // 行为命令通道: getup / getdown / passive / locomotion / vel_stop / zero / status
export const CMD_VEL_TOPIC     = 'cmd_vel';            // 底盘速度通道(扁平字段 vx/vy/yaw)
export const FEEDBACK_TOPIC    = 'feedback';           // 指令应答/状态反馈通道
export const POSE2D_TOPIC      = 'pose2d';             // 里程计位姿通道 float[7]
export const IMU_STREAM_ENABLE_TOPIC = 'imu_stream';   // IMU 数据流开关通道
export const NOTIFY_TOPIC      = 'notify';             // 机器人通知事件通道 (JSON: {id,type,content,timestamp})
export const NOTIFY_ACK_TOPIC  = 'notify_ack';         // 通知回复通道 (JSON: {id,ok})
export const CHECK_STAND_TOPIC     = 'check_stand';         // 站立检查触发通道(上位机→机器人)
export const CHECK_STAND_RESULT_TOPIC = 'check_stand_result'; // 站立检查结果通道(机器人→上位机, JSON)
export const BRIEFING_PLAY_TOPIC   = 'briefing';         // 作业交底语音播报通道(上位机→机器人: 场景名/stop/list)
export const BRIEFING_STATUS_TOPIC = 'briefing_status';  // 作业交底播报状态通道(机器人→上位机)

// ── Insight 9 相机通道名(保留常量供旧 UI 文案引用;相机 ROS 数据管线已下线,画面暂为空) ──
export const CAMERA_COLOR_TOPIC        = '/camera/camera/color/image_raw/compressed';
export const CAMERA_COLOR_RECT_TOPIC   = '/camera/camera/color/image_rect_raw/compressed';
export const CAMERA_INFRA1_TOPIC       = '/camera/camera/infra1/image_raw';
export const CAMERA_INFRA1_RECT_TOPIC  = '/camera/camera/infra1/image_rect_raw';
export const CAMERA_INFRA2_TOPIC       = '/camera/camera/infra2/image_raw';
export const CAMERA_INFRA2_RECT_TOPIC  = '/camera/camera/infra2/image_rect_raw';
export const CAMERA_IMU_TOPIC      = '/camera/camera/imu';
export const CAMERA_VIO_TOPIC      = '/camera/camera/vio_100hz';
export const CAMERA_VIO_STATUS_TOPIC = '/camera/camera/vio_status';
/** 相机画面通道合法值(setCameraChannel 校验用) */
const CAMERA_CHANNELS = new Set(['color', 'infra1', 'infra2']);

// ── Livox(MID-360)雷达通道名(同上,保留常量,数据管线已下线) ──
export const LIDAR_TOPIC     = '/livox/lidar';
export const LIDAR_IMU_TOPIC = '/livox/imu';

/**
 * 复现时屏蔽真机回调:置 true 后,真实 motor/imu/pose2d 消息全部丢弃,
 * 仅 injectMotorFrame / injectImuFrame 仍可注入,避免双数据流打架。
 */
let _replayMutingRealCallbacks = false;
export function setReplayMutingRealCallbacks(on) {
  _replayMutingRealCallbacks = !!on;
}
export function isReplayMutingRealCallbacks() {
  return _replayMutingRealCallbacks;
}

/** 运行时 WebSocket URL(可被 main.js 通过 setRosUrl 设置,来源于页面输入框的值) */
let _runtimeRosUrl = ROS_URL_DEFAULT;
/** 设置运行时 WebSocket 地址,只在未连接状态下生效(或下次连接时使用) */
export function setRosUrl(url) {
  const u = String(url || '').trim();
  if (u) _runtimeRosUrl = u;
}
/** 获取当前生效的 WebSocket 地址 */
export function getRosUrl() {
  return _runtimeRosUrl || ROS_URL_DEFAULT;
}

/**
 * 获取内部原生 WebSocket 连接实例(话题调试工具等复用)。
 * 未连接/已断开时返回 null;调用方需自行检查 connectionStatus === 'online'。
 */
export function getRosClient() {
  return (connectionStatus === 'online' && ws) ? ws : null;
}

/** 行为命令最近发布次数(UI 显示用) */
export let commandPublishCount = 0;
export let lastCommandPublishAt = 0;
export let lastCommandPayload = '';

/** cmd_vel 最近发布次数 + 最后一次速度(UI 显示用) */
export let cmdVelPublishCount = 0;
export let lastCmdVelPublishAt = 0;
export let lastCmdVel = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };

/**
 * IMU float[19] 字段定义:
 *   quat_w,quat_x,quat_y,quat_z, roll,pitch,yaw,
 *   acc_x,acc_y,acc_z, gyro_x,gyro_y,gyro_z,
 *   pos_north,pos_east,pos_down, vel_body_x,vel_body_y,vel_body_z
 */
const IMU_FIELDS = {
  QUAT_W: 0, QUAT_X: 1, QUAT_Y: 2, QUAT_Z: 3,
  ROLL: 4, PITCH: 5, YAW: 6,
  ACC_X: 7, ACC_Y: 8, ACC_Z: 9,
  GYRO_X: 10, GYRO_Y: 11, GYRO_Z: 12,
  POS_N: 13, POS_E: 14, POS_D: 15,
  VEL_X: 16, VEL_Y: 17, VEL_Z: 18,
};
/** IMU 最近帧数(对外只读,与 motor 的计数分开) */
export let imuMessageCount = 0;
/** IMU 最近一帧时间戳(ms) */
export let lastImuMessageAt = 0;

/**
 * pose2d float[7] 字段定义:
 *   x, y, z, yaw, vel_x, vel_y, yaw_rate
 */
const POSE2D_FIELDS = {
  X: 0, Y: 1, Z: 2, YAW: 3,
  VEL_X: 4, VEL_Y: 5, YAW_RATE: 6,
};
/** pose2d 最近帧数 + 时间戳(对外只读) */
export let pose2dMessageCount = 0;
export let lastPose2dMessageAt = 0;

/**
 * 各模型的 "配置序号 i" -> 仿真关节名 映射表。
 * i 是消息中 9 元素块的块号(0 开始),对应配置表中的"配置序号"。
 */
export const JOINT_MAPPING = {
  // 模型 1:人形 (actuator XML 定义顺序,如需调整可改此处)
  '1': [
    'fl_steering_joint',
    'fr_steering_joint',
    'bl_steering_joint',
    'br_steering_joint',
    'lift_joint',
    'robot_left_joint1',
    'robot_left_joint2',
    'robot_left_joint3',
    'robot_left_joint4',
    'robot_left_joint5',
    'robot_left_joint6',
    'robot_left_joint7',
    'robot_left_finger_joint1',
    'robot_right_joint1',
    'robot_right_joint2',
    'robot_right_joint3',
    'robot_right_joint4',
    'robot_right_joint5',
    'robot_right_joint6',
    'robot_right_joint7',
    'robot_right_finger_joint1',
    'robot_head_pitch_joint',
    'robot_head_yaw_joint',
  ],
  // 模型 2:四足机器人 (按用户提供映射表:配置序号 0~15)
  '2': [
    'FL_hip_joint',   // 0  左前侧摆
    'FL_thigh_joint', // 1  左前髋
    'FL_calf_joint',  // 2  左前膝
    'FL_foot_joint',  // 3  左前踝
    'FR_hip_joint',   // 4  右前侧摆
    'FR_thigh_joint', // 5  右前髋
    'FR_calf_joint',  // 6  右前膝
    'FR_foot_joint',  // 7  右前踝
    'RL_hip_joint',   // 8  左后侧摆
    'RL_thigh_joint', // 9  左后髋
    'RL_calf_joint',  // 10 左后膝
    'RL_foot_joint',  // 11 左后踝
    'RR_hip_joint',   // 12 右后侧摆
    'RR_thigh_joint', // 13 右后髋
    'RR_calf_joint',  // 14 右后膝
    'RR_foot_joint',  // 15 右后踝
  ],
};

/** 位置源:'real' = real_pos, 'target' = target_pos */
export let positionSource = 'real';
/** 连接状态:'offline' | 'connecting' | 'online' | 'error' */
export let connectionStatus = 'offline';
/** 最近一次错误信息 */
export let lastError = '';
/** 最近一帧消息的时间戳(ms),用于 UI 判断数据流是否正常 */
export let lastMessageAt = 0;
/** 累计收到的消息帧数 */
export let messageCount = 0;
/** 最近一次消息解包到的电机块数 */
export let lastBlockCount = 0;
/** 是否启用远端覆盖(即真机数据生效且禁用本地滑块) */
export let rosControlActive = false;

let ws = null;
const statusListeners = new Set();
const messageListeners = new Set();
const imuListeners = new Set();
const notifyListeners = new Set();
const checkStandListeners = new Set();
// 命令应答文本（如 "command accepted: zero_motor motor_id=5"）
const commandFeedbackListeners = new Set();
const briefingStatusListeners = new Set();
const cameraFrameListeners = new Set();
const cameraImuListeners = new Set();
const cameraVioListeners = new Set();
const cameraVioStatusListeners = new Set();
const lidarFrameListeners = new Set();
const lidarImuListeners = new Set();
// 原始帧监听(话题调试工具 ros-debug.js 复用;收到任何服务器帧都会回调)
const rawMessageListeners = new Set();
/** 通知消息累计收到的条数 */
export let notifyMessageCount = 0;
/** 最近一次通知的简要信息(UI 状态栏显示) */
export let lastNotifySummary = '';
/** 状态栏 emit 节流:高频数据流下每 30/60 条一条太快,叠加 200ms 时间门限(≤5Hz) */
let _lastMotorStatusEmitAt = 0;
let _lastImuStatusEmitAt = 0;

/** 20Hz = 50ms 间隔(对齐旧 CLI --rate 20) */
const CMD_VEL_INTERVAL_MS = 50;
/** 2Hz = 500ms 间隔(对齐旧 CLI --rate 2) */
const COMMAND_INTERVAL_MS = 500;

/** 当前对外"请求的目标速度";20Hz 定时器会持续把它发布到 cmd_vel 通道 */
const cmdVelRequest = { vx: 0, vy: 0, omega: 0, vz: 0 };
/** cmd_vel keep-alive 定时器句柄(非空=正在持续发布) */
let cmdVelTimerId = null;
/** cmd_vel request 非零时已持续帧数;用于"归零后还需多发 N 帧零速度再停 timer"的 N 计数 */
let cmdVelZeroFramesRemaining = 0;

/** status 事件(对外 UI):新增 command / cmd_vel 统计字段一起广播 */
export function getStatusSnapshot() {
  return {
    connectionStatus, lastError, rosControlActive, positionSource,
    lastMessageAt, messageCount, lastBlockCount,
    lastImuMessageAt, imuMessageCount,
    commandPublishCount, lastCommandPublishAt, lastCommandPayload,
    cmdVelPublishCount, lastCmdVelPublishAt, lastCmdVel,
    notifyMessageCount, lastNotifySummary,
  };
}
function emitStatus() {
  statusListeners.forEach((fn) => fn(getStatusSnapshot()));
}

/** 新消息到达时通知监听者(可选) */
function emitMessage(payload) {
  messageListeners.forEach((fn) => fn(payload));
}

/** IMU 新帧到达通知 */
function emitImu(payload) {
  imuListeners.forEach((fn) => fn(payload));
}

/** 通知事件到达时通知监听者 */
function emitNotify(payload) {
  notifyListeners.forEach((fn) => fn(payload));
}

/** 站立检查结果到达时通知监听者 */
function emitCheckStandResult(payload) {
  checkStandListeners.forEach((fn) => fn(payload));
}

/** 行为命令应答到达时通知监听者(纯文本) */
function emitCommandFeedback(text) {
  commandFeedbackListeners.forEach((fn) => fn(text));
}

/** 作业交底播报状态到达时通知监听者 */
function emitBriefingStatus(text) {
  briefingStatusListeners.forEach((fn) => fn(text));
}

export function addStatusListener(fn) { statusListeners.add(fn); emitStatus(); return () => statusListeners.delete(fn); }
export function addMessageListener(fn) { messageListeners.add(fn); return () => messageListeners.delete(fn); }
export function addImuListener(fn) { imuListeners.add(fn); return () => imuListeners.delete(fn); }
export function addNotifyListener(fn) { notifyListeners.add(fn); return () => notifyListeners.delete(fn); }
export function addCheckStandListener(fn) { checkStandListeners.add(fn); return () => checkStandListeners.delete(fn); }
export function addCommandFeedbackListener(fn) { commandFeedbackListeners.add(fn); return () => commandFeedbackListeners.delete(fn); }
export function addBriefingStatusListener(fn) { briefingStatusListeners.add(fn); return () => briefingStatusListeners.delete(fn); }
export function addCameraFrameListener(fn) { cameraFrameListeners.add(fn); return () => cameraFrameListeners.delete(fn); }
export function addCameraImuListener(fn) { cameraImuListeners.add(fn); return () => cameraImuListeners.delete(fn); }
export function addCameraVioListener(fn) { cameraVioListeners.add(fn); return () => cameraVioListeners.delete(fn); }
export function addCameraVioStatusListener(fn) { cameraVioStatusListeners.add(fn); return () => cameraVioStatusListeners.delete(fn); }
export function addLidarFrameListener(fn) { lidarFrameListeners.add(fn); return () => lidarFrameListeners.delete(fn); }
export function addLidarImuListener(fn) { lidarImuListeners.add(fn); return () => lidarImuListeners.delete(fn); }
export function addRawMessageListener(fn) { rawMessageListeners.add(fn); return () => rawMessageListeners.delete(fn); }

export function setPositionSource(source) {
  positionSource = source === 'target' ? 'target' : 'real';
  emitStatus();
}

/** 设置远端控制是否启用(通常连接成功后设为 true,断开设为 false) */
function setControlActive(active) {
  rosControlActive = active;
  emitStatus();
}

function setStatus(s, errorMsg = '') {
  connectionStatus = s;
  if (errorMsg) lastError = errorMsg;
  emitStatus();
}

/**
 * 将解包后的电机位置应用到对应关节。
 * 直接写入 state.value 与 state.target 并调用 setJointPose,绕过渲染循环中的阻尼插值,
 * 让仿真画面严格跟随真机数据,减少延迟。
 * 滑块 UI 同步改为「标脏 + ~20Hz 批量刷新」:高频消息下每条消息 × 每个电机都做
 * querySelector + input/output 写入会把主线程淹没在 DOM 工作里,导致浏览器严重卡顿。
 */
const _dirtySliderJoints = new Map(); // jointName -> 最近一次夹紧值
const _sliderRefsCache = new Map();   // jointName -> { input, out } | null(缺失/失效时重查)

/** 把标脏的关节同步到滑块 DOM(~20Hz 批量,代替每消息同步) */
function flushSliderSync() {
  if (!_dirtySliderJoints.size) return;
  for (const [jointName, clamped] of _dirtySliderJoints) {
    let refs = _sliderRefsCache.get(jointName);
    if (!refs || !refs.input.isConnected) {
      const input = document.querySelector(`input[type="range"][data-joint="${jointName}"]`);
      refs = input ? { input, out: input.parentElement.querySelector('output') } : null;
      _sliderRefsCache.set(jointName, refs);
    }
    if (!refs) continue;
    const state = jointStates.get(jointName);
    const precision = state ? (state.type === 'slide' ? 3 : state.controlMode === 'velocity' ? 1 : 2) : 2;
    refs.input.value = String(clamped);
    if (refs.out) refs.out.value = clamped.toFixed(precision);
  }
  _dirtySliderJoints.clear();
}
window.setInterval(flushSliderSync, 50);

function applyMotorPositions(blockIndex, pos) {
  const mapping = JOINT_MAPPING[currentModelId];
  if (!mapping) return;
  const jointName = mapping[blockIndex];
  if (!jointName) return;
  const state = jointStates.get(jointName);
  if (!state) return;
  const clamped = THREE.MathUtils.clamp(pos, state.min, state.max);
  state.value = clamped;
  state.target = clamped;
  setJointPose(state, clamped);
  // 滑块 UI:仅标脏,由 flushSliderSync ~20Hz 批量同步
  _dirtySliderJoints.set(jointName, clamped);
}

/**
 * 解析电机遥测消息并应用到关节。
 * 每 9/10 个 float 为一块(自适应识别);base+0 为 idx(与块号 i 做一致性校验,不一致时 warn)。
 *
 * 兼容多种消息封装:
 *  - 新协议:  { type:'motor_state', data: [float...] }
 *  - 旧封装:  message.data / message.msg.data / 直接数组
 */
function handleMotorStateMessage(message) {
  // ══════════════════════════════════════════════════════════════════
  // 复现时真机回调被屏蔽(避免真机帧与 inject 复现帧冲突)
  // injectMotorFrame 内部走 handleMotorStateMessageImpl 绕过此守卫
  if (_replayMutingRealCallbacks) return;
  _handleMotorStateMessageImpl(message);
}

/**
 * 原始电机遥测(jointName → 9/10 字段块,未夹紧未加工):
 *   { idx, targetPos, realPos, targetVel, realVel, targetTau, realTau, kp, kd, temp?, at }
 *   temp 仅 10-float 旧协议存在(°C)。
 * 数据看板 / 曲线从这里读原始值;applyMotorPositions 的夹紧值仅用于驱动仿真模型。
 */
export const motorTelemetry = new Map();

/** 电机消息实际处理逻辑(与 handleMotorStateMessage 分开以支持 inject 绕过 muting) */
function _handleMotorStateMessageImpl(message) {
  // ── 1. 用多种方式尝试取出 data 数组 ──────────────
  let data = null;
  if (Array.isArray(message)) {
    data = message;
  } else if (Array.isArray(message?.data) && message.data.length > 0) {
    data = message.data;
  } else if (Array.isArray(message?.msg?.data)) {
    data = message.msg.data;
  } else if (message?.msg && Array.isArray(message?.msg) && message.msg.length > 0) {
    data = message.msg;
  } else {
    for (const key of Object.keys(message || {})) {
      const v = message[key];
      if (Array.isArray(v) && v.length >= 9) { data = v; break; }
      if (v && typeof v === 'object' && Array.isArray(v.data) && v.data.length >= 9) { data = v.data; break; }
    }
  }
  if (!data || !Array.isArray(data) || data.length === 0) {
    if (!_handleMotorStateMessageImpl._warnedEmpty) {
      console.warn('[ros] 收到消息但未找到 data 数组,原始消息 dump:', JSON.stringify(message).slice(0, 500));
      _handleMotorStateMessageImpl._warnedEmpty = true;
    }
    return;
  }

  const mapping = JOINT_MAPPING[currentModelId];
  if (!mapping) {
    if (!_handleMotorStateMessageImpl._warnedModel) {
      console.warn('[ros] 当前模型', currentModelId, '无映射表,忽略消息。');
      _handleMotorStateMessageImpl._warnedModel = true;
    }
    return;
  }
  // stride 自适应:10-float 旧协议(含温度) / 9-float 新协议。
  // 优先按当前模型电机数精确匹配 —— 单纯用「长度能否被 10 整除」会误判:
  // 例如 20 个电机的旧协议长度 180 也能被 10 整除,会被错当成 stride=10。
  const jointCount = mapping.length || 0;
  let stride;
  if (jointCount > 0 && data.length === jointCount * 10) stride = 10;
  else if (jointCount > 0 && data.length === jointCount * 9) stride = 9;
  else stride = data.length % 10 === 0 ? 10 : 9;
  const blockCount = Math.floor(data.length / stride);
  if (blockCount === 0) {
    console.warn('[ros] data 长度不足一块(stride=', stride, ',长度=', data.length, '):', data.slice(0, 10));
    return;
  }
  lastMessageAt = performance.now();
  messageCount += 1;
  lastBlockCount = blockCount;
  if (messageCount % 30 === 1 && lastMessageAt - _lastMotorStatusEmitAt >= 200) {
    _lastMotorStatusEmitAt = lastMessageAt;
    emitStatus();
  }

  if (!_handleMotorStateMessageImpl._loggedFirst) {
    const preview = [];
    for (let i = 0; i < Math.min(blockCount, 4); i += 1) {
      const b = i * stride;
      const tempStr = stride === 10 ? ` temp=${data[b+9].toFixed(1)}°C` : '';
      preview.push(`i=${i}: idx=${data[b+0]} tgt=${data[b+1].toFixed(3)} real=${data[b+2].toFixed(3)} kp=${data[b+7]} kd=${data[b+8]}${tempStr}`);
    }
    console.info(`[ros] 首条消息:共 ${data.length} 个数 => ${blockCount} 个电机块。前几块:\n${preview.join('\n')}`);
    _handleMotorStateMessageImpl._loggedFirst = true;
  }

  let applied = 0;
  for (let i = 0; i < blockCount; i += 1) {
    const base = i * stride;
    const targetPos = Number(data[base + 1]);
    const realPos = Number(data[base + 2]);
    // 原始遥测入表(供数据看板/曲线使用,不做夹紧/位置源处理)
    const jointName = mapping[i];
    if (jointName) {
      // 原地复用既有对象,避免每条消息 × 每个电机分配新对象造成 GC 压力
      let tel = motorTelemetry.get(jointName);
      if (!tel) { tel = {}; motorTelemetry.set(jointName, tel); }
      tel.idx = Number(data[base + 0]);
      tel.targetPos = targetPos;
      tel.realPos = realPos;
      tel.targetVel = Number(data[base + 3]);
      tel.realVel = Number(data[base + 4]);
      tel.targetTau = Number(data[base + 5]);
      tel.realTau = Number(data[base + 6]);
      tel.kp = Number(data[base + 7]);
      tel.kd = Number(data[base + 8]);
      if (stride === 10) tel.temp = Number(data[base + 9]);
      else if (tel.temp !== undefined) delete tel.temp; // 源切回旧协议时清残留
      tel.at = lastMessageAt;
    }
    const pos = positionSource === 'target' ? targetPos : realPos;
    if (Number.isFinite(pos)) {
      applyMotorPositions(i, pos);
      applied += 1;
    }
  }
  // data 是每条消息解码出的新数组,监听者只在调用内使用,无需再 slice 拷贝
  emitMessage({ blockCount, applied, at: lastMessageAt, data });
}

/**
 * 从 message 里兜底取 data 数组。
 * 复用到 motor / imu 两个 float 数组通道。
 */
function extractFloat32Data(message) {
  if (Array.isArray(message)) return message;
  if (Array.isArray(message?.data) && message.data.length > 0) return message.data;
  if (Array.isArray(message?.msg?.data)) return message.msg.data;
  if (message?.msg && Array.isArray(message?.msg)) return message.msg;
  for (const key of Object.keys(message || {})) {
    const v = message[key];
    if (Array.isArray(v) && v.length >= 2) return v;
    if (v && typeof v === 'object' && Array.isArray(v.data) && v.data.length >= 2) return v.data;
  }
  return null;
}

/**
 * 解析 IMU 19-fields 消息。
 * 写入 shared.imu;优先用四元数计算欧拉(RPY 字段备用)。
 */
function handleImuStateMessage(message) {
  // 复现时真机回调屏蔽。injectImuFrame 内部走 _handleImuStateMessageImpl 绕过。
  if (_replayMutingRealCallbacks) return;
  _handleImuStateMessageImpl(message);
}

function _handleImuStateMessageImpl(message) {
  const data = extractFloat32Data(message);
  if (!data || !Array.isArray(data) || data.length < 19) {
    if (!_handleImuStateMessageImpl._warned) {
      console.warn('[imu] 消息无 data 或长度<19。len=', data?.length, '原始 sample=', JSON.stringify(message).slice(0, 300));
      _handleImuStateMessageImpl._warned = true;
    }
    return;
  }

  lastImuMessageAt = performance.now();
  imuMessageCount += 1;
  const imu = shared.imu;
  imu.lastMsgAt = lastImuMessageAt;
  imu.msgCount  = imuMessageCount;

  const f = IMU_FIELDS;
  imu.quat.w = Number(data[f.QUAT_W]);
  imu.quat.x = Number(data[f.QUAT_X]);
  imu.quat.y = Number(data[f.QUAT_Y]);
  imu.quat.z = Number(data[f.QUAT_Z]);
  imu.rpy.roll  = Number(data[f.ROLL]);
  imu.rpy.pitch = Number(data[f.PITCH]);
  imu.rpy.yaw   = Number(data[f.YAW]);
  imu.acc.x = Number(data[f.ACC_X]);
  imu.acc.y = Number(data[f.ACC_Y]);
  imu.acc.z = Number(data[f.ACC_Z]);
  // 角速度 rad/s
  imu.gyro.x = Number(data[f.GYRO_X]);
  imu.gyro.y = Number(data[f.GYRO_Y]);
  imu.gyro.z = Number(data[f.GYRO_Z]);
  // 位置(北东地)m
  imu.pos.north = Number(data[f.POS_N]);
  imu.pos.east  = Number(data[f.POS_E]);
  imu.pos.down  = Number(data[f.POS_D]);
  // 体坐标系速度 m/s
  imu.vel.x = Number(data[f.VEL_X]);
  imu.vel.y = Number(data[f.VEL_Y]);
  imu.vel.z = Number(data[f.VEL_Z]);

  if (!_handleImuStateMessageImpl._loggedFirst) {
    const n = Math.hypot(imu.quat.w, imu.quat.x, imu.quat.y, imu.quat.z);
    console.info(
      `[imu] 首帧。quat wxyz=(%s) 归一化模=%s  RPY(deg)=%s/%s/%s  |a|=%s m/s²`,
      [imu.quat.w,imu.quat.x,imu.quat.y,imu.quat.z].map(v=>v.toFixed(4)).join(','),
      n.toFixed(4),
      THREE.MathUtils.radToDeg(imu.rpy.roll).toFixed(1),
      THREE.MathUtils.radToDeg(imu.rpy.pitch).toFixed(1),
      THREE.MathUtils.radToDeg(imu.rpy.yaw).toFixed(1),
      Math.hypot(imu.acc.x,imu.acc.y,imu.acc.z).toFixed(3),
    );
    _handleImuStateMessageImpl._loggedFirst = true;
  }
  if (imuMessageCount % 60 === 1 && lastImuMessageAt - _lastImuStatusEmitAt >= 200) {
    _lastImuStatusEmitAt = lastImuMessageAt;
    emitStatus();
  }
  // data 是每条消息解码出的新数组,监听者只在调用内使用,无需再 slice 拷贝
  emitImu({ imu: imu /* 引用不变,录制层会 copy */, at: lastImuMessageAt, data });
}

/**
 * 外部注入一帧「电机帧」(复现播放使用)。
 * 效果等价于收到一条 motor_state 消息:
 *   写 lastMessageAt/messageCount/lastBlockCount → emitStatus 按需 → applyMotorPositions → emitMessage
 * 这样 main.js / curves.js 无需改,自然走同一套 UI 管道。
 * @param {number[]} data  N blocks × 9 floats 数组
 * @param {number} atMs    时间戳(performance.now() 基)
 */
export function injectMotorFrame(data, atMs) {
  // 绕过 muting 守卫,直接调用内部 impl
  _handleMotorStateMessageImpl(data);
  if (atMs) lastMessageAt = atMs;
}

/**
 * 外部注入一帧「IMU 帧」(复现播放使用)。
 * 效果等价于收到一条 imu_state 消息。
 * @param {number[]} data  19 floats 数组
 * @param {number} atMs    时间戳(performance.now() 基)
 */
export function injectImuFrame(data, atMs) {
  _handleImuStateMessageImpl(data);
  if (atMs) lastImuMessageAt = atMs;
}

/**
 * 处理 pose2d float[7] 一帧:
 * x, y, z, yaw, vel_x, vel_y, yaw_rate → 写入 shared.rosPose2d
 * @param {{data: number[]}} msg
 */
function _handlePose2dMessageImpl(msg) {
  const data = Array.isArray(msg?.data) ? msg.data : [];
  if (data.length < 7) return;
  const f = POSE2D_FIELDS;
  const odom = shared.rosPose2d;
  odom.x = Number(data[f.X]);
  odom.y = Number(data[f.Y]);
  odom.z = Number(data[f.Z]);
  odom.yaw = Number(data[f.YAW]);
  odom.velX = Number(data[f.VEL_X]);
  odom.velY = Number(data[f.VEL_Y]);
  odom.yawRate = Number(data[f.YAW_RATE]);
  pose2dMessageCount += 1;
  lastPose2dMessageAt = performance.now();
  if (pose2dMessageCount === 1) {
    console.info('[pose2d] 首帧。x,y,z=(%s) yaw=%s rad  vel=(%s, %s) m/s  yaw_rate=%s rad/s',
      [odom.x, odom.y, odom.z].map(v => v.toFixed(3)).join(','),
      odom.yaw.toFixed(3), odom.velX.toFixed(3), odom.velY.toFixed(3), odom.yawRate.toFixed(3));
  }
}

/**
 * 处理 feedback string 一帧:
 * 指令应答 / 状态反馈文本 → 打印日志
 * @param {{data: string}} msg
 */
function _handleFeedbackMessageImpl(msg) {
  const text = String(msg?.data ?? '');
  console.info(`[feedback] ⬇️ ${text}`);
  // 命令应答类消息提升给 UI toast（置零/保存置零执行约 2s, 是用户唯一的完成信号）；
  // 周期性 status 文本不上屏，避免刷屏
  if (/command accepted|command rejected|unknown command|not ready/i.test(text)) {
    emitCommandFeedback(text);
  }
}

/**
 * 机器人通知事件 (notify):解析 JSON → 发射事件 → 自动回复 ack。
 * 协议:{"id":N,"type":"info|warning|error|note","content":"...","timestamp":秒}
 * @param {{data: string}} msg
 */
function _handleNotifyMessageImpl(msg) {
  const raw = String(msg?.data ?? '');
  let parsed = null;
  try { parsed = JSON.parse(raw); } catch (_) {
    // 非 JSON:当作纯文本通知(id=0, type=info)
    parsed = { id: 0, type: 'info', content: raw, timestamp: Date.now() / 1000 };
  }
  const id     = Number(parsed.id) || 0;
  const type   = String(parsed.type || 'info');    // info / warning / error / note
  const content= String(parsed.content || '');
  const ts     = Number(parsed.timestamp) || (Date.now() / 1000);
  const payload = { id, type, content, timestamp: ts, raw };

  notifyMessageCount++;
  // 状态栏简要摘要(截断防过长)
  lastNotifySummary = `[${type}] ${content.length > 50 ? content.slice(0, 50) + '…' : content}`;
  console.info(`[notify] 📨 #${id} ${type}: ${content}`);

  // 发射给 UI 监听者
  emitNotify(payload);
  emitStatus();

  // 自动回复 ack(收到即确认,不让机器人重发)
  publishNotifyAck(id, true);
}

/**
 * 发布通知回复 (notify_ack)。
 * @param {number} id  通知消息编号
 * @param {boolean} ok 是否已确认
 */
export function publishNotifyAck(id, ok = true) {
  if (!isWsOpen()) return;
  const payload = JSON.stringify({ id, ok: !!ok });
  try {
    wsSend({ type: NOTIFY_ACK_TOPIC, data: payload });
    console.info(`[notify] ✅ ack sent #${id} ok=${ok}`);
  } catch (err) {
    console.warn('[notify] ack 发布失败:', err);
  }
}

/**
 * 站立检查结果 (check_stand_result):解析 JSON → 发射事件。
 * @param {{data: string}} msg
 */
function _handleCheckStandResultImpl(msg) {
  const raw = String(msg?.data ?? '');
  let parsed = null;
  try { parsed = JSON.parse(raw); } catch (_) {
    parsed = { ready: false, motors_ready: false, imu_ready: false, reason: raw };
  }
  const result = {
    ready: !!parsed.ready,
    motors_ready: !!parsed.motors_ready,
    imu_ready: !!parsed.imu_ready,
    reason: String(parsed.reason || ''),
    raw,
  };
  console.info(`[check-stand] 🔍 result: ready=${result.ready} motors=${result.motors_ready} imu=${result.imu_ready} reason=${result.reason}`);
  emitCheckStandResult(result);
}

/**
 * 发布站立检查触发 (check_stand)。
 */
export function publishCheckStand() {
  if (!isWsOpen()) return false;
  try {
    wsSend({ type: CHECK_STAND_TOPIC, data: 'check' });
    console.info('[check-stand] 📤 sent check_stand trigger');
    return true;
  } catch (err) {
    console.warn('[check-stand] 发布失败:', err);
    return false;
  }
}

/**
 * 作业交底播报状态 (briefing_status):原样发射(内容由 C++ 端定义)。
 * @param {{data: string}} msg
 */
function _handleBriefingStatusImpl(msg) {
  const raw = String(msg?.data ?? '');
  if (!raw) return;
  console.info('[briefing] 📢 status:', raw);
  emitBriefingStatus(raw);
}

/**
 * 发布作业交底播报命令 (briefing)。
 * @param {string} scene 场景名(elevator/forklift/lifting/warehouse/height)或 stop(打断)/list(列出场)
 */
export function publishBriefing(scene) {
  if (!isWsOpen()) return false;
  try {
    wsSend({ type: BRIEFING_PLAY_TOPIC, data: String(scene || '') });
    console.info('[briefing] 📤 sent:', scene);
    return true;
  } catch (err) {
    console.warn('[briefing] 发布失败:', err);
    return false;
  }
}

// ── 相机:数据管线已下线(相机 ROS 管道不再桥接),保留导出接口与期望状态 ──
/** UI 期望的相机状态 */
let cameraDesiredChannel = null;
/** 切换画面通道期望。channel: 'color'|'infra1'|'infra2'|null(取消)。 */
export function setCameraChannel(channel) {
  cameraDesiredChannel = CAMERA_CHANNELS.has(channel) ? channel : null;
}
export function getCameraChannel() { return cameraDesiredChannel; }

/** 相机 IMU / VIO 数据订阅开关(数据管线已下线,仅保存期望状态) */
export function setCameraTelemetryEnabled(on) {
  cameraTelemetryEnabled = !!on;
}
export function isCameraTelemetryEnabled() { return cameraTelemetryEnabled; }
let cameraTelemetryEnabled = false;

// ── Livox 雷达:数据管线已下线,保留导出接口 ──
/** 开关雷达点云 + 雷达 IMU 订阅(仅保存期望状态) */
export function setLidarEnabled(on) {
  lidarEnabled = !!on;
}
export function isLidarEnabled() { return lidarEnabled; }
let lidarEnabled = false;

// ─────────────────────────────────────────────
//   传输层:原生 WebSocket(handleServerMessage / wsSend / connect / disconnect)
// ─────────────────────────────────────────────

/** WebSocket 处于可发送状态 */
function isWsOpen() {
  return !!(ws && ws.readyState === WebSocket.OPEN);
}

/** 发送一帧 JSON;未连接时返回 false */
function wsSend(obj) {
  if (!isWsOpen()) return false;
  try {
    ws.send(JSON.stringify(obj));
    return true;
  } catch (err) {
    console.warn('[ws] send failed:', err);
    return false;
  }
}

/**
 * 处理服务器下发的一帧 JSON:{"type":"<通道>","data":<payload>}。
 * 统一包成 {data:...} 形状复用既有 _handle*Impl 解析;未知通道忽略。
 */
function handleServerMessage(raw) {
  let frame;
  try { frame = JSON.parse(raw); } catch (_) { return; }
  if (!frame || typeof frame !== 'object' || !frame.type) return;

  switch (frame.type) {
    case MOTOR_STATE_TOPIC:
      handleMotorStateMessage(frame);
      break;
    case IMU_STATE_TOPIC:
      handleImuStateMessage(frame);
      break;
    case POSE2D_TOPIC:
      if (!_replayMutingRealCallbacks) _handlePose2dMessageImpl(frame); // 复现时与 motor/imu 一致静默
      break;
    case FEEDBACK_TOPIC:
      _handleFeedbackMessageImpl(frame);
      break;
    case NOTIFY_TOPIC:
      _handleNotifyMessageImpl(frame);
      break;
    case CHECK_STAND_RESULT_TOPIC:
      _handleCheckStandResultImpl(frame);
      break;
    case BRIEFING_STATUS_TOPIC:
      _handleBriefingStatusImpl(frame);
      break;
    default:
      break; // 未知通道忽略
  }
  // 原始帧监听(调试工具)
  rawMessageListeners.forEach((fn) => {
    try { fn(frame); } catch { /* 忽略监听器异常 */ }
  });
}

/** 连接 WebSocket。返回 Promise,连接成功 resolve,失败 reject。 */
function connectRos() {
  return new Promise((resolve, reject) => {
    if (ws && (connectionStatus === 'online' || connectionStatus === 'connecting')) {
      resolve();
      return;
    }
    setStatus('connecting');
    setControlActive(false);
    const url = getRosUrl();
    console.info('[ws] 正在连接 WebSocket:', url);

    try {
      ws = new WebSocket(url);
    } catch (err) {
      const msg = `WebSocket 初始化失败:${err?.message || err}`;
      console.error('[ws]', msg);
      ws = null;
      setStatus('error', msg);
      reject(err);
      return;
    }

    let settled = false;

    ws.onopen = () => {
      settled = true;
      console.info('[ws] ✅ 已连接到 rl_real_JXG 内置服务器。');
      setStatus('online');
      setControlActive(true);
      resolve();
    };

    ws.onmessage = (ev) => {
      if (typeof ev.data !== 'string') return; // 二进制帧(协议层 pong 等)忽略
      handleServerMessage(ev.data);
    };

    ws.onerror = () => {
      const msg = `连接错误:无法连接 ${url}`;
      if (!settled) {
        settled = true;
        console.error('[ws]', msg);
        setStatus('error', msg);
        setControlActive(false);
        reject(new Error(msg));
      }
    };

    ws.onclose = () => {
      const wasOnline = connectionStatus === 'online';
      ws = null;
      console.warn('[ws] WebSocket 已关闭。');
      if (!settled) {
        settled = true;
        setStatus('error', `无法连接 ${url}`);
        setControlActive(false);
        reject(new Error('连接已关闭'));
      } else if (wasOnline) {
        setStatus('offline', '连接已关闭');
        setControlActive(false);
      }
    };
  });
}

/** 断开 WebSocket 连接。 */
function disconnectRos() {
  try {
    // 断开时恢复复现静默标志,避免 seek/回放后 muting 卡住导致重连后收不到任何数据
    try { setReplayMutingRealCallbacks(false); } catch { /* 忽略 */ }
    if (cmdVelTimerId) { clearInterval(cmdVelTimerId); cmdVelTimerId = null; }
    if (_commandBurstTimerId) { clearInterval(_commandBurstTimerId); _commandBurstTimerId = null; }
    cmdVelRequest.vx = 0; cmdVelRequest.vy = 0; cmdVelRequest.omega = 0;
    cmdVelZeroFramesRemaining = 0;
    if (ws) {
      try { ws.close(); } catch { /* 忽略 */ }
      ws = null;
    }
  } finally {
    messageCount = 0;
    lastBlockCount = 0;
    lastMessageAt = 0;
    imuMessageCount = 0;
    lastImuMessageAt = 0;
    commandPublishCount = 0;
    lastCommandPublishAt = 0;
    lastCommandPayload = '';
    cmdVelPublishCount = 0;
    lastCmdVelPublishAt = 0;
    lastCmdVel = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };
    cmdVelRequest.vx = 0; cmdVelRequest.vy = 0; cmdVelRequest.omega = 0; cmdVelRequest.vz = 0;
    notifyMessageCount = 0;
    lastNotifySummary = '';
    handleMotorStateMessage._warnedEmpty = false;
    handleMotorStateMessage._warnedModel = false;
    handleMotorStateMessage._warnedIdx = false;
    handleMotorStateMessage._loggedFirst = false;
    handleMotorStateMessage._cbHit = false;
    handleImuStateMessage._warned = false;
    handleImuStateMessage._loggedFirst = false;
    handleImuStateMessage._cbHit = false;
    // 清空 IMU 的显示值,避免断开后还保留旧数
    const imu = shared.imu;
    imu.quat.w = 1; imu.quat.x = imu.quat.y = imu.quat.z = 0;
    imu.rpy.roll = imu.rpy.pitch = imu.rpy.yaw = 0;
    imu.acc.x = imu.acc.y = imu.acc.z = 0;
    imu.gyro.x = imu.gyro.y = imu.gyro.z = 0;
    imu.pos.north = imu.pos.east = imu.pos.down = 0;
    imu.vel.x = imu.vel.y = imu.vel.z = 0;
    imu.msgCount = 0; imu.lastMsgAt = 0;
    setStatus('offline');
    setControlActive(false);
  }
}

/** 切换连接:在线则断开,离线则连接。 */
export async function toggleConnection() {
  if (connectionStatus === 'online' || connectionStatus === 'connecting') {
    disconnectRos();
    return false;
  }
  await connectRos();
  return true;
}

// ─────────────────────────────────────────────
//   Publishers: 行为命令 (command) + 底盘速度 (cmd_vel)
// ─────────────────────────────────────────────

/** 立即发一帧 string 命令;未连接时直接 return false,便于调试 + 状态计数。 */
function publishOneCommand(dataString) {
  if (!isWsOpen()) return false;
  try {
    wsSend({ type: COMMAND_TOPIC, data: dataString });
    commandPublishCount += 1;
    lastCommandPublishAt = performance.now();
    lastCommandPayload = dataString;
    emitStatus();
    return true;
  } catch (err) {
    console.warn('[ws] publish command failed:', err);
    return false;
  }
}

/**
 * 发布行为命令
 *  - 常规 FSM 命令: 3 帧 burst, interval=500ms,总计 1.0s(对齐旧 CLI --times 3 --rate 2)
 *  - 置零类事件命令 zero / zero_motor <id>: 只发 1 帧。需遍历电机写零、
 *    耗时约 2s, burst 叠发会打断/重复电机写零流程
 *  - 同一时刻新的行为命令进来会 cancel 上一个未发完的 burst(避免叠发)
 * @param {string} cmd 允许值:'getup' | 'getdown' | 'passive' | 'locomotion' | 'vel_stop' | 'zero' | 'zero_motor <motor_id 0-15>'
 * @returns {boolean} true = 已开始发布;false = 未连接/参数非法
 */
const SINGLE_SHOT_COMMANDS = new Set(['zero']);
const ZERO_MOTOR_CMD_RE = /^zero_motor\s+([0-9]{1,2})$/;
let _commandBurstTimerId = null;
export function publishBehaviorCommand(cmd) {
  const zeroMotorMatch = ZERO_MOTOR_CMD_RE.exec(cmd);
  const isZeroMotor = !!zeroMotorMatch && Number(zeroMotorMatch[1]) <= 15;
  const allowed = new Set(['getup', 'getdown', 'passive', 'locomotion', 'vel_stop', 'zero']);
  if (!allowed.has(cmd) && !isZeroMotor) {
    console.warn('[ws] publishBehaviorCommand:非法命令', cmd);
    return false;
  }
  if (connectionStatus !== 'online' || !isWsOpen()) {
    console.warn('[ws] publishBehaviorCommand 跳过:未连接');
    return false;
  }
  if (_commandBurstTimerId) {
    clearInterval(_commandBurstTimerId);
    _commandBurstTimerId = null;
  }
  // 置零类: 单发事件命令, 不进入 burst
  if (SINGLE_SHOT_COMMANDS.has(cmd) || isZeroMotor) {
    publishOneCommand(cmd);
    console.debug(`[ws] 📤 publish '${cmd}' (single-shot)`);
    return true;
  }
  let remaining = 3;
  const tick = () => {
    publishOneCommand(cmd);
    console.debug(`[ws] 📤 publish '${cmd}' (${3 - remaining + 1}/3)`);
    remaining -= 1;
    if (remaining <= 0 && _commandBurstTimerId) {
      clearInterval(_commandBurstTimerId);
      _commandBurstTimerId = null;
    }
  };
  tick(); // 不等待,立即发第 1 帧
  _commandBurstTimerId = setInterval(tick, COMMAND_INTERVAL_MS);
  return true;
}

/** 立即发一帧速度到 cmd_vel 通道(扁平字段 vx/vy/yaw;vz 仅本地记录) */
function publishOneTwist(vx, vy, omega, vz = 0) {
  if (!isWsOpen()) return false;
  const x = Number(vx) || 0;
  const y = Number(vy) || 0;
  const z = Number(omega) || 0;
  const w = Number(vz) || 0;
  try {
    wsSend({ type: CMD_VEL_TOPIC, vx: x, vy: y, yaw: z });
    cmdVelPublishCount += 1;
    lastCmdVelPublishAt = performance.now();
    lastCmdVel = {
      linear:  { x, y, z: w },
      angular: { x: 0, y: 0, z },
    };
    // 避免 UI 每 50ms 重绘整颗顶栏胶囊(太占主线程),cmdVel 计数改 ~1fps 才 emit
    if (cmdVelPublishCount % 20 === 1) emitStatus();
    return true;
  } catch (err) {
    console.warn('[ws] publish cmd_vel failed:', err);
    return false;
  }
}

/** 启动 cmd_vel 20Hz keep-alive 定时器(若未启动) */
function ensureCmdVelTimer() {
  if (cmdVelTimerId) return;
  cmdVelTimerId = setInterval(() => {
    const { vx, vy, omega, vz } = cmdVelRequest;
    const isZero = Math.abs(vx) < 1e-6 && Math.abs(vy) < 1e-6 && Math.abs(omega) < 1e-6 && Math.abs(vz) < 1e-6;
    if (isZero) {
      if (cmdVelZeroFramesRemaining > 0) {
        publishOneTwist(0, 0, 0, 0);
        cmdVelZeroFramesRemaining -= 1;
      } else {
        // 已经是 0,且零速度尾帧都发完了 → 停 timer 省 CPU
        if (cmdVelTimerId) { clearInterval(cmdVelTimerId); cmdVelTimerId = null; }
      }
    } else {
      // 非零 → 保持 20Hz 持续发布
      publishOneTwist(vx, vy, omega, vz);
    }
  }, CMD_VEL_INTERVAL_MS);
}

/**
 * 设置下一个持续对外发布的 cmd_vel
 *  - 非零:启动 20Hz keep-alive 定时器,每 50ms 发一帧
 *  - 归零:按停止约定,再追加 5 帧零速度才停 timer(避免真机收到一次零后又被 keepalive 逻辑卡)
 * @param {number} vx linear.x (m/s)
 * @param {number} vy linear.y (m/s)
 * @param {number} omega angular.z (rad/s)
 * @param {number=} vz linear.z 垂直速度 (m/s);不传(undefined)则保持当前值,传数字则覆盖
 * @returns {boolean} true=已接受请求,false=未连接
 */
export function setCmdVelRequest(vx, vy, omega, vz) {
  if (connectionStatus !== 'online' || !isWsOpen()) return false;
  cmdVelRequest.vx = Number(vx) || 0;
  cmdVelRequest.vy = Number(vy) || 0;
  cmdVelRequest.omega = Number(omega) || 0;
  if (vz !== undefined) cmdVelRequest.vz = Number(vz) || 0;
  const isZero =
    Math.abs(cmdVelRequest.vx) < 1e-6 &&
    Math.abs(cmdVelRequest.vy) < 1e-6 &&
    Math.abs(cmdVelRequest.omega) < 1e-6 &&
    Math.abs(cmdVelRequest.vz) < 1e-6;
  if (isZero) {
    cmdVelZeroFramesRemaining = 5; // 尾帧:发 5 帧零速度
  } else {
    cmdVelZeroFramesRemaining = 0; // 非零时清掉"尾帧计数",避免停掉新的速度请求
  }
  ensureCmdVelTimer();
  return true;
}

/** 单独设置垂直速度 linear.z(摇杆/平移滑块不影响该通道;复位与停止会清零) */
export function setCmdVelVz(vz) {
  return setCmdVelRequest(cmdVelRequest.vx, cmdVelRequest.vy, cmdVelRequest.omega, vz);
}

/** 手动"停止底盘" = 立即全部置零(含垂直速度) + 安排 5 帧零速度尾帧(更直观) */
export function stopCmdVel() {
  return setCmdVelRequest(0, 0, 0, 0);
}

/** 读当前对外的 cmd_vel 请求(UI/调试用) */
export function getCmdVelRequest() {
  return { ...cmdVelRequest };
}
