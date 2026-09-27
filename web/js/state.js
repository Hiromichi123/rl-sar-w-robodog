/**
 * 全局共享状态。
 *
 * 集中存放跨模块读写的集合与可变引用,避免模块间循环依赖。
 *  - 集合(jointStates/bodyObjects 等):在模型加载时填充,其他模块只读
 *  - shared 对象:持有 modelRoot 跨模块可变引用
 *  - annotationVectors:注释模块复用的临时向量,避免每帧分配
 *  - currentModelId:当前加载的模型目录名(唯一四足模型 '2')
 *  - MODEL_REGISTRY:可用模型列表(当前仅一个)
 */
import * as THREE from 'three';

// ── 模型注册表 ───────────────────────────────────────────
// 每个条目包含:
//   id:       唯一标识
//   label:    显示名
//   xmlPath:  XML 模型文件相对项目根的路径
//   meshDir:  网格文件相对项目根的目录
//   format:   网格格式 'obj' | 'stl'
// 新模型在 model/<id>/ 下放对应文件并在此注册即可
export const MODEL_REGISTRY = [
  { id: '2', label: '四足机器人', xmlPath: 'model/2/JXG.xml', meshDir: 'model/2/meshes/', format: 'obj' },
];

/** 当前加载的模型 ID(目录名),默认第一个 */
export let currentModelId = MODEL_REGISTRY[0].id;

/** 更新当前模型 ID(仅用于记录,不触发加载) */
export function setCurrentModelId(id) {
  currentModelId = id;
}

// ── 共享集合 ─────────────────────────────────────────────
// 关节状态表: name -> { name, pivot, content, axis, type, min, max, value, target, velocity, controlMode, ... }
export const jointStates = new Map();
// 刚体对象表: body name -> THREE.Group
export const bodyObjects = new Map();
// 受控关节列表(用于渲染控制面板)
export const visibleControls = [];
// 双臂笛卡尔控制器: arm -> { solver, states, page, status, controls, values, marker, frameRequest, initialJoints }
export const cartesianArms = new Map();
// 关节注释条目: { state, line, marker, label, element, layout }
export const jointAnnotations = [];
// 活跃步进器停止函数集合(用于全局中断长按)
export const activeStepperStops = new Set();

// ── 注释布局用的共享临时向量(避免每帧分配) ──────────────
export const annotationVectors = {
  anchorWorld: new THREE.Vector3(),
  anchorLocal: new THREE.Vector3(),
  endpointWorld: new THREE.Vector3(),
  endpointLocal: new THREE.Vector3(),
  rootWorld: new THREE.Vector3(),
  cameraRight: new THREE.Vector3(),
  cameraUp: new THREE.Vector3(),
};

// ── 跨模块共享的可变引用 ─────────────────────────────────
export const shared = {
  modelRoot: null,        // 机器人模型根 Group
  /**
   * 底盘速度钩子:所有"设置底盘速度命令"的入口(setChassisCommand / 四足 .setCommand/.stop)
   * 都会同步回调。main.js 用来把相同命令 publish 到真机 /rl_real/cmd_vel,ROS 离线时直接 return。
   * 参数:vx(m/s, +forward), vy(m/s, +left), omega(rad/s, +ccw yaw)
   */
  chassisCommandHook: null,
  /** 真机 IMU 读数(来自 /rl_real/imu_state Float32MultiArray[19]),字段定义见 ros-bridge.js IMU_FIELDS */
  imu: {
    quat: { w: 1, x: 0, y: 0, z: 0 },
    rpy:  { roll: 0, pitch: 0, yaw: 0 },
    acc:  { x: 0, y: 0, z: 0 },
    gyro: { x: 0, y: 0, z: 0 },
    pos:  { north: 0, east: 0, down: 0 },
    vel:  { x: 0, y: 0, z: 0 },
    lastMsgAt: 0,
    msgCount:  0,
  },
  /**
   * 虚拟摇杆最新输出(事件驱动,onMove 回调写入;每帧由 curves.js 采样)。
   * nx / ny: 归一化输入 [-1,1] (右=+nx, 上=+ny)
   * vx / vy: 换算后的底盘速度指令 (m/s, 人形满偏 0.6, 四足 0.5)
   * omega:   角速度指令 (rad/s, 手柄右摇杆左右, 满偏 0.6/0.5)
   */
  joystick: {
    nx: 0, ny: 0,
    vx: 0, vy: 0, omega: 0,
    lastUpdateAt: 0,  // 最近一次 onMove 的 performance.now() (ms)
    msgCount: 0,      // 累计 onMove 触发次数
    decayStartAt: 0,     // 0=不在衰减, >0=衰减开始时间戳
    decayDuration: 300,  // 衰减持续时间(ms)
    decayFrom: { nx: 0, ny: 0, vx: 0, vy: 0, omega: 0 }, // 衰减起点快照
  },
  /** 当前底盘摇杆回调(方向键/手柄复用): { onMove(nx,ny,omega?), onEnd() } — omega 为右摇杆归一化输入 [-1,1] */
  joystickInput: null,
  /** 真机里程计位姿(来自 /rl_real/pose2d Float32MultiArray[7]),字段定义见 ros-bridge.js POSE2D_FIELDS */
  rosPose2d: {
    x: 0, y: 0, z: 0,
    yaw: 0,
    velX: 0, velY: 0,
    yawRate: 0,
    lastMsgAt: 0,
    msgCount: 0,
  },
};

// ── 清理/重置 ────────────────────────────────────────────
/**
 * 清空所有模型相关的状态并从场景中移除 modelRoot。
 * 在切换模型前必须调用,避免新旧模型同时存在于场景/控制面板。
 * @param {THREE.Scene} scene - Three.js 场景(用于移除 modelRoot)
 * @param {HTMLElement} controlsEl - 关节控制 DOM 容器(清空 children)
 */
export function clearAllModelState(scene, controlsEl) {
  // 1. 移除 3D 场景中的模型根节点(含所有子对象 / 注释 / 连线 / 标记)
  if (shared.modelRoot) {
    scene.remove(shared.modelRoot);
    shared.modelRoot.traverse?.((node) => {
      if (node.isMesh) {
        node.geometry?.dispose?.();
        if (Array.isArray(node.material)) {
          node.material.forEach((m) => m.dispose?.());
        } else {
          node.material?.dispose?.();
        }
      }
    });
    shared.modelRoot = null;
  }

  // 2. 清空集合与列表
  jointStates.clear();
  bodyObjects.clear();
  visibleControls.length = 0;
  cartesianArms.clear();
  // 清理注释 DOM 元素
  for (const entry of jointAnnotations) {
    entry.element?.remove?.();
    entry.line?.geometry?.dispose?.();
    entry.line?.material?.dispose?.();
    entry.marker?.geometry?.dispose?.();
    entry.marker?.material?.dispose?.();
  }
  jointAnnotations.length = 0;
  for (const stop of activeStepperStops) {
    try { stop(); } catch { /* 忽略已结束的步进器 */ }
  }
  activeStepperStops.clear();

  // 3. 清空控制面板 DOM
  if (controlsEl) controlsEl.innerHTML = '';
}
