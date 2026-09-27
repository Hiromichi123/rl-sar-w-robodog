/**
 * 笛卡尔空间控制模块(四足机器人)。
 *
 * 职责:
 *  - 四足腿 IK 步进器 UI 绑定(位置 x/y/z)
 *  - 逆运动学(IK)求解调度与状态显示(收敛/不可达)
 *  - 腿部复位与位姿同步
 *  - 四足底盘运动控制(摇杆 + 键盘)
 *  - 机身位姿步进器与地面贴合约束
 *
 * 与 main.js 的协作:
 *  - 模型加载完成后由 model.js 调用 initializeCartesianControl 完成初始化
 *  - main.js 渲染循环调用 scheduleArmSolve/syncArmTarget 等驱动 IK 更新
 */
import * as THREE from 'three';
import { Matrix, inverse } from 'ml-matrix';
import { jointStates, bodyObjects, visibleControls, cartesianArms, activeStepperStops, shared, currentModelId } from './state.js?v=1';
import { setJointPose, syncStateInputs, stopActiveStepperMotion } from './joints.js?v=1';
import { createJoystick } from './joystick.js?v=1001';

// ── 摇杆速度上限:线速度 + 角速度两个连续可调滑条(D-pad 与手柄同样生效) ──
/** 速度上限的滑条量程 */
export const JOY_SPEED_MAX = 1.5;
/** 满偏线速度上限 m/s(滑条 0~1.5,默认 0.5) */
let joyMaxLinear = 0.5;
/** 满偏角速度上限 rad/s(滑条 0~1.5,默认 0.5,与原右摇杆硬编码一致) */
let joyMaxAngular = 0.5;
/** 夹到 [0, JOY_SPEED_MAX],非法值按 0 处理 */
function clampSpeedLimit(v) {
  const n = Number(v);
  if (!Number.isFinite(n)) return 0;
  return Math.max(0, Math.min(JOY_SPEED_MAX, n));
}
export function getJoystickMaxLinear() { return joyMaxLinear; }
export function setJoystickMaxLinear(v) { joyMaxLinear = clampSpeedLimit(v); return joyMaxLinear; }
export function getJoystickMaxAngular() { return joyMaxAngular; }
export function setJoystickMaxAngular(v) { joyMaxAngular = clampSpeedLimit(v); return joyMaxAngular; }

// ── 数值逆运动学求解器(DLS 阻尼最小二乘法)──────────────────

const _worldPosition = new THREE.Vector3();
const _perturbedPosition = new THREE.Vector3();
const _worldQuaternion = new THREE.Quaternion();
const _perturbedQuaternion = new THREE.Quaternion();
const _rootQuaternion = new THREE.Quaternion();

function rotationVector(from, to) {
  const delta = to.clone().multiply(from.clone().invert()).normalize();
  if (delta.w < 0) delta.set(-delta.x, -delta.y, -delta.z, -delta.w);
  const sine = Math.hypot(delta.x, delta.y, delta.z);
  if (sine < 1e-9) return new THREE.Vector3();
  const angle = 2 * Math.atan2(sine, Math.max(1e-9, delta.w));
  return new THREE.Vector3(delta.x / sine, delta.y / sine, delta.z / sine).multiplyScalar(angle);
}

class IKSolver {
  constructor({ states, tcp, root, applyJointPose }) {
    this.states = states;
    this.tcp = tcp;
    this.root = root;
    this.applyJointPose = applyJointPose;
    this.maxIterations = 60;
    this.positionTolerance = 0.0015;
    this.orientationTolerance = 0.025;
    this.damping = 0.055;
    this.stepLimit = 0.16;
  }

  currentLocalPose() {
    this.root.updateMatrixWorld(true);
    const position = this.tcp.getWorldPosition(new THREE.Vector3());
    this.root.worldToLocal(position);
    const tcpQuaternion = this.tcp.getWorldQuaternion(new THREE.Quaternion());
    const localQuaternion = this.root.getWorldQuaternion(new THREE.Quaternion()).invert().multiply(tcpQuaternion);
    return { position, quaternion: localQuaternion.normalize() };
  }

  solve(targetPositionLocal, targetQuaternionLocal = null, {
    positionWeight = 1,
    orientationWeight = 1,
  } = {}) {
    const targetPosition = this.root.localToWorld(targetPositionLocal.clone());
    this.root.getWorldQuaternion(_rootQuaternion);
    const targetQuaternion = targetQuaternionLocal
      ? _rootQuaternion.clone().multiply(targetQuaternionLocal).normalize()
      : null;

    const q = this.states.map((state) => state.value);
    let positionError = Infinity;
    let orientationError = targetQuaternion ? Infinity : 0;

    for (let iteration = 0; iteration < this.maxIterations; iteration += 1) {
      this.#apply(q);
      this.tcp.getWorldPosition(_worldPosition);
      this.tcp.getWorldQuaternion(_worldQuaternion);

      const positionDelta = targetPosition.clone().sub(_worldPosition);
      const orientationDelta = targetQuaternion
        ? rotationVector(_worldQuaternion, targetQuaternion)
        : new THREE.Vector3();
      positionError = positionDelta.length();
      orientationError = orientationDelta.length();

      const positionSatisfied = positionWeight === 0 || positionError <= this.positionTolerance;
      if (positionSatisfied && orientationError <= this.orientationTolerance) break;

      const rowCount = targetQuaternion ? 6 : 3;
      const jacobian = Array.from({ length: rowCount }, () => Array(this.states.length).fill(0));
      const epsilon = 1e-4;
      for (let jointIndex = 0; jointIndex < this.states.length; jointIndex += 1) {
        const original = q[jointIndex];
        q[jointIndex] = Math.min(this.states[jointIndex].max, original + epsilon);
        const actualStep = q[jointIndex] - original;
        if (actualStep < 1e-8) {
          q[jointIndex] = Math.max(this.states[jointIndex].min, original - epsilon);
        }
        const signedStep = q[jointIndex] - original;
        this.#apply(q);
        this.tcp.getWorldPosition(_perturbedPosition);
        this.tcp.getWorldQuaternion(_perturbedQuaternion);
        jacobian[0][jointIndex] = (_perturbedPosition.x - _worldPosition.x) / signedStep;
        jacobian[1][jointIndex] = (_perturbedPosition.y - _worldPosition.y) / signedStep;
        jacobian[2][jointIndex] = (_perturbedPosition.z - _worldPosition.z) / signedStep;
        if (targetQuaternion) {
          const angularStep = rotationVector(_worldQuaternion, _perturbedQuaternion).divideScalar(signedStep);
          jacobian[3][jointIndex] = angularStep.x;
          jacobian[4][jointIndex] = angularStep.y;
          jacobian[5][jointIndex] = angularStep.z;
        }
        q[jointIndex] = original;
      }

      const error = [positionDelta.x, positionDelta.y, positionDelta.z];
      if (targetQuaternion) error.push(orientationDelta.x, orientationDelta.y, orientationDelta.z);
      const rowWeights = targetQuaternion
        ? [positionWeight, positionWeight, positionWeight, orientationWeight, orientationWeight, orientationWeight]
        : [positionWeight, positionWeight, positionWeight];
      for (let row = 0; row < rowCount; row += 1) {
        error[row] *= rowWeights[row];
        for (let column = 0; column < this.states.length; column += 1) {
          jacobian[row][column] *= rowWeights[row];
        }
      }

      try {
        const matrix = new Matrix(jacobian);
        const transpose = matrix.transpose();
        const regularized = matrix.mmul(transpose).add(Matrix.eye(rowCount).mul(this.damping ** 2));
        const delta = transpose.mmul(inverse(regularized)).mmul(Matrix.columnVector(error));
        for (let jointIndex = 0; jointIndex < q.length; jointIndex += 1) {
          const step = THREE.MathUtils.clamp(delta.get(jointIndex, 0), -this.stepLimit, this.stepLimit);
          q[jointIndex] = THREE.MathUtils.clamp(q[jointIndex] + step, this.states[jointIndex].min, this.states[jointIndex].max);
        }
      } catch {
        break;
      }
    }

    this.#apply(q);
    this.states.forEach((state, index) => {
      state.value = q[index];
      state.target = q[index];
    });
    return {
      success: (positionWeight === 0 || positionError < 0.012) && orientationError < 0.12,
      positionError,
      orientationError,
      joints: [...q],
    };
  }

  #apply(values) {
    this.states.forEach((state, index) => {
      state.value = values[index];
      this.applyJointPose(state, values[index]);
    });
    this.root.updateMatrixWorld(true);
  }
}

// ── 笛卡尔步进器 UI ──────────────────────────────────────

/**
 * 为已存在的步进器 DOM 绑定长按重复触发行为。
 * @param {Object} opts
 * @param {HTMLElement} opts.row - 步进器容器(含 span/output/button/button)
 * @param {string} opts.arm - 'left' | 'right'
 * @param {number} opts.value - 初始值
 * @param {number} opts.min, opts.max - 范围(wrap=false 时生效)
 * @param {number} opts.step - 每次步进量
 * @param {number} opts.precision - 显示小数位
 * @param {boolean} opts.wrap - 为 true 时按角度环绕归一化
 * @param {Function} opts.onInput - 值变化回调
 */
function bindStepper({ row, arm, value, min, max, step, precision, wrap = false, onInput, getValue }) {
  const [decreaseButton, increaseButton] = row.querySelectorAll('button');
  const output = row.querySelector('output');
  const label = row.querySelector('span').textContent;
  const normalize = (next) => wrap
    ? Math.atan2(Math.sin(next), Math.cos(next))
    : THREE.MathUtils.clamp(next, min, max);
  const readValue = () => getValue ? getValue() : value;
  const update = (next) => {
    const normalized = normalize(next);
    output.value = normalized.toFixed(precision);
    onInput(normalized);
  };
  output.value = value.toFixed(precision);

  const LEG_LABELS = { FL: '左前腿', FR: '右前腿', RL: '左后腿', RR: '右后腿' };

  /** 配置单个步进按钮的长按重复触发 */
  const configureButton = (button, direction) => {
    const axisLabel = label.split(' /')[0];
    let armLabel;
    if (arm === 'left') armLabel = '左臂';
    else if (arm === 'right') armLabel = '右臂';
    else if (arm === 'body') armLabel = '机身';
    else armLabel = LEG_LABELS[arm] || arm;
    const action = direction < 0 ? '减小' : '增加';
    button.setAttribute('aria-label', `${armLabel} ${axisLabel} ${action}`);
    button.title = `${action} ${axisLabel}（步长 ${step}）`;
    let repeatDelay = 0;
    let repeatInterval = 0;
    const advance = () => update(readValue() + direction * step);
    const stop = () => {
      window.clearTimeout(repeatDelay);
      window.clearInterval(repeatInterval);
      repeatDelay = 0;
      repeatInterval = 0;
      button.classList.remove('pressed');
      activeStepperStops.delete(stop);
    };
    button.addEventListener('pointerdown', (event) => {
      if (event.button !== 0 || !event.isPrimary) return;
      event.preventDefault();
      stopActiveStepperMotion();
      button.classList.add('pressed');
      activeStepperStops.add(stop);
      advance();
      repeatDelay = window.setTimeout(() => {
        repeatInterval = window.setInterval(advance, 90);
      }, 280);
    });
    button.addEventListener('pointerup', stop);
    button.addEventListener('pointerleave', stop);
    button.addEventListener('pointercancel', stop);
    button.addEventListener('click', (event) => {
      if (event.detail === 0) advance();
    });
  };
  configureButton(decreaseButton, -1);
  configureButton(increaseButton, 1);
  return {
    row,
    setValue(next) {
      value = normalize(next);
      output.value = value.toFixed(precision);
    },
  };
}

// ── 逆运动学求解 ─────────────────────────────────────────

/**
 * 求解单臂/单腿 IK:将末端移动到 controller.values 指定的位姿。
 * 位置目标取自 values.x/y/z;姿态目标仅在 #ik-orientation 勾选时生效。
 * 对于腿 (FL/FR/RL/RR),始终为位置-only 求解 (3DOF → 3位置轴)。
 * @param {'left' | 'right' | 'FL' | 'FR' | 'RL' | 'RR'} arm - 目标臂/腿
 * @param {Object} solverOptions - 透传给 IKSolver.solve 的选项(如 positionWeight)
 * @returns {Object|null} IK 求解结果,含 success/positionError 等
 */
export function solveArm(arm, solverOptions = {}) {
  const controller = cartesianArms.get(arm);
  if (!controller) return null;
  const targetPosition = new THREE.Vector3(controller.values.x, controller.values.y, controller.values.z);
  controller.marker.position.copy(targetPosition);
  const result = controller.solver.solve(targetPosition, null, solverOptions);
  controller.states.forEach((state) => syncStateInputs(state));
  const errorMillimeters = result.positionError * 1000;
  setArmStatus(
    controller,
    result.success ? `已收敛 · ${errorMillimeters.toFixed(1)} mm` : `目标不可达 · ${errorMillimeters.toFixed(1)} mm`,
    !result.success,
  );
  const isLeg = ['FL', 'FR', 'RL', 'RR'].includes(arm);
  if (isLeg) enforceGroundContact();
  return result;
}

/** 更新臂部状态文字(收敛/不可达) */
function setArmStatus(controller, text, unreachable = false) {
  controller.status.textContent = text;
  controller.status.classList.toggle('unreachable', unreachable);
}

/** 计算两个四元数间的角距离(用于判断姿态是否已稳定) */
function quaternionDistance(first, second) {
  return 2 * Math.acos(Math.min(1, Math.abs(first.dot(second))));
}

/** 在下一帧调度一次 IK 求解(避免重复请求) */
export function scheduleArmSolve(arm) {
  const controller = cartesianArms.get(arm);
  if (!controller || controller.frameRequest) return;
  controller.frameRequest = requestAnimationFrame(() => {
    controller.frameRequest = 0;
    solveArm(arm);
  });
}

/** 从当前关节状态同步末端位姿到控制器(用于复位后刷新) */
export function syncArmTarget(arm) {
  const controller = cartesianArms.get(arm);
  if (!controller) return;
  const pose = controller.solver.currentLocalPose();
  Object.assign(controller.values, {
    x: pose.position.x,
    y: pose.position.y,
    z: pose.position.z,
  });
  Object.entries(controller.controls).forEach(([name, control]) => {
    if (name in controller.values) control.setValue(controller.values[name]);
  });
  controller.marker.position.copy(pose.position);
  setArmStatus(controller, '当前末端位姿');
}

/** 将指定臂复位到初始关节角度 */
export function resetArm(arm) {
  const controller = cartesianArms.get(arm);
  if (!controller) return;
  if (controller.frameRequest) cancelAnimationFrame(controller.frameRequest);
  controller.frameRequest = 0;
  controller.states.forEach((state, index) => {
    state.value = controller.initialJoints[index];
    state.target = controller.initialJoints[index];
    setJointPose(state, state.value);
    syncStateInputs(state);
  });
  shared.modelRoot.updateMatrixWorld(true);
  syncArmTarget(arm);
  setArmStatus(controller, '已复位');
}

// ── 底盘运动控制 ─────────────────────────────────────────

/** 设置底盘速度指令(vx 前后、vy 横移、omega 旋转)。键盘等直接入口共用。 */
export function setChassisCommand(vx, vy, omega) {
  quadChassisController.setCommand(vx, vy, omega);
  try { shared.chassisCommandHook?.(vx, vy, omega); } catch (err) { console.warn('[cart] chassisCommandHook 异常:', err); }
}

/** 把 shared.joystick 的当前值(vx/vy/omega)组合推送到本地仿真 + ROS */
export function pushJoystickCommand() {
  const j = shared.joystick;
  const r2 = (v) => Math.round(v * 100) / 100;
  const vx = r2(j.vx), vy = r2(j.vy), omega = r2(j.omega);
  setChassisCommand(vx, vy, omega);
}

// ── 右摇杆 Z 轴:人形升降滑台 / 四足蹲起 ──────────────────
let _joystickLiftNorm = 0;   // 右摇杆纵向归一化值(衰减起点用)

/** 读取当前 Z 归一化值(松开衰减用) */
export function getJoystickLift() { return _joystickLiftNorm; }

/**
 * 右摇杆纵向 → Z 轴目标(四足蹲起):
 * 前后腿 thigh/calf 镜像偏移,上推伸展(接近直腿,机身升高),
 * 下拉收拢(蹲低);enforceGroundContact 保持足端贴地。
 * @param {number} ny - 右摇杆纵向 [-1,1],上=+1
 */
export function applyJoystickLift(ny) {
  _joystickLiftNorm = ny;
  const linZ = document.getElementById('twist-lin-z');
  if (linZ) linZ.value = ny.toFixed(2);
  const r3 = (v) => Math.round(v * 1000) / 1000;
  // 四足蹲起:基准 = 站立姿态(STANDING_ANGLES),ny>0 向 0 靠拢(伸直升高)
  const CROUCH_THIGH = 0.25, CROUCH_CALF = 0.45;
  const BASES = {
    FL_thigh_joint: [-0.85, +CROUCH_THIGH], FR_thigh_joint: [-0.85, +CROUCH_THIGH],
    RL_thigh_joint: [0.85, -CROUCH_THIGH],  RR_thigh_joint: [0.85, -CROUCH_THIGH],
    FL_calf_joint: [1.71, -CROUCH_CALF],    FR_calf_joint: [1.71, -CROUCH_CALF],
    RL_calf_joint: [-1.71, +CROUCH_CALF],   RR_calf_joint: [-1.71, +CROUCH_CALF],
  };
  for (const [name, [base, slope]] of Object.entries(BASES)) {
    const st = jointStates.get(name);
    if (!st) continue;
    st.target = r3(base + slope * ny);
    syncStateInputs(st);
  }
}

/** 左摇杆(移动 X/Y)handlers 工厂:ny→vx(上=前进),nx→vy(右推=向右) */
function makeLeftJoyHandlers() {
  return {
    onMove: (nx, ny) => {
      const j = shared.joystick;
      j.decayLeftStartAt = 0;   // 重新握住即取消衰减
      j.vx = Math.round(ny * getJoystickMaxLinear() * 100) / 100;
      j.vy = Math.round(-nx * getJoystickMaxLinear() * 100) / 100;  // +Y=左,右推应向右 → 取负
      j.nx = nx; j.ny = ny;
      pushJoystickCommand();
      j.lastUpdateAt = performance.now();
      j.msgCount += 1;
    },
    onEnd: () => {
      const j = shared.joystick;
      j.decayLeftVx = j.vx; j.decayLeftVy = j.vy;
      j.decayLeftStartAt = performance.now();
      j.lastUpdateAt = performance.now();
      j.msgCount += 1;
    },
  };
}

/** 右摇杆(Z/ω)handlers 工厂:ny→升降 Z,nx→ω(左推=左转),满偏角速度取滑条上限 */
function makeRightJoyHandlers() {
  return {
    onMove: (nx, ny) => {
      const j = shared.joystick;
      j.decayRightStartAt = 0;  // 重新握住即取消衰减
      j.omega = Math.round(-nx * getJoystickMaxAngular() * 100) / 100;
      j.znx = nx; j.zny = ny;
      applyJoystickLift(ny);
      pushJoystickCommand();
      j.lastUpdateAt = performance.now();
      j.msgCount += 1;
    },
    onEnd: () => {
      const j = shared.joystick;
      j.decayRightOmega = j.omega;
      j.decayRightLift = getJoystickLift();
      j.decayRightStartAt = performance.now();
      j.lastUpdateAt = performance.now();
      j.msgCount += 1;
    },
  };
}

// ── 四足机器人腿部 IK 控制 ────────────────────────────────

const LEG_PREFIXES = ['FL', 'FR', 'RL', 'RR'];

// ── 地面接触约束 ──────────────────────────────────────────

/** 地面 Z 坐标 (与 scene.js 中地板平面一致) */
const GROUND_Z = 0;
/** 足端 body 名称列表,用于查找 bodyObjects */
const FOOT_NAMES = ['FL_foot', 'FR_foot', 'RL_foot', 'RR_foot'];
/** 足端关节名称列表,用于轮子转动示意 */
const FOOT_JOINT_NAMES = ['FL_foot_joint', 'FR_foot_joint', 'RL_foot_joint', 'RR_foot_joint'];
/** 各足端在底盘坐标系中的横向位置(用于旋转时切向速度计算) */
const FOOT_LATERAL_POS = { FL: 0.15, FR: -0.15, RL: 0.15, RR: -0.15 };
/** 足端轮子半径估算值(m),用于线速度→角速度转换 */
const FOOT_WHEEL_RADIUS = 0.04;
/** 世界包围盒计算用临时对象 */
const _tmpBox = new THREE.Box3();
const _tmpBox2 = new THREE.Box3();
const _tmpVec = new THREE.Vector3();

/**
 * 获取足端 body 下所有网格几何体的世界包围盒最小 Z 值。
 * 遍历 body 子树中所有 Mesh,累积包围盒后返回 min.z。
 * 注意:调用方需已调用 modelRoot.updateMatrixWorld(true) 确保矩阵最新。
 * @param {THREE.Object3D} footBody - 足端 body 节点
 * @returns {number} 世界坐标下的最小 Z 值
 */
function getFootMinWorldZ(footBody) {
  let hasGeometry = false;
  _tmpBox.makeEmpty();
  footBody.traverse((obj) => {
    if (obj.isMesh && obj.geometry) {
      const geom = obj.geometry;
      if (!geom.boundingBox) geom.computeBoundingBox();
      const localBox = geom.boundingBox;
      _tmpBox2.copy(localBox).applyMatrix4(obj.matrixWorld);
      _tmpBox.union(_tmpBox2);
      hasGeometry = true;
    }
  });
  if (!hasGeometry) {
    footBody.getWorldPosition(_tmpVec);
    return _tmpVec.z;
  }
  return _tmpBox.min.z;
}

/**
 * 强制四足机器人足端触地:
 * 计算所有足端网格几何体的世界包围盒最小 Z 坐标,
 * 若最低足端低于地面,则抬升模型根节点,
 * 确保足端始终接触地面但不陷入。
 * 同时自动调整机身 Z 步进器显示。
 */

/** 地面贴合模式:为 true 时自动调整机身 Z 使足端贴地 */
export let groundSnapMode = true;

/** 设置地面贴合模式(供外部模块调用) */
export function setGroundSnapMode(value) { groundSnapMode = value; }

/**
 * 足端最小 Z 的 EMA(指数移动平均)缓存。
 * 用于过滤轮子旋转引起的高频包围盒波动,同时保持贴地检测的连续性。
 */
let smoothedFootMinZ = null;
/** 贴地检测节流:不需要每帧执行,30fps 足够(约 33ms 间隔) */
let _groundCheckLastAt = 0;

/**
 * 为四足机器人执行足端触地保障。
 * 节流到 ~30fps(地面贴合不需要 60fps),使用 EMA 过滤轮子旋转导致的高频包围盒波动,
 * 每次执行都对机身 Z 进行平滑修正,确保足端始终贴地,消除启停时的跳动。
 * @param {number} dt - 帧间隔时间(秒)
 */
export function enforceGroundContact(dt = 1 / 60) {
  if (!shared.modelRoot) return;
  const isQuadruped = jointStates.has('FL_hip_joint');
  if (!isQuadruped) return;
  if (!groundSnapMode) return;

  // 节流:~30fps 足够,避免每帧 traverse 4 个足端子树 + 包围盒计算的开销
  const now = performance.now();
  if (now - _groundCheckLastAt < 33) return;
  _groundCheckLastAt = now;

  shared.modelRoot.updateMatrixWorld(true);

  let minFootZ = Infinity;

  for (const name of FOOT_NAMES) {
    const foot = bodyObjects.get(name);
    if (!foot) continue;
    const footMinZ = getFootMinWorldZ(foot);
    if (footMinZ < minFootZ) minFootZ = footMinZ;
  }

  if (!isFinite(minFootZ)) return;

  // 用 EMA 平滑足端 Z 测量值,过滤轮子旋转引起的高频波动
  // rate=15 足够快以响应姿态变化,足够慢以过滤轮子旋转噪声
  if (smoothedFootMinZ === null) {
    smoothedFootMinZ = minFootZ;
  } else {
    smoothedFootMinZ = THREE.MathUtils.damp(smoothedFootMinZ, minFootZ, 15, dt);
  }

  const currentBodyZ = shared.modelRoot.position.z;
  const targetZ = currentBodyZ + (GROUND_Z - smoothedFootMinZ);

  const diff = Math.abs(targetZ - currentBodyZ);

  // 偏差极小无需调整
  if (diff < 1e-4) return;

  // 使用阻尼平滑修正机身 Z,避免任何瞬态跳动
  const correctedZ = THREE.MathUtils.damp(currentBodyZ, targetZ, 12, dt);
  shared.modelRoot.position.z = correctedZ;
  shared.modelRoot.updateMatrixWorld(true);
  bodyPose.z = correctedZ;
  syncBodyPoseInputs();
  updateBodyPoseReadout();
}

// ── 姿态预设:站立 ──────────────────────────────────────────

/** 四足机器人站立关节角度(与模型初始加载姿态一致) */
const STANDING_ANGLES = {
  FL_hip_joint: 0, FL_thigh_joint: -0.85, FL_calf_joint: 1.71, FL_foot_joint: 0,
  FR_hip_joint: 0, FR_thigh_joint: -0.85, FR_calf_joint: 1.71, FR_foot_joint: 0,
  RL_hip_joint: 0, RL_thigh_joint: 0.85, RL_calf_joint: -1.71, RL_foot_joint: 0,
  RR_hip_joint: 0, RR_thigh_joint: 0.85, RR_calf_joint: -1.71, RR_foot_joint: 0,
};

/** 站立时机身 Z 高度 */
const STANDING_Z = 0.15;

/**
 * 应用姿态预设:平滑过渡关节角度和机身高度。
 * 通过设置 joint target 实现阻尼插值过渡,同时启用地面贴合。
 * @param {Object} angles - 关节角度映射表
 * @param {number} targetZ - 目标机身 Z 高度(初始值,实际由 enforceGroundContact 调整)
 */
function applyPosture(angles, targetZ) {
  // 重置 EMA 缓存,让 enforceGroundContact 重新测量新姿态下的足端位置
  smoothedFootMinZ = null;
  // 启用地面贴合模式
  groundSnapMode = true;
  for (const [name, angle] of Object.entries(angles)) {
    const state = jointStates.get(name);
    if (!state) continue;
    const clamped = THREE.MathUtils.clamp(angle, state.min, state.max);
    state.target = clamped;
    syncStateInputs(state);
  }
  // 设置初始机身 Z,enforceGroundContact 会持续调整
  bodyPose.z = targetZ;
  shared.modelRoot.position.z = targetZ;
  syncBodyPoseInputs();
  updateBodyPoseReadout();
  // 同步腿部 IK 控制器
  for (const legId of LEG_PREFIXES) {
    syncArmTarget(legId);
  }
}

/** 绑定姿态预设按钮(站立) */
function setupStandPostureControl() {
  document.querySelector('#posture-stand')?.addEventListener('click', () => {
    applyPosture(STANDING_ANGLES, STANDING_Z);
  });
}

// ── 四足机器人底盘运动控制 ──────────────────────────────────

/** 四足机器人底盘运动控制器:直接积分速度指令到 modelRoot */
const quadChassisController = {
  command: { vx: 0, vy: 0, omega: 0 },
  pose: { x: 0, y: 0, yaw: 0 },
  enabled: false,

  setCommand(vx, vy, omega) {
    this.command.vx = vx;
    this.command.vy = vy;
    this.command.omega = omega;
  },

  stop() {
    this.setCommand(0, 0, 0);
  },

  resetPose() {
    this.pose = { x: 0, y: 0, yaw: 0 };
    this.command = { vx: 0, vy: 0, omega: 0 };
    if (shared.modelRoot) {
      shared.modelRoot.position.x = 0;
      shared.modelRoot.position.y = 0;
      shared.modelRoot.rotation.z = 0;
    }
    bodyPose.x = 0;
    bodyPose.y = 0;
    bodyPose.yaw = 0;
    syncBodyPoseInputs();
    updateBodyPoseReadout();
    this.updateReadout();
  },

  updateReadout() {
    const el = document.querySelector('#quad-base-pose');
    if (!el) return;
    const deg = THREE.MathUtils.radToDeg(this.pose.yaw);
    el.textContent = `X ${this.pose.x.toFixed(2)} · Y ${this.pose.y.toFixed(2)} · ${deg.toFixed(0)}°`;
  },

  /** 判断底盘是否有非零速度指令 */
  isMoving() {
    const { vx, vy, omega } = this.command;
    return Math.abs(vx) + Math.abs(vy) + Math.abs(omega) > 1e-4;
  },

  /** 每帧步进:将速度指令积分到位姿,更新 modelRoot,驱动足端轮子转动 */
  step(dt) {
    if (!this.enabled || !shared.modelRoot) return;
    const { vx, vy, omega } = this.command;
    const cosine = Math.cos(this.pose.yaw);
    const sine = Math.sin(this.pose.yaw);
    this.pose.x += (vx * cosine - vy * sine) * dt;
    this.pose.y += (vx * sine + vy * cosine) * dt;
    this.pose.yaw = Math.atan2(Math.sin(this.pose.yaw + omega * dt), Math.cos(this.pose.yaw + omega * dt));

    // 更新 modelRoot,保留 Z 和 roll/pitch
    shared.modelRoot.position.x = this.pose.x;
    shared.modelRoot.position.y = this.pose.y;
    shared.modelRoot.rotation.z = this.pose.yaw;

    // 驱动足端关节旋转(轮子转动示意)
    // 每个足端的线速度 = 底盘速度 + 旋转切向速度
    for (const name of FOOT_JOINT_NAMES) {
      const state = jointStates.get(name);
      if (!state) continue;
      const legPrefix = name.slice(0, 2);
      const lateralPos = FOOT_LATERAL_POS[legPrefix] ?? 0;
      // 旋转产生的切向 x 分量: omega × y
      const footVx = vx - omega * lateralPos;
      // 线速度 → 角速度: ω = v / r
      const wheelAngularVel = footVx / FOOT_WHEEL_RADIUS;
      // 累加到目标角度,动画循环的阻尼插值会平滑旋转
      state.target += wheelAngularVel * dt;
    }

    // 同步到 bodyPose,保持步进器显示一致
    bodyPose.x = this.pose.x;
    bodyPose.y = this.pose.y;
    bodyPose.yaw = this.pose.yaw;
    syncBodyPoseInputs();
    updateBodyPoseReadout();
    this.updateReadout();
  },
};

/** 初始化四足机器人底盘控制:绑定双摇杆(左=移动 X/Y,右=高度 Z/ω)
 *  导出供 main.js 在仿真关闭时也能提前绑定 —— 摇杆指令经 chassisCommandHook
 *  直发真机 /rl_real/cmd_vel,不依赖 3D 模型;createJoystick 幂等,模型加载后
 *  initializeCartesianControl 再次调用安全。 */
export function setupQuadrupedChassisControl() {
  quadChassisController.enabled = true;

  // 左摇杆:前后移动 + 横移,满偏线速度由「线速度」滑条决定(0~1.5 m/s,默认 0.5)
  // 右摇杆:上=蹲起升高 下=降低,左右=ω 旋转,满偏由「角速度」滑条决定(rad/s)
  const left = makeLeftJoyHandlers();
  const right = makeRightJoyHandlers();
  createJoystick(document.querySelector('#quad-chassis-joystick'), left);
  createJoystick(document.querySelector('#quad-chassis-joystick-z'), right);
  shared.joystickInput = left;
  shared.joystickZInput = right;

  // 复位按钮
  document.querySelector('#quad-base-reset')?.addEventListener('click', () => {
    quadChassisController.resetPose();
    setChassisCommand(0, 0, 0);
    enforceGroundContact();
  });

  quadChassisController.updateReadout();
}

/** 获取四足机器人底盘控制器(供 main.js 渲染循环调用) */
export function getQuadChassisController() {
  return quadChassisController;
}

/**
 * 为单腿创建 IK 求解器、目标标记球,并绑定 3 个位置步进器。
 * 使用 IKSolver:3 关节 (hip/thigh/calf) → 3 位置自由度 (x/y/z)。
 * @param {'FL'|'FR'|'RL'|'RR'} legId - 腿标识
 */
function setupLegCartesianControl(legId) {
  const states = [
    jointStates.get(`${legId}_hip_joint`),
    jointStates.get(`${legId}_thigh_joint`),
    jointStates.get(`${legId}_calf_joint`),
  ].filter(Boolean);
  const tcp = bodyObjects.get(`${legId}_foot`);
  if (states.length < 3 || !tcp) return;

  const solver = new IKSolver({ states, tcp, root: shared.modelRoot, applyJointPose: setJointPose });
  const pose = solver.currentLocalPose();
  const page = document.querySelector(`.leg-control-page[data-leg="${legId}"]`);
  const status = page.querySelector('.ik-msg');

  const controller = {
    solver,
    states,
    page,
    status,
    controls: {},
    initialJoints: states.map((state) => state.value),
    values: { x: pose.position.x, y: pose.position.y, z: pose.position.z },
    frameRequest: 0,
  };
  cartesianArms.set(legId, controller);

  const stepperConfigs = [
    ['x', pose.position.x - 0.15, pose.position.x + 0.15, 0.005, 3],
    ['y', pose.position.y - 0.15, pose.position.y + 0.15, 0.005, 3],
    ['z', -0.1, 0.25, 0.005, 3],
  ];
  for (const [name, min, max, step, precision] of stepperConfigs) {
    const row = page.querySelector(`.cartesian-stepper[data-axis="${name}"]`);
    const control = bindStepper({
      row, arm: legId, value: controller.values[name], min, max, step, precision, wrap: false,
      getValue: () => controller.values[name],
      onInput: (value) => {
        controller.values[name] = value;
        control.setValue(value);
        scheduleArmSolve(legId);
      },
    });
    controller.controls[name] = control;
  }

  page.querySelector('.leg-reset').addEventListener('click', () => resetArm(legId));

  const marker = new THREE.Mesh(
    new THREE.SphereGeometry(0.018, 16, 10),
    new THREE.MeshStandardMaterial({
      color: 0x44d0a0,
      emissive: 0x1a6040,
      emissiveIntensity: 0.4,
      transparent: true,
      opacity: 0.85,
    }),
  );
  marker.position.copy(pose.position);
  marker.visible = false;
  shared.modelRoot.add(marker);
  controller.marker = marker;
}

// ── 机身位姿控制 ──────────────────────────────────────────

const bodyPose = {
  x: 0, y: 0, z: 0.15,
  roll: 0, pitch: 0, yaw: 0,
  initialZ: 0.15,
};

/** 创建机身位姿步进器绑定 */
function setupBodyPoseControl() {
  const initialZ = shared.modelRoot.position.z || 0.15;
  bodyPose.z = initialZ;
  bodyPose.initialZ = initialZ;

  const stepperConfigs = [
    ['x', -0.5, 0.5, 0.01, 3, false],
    ['y', -0.5, 0.5, 0.01, 3, false],
    ['z', 0.0, 0.35, 0.01, 3, false],
    ['roll', -0.5, 0.5, 0.02, 2, false],
    ['pitch', -0.5, 0.5, 0.02, 2, false],
    ['yaw', -Math.PI, Math.PI, 0.05, 2, true],
  ];
  for (const [name, min, max, step, precision, wrap] of stepperConfigs) {
    const row = document.querySelector(`.cartesian-stepper[data-body-axis="${name}"]`);
    if (!row) continue;
    bindStepper({
      row, arm: 'body', value: bodyPose[name], min, max, step, precision, wrap,
      getValue: () => bodyPose[name],
      onInput: (value) => {
        // 手动调整机身位姿时退出地面贴合模式
        if (name === 'z') groundSnapMode = false;
        bodyPose[name] = value;
        applyBodyPose();
      },
    });
  }

  document.querySelector('#body-reset').addEventListener('click', () => {
    bodyPose.x = 0; bodyPose.y = 0; bodyPose.z = bodyPose.initialZ;
    bodyPose.roll = 0; bodyPose.pitch = 0; bodyPose.yaw = 0;
    applyBodyPose();
    syncBodyPoseInputs();
    // 四足机器人:归零后重新启用地面贴合,确保贴地
    if (jointStates.has('FL_hip_joint')) {
      groundSnapMode = true;
      enforceGroundContact();
    }
  });

  updateBodyPoseReadout();
}

/** 应用机身位姿到模型根节点 */
function applyBodyPose() {
  if (!shared.modelRoot) return;
  shared.modelRoot.position.set(bodyPose.x, bodyPose.y, bodyPose.z);
  shared.modelRoot.rotation.set(bodyPose.roll, bodyPose.pitch, bodyPose.yaw);
  // 同步底盘控制器位姿,避免与步进器冲突
  if (jointStates.has('FL_hip_joint')) {
    quadChassisController.pose.x = bodyPose.x;
    quadChassisController.pose.y = bodyPose.y;
    quadChassisController.pose.yaw = bodyPose.yaw;
    quadChassisController.updateReadout();
  }
  updateBodyPoseReadout();
  enforceGroundContact();
}

/** 同步机身位姿输入控件显示 */
function syncBodyPoseInputs() {
  const stepperConfigs = ['x', 'y', 'z', 'roll', 'pitch', 'yaw'];
  for (const name of stepperConfigs) {
    const row = document.querySelector(`.cartesian-stepper[data-body-axis="${name}"]`);
    if (!row) continue;
    const output = row.querySelector('output');
    if (output) output.textContent = (name === 'x' || name === 'y' || name === 'z')
      ? bodyPose[name].toFixed(3) : bodyPose[name].toFixed(2);
  }
}

/** 更新机身位姿读数 */
function updateBodyPoseReadout() {
  const el = document.querySelector('#body-pose-readout');
  if (!el) return;
  el.textContent = `X ${bodyPose.x.toFixed(2)} · Y ${bodyPose.y.toFixed(2)} · Z ${bodyPose.z.toFixed(2)}`;
}

// ── 笛卡尔控制总初始化 ───────────────────────────────────

/** 初始化笛卡尔控制 */
export function initializeCartesianControl() {
  // 重置地面接触 EMA 缓存,确保模型切换后重新测量
  smoothedFootMinZ = null;
  document.querySelector('#quadruped-cartesian').hidden = false;
  document.querySelector('#movement-quadruped-chassis').hidden = false;
  setupStandPostureControl();
  setupQuadrupedChassisControl();
  setupBodyPoseControl();
  for (const legId of LEG_PREFIXES) setupLegCartesianControl(legId);
}
