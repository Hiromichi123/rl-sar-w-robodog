/**
 * 关节控制模块。
 *
 * 职责:
 *  - setJointPose:将关节值(角度/位移)应用到 3D 模型的 pivot/content
 *  - 关节中文名称映射(labelForJoint)与分组(groupKeyForJoint)
 *  - 控制面板渲染:按分组生成滑块(createControl / renderControlPanel)
 *  - 步进器长按中断(stopActiveStepperMotion)
 *
 * 关节状态对象结构:
 *  { name, pivot, content, axis, jointPosition, type('slide'|'hinge'),
 *    min, max, value, target, velocity, controlMode('position'|'velocity') }
 */
import * as THREE from 'three';
import { jointStates, visibleControls, activeStepperStops } from './state.js?v=1';
import { controlsElement, jointCountElement } from './scene.js?v=2';

// ── 关节姿态应用 ─────────────────────────────────────────

/** 将关节值应用到 3D 模型(slide 平移 / hinge 旋转) */
export function setJointPose(state, value) {
  const { pivot, content, axis, type, jointPosition } = state;
  if (type === 'slide') {
    pivot.position.copy(jointPosition).addScaledVector(axis, value);
    pivot.quaternion.identity();
  } else {
    pivot.position.copy(jointPosition);
    pivot.quaternion.setFromAxisAngle(axis, value);
  }
  content.position.copy(jointPosition).multiplyScalar(-1);
}

/** 中断所有正在进行的步进器长按动作 */
export function stopActiveStepperMotion() {
  [...activeStepperStops].forEach((stop) => stop());
}

// ── 关节标签与分组 ───────────────────────────────────────

/** 返回关节的中文名称 */
export function labelForJoint(name) {
  const explicit = {
    lift_joint: '升降滑台',
    robot_head_pitch_joint: '头部俯仰',
    robot_head_yaw_joint: '头部偏航',
    fl_steering_joint: '左前转向', fr_steering_joint: '右前转向',
    bl_steering_joint: '左后转向', br_steering_joint: '右后转向',
    fl_wheel_joint: '左前驱动轮速度', fr_wheel_joint: '右前驱动轮速度',
    bl_wheel_joint: '左后驱动轮速度', br_wheel_joint: '右后驱动轮速度',
  };
  if (explicit[name]) return explicit[name];
  const match = name.match(/^robot_(left|right)_joint(\d)$/);
  if (match) return `${match[1] === 'left' ? '左臂' : '右臂'} J${match[2]}`;
  const finger = name.match(/^robot_(left|right)_finger_joint1$/);
  if (finger) return finger[1] === 'left' ? '左手夹爪' : '右手夹爪';
  // 四足机器人腿关节: FL/FR/RL/RR + hip/thigh/calf/foot
  const leg = name.match(/^(FL|FR|RL|RR)_(hip|thigh|calf|foot)_joint$/);
  if (leg) {
    const side = { FL: '左前', FR: '右前', RL: '左后', RR: '右后' }[leg[1]];
    const part = { hip: '髋关节', thigh: '大腿', calf: '小腿', foot: '足端轮' }[leg[2]];
    return `${side}${part}`;
  }
  return name;
}

/** 返回关节所属的分组名称(用于控制面板分组显示) */
export function groupKeyForJoint(name) {
  if (/^(FL|FR)_/.test(name)) return '前腿';
  if (/^(RL|RR)_/.test(name)) return '后腿';
  if (name.includes('_left_')) return '左臂';
  if (name.includes('_right_')) return '右臂';
  if (name.includes('head')) return '头部';
  if (name.includes('steering') || name.includes('_wheel_')) return '底盘';
  return '躯干';
}

// ── 关节控制面板 ─────────────────────────────────────────

/** 联动模式开关(四足机器人专用) */
export let gangModeEnabled = false;
/** 联动时选中的腿集合(默认全部选中) */
export const gangSelectedLegs = new Set(['FL', 'FR', 'RL', 'RR']);
/** 联动时的符号映射:前后腿关节方向相反,需要取反 */
const GANG_SIGN_MAP = {
  FL: { hip: 1, thigh: 1, calf: 1 },
  FR: { hip: 1, thigh: 1, calf: 1 },
  RL: { hip: 1, thigh: -1, calf: -1 },
  RR: { hip: 1, thigh: -1, calf: -1 },
};

/**
 * 对四足机器人的同类关节应用联动同步。
 * 当前腿的某个关节被调整时,自动更新选中腿的同类关节。
 * 前后腿关节安装方向相反,thigh/calf 需取反。
 * @param {Object} state - 被修改的关节状态
 * @param {number} newValue - 新值
 */
function applyGangSync(state, newValue) {
  const match = state.name.match(/^(FL|FR|RL|RR)_(hip|thigh|calf)_joint$/);
  if (!match) return;
  const [, sourceLeg, jointType] = match;
  const sourceSign = GANG_SIGN_MAP[sourceLeg][jointType];
  for (const leg of ['FL', 'FR', 'RL', 'RR']) {
    if (!gangSelectedLegs.has(leg) || leg === sourceLeg) continue;
    const targetState = jointStates.get(`${leg}_${jointType}_joint`);
    if (!targetState) continue;
    const targetSign = GANG_SIGN_MAP[leg][jointType];
    const mappedValue = newValue * (targetSign / sourceSign);
    targetState.target = THREE.MathUtils.clamp(mappedValue, targetState.min, targetState.max);
    syncStateInputs(targetState);
  }
}

/** 根据关节类型返回显示精度 */
export function precisionForState(state) {
  return state.type === 'slide' ? 3 : state.controlMode === 'velocity' ? 1 : 2;
}

/** 同步同一关节的所有输入控件的值 */
export function syncStateInputs(state, source = null) {
  const precision = precisionForState(state);
  document.querySelectorAll(`input[data-joint="${state.name}"]`).forEach((input) => {
    if (input !== source) input.value = state.target;
    input.parentElement.querySelector('output').value = state.target.toFixed(precision);
  });
}

/** 为单个关节创建滑块控件行 */
export function createControl(state) {
  const row = document.createElement('label');
  row.className = 'control-row';
  const precision = precisionForState(state);
  row.innerHTML = `<span>${labelForJoint(state.name)}</span><output>${state.target.toFixed(precision)}</output>`;
  const input = document.createElement('input');
  input.type = 'range';
  input.dataset.joint = state.name;
  input.min = state.min;
  input.max = state.max;
  input.step = state.type === 'slide' ? '0.001' : state.controlMode === 'velocity' ? '0.1' : '0.01';
  input.value = state.target;
  input.addEventListener('input', () => {
    const newValue = Number(input.value);
    state.target = newValue;
    if (gangModeEnabled) applyGangSync(state, newValue);
    syncStateInputs(state, input);
  });
  row.append(input);
  return row;
}

/** 按分组渲染所有受控关节的滑块面板 */
export function renderControlPanel() {
  controlsElement.replaceChildren();
  // 四足机器人:显示站立按钮,隐藏招手/检查按钮
  // 人形机器人:显示招手/检查按钮,隐藏站立按钮
  const isQuadruped = jointStates.has('FL_hip_joint');
  document.querySelectorAll('.quadruped-only').forEach((el) => {
    el.classList.toggle('visible', isQuadruped);
  });
  document.querySelectorAll('.humanoid-only').forEach((el) => {
    el.classList.toggle('visible', !isQuadruped);
  });
  if (isQuadruped) {
    const gangSection = document.createElement('section');
    gangSection.className = 'control-group';
    gangSection.innerHTML = `<h2>联动控制</h2>`;

    // 主开关
    const toggleRow = document.createElement('label');
    toggleRow.className = 'control-row gang-toggle-row';
    toggleRow.innerHTML = `<span>启用同类关节联动</span>`;
    const toggle = document.createElement('input');
    toggle.type = 'checkbox';
    toggle.id = 'gang-mode-toggle';
    toggle.checked = gangModeEnabled;
    toggle.addEventListener('change', () => {
      gangModeEnabled = toggle.checked;
      updateLegCheckboxStates();
    });
    toggleRow.append(toggle);
    gangSection.append(toggleRow);

    // 腿选择
    const legSelectRow = document.createElement('div');
    legSelectRow.className = 'gang-leg-select';
    legSelectRow.innerHTML = `<span class="gang-leg-label">选择联动腿:</span>`;
    const legs = [
      { id: 'FL', label: '左前' },
      { id: 'FR', label: '右前' },
      { id: 'RL', label: '左后' },
      { id: 'RR', label: '右后' },
    ];
    for (const leg of legs) {
      const label = document.createElement('label');
      label.className = 'gang-leg-item';
      const cb = document.createElement('input');
      cb.type = 'checkbox';
      cb.className = 'gang-leg-cb';
      cb.dataset.leg = leg.id;
      cb.checked = gangSelectedLegs.has(leg.id);
      cb.addEventListener('change', () => {
        if (cb.checked) gangSelectedLegs.add(leg.id);
        else gangSelectedLegs.delete(leg.id);
      });
      label.append(cb, document.createTextNode(` ${leg.label}`));
      legSelectRow.append(label);
    }
    gangSection.append(legSelectRow);
    controlsElement.append(gangSection);
    updateLegCheckboxStates();
  }
  const groups = new Map();
  for (const state of visibleControls) {
    const group = groupKeyForJoint(state.name);
    if (!groups.has(group)) groups.set(group, []);
    groups.get(group).push(state);
  }
  for (const [groupKey, states] of groups) {
    const section = document.createElement('section');
    section.className = 'control-group';
    section.innerHTML = `<h2>${groupKey}</h2>`;
    states.forEach((state) => section.append(createControl(state)));
    controlsElement.append(section);
  }
  if (jointCountElement) jointCountElement.textContent = `${visibleControls.length} 关节`;
}

/** 更新腿选择复选框的禁用状态 */
function updateLegCheckboxStates() {
  document.querySelectorAll('.gang-leg-cb').forEach((cb) => {
    cb.disabled = !gangModeEnabled;
  });
}
