/**
 * 参数报警配置模块。
 *
 *  - 「报警配置」页:为电机/IMU 参数配置上下阈值(可留空表示不启用该侧);
 *  - 达到阈值 → ①右侧「当前报警」列表实时列出;②3D 模型对应关节上挂红色 ⚠ 标记
 *    (CSS2D 标签,参考注释标签实现,跟随关节运动);
 *  - 规则与启用状态持久化到 localStorage,刷新后自动恢复;
 *  - 评估频率约 5Hz(由 main.js 动画循环节流调用),数据源与数据看板一致:
 *    电机读 motorTelemetry 原始遥测,IMU 读 shared.imu。
 */
import * as THREE from 'three';
import { CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';
import { bodyObjects, jointStates, visibleControls, shared, currentModelId } from './state.js?v=1';
import {
  motorTelemetry, JOINT_MAPPING, getRosUrl,
  addMessageListener, addImuListener,
  MOTOR_STATE_TOPIC, IMU_STATE_TOPIC,
} from './ros-bridge.js?v=981';
import { labelForJoint } from './joints.js?v=1';
import { isReplaying } from './recorder-replay.js?v=967';
import {
  startRecording, stopRecording, recordFrame,
  resumeRecording, syncRecordingMeta, renameRecording,
} from './recorder-storage.js?v=967';

const LS_KEY = 'web_sim_alarm_rules_v1';
/** 报警记录元数据(同 key 合并一条;帧数据在 recordings store,复用录制复现管道) */
const LS_RECORDS_KEY = 'web_sim_alarm_records_v1';
/** 单条报警记录的最大 motor 帧数(超过则封盘,下次报警另起一条) */
const ALARM_CAPTURE_MAX_FRAMES = 120000;

/** 电机可报警字段(与 /rl_real/motor_state 9/10 字段块对应;temp 仅 10-float 新协议) */
const MOTOR_FIELD_DEFS = [
  { id: 'realPos',   label: '实际位置', unit: 'rad',   get: (t) => t.realPos },
  { id: 'targetPos', label: '目标位置', unit: 'rad',   get: (t) => t.targetPos },
  { id: 'realVel',   label: '实际速度', unit: 'rad/s', get: (t) => t.realVel },
  { id: 'targetVel', label: '目标速度', unit: 'rad/s', get: (t) => t.targetVel },
  { id: 'realTau',   label: '实际力矩', unit: 'N·m',   get: (t) => t.realTau },
  { id: 'targetTau', label: '目标力矩', unit: 'N·m',   get: (t) => t.targetTau },
  { id: 'temp',      label: '温度',     unit: '°C',    get: (t) => t.temp },
  { id: 'dev',       label: '位置偏差|目标-实际|', unit: 'rad', get: (t) => Math.abs(t.targetPos - t.realPos) },
];

/** IMU 可报警字段(shared.imu 结构,角度类阈值以 ° 配置,内部换算) */
const IMU_FIELD_DEFS = [
  { id: 'rpy.roll',  label: '横滚 Roll', unit: '°',   get: (imu) => THREE.MathUtils.radToDeg(imu.rpy.roll) },
  { id: 'rpy.pitch', label: '俯仰 Pitch', unit: '°',  get: (imu) => THREE.MathUtils.radToDeg(imu.rpy.pitch) },
  { id: 'rpy.yaw',   label: '航向 Yaw', unit: '°',    get: (imu) => THREE.MathUtils.radToDeg(imu.rpy.yaw) },
  { id: 'acc.x', label: '加速度 X', unit: 'm/s²', get: (imu) => imu.acc.x },
  { id: 'acc.y', label: '加速度 Y', unit: 'm/s²', get: (imu) => imu.acc.y },
  { id: 'acc.z', label: '加速度 Z', unit: 'm/s²', get: (imu) => imu.acc.z },
  { id: 'gyro.x', label: '角速度 X', unit: 'rad/s', get: (imu) => imu.gyro.x },
  { id: 'gyro.y', label: '角速度 Y', unit: 'rad/s', get: (imu) => imu.gyro.y },
  { id: 'gyro.z', label: '角速度 Z', unit: 'rad/s', get: (imu) => imu.gyro.z },
  { id: 'pos.north', label: '位置北向', unit: 'm', get: (imu) => imu.pos.north },
  { id: 'pos.east',  label: '位置东向', unit: 'm', get: (imu) => imu.pos.east },
  { id: 'pos.down',  label: '位置天向', unit: 'm', get: (imu) => imu.pos.down },
  { id: 'vel.x', label: '速度X(体)', unit: 'm/s', get: (imu) => imu.vel.x },
  { id: 'vel.y', label: '速度Y(体)', unit: 'm/s', get: (imu) => imu.vel.y },
  { id: 'vel.z', label: '速度Z(体)', unit: 'm/s', get: (imu) => imu.vel.z },
];

/** 字段定义查找表(O(1) 替代 find 线性查找) */
const _motorDefMap = new Map(MOTOR_FIELD_DEFS.map((d) => [d.id, d]));
const _imuDefMap = new Map(IMU_FIELD_DEFS.map((d) => [d.id, d]));

/** 模块状态:配置(持久化) + 活动报警 + 3D 标记池 */
const alarmState = {
  enabled: true,
  rules: [],           // { id, source:'motor'|'imu', joint:'ALL'|name, field, min, max, frames }
  active: new Map(),   // key -> { key, jointName|null, text, at }
  markers: new Map(),  // jointName -> { obj: CSS2DObject, el: HTMLElement }
};

/** evaluateAlarms 复用 Map(~5Hz 频率避免反复分配) */
const _evalNextMap = new Map();
const _evalBreachingMap = new Map();

// ── 报警周期(帧)去抖 + 报警记录捕获 ──────────────────────
/** motor_state 实收帧计数(复现注入的帧不计入) */
let motorFrameCounter = 0;
/** key -> { startFrame } 越限开始时的帧计数(未达周期阈值前的去抖状态) */
const pendingBreaches = new Map();
/** key -> { recordingId, label, frames, occurrences, lastEndedAt, closing } 已合并的报警记录捕获会话 */
const captures = new Map();

/** 读取报警记录元数据 localStorage */
function loadRecordMeta() {
  try { return JSON.parse(localStorage.getItem(LS_RECORDS_KEY) || '{}'); }
  catch (_) { return {}; }
}
function saveRecordMeta(meta) {
  try { localStorage.setItem(LS_RECORDS_KEY, JSON.stringify(meta)); } catch (_) { /* 忽略 */ }
}

/** 报警激活:开启(或复用)该 key 的合并记录会话(跨页面刷新也合并到同一条) */
async function ensureCapture(key, label) {
  if (captures.has(key)) return captures.get(key);
  const meta = loadRecordMeta();
  const prev = meta[key];
  let recordingId = null;
  // 已有合并记录且未封盘:续录到同一条(相同报警合并一条)
  if (prev?.recordingId && !prev.closed) {
    try { recordingId = await resumeRecording(prev.recordingId); } catch (_) { recordingId = null; }
  }
  if (recordingId == null) {
    const { recordingId: newId } = await startRecording({
      name: `⚠ ${label}`,
      modelId: currentModelId,
      rosUrl: getRosUrl(),
    });
    recordingId = newId;
  }
  const cap = {
    recordingId,
    label,
    frames: prev?.frames || 0,
    occurrences: (prev?.occurrences || 0) + 1,
    lastEndedAt: null,
    closing: false,
  };
  captures.set(key, cap);
  meta[key] = { recordingId, label, occurrences: cap.occurrences, frames: cap.frames, closed: false, modelId: String(currentModelId) };
  saveRecordMeta(meta);
  // 触发瞬间即通知「录制列表」刷新,让新记录立即出现(不等报警结束)
  try { window.dispatchEvent(new CustomEvent('web-sim:alarm-recordings-changed')); } catch (_) { /* 忽略 */ }
  return cap;
}

/** 报警结束:更新合并元数据 + 刷新记录帧数(会话保持打开,下次报警继续合并);帧数超限则封盘。
 *  记录条目统一显示在「录制列表」页(名称含 ⚠ 与合并次数),报警页不再单列。 */
async function finalizeCapture(key) {
  const cap = captures.get(key);
  if (!cap) return;
  cap.lastEndedAt = Date.now();
  const closed = cap.frames >= ALARM_CAPTURE_MAX_FRAMES;
  const meta = loadRecordMeta();
  meta[key] = { recordingId: cap.recordingId, label: cap.label, occurrences: cap.occurrences, frames: cap.frames, closed, modelId: String(currentModelId) };
  saveRecordMeta(meta);
  try { await syncRecordingMeta(cap.recordingId); } catch (_) { /* 忽略 */ }
  // 名称带上合并次数,如「⚠ xxx ×3」
  const name = `⚠ ${cap.label}${cap.occurrences > 1 ? ` ×${cap.occurrences}` : ''}`;
  try { await renameRecording(cap.recordingId, name); } catch (_) { /* 忽略 */ }
  if (closed && !cap.closing) {
    cap.closing = true;
    try { await stopRecording(cap.recordingId); } catch (_) { /* 忽略 */ }
    captures.delete(key);
  }
  // 报警结束(合并次数/帧数更新、封盘)后同样刷新列表
  try { window.dispatchEvent(new CustomEvent('web-sim:alarm-recordings-changed')); } catch (_) { /* 忽略 */ }
}

// ── 持久化 ────────────────────────────────────────────────
function loadConfig() {
  try {
    const raw = localStorage.getItem(LS_KEY);
    if (!raw) return;
    const cfg = JSON.parse(raw);
    alarmState.enabled = cfg?.enabled !== false;
    alarmState.rules = Array.isArray(cfg?.rules) ? cfg.rules.filter((r) => r && r.id) : [];
  } catch (err) {
    console.warn('[alarm] 读取本地配置失败:', err);
  }
}

function saveConfig() {
  try {
    localStorage.setItem(LS_KEY, JSON.stringify({ enabled: alarmState.enabled, rules: alarmState.rules }));
  } catch (err) {
    console.warn('[alarm] 保存本地配置失败:', err);
  }
}

// ── 规则编辑 UI ───────────────────────────────────────────
let rulesListEl = null;

/** 当前模型的电机名列表(优先 JOINT_MAPPING) */
/** 电机名列表缓存(模型切换时由 rebuildAlarmEditor 清除) */
let _motorNamesCache = null;
let _motorNamesModelId = null;
function getMotorNames() {
  if (_motorNamesCache && _motorNamesModelId === currentModelId) return _motorNamesCache;
  const mapped = JOINT_MAPPING[String(currentModelId)];
  _motorNamesCache = (Array.isArray(mapped) && mapped.length > 0) ? mapped : visibleControls.map((s) => s.name);
  _motorNamesModelId = currentModelId;
  return _motorNamesCache;
}

function el(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text != null) node.textContent = text;
  return node;
}

function fieldDefsFor(source) {
  return source === 'imu' ? IMU_FIELD_DEFS : MOTOR_FIELD_DEFS;
}

/** 构建单条规则行 */
function buildRuleRow(rule) {
  const row = el('div', 'alarm-rule');
  row.dataset.ruleId = rule.id;

  // 来源
  const sourceSel = el('select', 'alarm-rule-source');
  sourceSel.title = '参数来源';
  [['motor', '电机'], ['imu', 'IMU']].forEach(([v, t]) => {
    const opt = el('option', null, t);
    opt.value = v;
    sourceSel.appendChild(opt);
  });
  sourceSel.value = rule.source === 'imu' ? 'imu' : 'motor';

  // 参数字段
  const fieldSel = el('select', 'alarm-rule-field');
  fieldSel.title = '参数';
  const fillFields = () => {
    fieldSel.textContent = '';
    fieldDefsFor(sourceSel.value).forEach((def) => {
      const opt = el('option', null, `${def.label} (${def.unit})`);
      opt.value = def.id;
      fieldSel.appendChild(opt);
    });
    fieldSel.value = rule.field;
    if (fieldSel.value !== rule.field) fieldSel.selectedIndex = 0;
  };
  fillFields();

  // 关节(motor 来源时显示)
  const jointSel = el('select', 'alarm-rule-joint');
  jointSel.title = '作用关节';
  const fillJoints = () => {
    jointSel.textContent = '';
    const any = el('option', null, '任意关节');
    any.value = 'ALL';
    jointSel.appendChild(any);
    getMotorNames().forEach((n) => {
      const opt = el('option', null, labelForJoint(n));
      opt.value = n;
      jointSel.appendChild(opt);
    });
    jointSel.value = rule.joint || 'ALL';
    if (jointSel.value !== (rule.joint || 'ALL')) jointSel.value = 'ALL';
  };
  fillJoints();
  const syncJointVisibility = () => { jointSel.style.display = sourceSel.value === 'motor' ? '' : 'none'; };
  syncJointVisibility();

  // 启用开关(停用后该规则不参与评估,已有报警在下一评估轮自动消除)
  const enabledCb = el('input', 'alarm-rule-enabled');
  enabledCb.type = 'checkbox';
  enabledCb.checked = rule.enabled !== false;
  enabledCb.title = '启用/停用该规则';
  // 上下限
  const minInput = el('input', 'alarm-rule-min');
  minInput.type = 'number';
  minInput.step = 'any';
  minInput.placeholder = '下限';
  minInput.title = '下限(留空不启用)';
  minInput.value = Number.isFinite(rule.min) ? rule.min : '';
  const maxInput = el('input', 'alarm-rule-max');
  maxInput.type = 'number';
  maxInput.step = 'any';
  maxInput.placeholder = '上限';
  maxInput.title = '上限(留空不启用)';
  maxInput.value = Number.isFinite(rule.max) ? rule.max : '';
  // 报警周期(帧):越限需持续超过该帧数才判定报警(默认 1 = 立即)
  const framesInput = el('input', 'alarm-rule-frames');
  framesInput.type = 'number';
  framesInput.min = '1';
  framesInput.step = '1';
  framesInput.placeholder = '周期';
  framesInput.title = '报警周期(帧):越限持续超过该帧数才显示/记录,默认 1 帧';
  framesInput.value = Math.max(1, Number(rule.frames) || 1);

  // 删除
  const delBtn = el('button', 'alarm-rule-del', '✕');
  delBtn.type = 'button';
  delBtn.title = '删除该规则';
  delBtn.addEventListener('click', () => {
    alarmState.rules = alarmState.rules.filter((r) => r.id !== rule.id);
    saveConfig();
    row.remove();
    if (!rulesListEl?.children.length) renderEmptyHint();
  });

  const commit = () => {
    rule.source = sourceSel.value;
    rule.field = fieldSel.value;
    rule.joint = sourceSel.value === 'motor' ? jointSel.value : 'ALL';
    rule.min = minInput.value === '' ? null : Number(minInput.value);
    rule.max = maxInput.value === '' ? null : Number(maxInput.value);
    rule.frames = Math.max(1, Math.floor(Number(framesInput.value) || 1));
    rule.enabled = enabledCb.checked;
    row.classList.toggle('is-disabled', !enabledCb.checked);
    saveConfig();
  };
  enabledCb.addEventListener('change', commit);
  sourceSel.addEventListener('change', () => { fillFields(); fillJoints(); syncJointVisibility(); commit(); });
  fieldSel.addEventListener('change', commit);
  jointSel.addEventListener('change', commit);
  minInput.addEventListener('input', commit);
  maxInput.addEventListener('input', commit);
  framesInput.addEventListener('input', commit);

  row.append(enabledCb, sourceSel, fieldSel, jointSel, minInput, maxInput, framesInput, delBtn);
  row.classList.toggle('is-disabled', enabledCb.checked === false);
  return row;
}

function renderEmptyHint() {
  if (!rulesListEl) return;
  rulesListEl.appendChild(el('div', 'alarm-empty-hint', '暂无规则 — 点击「+ 添加规则」新增阈值报警'));
}

/** 重建规则编辑器(模型切换后关节列表变化时也调用) */
export function rebuildAlarmEditor() {
  if (!rulesListEl) return;
  rulesListEl.textContent = '';
  if (!alarmState.rules.length) { renderEmptyHint(); return; }
  alarmState.rules.forEach((rule) => rulesListEl.appendChild(buildRuleRow(rule)));
}

// ── 报警评估与 3D 标记 ────────────────────────────────────
function isBreaching(rule, v) {
  if (!Number.isFinite(v)) return false;
  const below = Number.isFinite(rule.min) && v < rule.min;
  const above = Number.isFinite(rule.max) && v > rule.max;
  return below || above;
}

/** 设置(或刷新)某关节上的报警标记 */
function setJointMarker(jointName, text) {
  // 优先按刚体名(bodyObjects = MJCF body),找不到再退回关节 pivot(jointStates)
  const target = bodyObjects.get(jointName) || jointStates.get(jointName)?.pivot;
  if (!target) return;
  let m = alarmState.markers.get(jointName);
  if (!m) {
    const node = el('div', 'alarm-marker');
    const obj = new CSS2DObject(node);
    obj.center.set(0.5, 0);
    target.add(obj);
    m = { obj, el: node };
    alarmState.markers.set(jointName, m);
  }
  if (m.el.textContent !== text) m.el.textContent = text;
}

function clearJointMarker(jointName) {
  const m = alarmState.markers.get(jointName);
  if (!m) return;
  m.obj.removeFromParent();
  m.el.remove();
  alarmState.markers.delete(jointName);
}

/** 模型切换/清场时调用:清空全部标记与活动报警 */
export function clearAlarmMarkers() {
  for (const name of [...alarmState.markers.keys()]) clearJointMarker(name);
  alarmState.active.clear();
  renderActiveList();
}

let activeListEl = null;
let summaryEl = null;

/** 渲染「当前报警」列表 */
function renderActiveList() {
  if (!activeListEl) return;
  activeListEl.textContent = '';
  const entries = [...alarmState.active.values()];
  if (!entries.length) {
    activeListEl.appendChild(el('li', 'alarm-empty', '暂无报警'));
  } else {
    entries.forEach((a) => {
      const t = new Date(a.at).toLocaleTimeString('zh-CN', { hour12: false });
      activeListEl.appendChild(el('li', 'alarm-item', `⚠ ${a.text} · ${t}`));
    });
  }
  if (summaryEl) summaryEl.textContent = `${entries.length} 条报警`;
}

/**
 * 评估所有规则(约 5Hz,由 main.js 动画循环节流调用)。
 *  - 电机:读 motorTelemetry 原始遥测(ALL = 当前模型全部电机,任一越限即报警);
 *  - IMU:读 shared.imu(要求 1s 内有数据);
 *  - 周期去抖:越限需持续超过规则配置的「周期(帧)」(motor_state 实收帧数)
 *    才判定为报警;未达周期即恢复的不显示、不记录;
 *  - 报警激活 → 记入活动报警 + 关节 3D ⚠ 标记(最多 12 个)+ 开始/继续帧捕获
 *    (同一 key 合并一条记录,记录整个报警周期数据);恢复 → 移除标记,封存该周期。
 */
export function evaluateAlarms() {
  // 复现(回放)期间:注入的是历史合成帧,不参与报警评估;
  // 并清掉当前报警与标记,避免回放数据冻结遥测导致报警"卡住"不消失。
  if (isReplaying()) {
    if (alarmState.active.size || alarmState.markers.size) clearAlarmMarkers();
    return;
  }
  evalTick += 1;
  const now = Date.now();
  // 复用 Map 避免 ~5Hz 频率分配 GC 压力
  const next = _evalNextMap;       // 通过去抖的活动报警
  const breaching = _evalBreachingMap;  // 本轮原始越限(含未达周期的),key -> {jointName, label}
  next.clear();
  breaching.clear();
  if (alarmState.enabled) {
    for (const rule of alarmState.rules) {
      if (rule.enabled === false) continue; // 单条停用
      const needFrames = Math.max(1, Number(rule.frames) || 1);
      if (rule.source === 'imu') {
        const imu = shared.imu;
        const fresh = imu?.msgCount && (performance.now() - (imu.lastMsgAt || 0) < 1000);
        if (!fresh) continue;
        const def = _imuDefMap.get(rule.field);
        if (!def) continue;
        const v = def.get(imu);
        if (isBreaching(rule, v)) {
          const dir = Number.isFinite(rule.min) && v < rule.min ? '低于下限' : '超出上限';
          const lim = dir === '低于下限' ? rule.min : rule.max;
          const key = `imu:${rule.id}:${def.id}`;
          breaching.set(key, { jointName: null, label: `IMU ${def.label} ${v.toFixed(1)}${def.unit} ${dir} ${lim}${def.unit}`, at: now });
          // IMU 无帧计数概念,用评估次数近似(5Hz):需持续 rule.frames 次
          const p = pendingBreaches.get(key) || { startTick: evalTick, count: 0 };
          p.count += 1;
          pendingBreaches.set(key, p);
          if (p.count >= needFrames) {
            next.set(key, { jointName: null, text: breaching.get(key).label, at: now });
          }
        } else {
          pendingBreaches.delete(`imu:${rule.id}:${def.id}`);
        }
      } else {
        const def = _motorDefMap.get(rule.field);
        if (!def) continue;
        const names = rule.joint && rule.joint !== 'ALL' ? [rule.joint] : getMotorNames();
        for (const n of names) {
          const tel = motorTelemetry.get(n);
          if (!tel) continue;
          const v = def.get(tel);
          const key = `motor:${rule.id}:${def.id}:${n}`;
          if (!isBreaching(rule, v)) { pendingBreaches.delete(key); continue; }
          const dir = Number.isFinite(rule.min) && v < rule.min ? '低于下限' : '超出上限';
          const lim = dir === '低于下限' ? rule.min : rule.max;
          const text = `${labelForJoint(n)} ${def.label} ${v.toFixed(2)}${def.unit} ${dir} ${lim}${def.unit}`;
          breaching.set(key, { jointName: n, label: text, at: now });
          // 周期去抖:以实收 motor 帧计数
          const p = pendingBreaches.get(key) || { startFrame: motorFrameCounter };
          pendingBreaches.set(key, p);
          if (motorFrameCounter - p.startFrame >= needFrames - 1) {
            next.set(key, { jointName: n, text, at: now, source: 'motor', field: def.id });
          }
        }
      }
    }
  }
  // 清掉已恢复越限的 pending
  for (const key of [...pendingBreaches.keys()]) {
    if (!breaching.has(key)) pendingBreaches.delete(key);
  }
  alarmState.active = next;

  // 报警记录捕获:激活的开始(续)捕获;消失的结束本周期
  for (const [key, a] of next) {
    if (!captures.has(key)) {
      ensureCapture(key, a.text).catch((err) => console.warn('[alarm] 开启报警记录失败:', err));
    }
  }
  for (const key of [...captures.keys()]) {
    if (!next.has(key)) finalizeCapture(key);
  }

  // 3D 标记同步(仅电机类,最多 12 个)
  const marked = new Set();
  let markerCount = 0;
  for (const a of next.values()) {
    if (!a.jointName || markerCount >= 12) continue;
    setJointMarker(a.jointName, `⚠ ${a.text}`);
    marked.add(a.jointName);
    markerCount += 1;
  }
  for (const name of [...alarmState.markers.keys()]) {
    if (!marked.has(name)) clearJointMarker(name);
  }

  renderActiveList();
}

/** 帧捕获监听器是否已注册 */
let captureListenersBound = false;
/** IMU 周期去抖用的评估轮次计数 */
let evalTick = 0;

// ── 报警记录(帧数据存于 IndexedDB recordings store,条目统一显示在「录制列表」页) ──
/** 删除报警记录时同步清理合并元数据(main.js 删除录制条目时调用) */
export function purgeAlarmRecordMeta(recordingId) {
  const rid = Number(recordingId);
  const meta = loadRecordMeta();
  let changed = false;
  for (const key of Object.keys(meta)) {
    if (Number(meta[key]?.recordingId) === rid) { delete meta[key]; changed = true; }
  }
  if (changed) saveRecordMeta(meta);
}

/** 查询某参数当前是否处于报警状态(数据看板标红用) */
export function isParamAlarmed(source, field, jointName = null) {
  for (const a of alarmState.active.values()) {
    if (a.source !== source || a.field !== field) continue;
    if (source === 'motor' && jointName != null && a.jointName !== jointName) continue;
    return true;
  }
  return false;
}

/** 注册帧捕获监听器:仅在对应报警处于活动状态期间写入帧(报警结束即停,会话保留以便下次合并) */
function bindCaptureListeners() {
  if (captureListenersBound) return;
  captureListenersBound = true;
  addMessageListener((payload) => {
    if (isReplaying()) return; // 复现注入的帧不参与报警计数/记录
    motorFrameCounter += 1;
    if (!captures.size) return;
    const tsMs = payload?.at || performance.now();
    const data = Array.isArray(payload?.data) ? payload.data : [];
    for (const [key, cap] of captures) {
      if (!alarmState.active.has(key)) continue; // 报警已结束:停止写帧
      recordFrame(cap.recordingId, { topic: MOTOR_STATE_TOPIC, timestampMs: tsMs, data });
      cap.frames += 1;
    }
  });
  addImuListener((payload) => {
    if (isReplaying()) return;
    if (!captures.size) return;
    const tsMs = payload?.at || performance.now();
    const data = Array.isArray(payload?.data) ? payload.data : [];
    for (const [key, cap] of captures) {
      if (!alarmState.active.has(key)) continue;
      recordFrame(cap.recordingId, { topic: IMU_STATE_TOPIC, timestampMs: tsMs, data });
    }
  });
}

// ── 初始化 ────────────────────────────────────────────────
/** 初始化报警配置页(绑定控件 + 恢复持久化配置) */
export function initAlarm() {
  loadConfig();
  rulesListEl = document.querySelector('#alarm-rules-list');
  activeListEl = document.querySelector('#alarm-active-list');
  summaryEl = document.querySelector('#alarm-summary');
  const enabledCb = document.querySelector('#alarm-enabled');
  const addBtn = document.querySelector('#alarm-add-rule');
  if (enabledCb) {
    enabledCb.checked = alarmState.enabled;
    enabledCb.addEventListener('change', () => {
      alarmState.enabled = enabledCb.checked;
      saveConfig();
      if (!alarmState.enabled) clearAlarmMarkers();
    });
  }
  if (addBtn) {
    addBtn.addEventListener('click', () => {
      alarmState.rules.push({
        id: `r${Date.now().toString(36)}${Math.floor(Math.random() * 1e4).toString(36)}`,
        source: 'motor', joint: 'ALL', field: 'realPos', min: null, max: null, frames: 1, enabled: true,
      });
      saveConfig();
      rebuildAlarmEditor();
    });
  }
  bindCaptureListeners();
  rebuildAlarmEditor();
  renderActiveList();
}
