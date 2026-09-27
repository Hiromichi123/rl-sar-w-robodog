/**
 * 电机数据采样与渲染模块。
 *
 * - 每帧从 jointStates 中为每个受控关节记录一个 (t, value, target) 样本。
 * - 存储在按关节分块的环形缓冲中(最长 MAX_SECONDS 秒 @60fps = 3600 样本),
 *   旧样本自动被覆盖,内存占用恒定。
 * - 渲染时按用户选择的时间窗口截取最近 N 秒,绘制双折线(青色=实际值、橙色=指令值)。
 */
import { jointStates, visibleControls, shared } from './state.js?v=1';
import { labelForJoint } from './joints.js?v=1';
import { motorTelemetry } from './ros-bridge.js?v=981';
import { lastMessageAt, messageCount } from './ros-bridge.js?v=981';
import * as THREE from 'three';

/** 最多记录的秒数(环形缓冲容量),比最大 UI 窗口略大 */
const MAX_SECONDS = 75;
/** 期望帧率上界,用于估算缓冲长度(实际每帧追加,不按固定步长) */
const EST_FPS = 60;
const BUF_LEN = MAX_SECONDS * EST_FPS;

/** 按关节名索引的环形缓冲 */
const buffers = new Map();

function ensureBufferFor(name) {
  let b = buffers.get(name);
  if (b) return b;
  b = {
    t: new Float32Array(BUF_LEN),          // 时间戳(s,相对首次记录)
    v: new Float32Array(BUF_LEN),          // 实际值 value
    tgt: new Float32Array(BUF_LEN),        // 指令值 target
    count: 0,                              // 累计写入数 (>= BUF_LEN 时代表已跑满)
    head: 0,                               // 下一个写入槽位
    t0: 0,                                 // 首次记录时的 performance.now() (ms)
  };
  buffers.set(name, b);
  return b;
}

/** 清空所有关节缓冲,在切换模型时调用 */
export function clearAllBuffers() {
  buffers.clear();
}

/** 清空单个关节缓冲(未使用,留作接口) */
export function clearBuffer(name) {
  buffers.delete(name);
}

let paused = false;
export function setPaused(v) { paused = !!v; }
export function isPaused() { return paused; }

/** 历史偏移(秒):0=实时(看最新数据),>0=往前看 offset 秒。由 main.js 拖 seek bar 或双击复位设置。 */
let _historyOffsetSec = 0;
export function getCurvesHistoryOffset() { return _historyOffsetSec; }
export function setCurvesHistoryOffset(v) {
  _historyOffsetSec = Math.max(0, Math.min(MAX_SECONDS - 1, Number(v) || 0));
}
export function resetCurvesHistory() { _historyOffsetSec = 0; }
export function getCurvesMaxHistorySec() { return MAX_SECONDS; }

/**
 * 关节显示模式:
 *   'curves' = 曲线模式(每行显示 canvas sparkline + 单独 value 标签)
 *   'values' = 数值模式(每行显示 real/target/delta 三列数值卡片,不画曲线,省电)
 */
let displayMode = 'curves';
export function setDisplayMode(mode) {
  displayMode = (mode === 'values') ? 'values' : 'curves';
}
export function getDisplayMode() { return displayMode; }

/** 向指定缓冲写入一个环形样本(位置/速度/力矩共用) */
function writeSample(key, v, tgt, nowMs) {
  const b = ensureBufferFor(key);
  if (b.count === 0) b.t0 = nowMs;
  const i = b.head;
  b.t[i] = (nowMs - b.t0) * 0.001;
  b.v[i] = v;
  b.tgt[i] = tgt;
  b.head = (i + 1) % BUF_LEN;
  b.count = Math.min(b.count + 1, BUF_LEN);
}

/**
 * 记录一帧样本。应由 requestAnimationFrame 驱动,dt 来自同一次 animate。
 * 与 recordImuSample 一致:无 ROS 电机数据推送(未连接/超时)时不记录,
 * 避免在无数据流入时用静态值画出平直曲线。
 * 数据源优先原始遥测 motorTelemetry(未夹紧的 target/real 原始值,与真机打印一致),
 * 无遥测时回退仿真 jointStates。每个关节写 4 条缓冲:
 *   name       位置(rad|m)   v=real_pos  tgt=target_pos
 *   name#vel   速度(rad/s)    v=real_vel  tgt=target_vel
 *   name#tau   力矩(N·m)      v=real_tau  tgt=target_tau
 *   name#temp  温度(°C)       v=temp      (无目标值,仅实际)
 * @param {DOMHighResTimeStamp} nowMs performance.now()
 */
export function recordSample(nowMs) {
  if (nowMs - (recordSample._dbg || 0) > 2000) {
    recordSample._dbg = nowMs;
    let bufInfo = '';
    if (buffers.size) { const first = buffers.values().next().value; bufInfo = `firstBufCount=${first.count}`; }
    const dbg = { paused, messageCount, ageMs: Math.round(nowMs - (lastMessageAt || 0)), visible: visibleControls.length, buffers: buffers.size, bufInfo };
    document.body.dataset.curvesDebug = JSON.stringify(dbg);
    console.log('[curves-debug]', dbg);
  }
  if (paused) return;
  // 无 ROS 电机数据推送时不记录(检查消息计数 + 新鲜度,阈值与 IMU 对齐 600ms)
  if (!messageCount) return;
  const age = nowMs - (lastMessageAt || 0);
  if (age > 600) return;
  for (const state of visibleControls) {
    const name = state.name;
    if (!buffers.has(name)) ensureBufferFor(name);
    const b = buffers.get(name);
    if (b.count === 0) b.t0 = nowMs;
    const i = b.head;
    b.t[i] = (nowMs - b.t0) * 0.001;
    const tel = motorTelemetry.get(name);
    b.v[i] = tel ? tel.realPos : state.value;
    b.tgt[i] = tel ? tel.targetPos : state.target;
    b.head = (i + 1) % BUF_LEN;
    b.count = Math.min(b.count + 1, BUF_LEN);
    // 速度 / 力矩 / 温度曲线(仅真机遥测存在时;temp 仅 10-float 新协议)
    if (tel) {
      writeSample(`${name}#vel`, tel.realVel, tel.targetVel, nowMs);
      writeSample(`${name}#tau`, tel.realTau, tel.targetTau, nowMs);
      if (Number.isFinite(tel.temp)) writeSample(`${name}#temp`, tel.temp, NaN, nowMs);
    }
  }
}

/**
 * 把环形缓冲的样本按时间升序展平到 [0..count) 的区间里,截取最近 windowSec 秒。
 * 返回 {t, v, tgt, start, n, yMin, yMax}。
 */
function readRecent(b, windowSec) {
  if (!b || !b.count) return null;
  const n = b.count;
  // 找到 b.head 之前的 n 个样本(这些样本在物理存储中的索引)
  const startIdx = (b.head - n + BUF_LEN) % BUF_LEN;
  // 最后一个样本时间 = 最新写入时间
  const latestT = b.t[(b.head - 1 + BUF_LEN) % BUF_LEN];
  // 历史偏移:_historyOffsetSec > 0 时窗口整体往前移
  const tMax = latestT - _historyOffsetSec;
  const tCut = tMax - windowSec;
  // 从后往前线性找第一个 >= tCut 的位置(因 n 不大,最多几千,够用)
  let keepFrom = 0;
  for (let k = n - 1; k >= 0; k -= 1) {
    const phys = (startIdx + k) % BUF_LEN;
    if (b.t[phys] < tCut) { keepFrom = k + 1; break; }
    if (k === 0) keepFrom = 0;
  }
  const keep = n - keepFrom;
  if (keep <= 0) return null;
  let yMin = +Infinity, yMax = -Infinity;
  for (let k = keepFrom; k < n; k += 1) {
    const phys = (startIdx + k) % BUF_LEN;
    const v1 = b.v[phys]; const v2 = b.tgt[phys];
    if (v1 < yMin) yMin = v1; if (v1 > yMax) yMax = v1;
    if (v2 < yMin) yMin = v2; if (v2 > yMax) yMax = v2;
  }
  return {
    b, startIdx, keepFrom, keep,
    tMin: tCut, tMax,
    yMin, yMax,
  };
}

/** 遍历 [keepFrom, n) 的每个样本,调用 fn(physIdx, order) */
function forEachSample(info, fn) {
  const { b, startIdx, keepFrom, keep } = info;
  const n = b.count;
  for (let k = 0; k < keep; k += 1) {
    const phys = (startIdx + keepFrom + k) % info.b.BUF_LEN || (startIdx + keepFrom + k) % BUF_LEN;
    fn(phys, k);
  }
}

/**
 * 绘制一条电机曲线到给定 canvas。
 * @param {HTMLCanvasElement} canvas
 * @param {{name: string, unit: string, precision: number, min: number, max: number}} spec
 *        行描述:name=缓冲键(位置 name / 速度 name#vel / 力矩 name#tau)
 * @param {number} windowSec 时间窗口秒
 */
function drawCurve(canvas, spec, windowSec, dpr) {
  const info = readRecent(buffers.get(spec.name), windowSec);
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  if (w <= 0 || h <= 0) return;
  const pxW = Math.max(1, Math.floor(w * dpr));
  const pxH = Math.max(1, Math.floor(h * dpr));
  if (canvas.width !== pxW || canvas.height !== pxH) {
    canvas.width = pxW; canvas.height = pxH;
  }
  const ctx = canvas.getContext('2d');
  ctx.save();
  ctx.clearRect(0, 0, pxW, pxH);
  ctx.scale(dpr, dpr);

  // ── Y 轴范围:无数据(info 为空)时固定 [-1, +1](与 IMU drawImuCurve L504-L506 完全一致,
  //   不使用 state.min/max,确保空态下所有曲线行网格视觉统一)。有数据时再按"数据动态范围 > 关节 min/max" 优先级。
  let yMin, yMax;
  if (!info) {
    yMin = -1; yMax = 1;
  } else {
    yMin = spec.min; yMax = spec.max;
    if (!Number.isFinite(yMin) || !Number.isFinite(yMax) || yMin === yMax) {
      yMin = info.yMin ?? -1; yMax = info.yMax ?? 1;
    }
  }
  const pad = Math.max(1e-6, (yMax - yMin) * 0.12);
  yMin -= pad; yMax += pad;
  const precision = spec.precision;
  const unit = spec.unit;

  // 网格(3 横线)
  ctx.strokeStyle = 'rgba(170,193,225,0.08)';
  ctx.lineWidth = 1;
  ctx.font = '9px ui-monospace, monospace';
  ctx.fillStyle = 'rgba(170,193,225,0.4)';
  ctx.textBaseline = 'top';
  ctx.fillText(yMax.toFixed(precision), 2, 1);
  ctx.textBaseline = 'bottom';
  ctx.fillText(yMin.toFixed(precision), 2, h - 1);
  for (let i = 0; i <= 3; i++) {
    const y = (h / 3) * i;
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  }
  // 0 值线(如果在范围内)
  if (yMin < 0 && yMax > 0) {
    const y0 = h * (1 - (0 - yMin) / (yMax - yMin));
    ctx.strokeStyle = 'rgba(170,193,225,0.22)';
    ctx.beginPath(); ctx.moveTo(0, y0); ctx.lineTo(w, y0); ctx.stroke();
  }

  const tMin = info ? info.tMin : 0;
  const tMax = info ? info.tMax : windowSec;
  const tSpan = Math.max(1e-3, tMax - tMin);
  const xOf = (t) => ((t - tMin) / tSpan) * w;
  const yOf = (v) => h * (1 - (v - yMin) / (yMax - yMin));

  let lastRealX = 0, lastRealY = 0, lastRealV = 0;
  let lastTgtX = 0, lastTgtY = 0, lastTgtV = 0;
  let hasReal = false, hasTgt = false;

  if (info) {
    // ── 指令值 target(橙色) ──
    ctx.lineJoin = 'round';
    ctx.lineWidth = 1.3;
    ctx.strokeStyle = '#f4a261';
    let first = true;
    forEachSample(info, (phys) => {
      const v = info.b.tgt[phys];
      const t = info.b.t[phys];
      if (!Number.isFinite(v)) return;
      const x = xOf(t), y = yOf(v);
      if (first) { ctx.beginPath(); ctx.moveTo(x, y); first = false; }
      else ctx.lineTo(x, y);
      lastTgtX = x; lastTgtY = y; lastTgtV = v; hasTgt = true;
    });
    if (!first) ctx.stroke();

    // ── 实际值 value(青绿,稍粗) ──
    ctx.lineWidth = 1.6;
    ctx.strokeStyle = '#4ed5c6';
    first = true;
    forEachSample(info, (phys) => {
      const v = info.b.v[phys];
      const t = info.b.t[phys];
      if (!Number.isFinite(v)) return;
      const x = xOf(t), y = yOf(v);
      if (first) { ctx.beginPath(); ctx.moveTo(x, y); first = false; }
      else ctx.lineTo(x, y);
      lastRealX = x; lastRealY = y; lastRealV = v; hasReal = true;
    });
    if (!first) ctx.stroke();
  }

  // 末端小圆点(与 IMU 同尺寸)
  if (hasTgt) {
    ctx.fillStyle = '#f4a261';
    ctx.beginPath(); ctx.arc(lastTgtX, lastTgtY, 2.2, 0, Math.PI * 2); ctx.fill();
  }
  if (hasReal) {
    ctx.fillStyle = '#4ed5c6';
    ctx.beginPath(); ctx.arc(lastRealX, lastRealY, 2.2, 0, Math.PI * 2); ctx.fill();
  }

  // ── 值标签盒跟随各自曲线末端点(与 IMU drawImuCurve L552-L564 算法一致) ──
  //   target 橙盒: 跟随 target 末端点(lastTgtX/lastTgtY)，放在圆点左上侧(-6,-12)，夹紧画布边界
  //   real   青盒: 跟随 real   末端点(lastRealX/lastRealY)，放在圆点右下侧(+6,+1)，避免与橙盒重叠
  if (hasTgt) {
    const label = `${(Number.isFinite(lastTgtV) ? lastTgtV : 0).toFixed(precision)}${unit}`;
    ctx.font = 'bold 10px ui-monospace, monospace';
    const tw = ctx.measureText(label).width;
    const padLR = 4; // 左右内边距各2 → 总4
    const boxW = tw + padLR;
    // 标签盒放在圆点左侧 6px 位置, 盒底与圆点顶端对齐 → y = lastTgtY - 12
    const lx = Math.max(2, Math.min(w - boxW - 2, lastTgtX - boxW - 6));
    const ly = Math.max(2, Math.min(h - 12 - 1, lastTgtY - 12 - 1));
    // 填充 + 橙色描边
    ctx.fillStyle = 'rgba(14, 23, 38, 0.82)';
    ctx.fillRect(lx, ly, boxW, 12);
    ctx.strokeStyle = '#f4a261';
    ctx.lineWidth = 1;
    ctx.strokeRect(lx + 0.5, ly + 0.5, boxW - 1, 11);
    ctx.fillStyle = '#f4a261';
    ctx.textBaseline = 'top';
    ctx.fillText(label, lx + 2, ly + 1);
  }
  if (hasReal) {
    const label = `${(Number.isFinite(lastRealV) ? lastRealV : 0).toFixed(precision)}${unit}`;
    ctx.font = 'bold 10px ui-monospace, monospace';
    const tw = ctx.measureText(label).width;
    const padLR = 4;
    const boxW = tw + padLR;
    // 标签盒放在圆点右侧 6px 位置, 盒顶与圆点下方对齐 → y = lastRealY + 1
    const lx = Math.max(2, Math.min(w - boxW - 2, lastRealX + 6));
    const ly = Math.max(2, Math.min(h - 12 - 1, lastRealY + 1));
    // 填充 + 青色描边
    ctx.fillStyle = 'rgba(14, 23, 38, 0.82)';
    ctx.fillRect(lx, ly, boxW, 12);
    ctx.strokeStyle = '#4ed5c6';
    ctx.lineWidth = 1;
    ctx.strokeRect(lx + 0.5, ly + 0.5, boxW - 1, 11);
    ctx.fillStyle = '#4ed5c6';
    ctx.textBaseline = 'top';
    ctx.fillText(label, lx + 2, ly + 1);
  }
  ctx.restore();
}

/**
 * 行描述表:bufferKey → { unit, precision, min, max }。
 * rebuildCurveList 时填充;renderCurves / refreshJointValues 按 canvas.dataset.joint 查询。
 */
const rowSpecs = new Map();

/**
 * 创建一条电机曲线行(位置/速度/力矩共用)。
 */
function buildMotorRow(frag, key, labelText, unit, precision, min, max, kind) {
  const row = document.createElement('div');
  row.className = 'curves-row motor-curve-row';
  row.dataset.joint = key;
  row.dataset.kind = kind;

  // ── 标签:左名(+单位小字) + 右值(初始 0.00unit,与 IMU 行一致) ──
  const label = document.createElement('div');
  label.className = 'curves-row-label';
  const nameEl = document.createElement('span');
  nameEl.className = 'curves-joint-name motor-curve-name';
  nameEl.innerHTML = `${labelText} <span class="motor-unit">${unit}</span>`;
  const valEl = document.createElement('span');
  valEl.className = 'curves-joint-value motor-curve-value';
  valEl.dataset.joint = key;
  valEl.textContent = `0.${'0'.repeat(precision)}${unit}`;
  label.appendChild(nameEl);
  label.appendChild(valEl);

  // ── Canvas 曲线块 ─────────────────────
  const canvasWrap = document.createElement('div');
  canvasWrap.className = 'curves-canvas-wrap';
  const canvas = document.createElement('canvas');
  canvas.className = 'curves-canvas motor-curve-canvas';
  canvas.dataset.joint = key;
  canvasWrap.appendChild(canvas);

  row.appendChild(label);
  row.appendChild(canvasWrap);
  frag.appendChild(row);
  // name 必须带上:drawCurve 通过 spec.name 查缓冲(buffers.get(spec.name)),
  // 缺失会导致 readRecent 永远拿到 null → 只画空网格、数据线不绘制
  rowSpecs.set(key, { name: key, unit, precision, min, max });
}

/**
 * 根据当前 visibleControls 重建(或复用)曲线 DOM 行。
 * 每个关节生成 4 行:位置(rad|m) / 速度(rad/s|m/s) / 力矩(N·m) / 温度(°C)。
 * 只在模型切换后调用一次。
 */
export function rebuildCurveList(container) {
  container.innerHTML = '';
  rowSpecs.clear();
  const frag = document.createDocumentFragment();
  for (const state of visibleControls) {
    const label = labelForJoint(state.name);
    const precision = state.type === 'slide' ? 3 : 2;
    const unit = state.type === 'slide' ? 'm' : 'rad';
    const velUnit = state.type === 'slide' ? 'm/s' : 'rad/s';
    buildMotorRow(frag, state.name, label, unit, precision, state.min, state.max, 'pos');
    buildMotorRow(frag, `${state.name}#vel`, `${label} 速度`, velUnit, 2, NaN, NaN, 'vel');
    buildMotorRow(frag, `${state.name}#tau`, `${label} 力矩`, 'N·m', 2, NaN, NaN, 'tau');
    buildMotorRow(frag, `${state.name}#temp`, `${label} 温度`, '°C', 1, NaN, NaN, 'temp');
  }
  container.appendChild(frag);
}

/**
 * 刷新每条电机曲线行右侧的当前实时值(位置/速度/力矩行通用)。
 * 取对应缓冲最新一个实际值;无数据时显示 0.00unit,与 IMU 行逻辑一致。
 */
export function refreshJointValues() {
  const container = document.querySelector('#curves-list');
  if (!container) return;
  for (const valEl of container.querySelectorAll('.motor-curve-value[data-joint]')) {
    const key = valEl.dataset.joint;
    const spec = rowSpecs.get(key);
    if (!spec) continue;
    const b = buffers.get(key);
    let v = 0;
    if (b && b.count > 0) {
      v = b.v[(b.head - 1 + BUF_LEN) % BUF_LEN];
    }
    if (!Number.isFinite(v)) {
      valEl.textContent = '—';
    } else {
      valEl.textContent = `${v.toFixed(spec.precision)}${spec.unit}`;
    }
  }
}

/**
 * 刷新当前显示:
 *   - curves 模式:重绘所有 canvas 曲线 + 同步每行实时数值文本。
 *   - values 模式:直接复用 refreshJointValues,不画 canvas。
 * 仅当曲线 tab 可见时调用。
 * @param {number} windowSec 时间窗口秒(只在曲线模式生效)
 */
export function renderCurves(windowSec) {
  if (displayMode === 'values') {
    refreshJointValues();
    return;
  }
  const container = document.querySelector('#curves-list');
  if (!container) return;
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  for (const canvas of container.querySelectorAll('canvas.curves-canvas[data-joint]')) {
    const key = canvas.dataset.joint;
    const spec = rowSpecs.get(key);
    if (spec) drawCurve(canvas, spec, windowSec, dpr);
  }
  refreshJointValues();
}

/* ══════════════════════════════════════════════════════════
   IMU 关键量曲线(与关节曲线模块独立:独立缓冲 / 独立重建 / 独立渲染)
   ══════════════════════════════════════════════════════════ */

/** IMU 关键量定义:15 条,绘制时按 5 组着色 */
export const IMU_CURVE_SPECS = [
  // 姿态 RPY(发送端已是弧度,记录时内部转成度显示更直观)
  { key: 'roll',  label: 'Roll',  group: 'rpy',  unit: '°',  precision: 2, get: (imu)=>THREE.MathUtils.radToDeg(imu.rpy.roll)  },
  { key: 'pitch', label: 'Pitch', group: 'rpy',  unit: '°',  precision: 2, get: (imu)=>THREE.MathUtils.radToDeg(imu.rpy.pitch) },
  { key: 'yaw',   label: 'Yaw',   group: 'rpy',  unit: '°',  precision: 2, get: (imu)=>THREE.MathUtils.radToDeg(imu.rpy.yaw)   },
  // 加速度 m/s²
  { key: 'ax',    label: 'a_x',   group: 'acc',  unit: 'm/s²', precision: 3, get: (imu)=>imu.acc.x },
  { key: 'ay',    label: 'a_y',   group: 'acc',  unit: 'm/s²', precision: 3, get: (imu)=>imu.acc.y },
  { key: 'az',    label: 'a_z',   group: 'acc',  unit: 'm/s²', precision: 3, get: (imu)=>imu.acc.z },
  // 角速度 rad/s → °/s 显示
  { key: 'gx',    label: 'ω_x',   group: 'gyro', unit: '°/s', precision: 2, get: (imu)=>THREE.MathUtils.radToDeg(imu.gyro.x) },
  { key: 'gy',    label: 'ω_y',   group: 'gyro', unit: '°/s', precision: 2, get: (imu)=>THREE.MathUtils.radToDeg(imu.gyro.y) },
  { key: 'gz',    label: 'ω_z',   group: 'gyro', unit: '°/s', precision: 2, get: (imu)=>THREE.MathUtils.radToDeg(imu.gyro.z) },
  // 位置 NED m
  { key: 'pN',    label: 'North', group: 'pos',  unit: 'm',    precision: 3, get: (imu)=>imu.pos.north },
  { key: 'pE',    label: 'East',  group: 'pos',  unit: 'm',    precision: 3, get: (imu)=>imu.pos.east  },
  { key: 'pD',    label: 'Down',  group: 'pos',  unit: 'm',    precision: 3, get: (imu)=>imu.pos.down  },
  // 体速度 m/s
  { key: 'vbx',   label: 'v_x',   group: 'vel',  unit: 'm/s',  precision: 3, get: (imu)=>imu.vel.x },
  { key: 'vby',   label: 'v_y',   group: 'vel',  unit: 'm/s',  precision: 3, get: (imu)=>imu.vel.y },
  { key: 'vbz',   label: 'v_z',   group: 'vel',  unit: 'm/s',  precision: 3, get: (imu)=>imu.vel.z },
];
/** IMU group → 曲线颜色 key(legend / stroke / dot) 对应 */
export const IMU_GROUP_COLOR = {
  rpy:  { stroke: '#4ed5c6', dot: 'legend-imu-rpy'  },
  acc:  { stroke: '#f4a261', dot: 'legend-imu-acc'  },
  gyro: { stroke: '#7cc3ff', dot: 'legend-imu-gyro' },
  pos:  { stroke: '#b794f6', dot: 'legend-imu-pos'  },
  vel:  { stroke: '#ff8fa3', dot: 'legend-imu-vel'  },
};

/** 每条 IMU 曲线单独一个环形缓冲 → 按 key 索引 */
const imuBuffers = new Map();
function ensureImuBuffer(key) {
  let b = imuBuffers.get(key);
  if (b) return b;
  b = {
    t: new Float32Array(BUF_LEN),
    v: new Float32Array(BUF_LEN),
    count: 0, head: 0, t0: 0,
  };
  imuBuffers.set(key, b);
  return b;
}

/** IMU 显示模式:与关节显示模式解耦 */
let imuDisplayMode = 'values';
export function setImuDisplayMode(mode) {
  imuDisplayMode = (mode === 'curves') ? 'curves' : 'values';
}
export function getImuDisplayMode() { return imuDisplayMode; }

export function clearAllImuBuffers() { imuBuffers.clear(); }

/**
 * 每帧记录一次 IMU 15 个关键量样本。
 * 如果 IMU 数据超时(>600ms)或 msgCount=0,代表真机还没发数据,跳过记录。
 */
export function recordImuSample(nowMs) {
  const imu = shared.imu;
  if (!imu || !imu.msgCount) return;
  const age = nowMs - (imu.lastMsgAt || 0);
  if (age > 600) return;
  if (paused) return;
  for (const spec of IMU_CURVE_SPECS) {
    const b = ensureImuBuffer(spec.key);
    const secs = b.t0 === 0 ? 0 : (nowMs - b.t0) / 1000;
    if (b.t0 === 0) b.t0 = nowMs;
    const i = b.head;
    b.t[i] = secs;
    try { b.v[i] = spec.get(imu); }
    catch { b.v[i] = NaN; }
    b.head = (b.head + 1) % BUF_LEN;
    b.count += 1;
  }
}

/** 读 IMU 某条曲线的最近 windowSec */
function readImuRecent(b, windowSec) {
  if (!b || !b.count) return null;
  const { count, head, t } = b;
  const now = t[(head - 1 + BUF_LEN) % BUF_LEN];
  const keepFrom = now - windowSec - _historyOffsetSec;
  const total = Math.min(count, BUF_LEN);
  // 找起点
  let start = 0;
  for (let j = 0; j < total; j++) {
    const idx = (head - 1 - j + BUF_LEN) % BUF_LEN;
    if (t[idx] < keepFrom) { start = j; break; }
    start = j + 1;
  }
  return { b, count: start, head, keepFrom };
}
function forEachImuSample(info, cb) {
  if (!info) return;
  const { b, count: n, head } = info;
  for (let j = n - 1; j >= 0; j--) {
    const idx = (head - 1 - j + BUF_LEN) % BUF_LEN;
    cb(b.t[idx], b.v[idx]);
  }
}

/** 画单条 IMU sparkline:单色折线 + 当前值点 + y 轴 label */
function drawImuCurve(canvas, spec, windowSec, dpr) {
  const info = readImuRecent(imuBuffers.get(spec.key), windowSec);
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  if (w <= 0 || h <= 0) return;
  const pxW = Math.max(1, Math.floor(w * dpr));
  const pxH = Math.max(1, Math.floor(h * dpr));
  if (canvas.width !== pxW || canvas.height !== pxH) {
    canvas.width = pxW; canvas.height = pxH;
  }
  const ctx = canvas.getContext('2d');
  ctx.save();
  ctx.clearRect(0, 0, pxW, pxH);
  ctx.scale(dpr, dpr);

  // Y 轴范围:先取过去窗口内的 min/max,再给点 margin,避免贴边
  let yMin = Infinity, yMax = -Infinity;
  if (info) {
    forEachImuSample(info, (_t, v) => {
      if (!Number.isFinite(v)) return;
      if (v < yMin) yMin = v;
      if (v > yMax) yMax = v;
    });
  }
  if (!Number.isFinite(yMin) || !Number.isFinite(yMax) || yMin === yMax) {
    yMin = -1; yMax = 1;
  } else {
    const pad = Math.max(1e-6, (yMax - yMin) * 0.12);
    yMin -= pad; yMax += pad;
  }
  const color = IMU_GROUP_COLOR[spec.group]?.stroke || '#4ed5c6';

  // 背景网格:1 条中线(0) + 上下各两条辅助线
  ctx.strokeStyle = 'rgba(170,193,225,0.08)';
  ctx.lineWidth = 1;
  ctx.font = '9px ui-monospace, monospace';
  ctx.fillStyle = 'rgba(170,193,225,0.4)';
  ctx.textBaseline = 'top';
  ctx.fillText(yMax.toFixed(spec.precision), 2, 1);
  ctx.textBaseline = 'bottom';
  ctx.fillText(yMin.toFixed(spec.precision), 2, h - 1);
  for (let i = 0; i <= 3; i++) {
    const y = (h / 3) * i;
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  }
  // 0 值线(如果在范围内)
  if (yMin < 0 && yMax > 0) {
    const y0 = h * (1 - (0 - yMin) / (yMax - yMin));
    ctx.strokeStyle = 'rgba(170,193,225,0.22)';
    ctx.beginPath(); ctx.moveTo(0, y0); ctx.lineTo(w, y0); ctx.stroke();
  }

  // 折线
  ctx.lineJoin = 'round';
  ctx.lineWidth = 1.3;
  ctx.strokeStyle = color;
  let started = false;
  let lastX = 0, lastY = 0, lastVal = 0;
  if (info) {
    const nowT = info.b.t[(info.head - 1 + BUF_LEN) % BUF_LEN];
    const tMin = nowT - windowSec - _historyOffsetSec;
    const tSpan = Math.max(1e-3, windowSec);
    const firstCount = info.count;
    let j = 0;
    forEachImuSample(info, (t, v) => {
      if (!Number.isFinite(v)) return;
      const x = ((t - tMin) / tSpan) * w;
      const y = h * (1 - (v - yMin) / (yMax - yMin));
      if (!started) { ctx.beginPath(); ctx.moveTo(x, y); started = true; }
      else ctx.lineTo(x, y);
      lastX = x; lastY = y; lastVal = v;
      j++;
    });
    if (started) ctx.stroke();
  }

  // 当前值圆点 + label
  if (started) {
    ctx.beginPath();
    ctx.arc(lastX, lastY, 2.2, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
    // 数值标签(画布右侧白色小数字)
    const labelTxt = `${lastVal.toFixed(spec.precision)}${spec.unit}`;
    ctx.font = 'bold 10px ui-monospace, monospace';
    const tw = ctx.measureText(labelTxt).width;
    const lx = Math.max(0, Math.min(w - tw - 3, lastX - tw - 6));
    const ly = Math.max(2, Math.min(h - 12, lastY - 12));
    ctx.fillStyle = 'rgba(14, 23, 38, 0.82)';
    ctx.fillRect(lx - 2, ly, tw + 4, 12);
    ctx.strokeStyle = color;
    ctx.lineWidth = 1;
    ctx.strokeRect(lx - 2 + 0.5, ly + 0.5, tw + 3, 11);
    ctx.fillStyle = '#ffffff';
    ctx.textBaseline = 'top';
    ctx.fillText(labelTxt, lx, ly + 1);
  }
  ctx.restore();
}

/** 构建 IMU 曲线行 DOM:按组名打 group 标签,每行 name + value 标签 + canvas */
export function rebuildImuCurveList(container) {
  container.innerHTML = '';
  const frag = document.createDocumentFragment();
  let lastGroup = null;
  for (const spec of IMU_CURVE_SPECS) {
    if (spec.group !== lastGroup) {
      const grpHeader = document.createElement('div');
      grpHeader.className = 'imu-group-header';
      const dot = document.createElement('i');
      dot.className = `legend-dot ${IMU_GROUP_COLOR[spec.group].dot}`;
      const label = document.createElement('span');
      const groupName = { rpy: '姿态 RPY', acc: '加速度计 Accel', gyro: '陀螺仪 Gyro', pos: '位置 NED', vel: '体速度 VelBody' }[spec.group] || spec.group;
      label.textContent = groupName;
      grpHeader.appendChild(dot);
      grpHeader.appendChild(label);
      frag.appendChild(grpHeader);
      lastGroup = spec.group;
    }
    const row = document.createElement('div');
    row.className = 'curves-row imu-curve-row';
    row.dataset.imuKey = spec.key;
    row.dataset.group = spec.group;

    const lab = document.createElement('div');
    lab.className = 'curves-row-label';
    const name = document.createElement('span');
    name.className = 'curves-joint-name imu-curve-name';
    // 允许 <sub> 标签 (ω_x)
    name.innerHTML = `${spec.label} <span class="imu-unit">${spec.unit}</span>`;
    const val = document.createElement('span');
    val.className = 'curves-joint-value imu-curve-value';
    val.dataset.imuKey = spec.key;
    lab.appendChild(name);
    lab.appendChild(val);

    const wrap = document.createElement('div');
    wrap.className = 'curves-canvas-wrap';
    const cv = document.createElement('canvas');
    cv.className = 'curves-canvas imu-curve-canvas';
    cv.dataset.imuKey = spec.key;
    wrap.appendChild(cv);

    row.appendChild(lab);
    row.appendChild(wrap);
    frag.appendChild(row);
  }
  container.appendChild(frag);
}

/**
 * 渲染 IMU 曲线:仅 imuDisplayMode==='curves' 才真正画 canvas。
 * 数值模式下什么都不做(留给 IMU 数值 DOM 自己刷新)。
 */
export function renderImuCurves(windowSec) {
  if (imuDisplayMode !== 'curves') return;
  const container = document.getElementById('imu-curves-list');
  if (!container) return;
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  for (const spec of IMU_CURVE_SPECS) {
    const canvas = container.querySelector(`canvas.imu-curve-canvas[data-imu-key="${spec.key}"]`);
    const valEl  = container.querySelector(`.imu-curve-value[data-imu-key="${spec.key}"]`);
    if (canvas) drawImuCurve(canvas, spec, windowSec, dpr);
    if (valEl) {
      const imu = shared.imu;
      let v = 0;
      try { v = imu && imu.msgCount ? spec.get(imu) : 0; } catch { v = NaN; }
      if (!Number.isFinite(v)) valEl.textContent = `—`;
      else valEl.textContent = `${v.toFixed(spec.precision)}${spec.unit}`;
    }
  }
}

/* ══════════════════════════════════════════════════════════
   虚拟摇杆输出曲线(与 IMU/电机独立:独立缓冲 / 独立重建 / 独立渲染)
   数据源: shared.joystick(事件驱动,onMove 回调写入,每帧采样)
   ══════════════════════════════════════════════════════════ */

/** 摇杆曲线规格:5 条,分 2 组着色 */
const JOY_CURVE_SPECS = [
  // 归一化输入 [-1, +1]
  { key: 'nx',    label: 'nx',    group: 'input', unit: '',       precision: 2, get: (j) => j.nx },
  { key: 'ny',    label: 'ny',    group: 'input', unit: '',       precision: 2, get: (j) => j.ny },
  // 底盘速度指令
  { key: 'vx',    label: 'vx',    group: 'vel',   unit: 'm/s',    precision: 2, get: (j) => j.vx },
  { key: 'vy',    label: 'vy',    group: 'vel',   unit: 'm/s',    precision: 2, get: (j) => j.vy },
  { key: 'omega', label: 'omega', group: 'vel',   unit: 'rad/s',  precision: 2, get: (j) => j.omega },
];
/** 摇杆 group → 曲线颜色 */
const JOY_GROUP_COLOR = {
  input: { stroke: '#e5c07b' },  // 金黄 - 归一化输入
  vel:   { stroke: '#61afef' },  // 蓝色 - 速度/角速度指令
};

/** 每条摇杆曲线单独一个环形缓冲 → 按 key 索引 */
const joyBuffers = new Map();
function ensureJoyBuffer(key) {
  let b = joyBuffers.get(key);
  if (b) return b;
  b = {
    t: new Float32Array(BUF_LEN),
    v: new Float32Array(BUF_LEN),
    count: 0, head: 0, t0: 0,
  };
  joyBuffers.set(key, b);
  return b;
}

export function clearAllJoyBuffers() { joyBuffers.clear(); }

/**
 * 每帧记录一次摇杆 5 个量样本。
 * 事件驱动的数据源:shared.joystick 始终有最新值(操作时写真实值,松开后写 0)。
 *
 * 松开后持续记录 0 值:曲线画出回落到 0 的竖直线后保持 0 平线,
 * 直至再次操作摇杆。
 */
/** 空闲超时兜底:摇杆不在拖拽中且 lastUpdateAt 停更超过该时长则强制归零,
 *  防 onEnd 未触发的异常残留导致曲线永远延伸。拖拽中(.joystick.dragging)绝不归零,
 *  否则按住不动(hold-to-drive)超过 500ms 曲线会错误回落到 0。 */
const JOY_IDLE_TIMEOUT_MS = 500;

/** 把当前 5 个量写入各环形缓冲 */
function writeJoySamples(nowMs, joy) {
  for (const spec of JOY_CURVE_SPECS) {
    const b = ensureJoyBuffer(spec.key);
    const secs = b.t0 === 0 ? 0 : (nowMs - b.t0) / 1000;
    if (b.t0 === 0) b.t0 = nowMs;
    const i = b.head;
    b.t[i] = secs;
    try { b.v[i] = spec.get(joy); }
    catch { b.v[i] = NaN; }
    b.head = (b.head + 1) % BUF_LEN;
    b.count += 1;
  }
}

export function recordJoystickSample(nowMs) {
  const joy = shared.joystick;
  if (!joy || !joy.msgCount) return;  // 从未操作过摇杆,跳过
  if (paused) return;
  // 空闲超时兜底(仅当没有摇杆在拖拽中且不在衰减中才生效)
  const dragging = !!document.querySelector('.joystick.dragging');
  if (!dragging && joy.decayStartAt === 0 && nowMs - (joy.lastUpdateAt || 0) > JOY_IDLE_TIMEOUT_MS) {
    joy.nx = 0; joy.ny = 0;
    joy.vx = 0; joy.vy = 0; joy.omega = 0;
  }
  // 持续记录:有值画值,松开(全 0)画 0 平线——不再跳过零值
  writeJoySamples(nowMs, joy);
}

/** 读摇杆某条曲线的最近 windowSec (与 IMU 相同算法) */
function readJoyRecent(b, windowSec) {
  if (!b || !b.count) return null;
  const { count, head, t } = b;
  const now = t[(head - 1 + BUF_LEN) % BUF_LEN];
  const keepFrom = now - windowSec - _historyOffsetSec;
  const total = Math.min(count, BUF_LEN);
  let start = 0;
  for (let j = 0; j < total; j++) {
    const idx = (head - 1 - j + BUF_LEN) % BUF_LEN;
    if (t[idx] < keepFrom) { start = j; break; }
    start = j + 1;
  }
  return { b, count: start, head, keepFrom };
}
function forEachJoySample(info, cb) {
  if (!info) return;
  const { b, count: n, head } = info;
  for (let j = n - 1; j >= 0; j--) {
    const idx = (head - 1 - j + BUF_LEN) % BUF_LEN;
    cb(b.t[idx], b.v[idx]);
  }
}

/** 画单条摇杆 sparkline:完全复用 IMU 的绘制逻辑(单色折线 + 圆点 + y 轴 label) */
function drawJoystickCurve(canvas, spec, windowSec, dpr) {
  const info = readJoyRecent(joyBuffers.get(spec.key), windowSec);
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  if (w <= 0 || h <= 0) return;
  const pxW = Math.max(1, Math.floor(w * dpr));
  const pxH = Math.max(1, Math.floor(h * dpr));
  if (canvas.width !== pxW || canvas.height !== pxH) {
    canvas.width = pxW; canvas.height = pxH;
  }
  const ctx = canvas.getContext('2d');
  ctx.save();
  ctx.clearRect(0, 0, pxW, pxH);
  ctx.scale(dpr, dpr);

  // Y 轴范围:优先数据动态范围,无数据时固定 [-1, +1]
  let yMin = Infinity, yMax = -Infinity;
  if (info) {
    forEachJoySample(info, (_t, v) => {
      if (!Number.isFinite(v)) return;
      if (v < yMin) yMin = v;
      if (v > yMax) yMax = v;
    });
  }
  if (!Number.isFinite(yMin) || !Number.isFinite(yMax) || yMin === yMax) {
    yMin = -1; yMax = 1;
  } else {
    const pad = Math.max(1e-6, (yMax - yMin) * 0.12);
    yMin -= pad; yMax += pad;
  }
  const color = JOY_GROUP_COLOR[spec.group]?.stroke || '#e5c07b';

  // 背景网格:1 条中线(0) + 上下各两条辅助线
  ctx.strokeStyle = 'rgba(170,193,225,0.08)';
  ctx.lineWidth = 1;
  ctx.font = '9px ui-monospace, monospace';
  ctx.fillStyle = 'rgba(170,193,225,0.4)';
  ctx.textBaseline = 'top';
  ctx.fillText(yMax.toFixed(spec.precision), 2, 1);
  ctx.textBaseline = 'bottom';
  ctx.fillText(yMin.toFixed(spec.precision), 2, h - 1);
  for (let i = 0; i <= 3; i++) {
    const y = (h / 3) * i;
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  }
  if (yMin < 0 && yMax > 0) {
    const y0 = h * (1 - (0 - yMin) / (yMax - yMin));
    ctx.strokeStyle = 'rgba(170,193,225,0.22)';
    ctx.beginPath(); ctx.moveTo(0, y0); ctx.lineTo(w, y0); ctx.stroke();
  }

  // 折线
  ctx.lineJoin = 'round';
  ctx.lineWidth = 1.3;
  ctx.strokeStyle = color;
  let started = false;
  let lastX = 0, lastY = 0, lastVal = 0;
  if (info) {
    const nowT = info.b.t[(info.head - 1 + BUF_LEN) % BUF_LEN];
    const tMin = nowT - windowSec - _historyOffsetSec;
    const tSpan = Math.max(1e-3, windowSec);
    forEachJoySample(info, (t, v) => {
      if (!Number.isFinite(v)) return;
      const x = ((t - tMin) / tSpan) * w;
      const y = h * (1 - (v - yMin) / (yMax - yMin));
      if (!started) { ctx.beginPath(); ctx.moveTo(x, y); started = true; }
      else ctx.lineTo(x, y);
      lastX = x; lastY = y; lastVal = v;
    });
    if (started) ctx.stroke();
  }

  // 当前值圆点 + label
  if (started) {
    ctx.beginPath();
    ctx.arc(lastX, lastY, 2.2, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
    const labelTxt = `${lastVal.toFixed(spec.precision)}${spec.unit}`;
    ctx.font = 'bold 10px ui-monospace, monospace';
    const tw = ctx.measureText(labelTxt).width;
    const lx = Math.max(0, Math.min(w - tw - 3, lastX - tw - 6));
    const ly = Math.max(2, Math.min(h - 12, lastY - 12));
    ctx.fillStyle = 'rgba(14, 23, 38, 0.82)';
    ctx.fillRect(lx - 2, ly, tw + 4, 12);
    ctx.strokeStyle = color;
    ctx.lineWidth = 1;
    ctx.strokeRect(lx - 2 + 0.5, ly + 0.5, tw + 3, 11);
    ctx.fillStyle = '#ffffff';
    ctx.textBaseline = 'top';
    ctx.fillText(labelTxt, lx, ly + 1);
  }
  ctx.restore();
}

/** 构建摇杆曲线行 DOM */
export function rebuildJoystickCurveList(container) {
  container.innerHTML = '';
  const frag = document.createDocumentFragment();
  let lastGroup = null;
  for (const spec of JOY_CURVE_SPECS) {
    if (spec.group !== lastGroup) {
      const grpHeader = document.createElement('div');
      grpHeader.className = 'imu-group-header joy-group-header';
      const dot = document.createElement('i');
      dot.className = `legend-dot`;
      dot.style.backgroundColor = JOY_GROUP_COLOR[spec.group].stroke;
      const label = document.createElement('span');
      const groupName = { input: '归一化输入 (nx, ny)', vel: '底盘速度/角速度 (vx, vy, ω)' }[spec.group] || spec.group;
      label.textContent = groupName;
      grpHeader.appendChild(dot);
      grpHeader.appendChild(label);
      frag.appendChild(grpHeader);
      lastGroup = spec.group;
    }
    const row = document.createElement('div');
    row.className = 'curves-row joy-curve-row';
    row.dataset.joyKey = spec.key;

    const lab = document.createElement('div');
    lab.className = 'curves-row-label';
    const name = document.createElement('span');
    name.className = 'curves-joint-name joy-curve-name';
    name.innerHTML = `${spec.label} ${spec.unit ? `<span class="joy-unit">${spec.unit}</span>` : ''}`;
    const val = document.createElement('span');
    val.className = 'curves-joint-value joy-curve-value';
    val.dataset.joyKey = spec.key;
    lab.appendChild(name);
    lab.appendChild(val);

    const wrap = document.createElement('div');
    wrap.className = 'curves-canvas-wrap';
    const cv = document.createElement('canvas');
    cv.className = 'curves-canvas joy-curve-canvas';
    cv.dataset.joyKey = spec.key;
    wrap.appendChild(cv);

    row.appendChild(lab);
    row.appendChild(wrap);
    frag.appendChild(row);
  }
  container.appendChild(frag);
}

/** 渲染摇杆曲线:每帧节流调用 */
export function renderJoystickCurves(windowSec) {
  const container = document.getElementById('joystick-curves-list');
  if (!container) return;
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  for (const spec of JOY_CURVE_SPECS) {
    const canvas = container.querySelector(`canvas.joy-curve-canvas[data-joy-key="${spec.key}"]`);
    const valEl  = container.querySelector(`.joy-curve-value[data-joy-key="${spec.key}"]`);
    if (canvas) drawJoystickCurve(canvas, spec, windowSec, dpr);
    if (valEl) {
      const joy = shared.joystick;
      let v = 0;
      try { v = joy && joy.msgCount ? spec.get(joy) : 0; } catch { v = NaN; }
      if (!Number.isFinite(v)) valEl.textContent = `—`;
      else valEl.textContent = `${v.toFixed(spec.precision)}${spec.unit}`;
    }
  }
}
