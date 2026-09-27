/**
 * 雷达页签模块(Livox MID-360)。
 *
 * 点云:/livox/lidar  sensor_msgs/msg/PointCloud2, 50Hz, 约 4000 点/帧
 *      驱动默认 fields: x,y,z,intensity 均 float32(point_step=16);
 *      解码器按 msg.fields 的 offset/datatype 泛化解析,兼容 uint8 intensity。
 * IMU: /livox/imu sensor_msgs/msg/Imu, 200Hz 内置 BMI088(orientation 通常为空,姿态由重力估计)。
 *
 * 两话题都是 best_effort QoS,ros-bridge 已注入 qos;订阅仅在「雷达」页签可见且在线时建立
 * (点云节流到 10Hz,IMU 20Hz)。
 *
 * 单导入方模块(仅 main.js 引用),可独立 ?v= 版本号。
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import {
  addStatusListener,
  addLidarFrameListener, addLidarImuListener,
  setLidarEnabled,
} from './ros-bridge.js?v=981';

let viewVisible = false;
let rosOnline = false;
let els = {};

// ── Three.js 资源 ─────────────────────────────────────────
let renderer = null;
let scene = null;
let camera3d = null;
let controls = null;
let pointsObj = null;
let pointsGeo = null;
let dataGroup = null;
let rafId = 0;
let autoRotate = false;
let colorMode = 'height'; // 'height' | 'intensity'
/** 最新一帧解码结果(切着色模式时重算颜色) */
let latestFrame = null;

// ── 累加建图:Livox 是非重复扫描,单帧只覆盖场景的一小部分,
//    多帧持续接收并在固定坐标系重叠 → 体素去重后逐步还原完整场景。
//    静止(匀速)采集前提;机器人移动后需配合位姿(里程计)做坐标变换。
const ACC_VOXEL = 0.05;  // 体素边长(m):同一 5cm 立方体内只保留首个点
const ACC_MAX = 600000;  // 最大体素点数(position/color 缓冲按此预分配)
const ACC_KEY_BASE = 4096;
const ACC_KEY_OFFSET = 2048; // 覆盖 ±102m(voxel 坐标 0..4095)
let accumulate = true;
let accMap = null;          // 体素键 → 1(去重集合)
let accObj = null;          // 累加点云 Points(与单帧 pointsObj 切换显示)
let accGeo = null;
let accPos = null;          // 预分配 Float32Array(ACC_MAX*3),只增量写入
let accCol = null;
let accInten = null;        // 每点反射率(切反射率着色时用)
let accCount = 0;
let accFull = false;        // 达到容量上限

// ── 点大小(用户可调,与相机缩放解耦;两套材质同一尺寸) ──
const POINT_SIZE_KEY = 'web_sim_lidar_point_size';
const POINT_SIZE_MIN = 0.01;
const POINT_SIZE_MAX = 0.30;
const POINT_SIZE_DEFAULT = 0.05;
let pointSize = POINT_SIZE_DEFAULT;
let frameMat = null;
let accMat = null;

// ── 统计 ─────────────────────────────────────────────────
let frameCount = 0;
let windowFrames = 0;
let windowBytes = 0;
let windowStartedAt = 0;
let lastFrameAt = 0;
let lastPointCount = 0;
let lastMaxRange = 0;
let lastPointStep = 0;
let lastFrameId = '-';

// ── IMU ──────────────────────────────────────────────────
let lastLidarImu = null;
let lastRoll = 0;
let lastPitch = 0;
/** 6 通道滚动波形缓冲:ax,ay,az,gx,gy,gz */
const WAVE_LEN = 300;
const waveBuf = [
  new Float32Array(WAVE_LEN), new Float32Array(WAVE_LEN), new Float32Array(WAVE_LEN),
  new Float32Array(WAVE_LEN), new Float32Array(WAVE_LEN), new Float32Array(WAVE_LEN),
];
let waveHead = 0;
let waveSamples = 0;
let waveDirty = true;

const ACC_SCALE = 20;  // m/s² 波形满量程
const GYR_SCALE = 3;   // rad/s 波形满量程
const WAVE_COLORS = ['#ff7a7a', '#7ee08a', '#6db6ff', '#c96565', '#5fae6b', '#568bc9'];

function fmt(n, d = 2) { return Number.isFinite(n) ? n.toFixed(d) : '-'; }
function radToDeg(r) { return r * 180 / Math.PI; }

/** base64 → Uint8Array */
function b64ToBytes(b64) {
  const bin = atob(b64);
  const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out;
}

// ── PointCloud2 解码 ─────────────────────────────────────
// ROS PointField datatype 枚举
const PF_INT8 = 1, PF_UINT8 = 2, PF_INT16 = 3, PF_UINT16 = 4, PF_INT32 = 5, PF_UINT32 = 6, PF_FLOAT32 = 7, PF_FLOAT64 = 8;

/**
 * 解析 PointCloud2 → { positions:Float32Array(n*3), intensity:Float32Array(n), count, maxRange, minZ, maxZ }
 * 非有限点(NaN/inf 无效回波)丢弃。
 */
function decodePointCloud2(msg) {
  const raw = typeof msg.data === 'string' ? b64ToBytes(msg.data) : msg.data;
  if (!raw) return null;
  const u8 = raw instanceof Uint8Array ? raw : new Uint8Array(raw);
  const pointStep = msg.point_step | 0;
  if (!pointStep) return null;
  const n = (msg.width | 0) * (msg.height > 0 ? msg.height | 0 : 1);
  if (!n) return null;
  const le = !msg.is_bigendian;

  // 按 fields 找偏移(默认 livox 驱动布局 x@0 y@4 z@8 intensity@12, 全 float32)
  let ox = 0, oy = 4, oz = 8, oi = 12, itype = PF_FLOAT32, hasIntensity = false;
  for (const f of msg.fields || []) {
    if (f.name === 'x') ox = f.offset;
    else if (f.name === 'y') oy = f.offset;
    else if (f.name === 'z') oz = f.offset;
    else if (f.name === 'intensity' || f.name === 'i') { oi = f.offset; itype = f.datatype; hasIntensity = true; }
  }

  const dv = new DataView(u8.buffer, u8.byteOffset, u8.byteLength);
  const positions = new Float32Array(n * 3);
  const intensity = new Float32Array(n);
  let valid = 0;
  let maxRange = 0, minZ = Infinity, maxZ = -Infinity;
  const readIntensity = (p) => {
    switch (itype) {
      case PF_FLOAT32: return dv.getFloat32(p + oi, le);
      case PF_FLOAT64: return dv.getFloat64(p + oi, le);
      case PF_UINT8: return dv.getUint8(p + oi);
      case PF_INT8: return dv.getInt8(p + oi);
      case PF_UINT16: return dv.getUint16(p + oi, le);
      case PF_INT16: return dv.getInt16(p + oi, le);
      case PF_UINT32: return dv.getUint32(p + oi, le);
      case PF_INT32: return dv.getInt32(p + oi, le);
      default: return 0;
    }
  };
  for (let i = 0; i < n; i++) {
    const p = i * pointStep;
    if (p + Math.max(ox, oy, oz) + 4 > u8.length) break;
    const x = dv.getFloat32(p + ox, le);
    const y = dv.getFloat32(p + oy, le);
    const z = dv.getFloat32(p + oz, le);
    if (!Number.isFinite(x + y + z)) continue;
    const o = valid * 3;
    positions[o] = x; positions[o + 1] = y; positions[o + 2] = z;
    if (hasIntensity && p + oi + 1 <= u8.length) intensity[valid] = readIntensity(p);
    const r = Math.sqrt(x * x + y * y + z * z);
    if (r > maxRange) maxRange = r;
    if (z < minZ) minZ = z;
    if (z > maxZ) maxZ = z;
    valid++;
  }
  if (!valid) return null;
  return {
    positions: positions.slice(0, valid * 3),
    intensity: intensity.slice(0, valid),
    count: valid, maxRange, minZ: minZ === Infinity ? 0 : minZ, maxZ: maxZ === -Infinity ? 0 : maxZ,
  };
}

/** 高度→颜色:蓝(低)→青→绿→黄→红(高),固定量程 -2~3m,避免帧间闪烁 */
const _cTmp = new THREE.Color();
function heightColor(z, out, idx) {
  const t = THREE.MathUtils.clamp((z + 2) / 5, 0, 1);
  _cTmp.setHSL((1 - t) * 0.66, 0.85, 0.55);
  out[idx] = _cTmp.r; out[idx + 1] = _cTmp.g; out[idx + 2] = _cTmp.b;
}
/** 反射率→灰度(Livox intensity 常见 0~255) */
function intensityColor(v, out, idx) {
  const g = THREE.MathUtils.clamp(v / 255, 0.04, 1);
  out[idx] = g * 0.85; out[idx + 1] = g * 0.95; out[idx + 2] = g;
}

function recomputeColors(frame) {
  const colors = new Float32Array(frame.count * 3);
  const pos = frame.positions;
  for (let i = 0; i < frame.count; i++) {
    if (colorMode === 'height') heightColor(pos[i * 3 + 2], colors, i * 3);
    else intensityColor(frame.intensity[i], colors, i * 3);
  }
  return colors;
}

// ── Three 场景 ───────────────────────────────────────────
function initThree() {
  const canvas = els.canvas;
  renderer = new THREE.WebGLRenderer({ canvas, antialias: false, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setClearColor(0x000000, 0);

  scene = new THREE.Scene();
  camera3d = new THREE.PerspectiveCamera(60, 1, 0.05, 200);
  camera3d.position.set(5.5, -5.5, 4.2);

  controls = new OrbitControls(camera3d, canvas);
  controls.enableDamping = true;
  controls.dampingFactor = 0.12;
  controls.minDistance = 0.3;
  controls.maxDistance = 80;
  controls.target.set(0, 0, 0);

  // 雷达数据是 z-up,Three 默认 y-up:全部数据放进绕 X 轴 -90° 的组(local z→world y)
  dataGroup = new THREE.Group();
  dataGroup.rotation.x = -Math.PI / 2;
  scene.add(dataGroup);

  // 地面网格(传感器原点为中心)
  const grid = new THREE.GridHelper(40, 40, 0x2a4458, 0x18262f);
  grid.material.transparent = true;
  grid.material.opacity = 0.55;
  grid.material.depthWrite = false;
  dataGroup.add(grid);
  dataGroup.add(new THREE.AxesHelper(1.2));

  pointsGeo = new THREE.BufferGeometry();
  pointsGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3), 3).setUsage(THREE.DynamicDrawUsage));
  pointsGeo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(3), 3).setUsage(THREE.DynamicDrawUsage));
  pointsGeo.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 100); // 固定大球,跳过逐帧包围体重算
  frameMat = new THREE.PointsMaterial({
    size: pointSize, sizeAttenuation: true, vertexColors: true,
    transparent: true, opacity: 0.92, depthWrite: false,
  });
  const mat = frameMat;
  pointsObj = new THREE.Points(pointsGeo, mat);
  pointsObj.frustumCulled = false;
  dataGroup.add(pointsObj);

  // 累加点云:容量一次性预分配,运行时只往尾部追加并扩大 drawRange,
  // GPU 仅上传新增区间(addUpdateRange),不做任何全量重建/拷贝
  accMap = new Map();
  accPos = new Float32Array(ACC_MAX * 3);
  accCol = new Float32Array(ACC_MAX * 3);
  accInten = new Float32Array(ACC_MAX);
  accGeo = new THREE.BufferGeometry();
  accGeo.setAttribute('position', new THREE.BufferAttribute(accPos, 3).setUsage(THREE.DynamicDrawUsage));
  accGeo.setAttribute('color', new THREE.BufferAttribute(accCol, 3).setUsage(THREE.DynamicDrawUsage));
  accGeo.setDrawRange(0, 0);
  accGeo.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 100);
  accMat = new THREE.PointsMaterial({
    size: pointSize, sizeAttenuation: true, vertexColors: true,
    transparent: true, opacity: 0.95, depthWrite: false,
  });
  accObj = new THREE.Points(accGeo, accMat);
  accObj.frustumCulled = false;
  accObj.visible = accumulate;
  dataGroup.add(accObj);
  pointsObj.visible = !accumulate;

  resizeRenderer();
  if (typeof ResizeObserver !== 'undefined') {
    const ro = new ResizeObserver(() => resizeRenderer());
    ro.observe(canvas.parentElement);
  }
  window.addEventListener('resize', resizeRenderer);

  const loop = () => {
    rafId = requestAnimationFrame(loop);
    if (!viewVisible) return;
    if (autoRotate) dataGroup.rotation.z += 0.004; // 绕竖直(雷达 z)轴缓慢转
    controls.update();
    renderer.render(scene, camera3d);
    if (waveDirty) { drawWaveform(); waveDirty = false; }
  };
  loop();
}

function resizeRenderer() {
  if (!renderer || !els.canvas) return;
  const w = els.canvas.clientWidth || 0;
  const h = els.canvas.clientHeight || 0;
  if (!w || !h) return;
  renderer.setSize(w, h, false);
  camera3d.aspect = w / h;
  camera3d.updateProjectionMatrix();
}

/** 用最新帧替换点云几何(单帧模式) */
function uploadFrame(frame) {
  latestFrame = frame;
  const colors = recomputeColors(frame);
  pointsGeo.setAttribute('position', new THREE.BufferAttribute(frame.positions, 3).setUsage(THREE.DynamicDrawUsage));
  pointsGeo.setAttribute('color', new THREE.BufferAttribute(colors, 3).setUsage(THREE.DynamicDrawUsage));
}

// ── 累加建图 ─────────────────────────────────────────────
/** 体素哈希键:空间位置 → 0..4095³ 网格内的唯一整数 */
function voxelKey(x, y, z) {
  let ix = Math.floor(x / ACC_VOXEL) + ACC_KEY_OFFSET;
  let iy = Math.floor(y / ACC_VOXEL) + ACC_KEY_OFFSET;
  let iz = Math.floor(z / ACC_VOXEL) + ACC_KEY_OFFSET;
  if (ix < 0) ix = 0; else if (ix >= ACC_KEY_BASE) ix = ACC_KEY_BASE - 1;
  if (iy < 0) iy = 0; else if (iy >= ACC_KEY_BASE) iy = ACC_KEY_BASE - 1;
  if (iz < 0) iz = 0; else if (iz >= ACC_KEY_BASE) iz = ACC_KEY_BASE - 1;
  return (ix * ACC_KEY_BASE + iy) * ACC_KEY_BASE + iz;
}

/**
 * 把一帧并入累加地图:仅新体素追加到预分配缓冲尾部。
 * 返回本帧新增体素数。
 */
function insertAccumulated(frame) {
  if (!accMap || accCount >= ACC_MAX) { accFull = true; return 0; }
  const pos = frame.positions;
  const inten = frame.intensity;
  const start = accCount;
  for (let i = 0; i < frame.count; i++) {
    if (accCount >= ACC_MAX) { accFull = true; break; }
    const o = i * 3;
    const x = pos[o], y = pos[o + 1], z = pos[o + 2];
    const key = voxelKey(x, y, z);
    if (accMap.has(key)) continue;
    accMap.set(key, 1);
    const w = accCount * 3;
    accPos[w] = x; accPos[w + 1] = y; accPos[w + 2] = z;
    accInten[accCount] = inten[i];
    if (colorMode === 'height') heightColor(z, accCol, w);
    else intensityColor(inten[i], accCol, w);
    accCount++;
  }
  const added = accCount - start;
  if (added > 0) {
    accGeo.setDrawRange(0, accCount);
    const pa = accGeo.getAttribute('position');
    const ca = accGeo.getAttribute('color');
    // addUpdateRange 的单位是"分量下标"(Float32 下标),顶点要 ×3
    pa.addUpdateRange(start * 3, added * 3);
    ca.addUpdateRange(start * 3, added * 3);
    pa.needsUpdate = true;
    ca.needsUpdate = true;
  }
  return added;
}

/** 切换着色模式后重刷全部累加点颜色(仅此时做一次全量上传) */
function recolorAccumulatedAll() {
  if (!accMap || accCount === 0) return;
  for (let i = 0; i < accCount; i++) {
    const w = i * 3;
    if (colorMode === 'height') heightColor(accPos[w + 2], accCol, w);
    else intensityColor(accInten[i], accCol, w);
  }
  const ca = accGeo.getAttribute('color');
  ca.addUpdateRange(0, accCount * 3);
  ca.needsUpdate = true;
}

/** 清空累加地图 */
function clearAccumulated() {
  if (accMap) accMap.clear();
  accCount = 0;
  accFull = false;
  if (accGeo) accGeo.setDrawRange(0, 0);
}

/** 单帧/累加显示切换 */
function setAccumulateMode(on) {
  accumulate = !!on;
  if (accObj) accObj.visible = accumulate;
  if (pointsObj) pointsObj.visible = !accumulate;
}

/** 设置点大小(m,世界尺寸):同步两套材质并持久化 */
function applyPointSize(v) {
  pointSize = THREE.MathUtils.clamp(+v || POINT_SIZE_DEFAULT, POINT_SIZE_MIN, POINT_SIZE_MAX);
  if (frameMat) frameMat.size = pointSize;
  if (accMat) accMat.size = pointSize;
  if (els.sizeVal) els.sizeVal.textContent = pointSize.toFixed(2);
  try { localStorage.setItem(POINT_SIZE_KEY, String(pointSize)); } catch { /* 忽略 */ }
}

// ── 帧/IMU 回调 ─────────────────────────────────────────
function onLidarFrame(msg) {
  if (!viewVisible) return;
  const frame = decodePointCloud2(msg);
  if (!frame) return;
  if (accumulate) insertAccumulated(frame);
  else uploadFrame(frame);
  frameCount++;
  windowFrames++;
  // PointCloud2 二进制 base64 长度 ≈ 字节数 × 4/3
  const rawLen = (msg.point_step | 0) * frame.count;
  windowBytes += rawLen;
  lastPointCount = frame.count;
  lastMaxRange = frame.maxRange;
  lastPointStep = msg.point_step | 0;
  lastFrameId = String(msg.header?.frame_id || '-');
  const now = performance.now();
  if (!windowStartedAt) windowStartedAt = now;
  lastFrameAt = now;
  els.placeholder.style.display = 'none';
}

function onLidarImu(msg) {
  lastLidarImu = msg;
  const a = msg.linear_acceleration || {};
  const g = msg.angular_velocity || {};
  // BMI088 orientation 通常全 0:用重力向量估计 roll/pitch(静止/匀速时准)
  const ax = +a.x || 0, ay = +a.y || 0, az = +a.z || 0;
  const gNorm = Math.hypot(ax, ay, az);
  if (gNorm > 1e-3) {
    lastPitch = Math.asin(THREE.MathUtils.clamp(-ax / gNorm, -1, 1));
    lastRoll = Math.atan2(ay, az);
  }
  waveBuf[0][waveHead] = ax; waveBuf[1][waveHead] = ay; waveBuf[2][waveHead] = az;
  waveBuf[3][waveHead] = +g.x || 0; waveBuf[4][waveHead] = +g.y || 0; waveBuf[5][waveHead] = +g.z || 0;
  waveHead = (waveHead + 1) % WAVE_LEN;
  waveSamples++;
  waveDirty = true;
}

// ── 姿态球(由重力估计的 roll/pitch) ─────────────────────
function drawAttitude() {
  const c = els.attiCanvas;
  if (!c) return;
  const ctx = c.getContext('2d');
  const W = c.width, H = c.height, cx = W / 2, cy = H / 2, R = W / 2 - 2;
  ctx.clearRect(0, 0, W, H);
  ctx.save();
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, Math.PI * 2); ctx.clip();
  // 天/地分界线随 roll 旋转、随 pitch 上下平移
  ctx.translate(cx, cy);
  ctx.rotate(-lastRoll);
  const off = THREE.MathUtils.clamp(radToDeg(lastPitch) / 45, -1, 1) * R;
  ctx.translate(0, off);
  ctx.fillStyle = '#6b4a1e'; // 地(褐)
  ctx.fillRect(-R, 0, 2 * R, 2 * R);
  ctx.fillStyle = '#2f6fae'; // 天(蓝)
  ctx.fillRect(-R, -2 * R, 2 * R, 2 * R);
  // 地平线
  ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(-R, 0); ctx.lineTo(R, 0); ctx.stroke();
  // 俯仰刻度
  ctx.lineWidth = 1;
  for (const d of [-30, -20, -10, 10, 20, 30]) {
    const y = -d / 45 * R;
    ctx.beginPath(); ctx.moveTo(-14, y); ctx.lineTo(14, y); ctx.stroke();
  }
  ctx.restore();
  // 外圆 + 固定飞机符号(不随球转)
  ctx.strokeStyle = '#9fb4c4'; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, Math.PI * 2); ctx.stroke();
  ctx.strokeStyle = '#ffd35c'; ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.moveTo(cx - 26, cy); ctx.lineTo(cx - 8, cy);
  ctx.moveTo(cx + 8, cy); ctx.lineTo(cx + 26, cy);
  ctx.moveTo(cx - 8, cy); ctx.lineTo(cx - 8, cy - 6);
  ctx.moveTo(cx + 8, cy); ctx.lineTo(cx + 8, cy - 6);
  ctx.stroke();
  // 顶部滚转刻线
  ctx.fillStyle = '#9fb4c4'; ctx.font = '9px monospace'; ctx.textAlign = 'center';
  ctx.fillText(`R ${radToDeg(lastRoll).toFixed(1)}°  P ${radToDeg(lastPitch).toFixed(1)}°`, cx, H - 5);
}

// ── 6 通道滚动波形 ───────────────────────────────────────
function drawWaveform() {
  const c = els.waveCanvas;
  if (!c) return;
  const ctx = c.getContext('2d');
  const W = c.width, H = c.height;
  ctx.clearRect(0, 0, W, H);
  // 零轴 + 量程参考线
  ctx.strokeStyle = 'rgba(255,255,255,0.12)'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0, H / 2); ctx.lineTo(W, H / 2); ctx.stroke();
  ctx.strokeStyle = 'rgba(255,255,255,0.05)';
  for (const q of [0.25, 0.75]) { ctx.beginPath(); ctx.moveTo(0, H * q); ctx.lineTo(W, H * q); ctx.stroke(); }

  for (let ch = 0; ch < 6; ch++) {
    const buf = waveBuf[ch];
    const scale = ch < 3 ? ACC_SCALE : GYR_SCALE;
    ctx.strokeStyle = WAVE_COLORS[ch];
    ctx.lineWidth = 1;
    ctx.beginPath();
    const filled = Math.min(waveSamples, WAVE_LEN);
    for (let px = 0; px < filled; px++) {
      const idx = (waveHead - filled + px + WAVE_LEN) % WAVE_LEN;
      const v = THREE.MathUtils.clamp(buf[idx] / scale, -1, 1);
      const x = px / (WAVE_LEN - 1) * W;
      const y = H / 2 - v * (H / 2 - 3);
      if (px === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
}

// ── 数字面板 8Hz 刷新(高频消息不直写 DOM) ────────────────
let flushTimer = null;
function flushDom() {
  if (!viewVisible) return;
  const now = performance.now();
  const dt = windowStartedAt ? (now - windowStartedAt) / 1000 : 0;
  if (dt >= 0.5) {
    els.fps.textContent = `${fmt(windowFrames / dt, 1)} FPS`;
    els.kbps.textContent = `${(windowBytes / dt / 1024).toFixed(0)} KB/s`;
    els.fps.classList.toggle('is-stale', now - lastFrameAt > 1000);
    windowFrames = 0; windowBytes = 0; windowStartedAt = now;
  }
  els.points.textContent = `${lastPointCount} 点`;
  if (els.accPoints) els.accPoints.textContent = accFull ? `累计 ${accCount} 点(满)` : `累计 ${accCount} 点`;
  els.range.textContent = `${fmt(lastMaxRange, 1)} m`;
  els.step.textContent = lastPointStep ? `step ${lastPointStep}B` : '-';
  els.frame.textContent = lastFrameId;
  els.frameCount.textContent = `帧 ${frameCount}`;

  if (lastLidarImu) {
    const a = lastLidarImu.linear_acceleration || {};
    const g = lastLidarImu.angular_velocity || {};
    els.accX.textContent = fmt(a.x, 3); els.accY.textContent = fmt(a.y, 3); els.accZ.textContent = fmt(a.z, 3);
    els.gyrX.textContent = fmt(g.x, 3); els.gyrY.textContent = fmt(g.y, 3); els.gyrZ.textContent = fmt(g.z, 3);
    els.gyrXD.textContent = fmt(radToDeg(g.x), 1);
    els.gyrYD.textContent = fmt(radToDeg(g.y), 1);
    els.gyrZD.textContent = fmt(radToDeg(g.z), 1);
    drawAttitude();
  }
}

// ── 可见性与订阅开关 ─────────────────────────────────────
function applyDesired() {
  const want = viewVisible && rosOnline;
  setLidarEnabled(want);
  if (want) {
    els.streamState.textContent = '● 订阅中(best_effort)';
    els.streamState.classList.add('is-live');
  } else {
    els.streamState.textContent = rosOnline ? '未订阅' : 'ROS 未连接';
    els.streamState.classList.remove('is-live');
  }
  if (want) resizeRenderer();
}

export function setLidarViewVisible(visible) {
  viewVisible = !!visible;
  applyDesired();
  if (viewVisible) {
    // 从隐藏恢复:canvas 尺寸可能为 0,重新对齐
    requestAnimationFrame(resizeRenderer);
  }
}

export function initLidarView() {
  els = {
    canvas: document.querySelector('#lidar-3d-canvas'),
    placeholder: document.querySelector('#lidar-placeholder'),
    streamState: document.querySelector('#lidar-stream-state'),
    fps: document.querySelector('#lidar-fps'),
    kbps: document.querySelector('#lidar-kbps'),
    points: document.querySelector('#lidar-points'),
    accPoints: document.querySelector('#lidar-acc-points'),
    range: document.querySelector('#lidar-range'),
    step: document.querySelector('#lidar-step'),
    frame: document.querySelector('#lidar-frame'),
    frameCount: document.querySelector('#lidar-frame-count'),
    btnReset: document.querySelector('#lidar-btn-reset'),
    btnTop: document.querySelector('#lidar-btn-top'),
    btnSide: document.querySelector('#lidar-btn-side'),
    btnRotate: document.querySelector('#lidar-btn-rotate'),
    btnColor: document.querySelector('#lidar-btn-color'),
    btnAcc: document.querySelector('#lidar-btn-acc'),
    btnClear: document.querySelector('#lidar-btn-clear'),
    sizeRange: document.querySelector('#lidar-point-size'),
    sizeVal: document.querySelector('#lidar-point-size-val'),
    attiCanvas: document.querySelector('#lidar-atti'),
    waveCanvas: document.querySelector('#lidar-wave'),
    accX: document.querySelector('#lidar-acc-x'), accY: document.querySelector('#lidar-acc-y'), accZ: document.querySelector('#lidar-acc-z'),
    gyrX: document.querySelector('#lidar-gyr-x'), gyrY: document.querySelector('#lidar-gyr-y'), gyrZ: document.querySelector('#lidar-gyr-z'),
    gyrXD: document.querySelector('#lidar-gyr-x-d'), gyrYD: document.querySelector('#lidar-gyr-y-d'), gyrZD: document.querySelector('#lidar-gyr-z-d'),
  };
  if (!els.canvas) return;

  // 波形/姿态球固定内部分辨率(CSS 负责显示尺寸)
  els.waveCanvas.width = 600; els.waveCanvas.height = 110;
  els.attiCanvas.width = 132; els.attiCanvas.height = 132;

  initThree();

  const setView = (x, y, z, tx = 0, ty = 0, tz = 0) => {
    camera3d.position.set(x, y, z);
    controls.target.set(tx, ty, tz);
    controls.update();
  };
  els.btnReset?.addEventListener('click', () => setView(5.5, -5.5, 4.2));
  els.btnTop?.addEventListener('click', () => setView(0.01, 0.01, 14));
  els.btnSide?.addEventListener('click', () => setView(0.01, -10, 0.6));
  els.btnRotate?.addEventListener('click', () => {
    autoRotate = !autoRotate;
    els.btnRotate.classList.toggle('is-active', autoRotate);
    if (!autoRotate) dataGroup.rotation.z = 0;
  });
  els.btnColor?.addEventListener('click', () => {
    colorMode = colorMode === 'height' ? 'intensity' : 'height';
    els.btnColor.textContent = colorMode === 'height' ? '着色:高度' : '着色:反射率';
    if (latestFrame) uploadFrame(latestFrame);
    recolorAccumulatedAll();
  });
  els.btnAcc?.addEventListener('click', () => {
    setAccumulateMode(!accumulate);
    els.btnAcc.classList.toggle('is-active', accumulate);
  });
  els.btnClear?.addEventListener('click', () => {
    clearAccumulated();
  });

  // 点大小滑杆(拖动即时生效,刷新后保留)
  if (els.sizeRange) {
    els.sizeRange.min = String(POINT_SIZE_MIN);
    els.sizeRange.max = String(POINT_SIZE_MAX);
    els.sizeRange.step = '0.01';
    let saved = POINT_SIZE_DEFAULT;
    try { saved = parseFloat(localStorage.getItem(POINT_SIZE_KEY)) || POINT_SIZE_DEFAULT; } catch { /* 忽略 */ }
    els.sizeRange.value = String(saved);
    els.sizeRange.addEventListener('input', () => applyPointSize(els.sizeRange.value));
    applyPointSize(saved);
  }

  addLidarFrameListener(onLidarFrame);
  addLidarImuListener(onLidarImu);
  addStatusListener((s) => {
    const online = s.connectionStatus === 'online';
    if (online !== rosOnline) {
      rosOnline = online;
      applyDesired();
    }
  });

  applyDesired();
  if (flushTimer) clearInterval(flushTimer);
  flushTimer = setInterval(flushDom, 125);
}
