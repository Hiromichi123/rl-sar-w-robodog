/**
 * 相机页签模块(Insight 9)。
 *
 * 画面:/camera/camera/color/image_raw/compressed (JPEG CompressedImage)
 *      /camera/camera/infra1|infra2/image_raw    (raw Image,多为 mono8)
 * 数据:/camera/camera/imu (Imu)、/camera/camera/vio_100hz (Odometry)、
 *      /camera/camera/vio_status (String)
 *
 * 相机话题全部是 best_effort QoS,订阅由 ros-bridge._subscribeWithQos 注入 qos。
 * 订阅按需建立:仅「相机」页签可见且 ROS 在线时订阅画面+遥测,切走即退订省带宽。
 *
 * 单导入方模块(仅 main.js 引用),可独立 ?v= 版本号。
 */
import {
  addStatusListener,
  addCameraFrameListener, addCameraImuListener, addCameraVioListener, addCameraVioStatusListener,
  setCameraChannel, setCameraTelemetryEnabled,
} from './ros-bridge.js?v=981';

/** 当前选中的画面通道 */
let currentChannel = 'color';
/** 页签是否可见 */
let viewVisible = false;
/** ROS 是否在线 */
let rosOnline = false;

// ── DOM 引用(init 时缓存) ─────────────────────────────────
let els = {};
/** 各通道可复用的 ImageData(raw 渲染) */
let rawImageData = null;
let blobUrl = null;
/** 彩色首帧 JPEG 是否已成功解码并显示(空 src 的 <img> 会显示破损图标,必须等 onload 后才取消 hidden) */
let colorReady = false;

// ── 统计 ─────────────────────────────────────────────────
let frameCount = 0;
let windowFrames = 0;
let windowBytes = 0;
let windowStartedAt = 0;
let lastFrameAt = 0;
let lastByteLen = 0;
let lastDim = '-';
let lastEncoding = '-';

// ── 最新一帧遥测(只在定时器里刷 DOM,避免高频写 DOM) ──────
let lastImu = null;
let lastVio = null;
let lastVioStatus = '';
let flushTimer = null;

/** base64 → Uint8Array(rosbridge JSON 传输 uint8[] 时为 base64 字符串) */
function b64ToBytes(b64) {
  const bin = atob(b64);
  const n = bin.length;
  const out = new Uint8Array(n);
  for (let i = 0; i < n; i++) out[i] = bin.charCodeAt(i);
  return out;
}

function fmt(n, digits = 2) {
  return Number.isFinite(n) ? n.toFixed(digits) : '-';
}

/** CompressedImage 帧 → <img>。返回 true 表示已提交解码(占位层由 onload 负责撤掉) */
function renderCompressed(channel, msg) {
  if (channel !== 'color') return false;
  const data = msg.data;
  const mime = /png/i.test(String(msg.format || '')) ? 'image/png' : 'image/jpeg';
  // 每次赋值前重挂 load/error:data/blob URL 的 load 是异步的,
  // 只有真正解码成功才取消 img 的 hidden,避免空 src/坏帧露出破损图标与 alt 文案
  els.colorImg.onload = () => {
    colorReady = true;
    els.colorImg.hidden = false;
    els.placeholder.style.display = 'none';
    hideOverlay();
  };
  els.colorImg.onerror = () => {
    colorReady = false;
    els.colorImg.hidden = true;
    showOverlay('彩色图像解码失败(数据不完整或编码非 JPEG/PNG)');
  };
  let submitted = false;
  if (typeof data === 'string' && data.length > 0) {
    els.colorImg.src = `data:${mime};base64,${data}`;
    submitted = true;
  } else if (data instanceof Uint8Array || data instanceof ArrayBuffer) {
    const bytes = data instanceof ArrayBuffer ? new Uint8Array(data) : data;
    if (bytes && bytes.byteLength > 0) {
      if (blobUrl) URL.revokeObjectURL(blobUrl);
      blobUrl = URL.createObjectURL(new Blob([bytes], { type: mime }));
      els.colorImg.src = blobUrl;
      submitted = true;
    }
  }
  lastByteLen = typeof data === 'string' ? Math.round((data.length * 3) / 4) : (data?.byteLength || 0);
  lastEncoding = String(msg.format || mime.split('/')[1]).toUpperCase();
  lastDim = '-';
  return submitted;
}

/** raw Image 帧 → canvas(支持 mono8/8UC1、rgb8、bgr8、rgba8、bgra8) */
function renderRawImage(channel, msg) {
  const canvas = channel === 'infra1' ? els.infra1Canvas : els.infra2Canvas;
  if (!canvas) return;
  const W = msg.width | 0;
  const H = msg.height | 0;
  const enc = String(msg.encoding || 'mono8');
  if (!W || !H) return;
  if (canvas.width !== W || canvas.height !== H || !rawImageData || rawImageData.width !== W || rawImageData.height !== H) {
    canvas.width = W;
    canvas.height = H;
    rawImageData = canvas.getContext('2d', { willReadFrequently: true }).createImageData(W, H);
  }
  const ctx = canvas.getContext('2d', { willReadFrequently: true });
  const bytes = typeof msg.data === 'string' ? b64ToBytes(msg.data) : msg.data;
  if (!bytes) return;
  const dst = rawImageData.data;
  const step = msg.step || W;
  let supported = true;
  if (enc === 'mono8' || enc === '8UC1') {
    for (let y = 0; y < H; y++) {
      const row = y * step;
      for (let x = 0; x < W; x++) {
        const v = bytes[row + x];
        const o = (y * W + x) * 4;
        dst[o] = v; dst[o + 1] = v; dst[o + 2] = v; dst[o + 3] = 255;
      }
    }
  } else if (enc === 'rgb8') {
    for (let i = 0, p = 0; i < bytes.length; i += 3, p += 4) {
      dst[p] = bytes[i]; dst[p + 1] = bytes[i + 1]; dst[p + 2] = bytes[i + 2]; dst[p + 3] = 255;
    }
  } else if (enc === 'bgr8') {
    for (let i = 0, p = 0; i < bytes.length; i += 3, p += 4) {
      dst[p] = bytes[i + 2]; dst[p + 1] = bytes[i + 1]; dst[p + 2] = bytes[i]; dst[p + 3] = 255;
    }
  } else if (enc === 'rgba8') {
    dst.set(bytes);
  } else if (enc === 'bgra8') {
    for (let i = 0, p = 0; i < bytes.length; i += 4, p += 4) {
      dst[p] = bytes[i + 2]; dst[p + 1] = bytes[i + 1]; dst[p + 2] = bytes[i]; dst[p + 3] = bytes[i + 3];
    }
  } else {
    supported = false;
  }
  if (!supported) {
    showOverlay(`不支持的图像编码:${enc}(已支持 mono8/rgb8/bgr8/rgba8/bgra8)`);
    return;
  }
  ctx.putImageData(rawImageData, 0, 0);
  lastByteLen = bytes.byteLength || bytes.length || 0;
  lastEncoding = enc;
  lastDim = `${W}×${H}`;
  hideOverlay();
}

function showOverlay(text) {
  if (!els.overlay) return;
  els.overlay.textContent = text;
  els.overlay.classList.add('is-visible');
}
function hideOverlay() {
  els.overlay?.classList.remove('is-visible');
}

function showLayerForChannel(channel) {
  // 彩色 img 仅在「当前通道是 color 且已有成功解码的帧」时显示,否则保持 hidden 不露破损图标
  if (els.colorImg) els.colorImg.hidden = !(channel === 'color' && colorReady);
  if (els.infra1Canvas) els.infra1Canvas.hidden = channel !== 'infra1';
  if (els.infra2Canvas) els.infra2Canvas.hidden = channel !== 'infra2';
}

/** 到达一帧画面 */
function onCameraFrame({ channel, topic, msg }) {
  if (channel !== currentChannel || !viewVisible) return;
  if (channel === 'color') {
    // 占位层在 <img> onload 成功后才撤掉;空数据帧不计数,避免统计假活
    if (!renderCompressed(channel, msg)) return;
  } else {
    renderRawImage(channel, msg);
  }
  if (topic && els.topic) {
    els.topic.textContent = topic;
    els.topic.title = topic;
  }
  frameCount++;
  windowFrames++;
  windowBytes += lastByteLen;
  const now = performance.now();
  if (!windowStartedAt) windowStartedAt = now;
  lastFrameAt = now;
  if (channel !== 'color') els.placeholder.style.display = 'none';
}

/** 切换画面通道(UI 层) */
function switchChannel(channel) {
  if (!['color', 'infra1', 'infra2'].includes(channel)) return;
  currentChannel = channel;
  colorReady = false; // 切通道后彩色需重新等首帧 onload,期间 img 保持隐藏
  els.channelBtns.forEach((b) => b.classList.toggle('is-active', b.dataset.camChannel === channel));
  frameCount = 0; windowFrames = 0; windowBytes = 0; windowStartedAt = 0;
  if (els.topic) { els.topic.textContent = '等待首帧…'; els.topic.title = ''; }
  lastDim = '-'; lastEncoding = '-'; lastByteLen = 0;
  showLayerForChannel(channel);
  els.placeholder.style.display = '';
  els.placeholder.innerHTML = '等待该通道首帧画面…';
  // 画面订阅只有在页签可见且在线时才真正建立
  if (viewVisible && rosOnline) setCameraChannel(channel);
}

/** 按「页签可见 ∧ ROS 在线」统一开关相机订阅 */
function applyDesired() {
  const want = viewVisible && rosOnline;
  setCameraChannel(want ? currentChannel : null);
  setCameraTelemetryEnabled(want);
  if (want) {
    els.streamState.textContent = '● 订阅中(best_effort)';
    els.streamState.classList.add('is-live');
    els.placeholder.innerHTML = '等待相机首帧…<br>若长时间无画面:确认相机话题存在且 rosbridge 支持 qos(best_effort)';
  } else {
    els.streamState.textContent = rosOnline ? '未订阅' : 'ROS 未连接';
    els.streamState.classList.remove('is-live');
    if (!rosOnline) els.placeholder.innerHTML = '连接 ROS 并停留在本页签后显示相机画面<br>彩色/红外话题使用 best_effort QoS';
  }
}

/** 定时刷新统计与遥测 DOM(8Hz),页签不可见时跳过 */
function flushDom() {
  if (!viewVisible) return;
  const now = performance.now();
  const dt = windowStartedAt ? (now - windowStartedAt) / 1000 : 0;
  if (dt >= 0.5) {
    const fps = windowFrames / dt;
    const kbps = (windowBytes / dt / 1024).toFixed(0);
    els.fps.textContent = `${fmt(fps, 1)} FPS`;
    els.fps.classList.toggle('is-stale', now - lastFrameAt > 1000);
    els.bytes.textContent = `${kbps} KB/s`;
    windowFrames = 0; windowBytes = 0; windowStartedAt = now;
  }
  els.resolution.textContent = lastDim;
  els.encoding.textContent = lastEncoding;
  els.frames.textContent = `帧 ${frameCount}`;
  // 2 秒没帧 → 提示中断(真机彩色流突发,帧间隔偶尔 >1s,阈值放宽防闪烁)
  if (frameCount > 0 && now - lastFrameAt > 2000) {
    els.placeholder.style.display = '';
    els.placeholder.textContent = '画面中断(2s 未收到帧)…';
  }

  if (lastImu) {
    const a = lastImu.linear_acceleration || {};
    const g = lastImu.angular_velocity || {};
    els.accX.textContent = fmt(a.x, 3); els.accY.textContent = fmt(a.y, 3); els.accZ.textContent = fmt(a.z, 3);
    els.gyrX.textContent = fmt(g.x, 3); els.gyrY.textContent = fmt(g.y, 3); els.gyrZ.textContent = fmt(g.z, 3);
  }
  if (lastVio) {
    // 兼容 nav_msgs/Odometry(pose.pose) 与 geometry_msgs/PoseStamped(pose)
    const pose = lastVio.pose?.pose || lastVio.pose;
    const pos = pose?.position;
    const q = pose?.orientation;
    if (pos) {
      els.vioX.textContent = fmt(pos.x, 3); els.vioY.textContent = fmt(pos.y, 3); els.vioZ.textContent = fmt(pos.z, 3);
    }
    if (q) {
      els.vioQ.textContent = `w=${fmt(q.w, 2)} x=${fmt(q.x, 2)} y=${fmt(q.y, 2)} z=${fmt(q.z, 2)}`;
    }
  }
  if (lastVioStatus) els.vioStatus.textContent = lastVioStatus;
}

/** 页签可见性(main.js setControlTab 调用) */
export function setCameraViewVisible(visible) {
  viewVisible = !!visible;
  applyDesired();
}

export function initCameraView() {
  els = {
    streamState: document.querySelector('#cam-stream-state'),
    channelBtns: Array.from(document.querySelectorAll('[data-cam-channel]')),
    colorImg: document.querySelector('#cam-color-img'),
    infra1Canvas: document.querySelector('#cam-infra1-canvas'),
    infra2Canvas: document.querySelector('#cam-infra2-canvas'),
    placeholder: document.querySelector('#cam-placeholder'),
    overlay: document.querySelector('#cam-overlay'),
    fps: document.querySelector('#cam-fps'),
    topic: document.querySelector('#cam-topic'),
    resolution: document.querySelector('#cam-resolution'),
    encoding: document.querySelector('#cam-encoding'),
    frames: document.querySelector('#cam-frames'),
    bytes: document.querySelector('#cam-bytes'),
    accX: document.querySelector('#cam-acc-x'), accY: document.querySelector('#cam-acc-y'), accZ: document.querySelector('#cam-acc-z'),
    gyrX: document.querySelector('#cam-gyr-x'), gyrY: document.querySelector('#cam-gyr-y'), gyrZ: document.querySelector('#cam-gyr-z'),
    vioX: document.querySelector('#cam-vio-x'), vioY: document.querySelector('#cam-vio-y'), vioZ: document.querySelector('#cam-vio-z'),
    vioQ: document.querySelector('#cam-vio-q'),
    vioStatus: document.querySelector('#cam-vio-status-text'),
  };
  if (!els.streamState) return; // DOM 不在当前页面(理论上两版 HTML 都有)

  els.channelBtns.forEach((btn) => {
    btn.addEventListener('click', () => switchChannel(btn.dataset.camChannel));
  });

  addCameraFrameListener(onCameraFrame);
  addCameraImuListener((msg) => { lastImu = msg; });
  addCameraVioListener((msg) => { lastVio = msg; });
  addCameraVioStatusListener((text) => { lastVioStatus = text; });
  addStatusListener((s) => {
    const online = s.connectionStatus === 'online';
    if (online !== rosOnline) {
      rosOnline = online;
      applyDesired();
    }
  });

  showLayerForChannel(currentChannel);
  applyDesired();
  if (flushTimer) clearInterval(flushTimer);
  flushTimer = setInterval(flushDom, 125);
}
