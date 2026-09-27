/**
 * OpenFlex 仿真主入口。
 *
 * 职责:
 *  - 绑定 UI 事件(面板切换、预设动作、视角按钮、键盘底盘控制等)
 *  - 维护预设动作配置(presets)
 *  - 运行渲染循环(animate):底盘步进、关节阻尼插值、注释刷新、场景渲染
 *  - 启动模型加载(loadModel)
 *  - 提供模型重新加载(switchModel)能力:清理旧状态 → 重新加载(回放内部使用)
 *
 * 不直接持有 Three.js 场景对象,所有场景/渲染器/DOM 引用均从各子模块导入。
 */
import * as THREE from 'three';
import { canvas, renderer, labelRenderer, scene, camera, orbit, defaultView, setView, status, controlsElement } from './scene.js?v=2';
import {
  jointStates, visibleControls, cartesianArms, shared,
  clearAllModelState, MODEL_REGISTRY, currentModelId,
} from './state.js?v=1';
import { stopActiveStepperMotion, setJointPose, syncStateInputs } from './joints.js?v=1';
import { loadModel, nextLoadGeneration, updateWheelSpokeSides } from './model.js?v=987';
import { setAnnotationsVisible, updateJointAnnotations, clearStaticAnnotations } from './annotations.js?v=949';
import { setComVisible, updateComViz } from './com-viz.js?v=3';
import { rebuildDashboard, updateDashboard } from './dashboard.js?v=970';
import { setChassisCommand, pushJoystickCommand, applyJoystickLift, getJoystickLift, scheduleArmSolve, syncArmTarget, enforceGroundContact, getQuadChassisController, getJoystickMaxLinear, setJoystickMaxLinear, getJoystickMaxAngular, setJoystickMaxAngular, JOY_SPEED_MAX, setupQuadrupedChassisControl } from './cartesian.js?v=1018';
import {
  addStatusListener, setPositionSource, toggleConnection, setRosUrl, getRosUrl,
  addMessageListener, addImuListener, addNotifyListener, publishNotifyAck,
  addCheckStandListener, addCommandFeedbackListener, publishCheckStand,
  addBriefingStatusListener, publishBriefing,
  lastImuMessageAt, imuMessageCount,
  publishBehaviorCommand, setCmdVelRequest, setCmdVelVz, stopCmdVel,
  MOTOR_STATE_TOPIC, IMU_STATE_TOPIC, getStatusSnapshot,
  injectMotorFrame, injectImuFrame, motorTelemetry, JOINT_MAPPING,
} from './ros-bridge.js?v=982';
import {
  openRecorderDB, startRecording, stopRecording, recordFrame, isRecording,
  listRecordings, renameRecording, deleteRecording, getRecording,
} from './recorder-storage.js?v=967';
import {
  startReplay, stopReplay, addReplayListener, isReplaying, getReplayState,
  seekTo as replaySeekTo, seekAndInject, hasFrames, clearFrames, getCurrentSeekMs,
  loadFramesForPreview,
} from './recorder-replay.js?v=969';
import {
  recordSample, renderCurves, rebuildCurveList, clearAllBuffers,
  setPaused as setCurvesPaused,
  setDisplayMode as setCurvesDisplayMode, getDisplayMode as getCurvesDisplayMode,
  recordImuSample, renderImuCurves, rebuildImuCurveList, clearAllImuBuffers,
  setImuDisplayMode, getImuDisplayMode,
  recordJoystickSample, renderJoystickCurves, rebuildJoystickCurveList, clearAllJoyBuffers,
  getCurvesHistoryOffset, setCurvesHistoryOffset, resetCurvesHistory, getCurvesMaxHistorySec,
} from './curves.js?v=982';
import { initRosDebug, onRosDebugTabShown } from './ros-debug.js?v=976';
import { initAlarm, evaluateAlarms, clearAlarmMarkers, rebuildAlarmEditor, purgeAlarmRecordMeta } from './alarm.js?v=972';
import { initCameraView, setCameraViewVisible } from './camera-view.js?v=5';
import { initLidarView, setLidarViewVisible } from './lidar-view.js?v=4';

/** 当前激活的控制面板标签:桌面默认「话题调试」;移动端精简为
 *  移动控制/ROS 指令/数据看板/仿真四个页签,默认「移动控制」。刷新后恢复上次选择 */
const CONTROL_TAB_STORAGE_KEY = 'web_sim_control_tab_v1';
const VALID_CONTROL_TABS = ['rosdebug', 'alarm', 'config', 'movement', 'joint', 'cartesian', 'curves', 'dashboard', 'camera', 'lidar', 'zero'];
const MOBILE_CONTROL_TABS = ['movement', 'ros', 'zero', 'dashboard', 'sim'];
function loadPersistedControlTab() {
  try {
    const saved = localStorage.getItem(CONTROL_TAB_STORAGE_KEY);
    if (window.__WEB_MOBILE__) {
      // 移动端:仅接受四个精简页签;桌面端遗留的其它页签(如 rosdebug)回退到移动控制
      return MOBILE_CONTROL_TABS.includes(saved) ? saved : 'movement';
    }
    // 桌面端不识别 sim(移动端独占),脏数据回退默认
    return VALID_CONTROL_TABS.includes(saved) ? saved : 'rosdebug';
  } catch {
    return window.__WEB_MOBILE__ ? 'movement' : 'rosdebug'; // 隐私模式等 localStorage 不可用时回退默认
  }
}
let activeControlTab = loadPersistedControlTab();

// ════════════════════════════════════════════════════════
// 3D 仿真画布开关(默认关闭)
// 关闭时:不加载模型网格(~40MB OBJ)、不创建渲染帧;ROS 连接、数据看板、
// 曲线、录制、真机遥控(摇杆/键盘 → cmd_vel)均独立可用。
// 再次开启时模型仍在内存(关闭只暂停渲染),秒恢复;选择持久化到 localStorage。
// ════════════════════════════════════════════════════════
const SIM_ENABLED_KEY = 'web_sim_enabled_v1';
/** 用户开关(持久化,默认 false) */
let simEnabled = false;
/** 模型已加载完成 → 场景可渲染 */
let simReady = false;
/** 模型加载中(防重复点击) */
let simLoading = false;

function loadSimEnabledPref() {
  try { return localStorage.getItem(SIM_ENABLED_KEY) === '1'; } catch { return false; }
}

/** 同步 body.sim-off / 开关按钮 / 状态栏文案 */
function applySimUi() {
  document.body.classList.toggle('sim-off', !simEnabled);
  const btn = document.querySelector('#sim-toggle-btn');
  if (btn) {
    btn.classList.toggle('sim-on', simEnabled);
    btn.textContent = simEnabled ? '🧊 仿真:开' : '🧊 仿真:关';
    btn.title = simEnabled
      ? '关闭 3D 仿真画布(停止渲染,模型保留在内存)'
      : '启动 3D 仿真画布(加载机器人模型网格,约 40MB)';
  }
  if (!simEnabled) status.textContent = '3D 仿真已关闭(模型未加载)';
}

function persistSimPref() {
  try { localStorage.setItem(SIM_ENABLED_KEY, simEnabled ? '1' : '0'); } catch { /* 忽略 */ }
}

/** 启动仿真:首次加载模型;已加载过则仅恢复渲染(禁止重复 loadModel,
 *  否则旧 modelRoot 仍挂在 scene 上,会出现多个模型重叠) */
async function enableSim() {
  if (simEnabled || simLoading) return;
  simEnabled = true;
  simLoading = true;
  applySimUi();
  persistSimPref();
  const overlay = document.querySelector('#loading-overlay');
  try {
    if (simReady) {
      // 模型已在内存:仅恢复画布,不重复加载
      resize();                 // canvas 从 display:none 恢复后需重新计算尺寸
      setControlTab(activeControlTab);
    } else {
      overlay?.classList.remove('is-hidden');
      await loadModel();
      simReady = true;
      resize();                 // canvas 从 display:none 恢复后需重新计算尺寸
      setControlTab(activeControlTab);
      // 若 ROS 已在线,关节滑块应切为禁用(真机接管)
      import('./ros-bridge.js?v=982').then((m) => refreshSliderDisabledState(!!m.rosControlActive)).catch(() => {});
    }
  } catch (error) {
    console.error(error);
    simEnabled = false;       // 回滚开关
    applySimUi();
    persistSimPref();
    status.textContent = `模型加载失败：${error.message}`;
  } finally {
    simLoading = false;
    overlay?.classList.add('is-hidden');
  }
}

/** 关闭仿真:停止渲染并隐藏画布;不 dispose 模型,再次开启秒恢复 */
function disableSim() {
  if (!simEnabled) return;
  simEnabled = false;
  applySimUi();
  persistSimPref();
}

/** 标记切换模型进行中(避免重复触发) */
let switchingModel = false;

// ════════════════════════════════════════════════════════
// 轻量 UI 反馈:toast 提示 + 自定义确认框
// (替代浏览器原生 alert/confirm:原生弹窗阻塞主线程,移动端样式突兀)
// ════════════════════════════════════════════════════════
let _uiFeedbackInited = false;
function _ensureUiFeedbackDom() {
  if (_uiFeedbackInited) return;
  _uiFeedbackInited = true;
  const style = document.createElement('style');
  style.id = 'ui-feedback-style';
  style.textContent = `
.ui-toast-wrap{position:fixed;top:14px;left:50%;transform:translateX(-50%);z-index:10000;display:flex;flex-direction:column;gap:8px;align-items:center;pointer-events:none;max-width:min(86vw,420px)}
.ui-toast{pointer-events:auto;background:rgba(16,25,42,.96);color:#e8eef7;border:1px solid var(--c-accent,#4dd0c1);border-radius:8px;padding:9px 14px;font-size:13px;line-height:1.45;box-shadow:0 6px 20px rgba(0,0,0,.45);opacity:0;transform:translateY(-8px);transition:opacity .18s,transform .18s;text-align:center;word-break:break-word}
.ui-toast-warn{border-color:#e07070}
.ui-toast-show{opacity:1;transform:translateY(0)}
.ui-modal-mask{position:fixed;inset:0;z-index:10001;background:rgba(4,8,14,.62);display:flex;align-items:center;justify-content:center;padding:20px;opacity:0;transition:opacity .15s}
.ui-modal-mask.ui-modal-show{opacity:1}
.ui-modal{width:min(88vw,360px);background:#10192a;border:1px solid var(--c-border,#24314a);border-radius:12px;padding:16px 16px 14px;box-shadow:0 12px 40px rgba(0,0,0,.6);transform:scale(.96);transition:transform .15s}
.ui-modal-mask.ui-modal-show .ui-modal{transform:scale(1)}
.ui-modal-title{font-size:15px;font-weight:700;color:#eef3fa;margin:0 0 8px}
.ui-modal-msg{font-size:13px;line-height:1.55;color:var(--c-muted,#9fb0c8);margin:0 0 16px;white-space:pre-wrap;word-break:break-word}
.ui-modal-actions{display:flex;gap:10px;justify-content:flex-end}
.ui-modal-actions button{min-height:34px;padding:0 16px;border-radius:7px;font-size:13px;cursor:pointer;touch-action:manipulation}
.ui-modal-cancel{background:transparent;border:1px solid var(--c-input-border,#2c3a55);color:var(--c-label,#c6d2e4)}
.ui-modal-ok{background:var(--c-accent,#4dd0c1);border:1px solid var(--c-accent,#4dd0c1);color:#0d2326;font-weight:700}
.ui-modal-danger{background:#e07070;border-color:#e07070;color:#fff}`;
  document.head.appendChild(style);
  const wrap = document.createElement('div');
  wrap.id = 'ui-toast-wrap';
  wrap.className = 'ui-toast-wrap';
  document.body.appendChild(wrap);
}

/** 顶部 toast 轻提示(不阻塞)。type: 'info' | 'warn'(红色描边) */
function showToast(message, type = 'info') {
  _ensureUiFeedbackDom();
  const el = document.createElement('div');
  el.className = `ui-toast${type === 'warn' ? ' ui-toast-warn' : ''}`;
  el.textContent = message; // textContent 防注入
  document.getElementById('ui-toast-wrap').appendChild(el);
  requestAnimationFrame(() => el.classList.add('ui-toast-show'));
  window.setTimeout(() => {
    el.classList.remove('ui-toast-show');
    window.setTimeout(() => el.remove(), 200);
  }, 2400);
}

/**
 * 自定义确认对话框,返回 Promise<boolean>(true=确认)。
 * 支持 Esc/点遮罩取消、Enter 确认。
 * @param {{title?:string, message?:string, confirmText?:string, cancelText?:string, danger?:boolean}} opts
 */
function confirmDialog({ title = '请确认', message = '', confirmText = '确定', cancelText = '取消', danger = false } = {}) {
  _ensureUiFeedbackDom();
  return new Promise((resolve) => {
    const mask = document.createElement('div');
    mask.className = 'ui-modal-mask';
    mask.innerHTML = `
      <div class="ui-modal" role="alertdialog" aria-modal="true">
        <h3 class="ui-modal-title"></h3>
        <p class="ui-modal-msg"></p>
        <div class="ui-modal-actions">
          <button type="button" class="ui-modal-cancel"></button>
          <button type="button" class="ui-modal-ok"></button>
        </div>
      </div>`;
    mask.querySelector('.ui-modal-title').textContent = title;
    mask.querySelector('.ui-modal-msg').textContent = message;
    mask.querySelector('.ui-modal-cancel').textContent = cancelText;
    const okBtn = mask.querySelector('.ui-modal-ok');
    okBtn.textContent = confirmText;
    okBtn.classList.toggle('ui-modal-danger', danger);
    document.body.appendChild(mask);
    let settled = false;
    const close = (ok) => {
      if (settled) return;
      settled = true;
      document.removeEventListener('keydown', onKey, true);
      mask.classList.remove('ui-modal-show');
      window.setTimeout(() => mask.remove(), 150);
      resolve(ok);
    };
    const onKey = (e) => {
      if (e.key === 'Escape') close(false);
      else if (e.key === 'Enter') close(true);
    };
    mask.addEventListener('click', (e) => { if (e.target === mask) close(false); });
    mask.querySelector('.ui-modal-cancel').addEventListener('click', () => close(false));
    okBtn.addEventListener('click', () => close(true));
    document.addEventListener('keydown', onKey, true);
    requestAnimationFrame(() => {
      mask.classList.add('ui-modal-show');
      okBtn.focus();
    });
  });
}

/** 上次报警评估时间戳(动画循环节流) */
let _lastAlarmEvalAt = 0;

// ════════════════════════════════════════════════════════
// 机器人通知事件 (/rl_real/notify):状态 + 渲染函数
// ════════════════════════════════════════════════════════
const _notifyHistory = [];
const NOTIFY_HISTORY_MAX = 100;
let _notifyUnreadCount = 0;

function _notifyTypeIcon(type) {
  switch (type) {
    case 'error':   return '\u2716';
    case 'warning': return '\u26a0';
    case 'note':    return '\u2605';
    default:        return '\u2139';
  }
}
function _notifyTypeLabel(type) {
  switch (type) {
    case 'error':   return '错误';
    case 'warning': return '警告';
    case 'note':    return '备注';
    default:        return '信息';
  }
}
function _formatNotifyTime(tsSec) {
  const d = new Date(tsSec * 1000);
  const hh = String(d.getHours()).padStart(2, '0');
  const mm = String(d.getMinutes()).padStart(2, '0');
  const ss = String(d.getSeconds()).padStart(2, '0');
  return `${hh}:${mm}:${ss}`;
}
function _renderNotifyPanel() {
  const list = document.getElementById('notify-history-list');
  if (!list) return;
  if (_notifyHistory.length === 0) {
    list.innerHTML = '<div class="notify-empty">暂无通知</div>';
    return;
  }
  const items = _notifyHistory.slice().reverse().map((n) => {
    const icon = _notifyTypeIcon(n.type);
    const label = _notifyTypeLabel(n.type);
    const time = _formatNotifyTime(n.timestamp);
    return `<div class="notify-item notify-${n.type}" data-id="${n.id}">
      <span class="notify-icon">${icon}</span>
      <div class="notify-body">
        <div class="notify-meta"><span class="notify-tag">${label}</span><span class="notify-time">${time}</span><span class="notify-id">#${n.id}</span></div>
        <div class="notify-content"></div>
      </div>
    </div>`;
  }).join('');
  list.innerHTML = items;
  list.querySelectorAll('.notify-item').forEach((el) => {
    const id = el.getAttribute('data-id');
    const n = _notifyHistory.find((x) => String(x.id) === id);
    if (n) el.querySelector('.notify-content').textContent = n.content;
  });
}
function _updateNotifyBadge() {
  const badge = document.getElementById('notify-badge');
  if (!badge) return;
  if (_notifyUnreadCount > 0) {
    badge.textContent = _notifyUnreadCount > 99 ? '99+' : String(_notifyUnreadCount);
    badge.classList.add('notify-badge-show');
  } else {
    badge.classList.remove('notify-badge-show');
  }
}

// 通知监听器:必须在模块顶层注册,不能放在 async initRecorderAndReplay() 内部,
// 否则 IndexedDB 初始化耗时期间通知可能已到达但 listener 尚未注册。
addNotifyListener((payload) => {
  _notifyHistory.push({ ...payload });
  if (_notifyHistory.length > NOTIFY_HISTORY_MAX) _notifyHistory.shift();
  _notifyUnreadCount++;
  _updateNotifyBadge();
  _renderNotifyPanel();
  const toastType = (payload.type === 'error' || payload.type === 'warning') ? 'warn' : 'info';
  const prefix = _notifyTypeIcon(payload.type);
  showToast(`${prefix} ${payload.content}`, toastType);
  if (payload.type === 'error') console.error(`[notify #${payload.id}] ${payload.content}`);
});

// ════════════════════════════════════════════════════════
// 录制 & 复现:全局状态(必须放在 addStatusListener 之前,避免 TDZ 报错)
// ════════════════════════════════════════════════════════
/** 当前正在录制的 recordingId(null = 未在录) */
let _activeRecordingId = null;
/** 当前录制开始时刻(Date.now() 基,用于按钮秒数本地计算) */
let _activeRecordingStartedAt = 0;
/** 节流:录制状态栏每秒刷一次 */
let _lastRecStatusRefreshAt = 0;
/** 1s 刷新一次录制状态 / 列表(新增后重排)/ 复现进度 */
const REC_STATUS_REFRESH_MS = 1000;
/** 录制中定时器:每 REC_STATUS_REFRESH_MS 触发 applyRecorderUiState,
 *  让顶部状态栏秒/帧统计实时更新(否则只会在按钮点击时刷新一次,统计永远停在快照) */
let _recStatusTimer = 0;
function startRecStatusTimer() {
  if (_recStatusTimer) return;
  _recStatusTimer = window.setInterval(() => {
    if (_activeRecordingId == null) { stopRecStatusTimer(); return; }
    applyRecorderUiState();
  }, REC_STATUS_REFRESH_MS);
}
function stopRecStatusTimer() {
  if (_recStatusTimer) { clearInterval(_recStatusTimer); _recStatusTimer = 0; }
}

/** 把「正在录制 / 正在复现」附加到主 ros-status 胶囊尾部(返回 appended 字符串) */
function rosStatusAppendExtra() {
  const parts = [];
  if (_activeRecordingId != null) {
    parts.push('● REC');
  }
  if (typeof isReplaying === 'function' && isReplaying()) {
    const rs = getReplayState();
    const pct = rs.totalDurationMs > 0 ? Math.round(100 * rs.elapsedMs / rs.totalDurationMs) : 0;
    parts.push(`▶ REPLAY ${pct}%`);
  }
  return parts.length ? ' · ' + parts.join(' · ') : '';
}

// ── 全局步进器中断事件 ───────────────────────────────────
// 任何可能让用户离开当前交互的场景(松开指针、切到后台、页面失焦),
// 都应中断正在长按重复触发的步进器,避免关节持续运动。
window.addEventListener('pointerup', stopActiveStepperMotion);
window.addEventListener('pointercancel', stopActiveStepperMotion);
window.addEventListener('blur', stopActiveStepperMotion);
document.addEventListener('visibilitychange', () => {
  if (document.hidden) stopActiveStepperMotion();
});

// ── 模型重新加载(报警记录回放内部使用) ───────────────────

/**
 * 重新加载机器人模型:清理旧模型(场景、状态、控制面板)后调用 loadModel。
 * 当前注册表里只有唯一模型(四足),无用户可选模型;本函数仅供回放路径
 * 在记录的 modelId 与当前不一致时兜底使用。
 *
 * @param {string} modelId - 目标模型目录名
 */
export async function switchModel(modelId) {
  if (switchingModel) return;
  switchingModel = true;
  const simWasEnabled = simEnabled;   // 加载失败时回滚到原开关态
  try {
    const entry = MODEL_REGISTRY.find((m) => m.id === modelId);
    if (!entry) throw new Error(`未知模型: ${modelId}`);

    stopActiveStepperMotion();
    status.textContent = `正在重新加载 ${entry.label}...`;

    // 仿真处于关闭态时(如报警记录回放触发切模型):仅翻开开关,
    // 模型由本函数随后的 loadModel 完成首次加载,避免重复拉取。
    if (!simEnabled) {
      simEnabled = true;
      applySimUi();
      persistSimPref();
    }

    // 作废旧模型的异步加载(防止旧模型加载完成后覆盖新模型 UI)
    nextLoadGeneration();

    // 2. 清理旧模型:从场景移除 / 清空状态 / 清空控制面板 DOM
    clearAllModelState(scene, controlsElement);
    clearStaticAnnotations();
    clearAlarmMarkers();
    // 重置四足机器人底盘控制器
    getQuadChassisController().enabled = false;
    getQuadChassisController().pose = { x: 0, y: 0, yaw: 0 };
    getQuadChassisController().command = { vx: 0, vy: 0, omega: 0 };

    // 3. 加载新模型
    await loadModel(modelId);
    simReady = true;
    resize();
    // 模型切换后:清空曲线 + IMU + 摇杆缓冲,若曲线 tab 当前可见则重建 DOM 并重绘
    clearAllBuffers();
    clearAllImuBuffers();
    clearAllJoyBuffers();
    if (activeControlTab === 'curves') {
      const list = document.querySelector('#curves-list');
      if (list) rebuildCurveList(list);
      const imuList = document.getElementById('imu-curves-list');
      if (imuList) rebuildImuCurveList(imuList);
      const joyList = document.getElementById('joystick-curves-list');
      if (joyList) rebuildJoystickCurveList(joyList);
      tryRenderCurves(true);
      tryRenderImuCurves(true);
      tryRenderJoystickCurves(true);
    }
    // 切模型后控制面板重新渲染,按 ROS 状态重新刷新滑块禁用态
    try {
      const stateModule = await import('./ros-bridge.js?v=982');
      refreshSliderDisabledState(!!stateModule.rosControlActive);
    } catch { /* 忽略 */ }
    // 重新激活当前控制标签,确保底盘控制器被正确启用
    setControlTab(activeControlTab);
  } catch (error) {
    console.error(error);
    if (!simWasEnabled) { simEnabled = false; applySimUi(); persistSimPref(); }
    status.textContent = `模型加载失败：${error.message}`;
  } finally {
    switchingModel = false;
  }
}

// ── 面板切换 ─────────────────────────────────────────────

/**
 * 切换控制面板标签(移动/关节/笛卡尔)。
 */
function setControlTab(tab) {
  activeControlTab = tab;
  // 持久化页签选择:刷新/重开后恢复(隐私模式等写入失败时静默忽略)
  try { localStorage.setItem(CONTROL_TAB_STORAGE_KEY, tab); } catch { /* 忽略 */ }
  document.querySelectorAll('[data-control-tab]').forEach((button) => {
    const active = button.dataset.controlTab === tab;
    button.classList.toggle('active', active);
    button.setAttribute('aria-selected', String(active));
  });
  document.querySelector('#config-control-view').hidden   = tab !== 'config';
  document.querySelector('#joint-control-view').hidden    = tab !== 'joint';
  document.querySelector('#movement-control-view').hidden = tab !== 'movement';
  document.querySelector('#cartesian-control-view').hidden= tab !== 'cartesian';
  document.querySelector('#curves-control-view').hidden   = tab !== 'curves';
  document.querySelector('#dashboard-control-view').hidden = tab !== 'dashboard';
  document.querySelector('#camera-control-view').hidden = tab !== 'camera';
  document.querySelector('#lidar-control-view').hidden = tab !== 'lidar';
  document.querySelector('#rosdebug-control-view').hidden = tab !== 'rosdebug';
  document.querySelector('#alarm-control-view').hidden    = tab !== 'alarm';
  document.querySelector('#zero-control-view').hidden     = tab !== 'zero';
  // 移动端独占的仿真页签容器(桌面 HTML 无此节点,可选链跳过)
  const simViewEl = document.querySelector('#sim-control-view');
  if (simViewEl) simViewEl.hidden = tab !== 'sim';
  // 移动端独占的 ROS 指令页签容器
  const rosViewEl = document.querySelector('#ros-control-view');
  if (rosViewEl) rosViewEl.hidden = tab !== 'ros';

  // 切换页签后把面板滚动位置归零:新页签内容可能比上一个短,
  // 保留旧 scrollTop 会让首屏一片空白,看起来像"没加载"
  const panelEl = document.querySelector('#control-panel');
  if (panelEl) panelEl.scrollTop = 0;

  // 以下为各页签的内容重建:逐个 try/catch,
  // 任一重建异常都不得中断切换流程(否则底盘 enable 等后续状态会被跳过)
  try {
    // 切到话题调试页签时刷新话题列表(在线时)
    if (tab === 'rosdebug') onRosDebugTabShown();

    // 切到报警配置页签时重建规则编辑器(适配当前模型关节列表)
    if (tab === 'alarm') rebuildAlarmEditor();

    // 切到数据看板页签时重建行/芯片(适配当前模型的关节列表)并立即刷一帧
    if (tab === 'dashboard') rebuildDashboard();

    // 可视化面板显示时重建 DOM 行(适配模型切换后的关节数 + IMU 15 条量固定 + 摇杆 5 条固定),并立即刷一帧
    if (tab === 'curves') {
      const list = document.querySelector('#curves-list');
      if (list) rebuildCurveList(list);
      const imuList = document.getElementById('imu-curves-list');
      if (imuList) rebuildImuCurveList(imuList);
      const joyList = document.getElementById('joystick-curves-list');
      if (joyList) rebuildJoystickCurveList(joyList);
      // 当前数值模式与曲线模式默认同页显示(不再切换互斥类);保留接口占位以兼容后续扩展。
      tryRenderCurves(true);
      tryRenderImuCurves(true);
      tryRenderJoystickCurves(true);
    }
  } catch (err) {
    console.warn('[control-tab] 页签内容重建异常:', err);
  }

  // 相机页签:可见时才订阅画面/遥测(best_effort,占带宽),切走立即退订
  try { setCameraViewVisible(tab === 'camera'); } catch { /* 忽略 */ }
  // 雷达页签同理:可见时订阅点云(节流10Hz)+内置 IMU(20Hz)
  try { setLidarViewVisible(tab === 'lidar'); } catch { /* 忽略 */ }

  // 底盘控制器仅在移动控制模式启用
  const isMovement = tab === 'movement';
  if (jointStates.has('FL_hip_joint')) getQuadChassisController().enabled = isMovement;
  const isCartesian = tab === 'cartesian';
  const isQuadruped = jointStates.has('FL_hip_joint');
  cartesianArms.forEach((controller, arm) => {
    controller.marker.visible = isCartesian;
    if (isCartesian) syncArmTarget(arm);
  });
  if (isCartesian && isQuadruped) {
    const activeLeg = document.querySelector('[data-leg-tab].active');
    if (activeLeg) {
      const legId = activeLeg.dataset.legTab;
      const controller = cartesianArms.get(legId);
      if (controller) controller.marker.visible = true;
    }
  }

  // 移动端独占「仿真」页签:与页签联动(顶部开关已隐藏):
  //  - 进入:模型已加载则秒恢复渲染;未加载不自动下载(~40MB),由占位卡片按钮启动
  //  - 离开:停止渲染但保留模型内存(下次进入秒恢复)
  // 模型加载中切走不打断:加载完后 simTabActive 门控保证不在后台渲染
  if (window.__WEB_MOBILE__) {
    if (tab === 'sim') {
      if (simReady && !simEnabled && !simLoading) {
        enableSim().catch(() => { /* 失败已在 enableSim 内回滚 + 提示 */ });
      }
      if (simEnabled && simReady) {
        try { resize(); } catch (err) { console.warn('[sim-tab] resize 失败:', err); }
      }
    } else if (simEnabled && !simLoading) {
      disableSim();
    }
  }
}

// ── ROS 连接 UI ──────────────────────────────────────────

const rosConnectBtn = document.querySelector('#ros-connect-btn');
const rosStatusEl = document.querySelector('#ros-status');
const rosPosSourceEl = document.querySelector('#ros-pos-source');
const rosUrlInputEl  = document.querySelector('#ros-url-input');

// 初始化:恢复上次连接地址(localStorage 持久化,优先级高于页面默认值),并自动重连。
(() => {
  if (!rosUrlInputEl) return;
  // ── ROS URL 历史管理(localStorage, max 10) ──
  const URL_HIST_KEY = 'web_sim_ros_url_history_v1';
  const URL_HIST_MAX = 10;
  const urlPanelEl   = document.getElementById('ros-url-dd-panel');
  /** 从 localStorage 加载历史数组 */
  const loadUrlHist = () => {
    try { return JSON.parse(localStorage.getItem(URL_HIST_KEY)) || []; } catch { return []; }
  };
  /** 持久化历史 */
  const saveUrlHist = (list) => {
    try { localStorage.setItem(URL_HIST_KEY, JSON.stringify(list)); } catch { /* 忽略 */ }
  };
  /** 把 URL push 到历史(去重,新地址放最前,截断到 max) */
  const pushUrlHist = (url) => {
    if (!url) return;
    const list = loadUrlHist().filter((h) => h && h !== url);
    list.unshift(url);
    saveUrlHist(list.slice(0, URL_HIST_MAX));
  };
  /** 渲染下拉面板(按输入过滤,历史去重在前) */
  const showUrlHistDd = (filter = '') => {
    if (!urlPanelEl || rosUrlInputEl.disabled) return;
    const q = filter.trim().toLowerCase();
    const list = loadUrlHist().filter((h) => h && (!q || h.toLowerCase().includes(q)));
    urlPanelEl.innerHTML = '';
    if (!list.length) {
      const empty = document.createElement('div');
      empty.className = 'ros-dd-empty';
      empty.textContent = q ? '无匹配历史地址' : '连接成功后自动记录';
      urlPanelEl.appendChild(empty);
      urlPanelEl.hidden = false;
      return;
    }
    const frag = document.createDocumentFragment();
    const head = document.createElement('div');
    head.className = 'ros-dd-group';
    head.textContent = '连接历史';
    frag.appendChild(head);
    for (const url of list) {
      const item = document.createElement('div');
      item.className = 'ros-dd-item';
      item.dataset.url = url;
      const n = document.createElement('span');
      n.className = 'dd-name';
      n.textContent = url;
      const d = document.createElement('span');
      d.className = 'dd-del';
      d.title = '从历史中删除';
      d.textContent = '×';
      item.append(n, d);
      frag.appendChild(item);
    }
    urlPanelEl.appendChild(frag);
    urlPanelEl.hidden = false;
  };
  const hideUrlHistDd = () => { if (urlPanelEl) { urlPanelEl.hidden = true; urlPanelEl.innerHTML = ''; } };

  // ── 现有初始化逻辑 ──
  const saved = localStorage.getItem('ros_url_last');
  const v = rosUrlInputEl.value?.trim();
  // 优先用保存的地址;其次输入框现值;最后库内默认值
  const url = saved || v || getRosUrl();
  rosUrlInputEl.value = url;
  setRosUrl(url);
  // 把当前生效地址也塞进历史(如果还不在的话)
  if (url) pushUrlHist(url);

  // 每次修改输入框:即时同步到 ros-bridge 运行时 + 持久化
  rosUrlInputEl.addEventListener('input', () => {
    const u = rosUrlInputEl.value.trim();
    if (u) {
      setRosUrl(u);
      localStorage.setItem('ros_url_last', u);
    }
    showUrlHistDd(rosUrlInputEl.value);
  });
  // focus 展开、失焦关闭(延迟,给 mousedown 留机会)、Esc 关闭
  rosUrlInputEl.addEventListener('focus', () => showUrlHistDd(rosUrlInputEl.value));
  rosUrlInputEl.addEventListener('blur', () => setTimeout(hideUrlHistDd, 120));
  rosUrlInputEl.addEventListener('keydown', (e) => { if (e.key === 'Escape') hideUrlHistDd(); });

  // 下拉项点击:mousedown 防止 blur 先触发,填充地址后关闭;× 按钮从历史中删除
  urlPanelEl?.addEventListener('mousedown', (e) => {
    const item = e.target.closest('.ros-dd-item');
    if (!item) return;
    e.preventDefault(); // 阻止 input blur
    const clickedDel = e.target.classList.contains('dd-del');
    const clickedUrl = item.dataset.url;
    if (clickedDel) {
      // 从历史中删除
      const list = loadUrlHist().filter((h) => h !== clickedUrl);
      saveUrlHist(list);
      showUrlHistDd(rosUrlInputEl.value); // 重绘
    } else {
      // 选中填充
      rosUrlInputEl.value = clickedUrl;
      setRosUrl(clickedUrl);
      localStorage.setItem('ros_url_last', clickedUrl);
      pushUrlHist(clickedUrl);
      hideUrlHistDd();
      rosUrlInputEl.focus();
    }
  });

  // 先读「刷新前是否在线」(注册监听器会同步触发一次回调,覆盖为当前状态)
  const wasConnectedBeforeReload = localStorage.getItem('ros_connected_last') === '1';
  // 连接成功时:持久化地址、push 历史、持久化「刷新前是否在线」
  addStatusListener((state) => {
    if (state.connectionStatus === 'online') {
      const u = getRosUrl();
      try { localStorage.setItem('ros_url_last', u); } catch (_) { /* 忽略 */ }
      pushUrlHist(u);
      rosUrlInputEl.value = u;
    }
    try { localStorage.setItem('ros_connected_last', state.connectionStatus === 'online' ? '1' : '0'); } catch (_) { /* 忽略 */ }
  });
  // 仅当刷新前处于连接状态时才自动重连(地址持久化不等于要自动连接)
  if (saved && wasConnectedBeforeReload) {
    setTimeout(() => {
      toggleConnection().catch((err) => console.warn('[main] 自动重连失败:', err));
    }, 600);
  }
})();

// 参数报警模块:绑定报警配置页控件并恢复持久化规则
initAlarm();

// 报警记录在触发/结束时由 alarm.js 派发事件 → 立即刷新「录制列表」(触发瞬间即出现,不等报警结束)
window.addEventListener('web-sim:alarm-recordings-changed', () => {
  renderRecordingsList();
});


/** 按 rosControlActive + connectionStatus 联动输入框/按钮的 disabled 状态 */
function setRosInputsDisabled(disabled) {
  if (rosUrlInputEl)  rosUrlInputEl.disabled = !!disabled;
  if (rosConnectBtn) {
    if (disabled) rosConnectBtn.setAttribute('aria-busy', 'true');
    else rosConnectBtn.removeAttribute('aria-busy');
  }
}

/** 按 rosControlActive 切换所有关节滑块的 disabled 状态 */
function refreshSliderDisabledState(rosActive) {
  const active = !!rosActive;
  // 同值守卫:状态 emit 高频重复调用,写相同值也会触发属性变更/失效;仅在变化时写入
  document.querySelectorAll('input[type="range"][data-joint]').forEach((input) => {
    if (input.disabled !== active) input.disabled = active;
  });
  // 联动开关与腿选择复选框:ROS 接管后一并禁用
  document.querySelectorAll('.gang-leg-cb, #gang-mode-toggle').forEach((el) => {
    if (el.disabled !== active) el.disabled = active;
  });
  // 预设按钮也一同禁用,避免本地动作覆盖 ROS 数据
  document.querySelectorAll('[data-preset]').forEach((btn) => {
    if (active) {
      if (!btn.hasAttribute('disabled')) {
        btn.setAttribute('disabled', 'true');
        btn.style.opacity = '0.45';
        btn.style.cursor = 'not-allowed';
      }
    } else if (btn.hasAttribute('disabled')) {
      btn.removeAttribute('disabled');
      btn.style.opacity = '';
      btn.style.cursor = '';
    }
  });
}

/** 更新"移动控制视图顶部"的 ROS 状态胶囊 + Twist 读数 */
function applyRosMovementUiState(state) {
  const {
    connectionStatus: s, rosControlActive,
    lastCommandPayload, commandPublishCount, lastCommandPublishAt,
    lastCmdVel, cmdVelPublishCount, lastCmdVelPublishAt,
  } = state;
  const pill = document.getElementById('ros-cmd-status');
  if (pill) {
    pill.classList.remove('ros-offline', 'ros-connecting', 'ros-online', 'ros-error');
    let text = 'ROS 离线';
    switch (s) {
      case 'connecting': text = 'ROS 连接中…'; pill.classList.add('ros-connecting'); break;
      case 'online': {
        const cmdInfo = (commandPublishCount > 0)
          ? `命令 ${commandPublishCount} 次 · 上次 ${lastCommandPayload || '-'} · ${((performance.now() - (lastCommandPublishAt || 0)) / 1000).toFixed(1)}s 前`
          : '命令未发布';
        text = `ROS 已连接 · ${cmdInfo}`;
        pill.classList.add('ros-online');
        break;
      }
      case 'error': text = 'ROS 错误'; pill.classList.add('ros-error'); break;
      default: text = 'ROS 离线'; pill.classList.add('ros-offline'); break;
    }
    pill.textContent = text;
    pill.title = text;
  }
  const online = s === 'online';
  document.querySelectorAll('.behavior-pill, .scene-chip').forEach((btn) => {
    // 纯 UI 按钮(不发布 ROS 命令,如「全选标红电机」)不受离线禁用影响
    if (btn.hasAttribute('data-ui-only')) return;
    if (online) {
      btn.removeAttribute('disabled');
      btn.classList.remove('is-disabled');
    } else {
      btn.setAttribute('disabled', 'true');
      btn.classList.add('is-disabled');
    }
  });
  // 底盘方向控制按钮:ROS 在线时不应该"禁用",因为它们还应该 publish cmd_vel。
  // 但为了避免本地仿真 + cmd_vel 同时驱动,本地底盘控制器 setEnabled 会在 applyRosUiState 里被关掉。
  const twistLinX = document.getElementById('twist-lin-x');
  const twistLinY = document.getElementById('twist-lin-y');
  const twistLinZ = document.getElementById('twist-lin-z');
  const twistAngZ = document.getElementById('twist-ang-z');
  const twistCount = document.getElementById('twist-count');
  if (twistLinX) twistLinX.value = lastCmdVel ? lastCmdVel.linear.x.toFixed(2) : '0.00';
  if (twistLinY) twistLinY.value = lastCmdVel ? lastCmdVel.linear.y.toFixed(2) : '0.00';
  if (twistLinZ) twistLinZ.value = lastCmdVel ? lastCmdVel.linear.z.toFixed(2) : '0.00';
  if (twistAngZ) twistAngZ.value = lastCmdVel ? lastCmdVel.angular.z.toFixed(2) : '0.00';
  if (twistCount) {
    const age = (performance.now() - (lastCmdVelPublishAt || 0)) / 1000;
    twistCount.textContent = (cmdVelPublishCount > 0)
      ? `${cmdVelPublishCount} 帧 · ${age.toFixed(1)}s 前`
      : '0 帧';
  }
  // 提示:未启用 / 不使用 rosControlActive 时,本地底盘 step 继续工作(仿真驱动)
  void rosControlActive;
}

/**
 * 根据 ros-bridge 状态刷新顶栏 ROS 控件外观 + 移动控制视图胶囊。
 */
function applyRosUiState(state) {
  const {
    connectionStatus: s, rosControlActive, positionSource, lastError,
    messageCount, lastBlockCount, lastMessageAt,
    imuMessageCount, lastImuMessageAt,
  } = state;
  rosConnectBtn.classList.toggle('ros-active', s === 'online' || s === 'connecting');
  rosConnectBtn.textContent = (s === 'online' || s === 'connecting') ? '断开 ROS' : '连接 ROS';

  rosStatusEl.classList.remove('ros-offline', 'ros-connecting', 'ros-online', 'ros-error');
  let text = '未连接';
  let fullText = ''; // 窄屏时 title 保留完整文案(含 IP/数据年龄)
  // 把当前生效的 rosbridge URL 简化显示(去掉 ws://)
  const rosUrlNice = (() => {
    try {
      const u = getRosUrl();
      return u.replace(/^wss?:\/\//i, '');
    } catch (_) { return 'unknown'; }
  })();
  switch (s) {
    case 'connecting': text = `连接中 ${rosUrlNice}...`; rosStatusEl.classList.add('ros-connecting'); break;
    case 'online': {
      const motorInfo = (messageCount > 0)
        ? `电机 ${messageCount} 帧 · ${lastBlockCount} · ${((performance.now() - lastMessageAt) / 1000).toFixed(1)}s 前`
        : '电机 等待数据...';
      const imuInfo = (imuMessageCount > 0)
        ? `IMU ${imuMessageCount} 帧 · ${((performance.now() - (lastImuMessageAt || 0)) / 1000).toFixed(1)}s 前`
        : 'IMU 等待数据...';
      text = `已连接 ${rosUrlNice} · ${motorInfo} · ${imuInfo}`;
      fullText = text;
      // 窄屏(手机):顶栏空间有限,精简为单行 —— IP 在工具栏地址栏已显示,
      // 去掉 IP/年龄/重复"帧"字:「已连接 · 电机511/16 · IMU510」;完整文案留 title
      if (window.innerWidth <= 768) {
        const m = (messageCount > 0) ? `电机${messageCount}/${lastBlockCount}` : '电机等待';
        const i = (imuMessageCount > 0) ? `IMU${imuMessageCount}` : 'IMU等待';
        text = `已连接 · ${m} · ${i}`;
      }
      rosStatusEl.classList.add('ros-online');
      break;
    }
    case 'error':      text = `错误 ${rosUrlNice}:${lastError || '连接失败'}`; rosStatusEl.classList.add('ros-error'); break;
    default:           text = '未连接'; rosStatusEl.classList.add('ros-offline');
  }
  rosStatusEl.textContent = text + rosStatusAppendExtra();
  rosStatusEl.title = lastError || fullText || text;

  // 连接中/在线 → 禁用地址输入框;离线/错误 → 可改地址
  const locked = s === 'online' || s === 'connecting';

  // 复现时,强制锁死 ROS 地址输入框(即使现在 offline 也不能切)
  const replayLocked = isReplaying();
  setRosInputsDisabled(locked || replayLocked);

  // 如果正在复现,同时把 ROS 连接按钮也禁用(避免播放中再连接真 ROS 数据流冲突)
  if (rosConnectBtn) rosConnectBtn.disabled = replayLocked;
  else void (0);

  const available = s === 'online' || s === 'connecting';
  rosPosSourceEl.disabled = !available;
  if (available && rosPosSourceEl.value !== positionSource) {
    rosPosSourceEl.value = positionSource;
  }
  refreshSliderDisabledState(!!rosControlActive);
  applyRosMovementUiState(state);

  // ROS 在线 → 关闭本地的底盘仿真积分(不再让 quad chassis 在仿真内自己 step 产生运动),
  // 改成 ① 真机电机位置 /rl_real/motor_state 从外部驱动仿真模型,② 发布 cmd_vel 让真机执行。
  if (jointStates.has('FL_hip_joint')) {
    getQuadChassisController().enabled = (activeControlTab === 'movement' && !available);
  }
  // ROS 在线状态变化 → 同步录制按钮启用态(只有 ROS 在线时未录制态的按钮才可点)
  // 首次注册 addStatusListener 时 ros-bridge.js 会立即同步触发一次回调,
  // 此时 replaySeekEl 等 const(L795)尚未初始化(TDZ),用 try/catch 守卫跳过;
  // 后续状态变化时 const 已就绪,正常调用。
  try {
    if (typeof applyRecorderUiState === 'function') applyRecorderUiState();
  } catch (_) { /* TDZ / DOM 尚未就绪,跳过本次同步 */ }
}

rosConnectBtn?.addEventListener('click', () => {
  toggleConnection().catch((err) => console.warn('[main] ROS 连接失败:', err));
});
rosPosSourceEl?.addEventListener('change', () => {
  setPositionSource(rosPosSourceEl.value);
});
addStatusListener(applyRosUiState);

// ── 真机命令:行为按钮 + 底盘 cmd_vel 路由 ────────────────
/**
 * 底盘命令钩子:cartesian.js 里的四足 chassis 所有 vx/vy/omega 命令入口
 * 都会回调到这里。ROS 在线时把它转发为 /rl_real/cmd_vel 持续发布(20Hz keep-alive),离线则忽略。
 */
shared.chassisCommandHook = (vx, vy, omega) => {
  // 即便 ROS 未连接也允许 setCmdVelRequest(内部会 return false,副作用很小),
  // 这样 UI 按钮/键盘不需要额外判断,所有输入路径保持一致。
  // 第 4 参不传 → 保持当前垂直速度(vz 由独立滑块控制,摇杆/旋转不清掉它)。
  setCmdVelRequest(vx, vy, omega);
  // 同步 twist 读数 DOM(即时,因为 cmdVelPublishCount %20===1 emit 节流可能跟不上手拨滑块)
  const linX = document.getElementById('twist-lin-x'); if (linX) linX.value = Number(vx).toFixed(2);
  const linY = document.getElementById('twist-lin-y'); if (linY) linY.value = Number(vy).toFixed(2);
  const angZ = document.getElementById('twist-ang-z'); if (angZ) angZ.value = Number(omega).toFixed(2);
};

// ── 垂直速度(linear.z):vz 滑块已删除,仅在底盘复位时清零 ROS vz 通道 ──
/** 复位/停止时把 ROS vz 请求清零 */
function resetCmdVelVz() {
  const linZ = document.getElementById('twist-lin-z');
  if (linZ) linZ.value = '0.00';
  setCmdVelVz(0);
}
// 底盘复位按钮(cartesian.js 已有自己的复位逻辑,这里追加清 vz)
document.getElementById('base-reset')?.addEventListener('click', resetCmdVelVz);
document.getElementById('quad-base-reset')?.addEventListener('click', resetCmdVelVz);
/** 行为按钮点击 → 对应 String burst 发布 3 次 */
// 置零类事件命令: 单发、耗时约 2s, 点击后立即提示并在冷却期内禁用按钮防重复点击
const ZERO_CMD_COOLDOWN_MS = { zero: 5000 };
const ZERO_CMD_LABEL = { zero: '电机置零' };
document.querySelectorAll('.behavior-pill').forEach((btn) => {
  btn.addEventListener('click', () => {
    const cmd = btn.dataset.behavior;
    if (!cmd) return;
    const cooldown = ZERO_CMD_COOLDOWN_MS[cmd] || 0;
    if (cooldown && btn.disabled) return;
    const ok = publishBehaviorCommand(cmd);
    console.debug(`[main] 点击行为按钮 cmd=${cmd} accepted=${ok}`);
    if (!ok) return;
    if (cooldown) {
      showToast(`${ZERO_CMD_LABEL[cmd]}执行中，约 2 秒，请勿重复点击`, 'info');
      btn.disabled = true;
      btn.classList.add('is-pressed');
      setTimeout(() => {
        btn.disabled = false;
        btn.classList.remove('is-pressed');
      }, cooldown);
    } else {
      // 视觉反馈:按钮 click 状态 → CSS :active + 0.4s is-pressed 高亮化
      btn.classList.add('is-pressed');
      setTimeout(() => btn.classList.remove('is-pressed'), 600);
    }
  });
});

// 机器人命令应答(/rl_real/feedback) → toast: 置零完成的唯一可见信号
addCommandFeedbackListener((text) => {
  const warn = /reject|unknown|not ready|fail/i.test(text);
  showToast(`机器人应答:${text}`, warn ? 'warn' : 'info');
});

// ── 置零页签:4 腿 × 4 关节电机选择矩阵(多选) + 实时位置标红 + 单/多电机置零 ──
const ZERO_MOTOR_LEGS = ['FL', 'FR', 'RL', 'RR'];
const ZERO_MOTOR_JOINTS = ['髋', '大腿', '小腿', '足轮'];
/** 位置偏差超过该值(rad)视为「不在零位」,格子标红 */
const ZERO_POS_TOLERANCE_RAD = 0.1;
const zeroMotorGrid = document.getElementById('zero-motor-grid');
const zeroMotorBtn = document.getElementById('zero-motor-btn');
const zeroSelectRedBtn = document.getElementById('zero-select-red-btn');
const zeroSelectedLabel = document.getElementById('zero-selected-label');
/** 已选电机 motor_id 集合(多选) */
const zeroSelectedMotors = new Set();
/** 电机格元素表:{ cell, posEl, motorId, shortLabel } */
const zeroMotorCells = [];

/** 读取某 motor_id 的实时位置(rad);无遥测时返回 undefined */
function zeroMotorPos(motorId) {
  const jointName = JOINT_MAPPING[currentModelId]?.[motorId];
  if (!jointName) return undefined;
  const pos = motorTelemetry.get(jointName)?.realPos;
  return Number.isFinite(pos) ? pos : undefined;
}

/** 刷新已选摘要文本与置零按钮可用态 */
function refreshZeroSelectionUi() {
  if (!zeroMotorBtn || !zeroSelectedLabel) return;
  const n = zeroSelectedMotors.size;
  if (n === 0) {
    zeroSelectedLabel.textContent = '未选择电机(点击电机格多选)';
    zeroMotorBtn.disabled = true;
    zeroMotorBtn.textContent = '置零选中电机';
    return;
  }
  const labels = [...zeroSelectedMotors].sort((a, b) => a - b)
    .map((id) => zeroMotorCells[id]?.shortLabel || `#${id}`);
  zeroSelectedLabel.textContent = `已选 ${n} 个:${labels.join('、')}`;
  zeroMotorBtn.disabled = false;
  zeroMotorBtn.textContent = `置零选中电机 (${n})`;
}

if (zeroMotorGrid && zeroMotorBtn && zeroSelectedLabel) {
  // 生成 16 电机选择矩阵:motor_id = 腿*4 + 关节(与后端 motor_map_ 顺序一致)
  ZERO_MOTOR_LEGS.forEach((leg, legIdx) => {
    ZERO_MOTOR_JOINTS.forEach((joint, jointIdx) => {
      const motorId = legIdx * 4 + jointIdx;
      const cell = document.createElement('button');
      cell.type = 'button';
      cell.className = 'zero-motor-cell';
      cell.dataset.motorId = String(motorId);
      cell.title = `motor_id=${motorId}(ttycan${legIdx + 1} / CAN ID ${4 - jointIdx})`;
      const nameEl = document.createElement('span');
      nameEl.className = 'zero-name';
      nameEl.textContent = `${leg}·${joint}`;
      const posEl = document.createElement('span');
      posEl.className = 'zero-pos';
      posEl.textContent = '—';
      cell.append(nameEl, posEl);
      cell.addEventListener('click', () => {
        // 多选:再次点击取消
        if (zeroSelectedMotors.has(motorId)) zeroSelectedMotors.delete(motorId);
        else zeroSelectedMotors.add(motorId);
        cell.classList.toggle('selected', zeroSelectedMotors.has(motorId));
        refreshZeroSelectionUi();
      });
      zeroMotorCells[motorId] = { cell, posEl, motorId, shortLabel: `${leg}·${joint}` };
      zeroMotorGrid.appendChild(cell);
    });
  });

  // 置零选中电机:二次确认后逐个单发(间隔 600ms,对齐后端单台处理耗时)
  zeroMotorBtn.addEventListener('click', () => {
    if (zeroSelectedMotors.size === 0 || zeroMotorBtn.disabled) return;
    const ids = [...zeroSelectedMotors].sort((a, b) => a - b);
    const labels = ids.map((id) => zeroMotorCells[id]?.shortLabel || `#${id}`).join('、');
    if (!window.confirm(`确认对以下 ${ids.length} 个电机执行置零?\n${labels}\n\n这些电机的当前位置将被写为零点!`)) return;
    let sent = 0;
    ids.forEach((id, i) => {
      setTimeout(() => {
        const ok = publishBehaviorCommand(`zero_motor ${id}`);
        sent += 1;
        if (!ok && sent === 1) showToast('ROS 未连接,无法置零', 'warn');
      }, i * 600);
    });
    showToast(`正在置零 ${ids.length} 个电机,请勿重复点击`, 'info');
    zeroMotorBtn.disabled = true;
    zeroMotorBtn.classList.add('is-pressed');
    setTimeout(() => {
      zeroMotorBtn.classList.remove('is-pressed');
      refreshZeroSelectionUi();
    }, (ids.length - 1) * 600 + ZERO_CMD_COOLDOWN_MS.zero);
  });

  // 全选标红电机:一键选中所有偏离零位超过阈值的电机
  zeroSelectRedBtn?.addEventListener('click', () => {
    let picked = 0;
    zeroMotorCells.forEach(({ cell, motorId }) => {
      const pos = zeroMotorPos(motorId);
      const offZero = pos !== undefined && Math.abs(pos) > ZERO_POS_TOLERANCE_RAD;
      if (offZero) {
        zeroSelectedMotors.add(motorId);
        cell.classList.add('selected');
        picked += 1;
      }
    });
    if (picked === 0) showToast('当前没有标红(偏零)电机', 'info');
    refreshZeroSelectionUi();
  });

  refreshZeroSelectionUi();
}

/** 置零页实时刷新:位置数值 + 偏零标红(仅页签可见时,~5Hz 节流,由动画循环调用) */
let zeroPanelLastRefreshAt = 0;
function updateZeroPanel() {
  const view = document.getElementById('zero-control-view');
  if (!view || view.hidden || zeroMotorCells.length === 0) return;
  const now = performance.now();
  if (now - zeroPanelLastRefreshAt < 200) return;
  zeroPanelLastRefreshAt = now;
  zeroMotorCells.forEach(({ cell, posEl, motorId }) => {
    const pos = zeroMotorPos(motorId);
    const text = pos === undefined ? '—' : pos.toFixed(3);
    if (posEl.textContent !== text) posEl.textContent = text;
    cell.classList.toggle('off-zero', pos !== undefined && Math.abs(pos) > ZERO_POS_TOLERANCE_RAD);
  });
}

// ── 检查站立按钮(ROS 指令页主按钮 + 移动控制页快捷按钮) ──
const checkStandBtns = document.querySelectorAll('#check-stand-btn, .check-stand-quick');
checkStandBtns.forEach((checkStandBtn) => {
  checkStandBtn.addEventListener('click', () => {
    const ok = publishCheckStand();
    if (ok) {
      showToast('正在检查站立条件…', 'info');
      checkStandBtn.classList.add('is-pressed');
      setTimeout(() => checkStandBtn.classList.remove('is-pressed'), 600);
    } else {
      showToast('ROS 未连接,无法检查', 'warn');
    }
  });
});
// 站立检查结果 → toast + console
addCheckStandListener((result) => {
  if (result.ready) {
    showToast('✅ 站立条件就绪:电机零位达标,IMU 正常', 'info');
  } else {
    showToast(`⚠ ${result.reason}`, 'warn');
    console.warn('[check-stand] 不满足站立条件:', result.reason);
  }
});

// ── 作业交底语音播报按钮 (/rl_briefing/play,支持多选按点选顺序连播) ──
const briefingSceneEl = document.querySelector('#briefing-scenes');
const briefingPlayBtn = document.querySelector('#briefing-play-btn');
const briefingStopBtn = document.querySelector('#briefing-stop-btn');
const BRIEFING_SCENE_LABELS = { elevator: '电梯', forklift: '叉车', lifting: '吊装', warehouse: '仓库', height: '高处' };
/** 已选场景队列(按点选顺序,与 C++ 端逗号分隔多选协议一致) */
const _briefingOrder = [];
/** 刷新 chips 的选中态与播放顺序序号 */
function _refreshBriefingChips() {
  briefingSceneEl?.querySelectorAll('.scene-chip').forEach((chip) => {
    const n = _briefingOrder.indexOf(chip.dataset.scene);
    chip.classList.toggle('is-active', n >= 0);
    const orderEl = chip.querySelector('.chip-order');
    if (orderEl) orderEl.textContent = n >= 0 ? `${n + 1}` : '';
  });
}
briefingSceneEl?.querySelectorAll('.scene-chip').forEach((chip) => {
  chip.addEventListener('click', () => {
    const scene = chip.dataset.scene;
    if (!scene) return;
    const idx = _briefingOrder.indexOf(scene);
    if (idx >= 0) _briefingOrder.splice(idx, 1);
    else _briefingOrder.push(scene);
    _refreshBriefingChips();
  });
});
briefingPlayBtn?.addEventListener('click', () => {
  if (!briefingSceneEl || !_briefingOrder.length) {
    showToast('请先点选至少一个交底场景', 'warn');
    return;
  }
  const payload = _briefingOrder.join(',');
  const ok = publishBriefing(payload);
  if (ok) {
    const labelStr = _briefingOrder.map((s) => BRIEFING_SCENE_LABELS[s] || s).join('→');
    showToast(`🔊 正在播报作业交底(${_briefingOrder.length} 个场景顺序连播):${labelStr}`, 'info');
    briefingPlayBtn.classList.add('is-pressed');
    setTimeout(() => briefingPlayBtn.classList.remove('is-pressed'), 600);
  } else {
    showToast('ROS 未连接,无法播报', 'warn');
  }
});
briefingStopBtn?.addEventListener('click', () => {
  const ok = publishBriefing('stop');
  if (ok) {
    showToast('已请求停止播报', 'info');
    briefingStopBtn.classList.add('is-pressed');
    setTimeout(() => briefingStopBtn.classList.remove('is-pressed'), 600);
  } else {
    showToast('ROS 未连接', 'warn');
  }
});
// 播报状态反馈 → toast(相同内容 2s 内去重,防刷屏)
let _lastBriefingStatusText = '';
let _lastBriefingStatusAt = 0;
addBriefingStatusListener((text) => {
  const now = Date.now();
  if (text === _lastBriefingStatusText && now - _lastBriefingStatusAt < 2000) return;
  _lastBriefingStatusText = text;
  _lastBriefingStatusAt = now;
  showToast(`🔊 ${text}`, 'info');
});

// 每次重新渲染控制面板(切模型后)也重新刷新禁用状态
const _origRenderControlPanel_patched = false;

// ── 预设动作 ─────────────────────────────────────────────

/**
 * 预设关节角度组(单位 rad / m)。
 * - home:全部归零(空对象,未列出的关节默认取 0)
 */
const presets = {
  home: {},
};

/**
 * 应用预设动作到所有受控关节的 target。
 * 渲染循环会通过阻尼插值让关节平滑过渡到该目标。
 * @param {string} name - presets 中的键名(home)
 */
function applyPreset(name) {
  const preset = presets[name];
  visibleControls.forEach((state) => {
    state.target = THREE.MathUtils.clamp(preset[state.name] ?? 0, state.min, state.max);
    syncStateInputs(state);
  });
}

// ── 事件绑定 ─────────────────────────────────────────────

// 预设按钮:点击后应用对应预设动作
document.querySelectorAll('[data-preset]').forEach((button) => {
  button.addEventListener('click', () => applyPreset(button.dataset.preset));
});

// 控制面板标签切换:移动控制 / 关节控制 / 笛卡尔控制 / 曲线
document.querySelectorAll('[data-control-tab]').forEach((button) => {
  button.addEventListener('click', () => setControlTab(button.dataset.controlTab));
});
// 相机页签(先于首次 setControlTab 初始化,使持久化页签为 camera 时可见性立即生效)
initCameraView();
// Livox 雷达页签同理
initLidarView();

// 立即应用持久化的页签,避免刷新后先闪一下默认「话题调试」;
// 此时模型可能未加载,重建逻辑均有 try/catch,模型就绪后会再应用一次
setControlTab(activeControlTab);

// ROS 话题调试工具(复用 ros-bridge 连接;内部含连接状态监听与断开清理)
initRosDebug();

// ── 曲线工具栏事件 ──────────────────────────────────────
let curvesWindowSec = 10;
document.querySelector('#curves-window')?.addEventListener('change', (e) => {
  curvesWindowSec = Math.max(1, Number(e.target.value) || 10);
});
document.querySelector('#curves-pause')?.addEventListener('change', (e) => {
  setCurvesPaused(e.target.checked);
});
document.querySelector('#curves-clear')?.addEventListener('click', () => {
  clearAllBuffers();
  clearAllImuBuffers();
  clearAllJoyBuffers();
  tryRenderCurves(true);
  tryRenderImuCurves(true);
  tryRenderJoystickCurves(true);
});
// 数值与曲线同页显示:模式始终同时渲染
setCurvesDisplayMode('curves');
setImuDisplayMode('curves');

// ── 滚轮缩放 + seek bar 拖拽 + 双击复位 ──────────────────
const _seekbar = document.getElementById('curves-seekbar');
const _seekThumb = _seekbar?.querySelector('.seekbar-thumb');
const _seekLabel = _seekbar?.querySelector('.seekbar-label');
const MAX_HIST = getCurvesMaxHistorySec();

/** 滚轮缩放:无修饰键 ±20%, Ctrl ±50%, Shift ±100%;钳制到 1–75 秒 */
document.querySelectorAll('.curves-list').forEach((list) => {
  list.addEventListener('wheel', (e) => {
    e.preventDefault();
    const factor = e.ctrlKey ? 0.5 : e.shiftKey ? 1.0 : 0.2;
    const dir = e.deltaY < 0 ? -1 : 1;
    const next = curvesWindowSec * (1 + dir * factor);
    curvesWindowSec = Math.max(1, Math.min(75, Math.round(next * 10) / 10));
    // 同步下拉(仅当下拉有对应 option 时才设值,否则保持原选——滚轮是细粒度控制)
    const sel = document.getElementById('curves-window');
    const rounded = String(Math.round(curvesWindowSec));
    if (sel && [...sel.options].some((o) => o.value === rounded)) sel.value = rounded;
    // 偏移不能超出总范围
    const maxOff = Math.max(0, MAX_HIST - curvesWindowSec);
    if (getCurvesHistoryOffset() > maxOff) setCurvesHistoryOffset(maxOff);
    tryRenderCurves(true); tryRenderImuCurves(true); tryRenderJoystickCurves(true);
    updateSeekBar();
  }, { passive: false });
});

/** 双击 canvas 复位到实时模式 */
document.querySelectorAll('.curves-list').forEach((list) => {
  list.addEventListener('dblclick', (e) => {
    if (!e.target.closest('canvas')) return;
    resetCurvesHistory();
    tryRenderCurves(true); tryRenderImuCurves(true); tryRenderJoystickCurves(true);
    updateSeekBar();
  });
});

/** seek bar 拖拽:拖 thumb 浏览历史 */
let _seekDragging = false;
function updateSeekBar() {
  if (!_seekbar || !_seekThumb) return;
  const offset = getCurvesHistoryOffset();
  const zoom = curvesWindowSec;
  const maxOff = Math.max(0, MAX_HIST - zoom);
  // thumb 左端 = offset / maxOff,右端 = (offset + zoom) / MAX_HIST
  const leftPct = maxOff > 0 ? (offset / MAX_HIST) * 100 : 0;
  const widthPct = Math.min(100, (zoom / MAX_HIST) * 100);
  _seekThumb.style.left = `${leftPct}%`;
  _seekThumb.style.width = `${widthPct}%`;
  if (_seekLabel) {
    _seekLabel.textContent = offset > 0.5 ? `-${offset.toFixed(0)}s` : '实时';
  }
}
if (_seekbar) {
  const onDown = (e) => {
    _seekDragging = true;
    e.preventDefault();
    onMove(e);
  };
  const onMove = (e) => {
    if (!_seekDragging) return;
    const rect = _seekbar.getBoundingClientRect();
    const x = (e.clientX ?? e.touches?.[0]?.clientX) - rect.left;
    const pct = Math.max(0, Math.min(1, x / rect.width));
    const maxOff = Math.max(0, MAX_HIST - curvesWindowSec);
    setCurvesHistoryOffset(pct * maxOff);
    tryRenderCurves(true); tryRenderImuCurves(true); tryRenderJoystickCurves(true);
    updateSeekBar();
  };
  const onUp = () => { _seekDragging = false; };
  _seekbar.addEventListener('pointerdown', onDown);
  window.addEventListener('pointermove', onMove);
  window.addEventListener('pointerup', onUp);
  updateSeekBar();
}

/**
 * 摇杆松开衰减:左摇杆(vx/vy)与右摇杆(ω/Z)各自独立,逐帧从松开时的值线性衰减到 0,
 * 同步推送到本地仿真 + ROS cmd_vel。两组互不影响(左杆松开衰减时右杆的 ω 保持)。
 * 衰减期间 shared.joystick 的值持续更新,recordJoystickSample 会自然记录到曲线。
 * @param {number} nowMs performance.now()
 */
function updateJoystickDecay(nowMs) {
  const joy = shared.joystick;
  if (!joy) return;
  const dur = joy.decayDuration || 300;
  const r2 = (v) => Math.round(v * 100) / 100;
  let active = false;
  if (joy.decayLeftStartAt) {
    const t = 1 - Math.min(1, (nowMs - joy.decayLeftStartAt) / dur);
    joy.vx = r2((joy.decayLeftVx || 0) * t);
    joy.vy = r2((joy.decayLeftVy || 0) * t);
    active = true;
    if (t <= 0) { joy.vx = 0; joy.vy = 0; joy.decayLeftStartAt = 0; }
  }
  if (joy.decayRightStartAt) {
    const t = 1 - Math.min(1, (nowMs - joy.decayRightStartAt) / dur);
    joy.omega = r2((joy.decayRightOmega || 0) * t);
    applyJoystickLift(r2((joy.decayRightLift || 0) * t));
    active = true;
    if (t <= 0) { joy.omega = 0; applyJoystickLift(0); joy.decayRightStartAt = 0; }
  }
  if (!active) return;
  pushJoystickCommand();
  joy.lastUpdateAt = nowMs;
}

/** 安全地渲染一次关节曲线;只有可视化 tab 可见时才真正绘制。force=true 忽略节流。 */
let _lastCurvesRenderAt = 0;
function tryRenderCurves(force = false) {
  if (activeControlTab !== 'curves') return;
  const now = performance.now();
  if (!force && now - _lastCurvesRenderAt < 66) return;
  _lastCurvesRenderAt = now;
  renderCurves(curvesWindowSec);
}

/** 安全地渲染一次 IMU 曲线(与关节独立计数节流,避免互相卡掉) */
let _lastImuCurvesRenderAt = 0;
function tryRenderImuCurves(force = false) {
  if (activeControlTab !== 'curves') return;
  const now = performance.now();
  if (!force && now - _lastImuCurvesRenderAt < 66) return;
  _lastImuCurvesRenderAt = now;
  renderImuCurves(curvesWindowSec);
}

/** 安全地渲染一次摇杆曲线(独立计数节流) */
let _lastJoyCurvesRenderAt = 0;
function tryRenderJoystickCurves(force = false) {
  if (activeControlTab !== 'curves') return;
  const now = performance.now();
  if (!force && now - _lastJoyCurvesRenderAt < 66) return;
  _lastJoyCurvesRenderAt = now;
  renderJoystickCurves(curvesWindowSec);
}

// ════════════════════════════════════════════════════════
// 点击状态持久化:工具栏开关(注释/重心)、IK 姿态约束(人形/四足)、
// 笛卡尔子页签(左臂/右臂、四腿)的点击选择统一存 localStorage,
// 刷新/重开页面(含手机浏览器回收标签页后重开)自动恢复,免去重复点击。
// 桌面/移动共用 DOM 与 main.js,双端同时生效。
// ════════════════════════════════════════════════════════
const UI_CLICK_STATE_KEY = 'web_sim_ui_click_state_v1';
function loadUiClickState() {
  try { return JSON.parse(localStorage.getItem(UI_CLICK_STATE_KEY)) || {}; }
  catch { return {}; } // 隐私模式等 localStorage 不可用时全部回退默认
}
function persistUiClickState(patch) {
  try {
    localStorage.setItem(UI_CLICK_STATE_KEY, JSON.stringify({ ...loadUiClickState(), ...patch }));
  } catch { /* 忽略 */ }
}

// 双臂页签切换:在笛卡尔控制视图内切换左臂/右臂页面
document.querySelectorAll('[data-arm-tab]').forEach((button) => {
  button.addEventListener('click', () => {
    document.querySelectorAll('[data-arm-tab]').forEach((candidate) => {
      const active = candidate === button;
      candidate.classList.toggle('active', active);
      candidate.setAttribute('aria-selected', String(active));
    });
    document.querySelectorAll('.arm-control-page').forEach((page) => {
      page.hidden = page.dataset.arm !== button.dataset.armTab;
    });
    persistUiClickState({ armTab: button.dataset.armTab });
  });
});

// 四足机器人腿部页签切换
document.querySelectorAll('[data-leg-tab]').forEach((button) => {
  button.addEventListener('click', () => {
    document.querySelectorAll('[data-leg-tab]').forEach((candidate) => {
      const active = candidate === button;
      candidate.classList.toggle('active', active);
      candidate.setAttribute('aria-selected', String(active));
    });
    document.querySelectorAll('.leg-control-page').forEach((page) => {
      page.hidden = page.dataset.leg !== button.dataset.legTab;
    });
    persistUiClickState({ legTab: button.dataset.legTab });
  });
});

// IK 姿态优先开关:勾选后求解时同时约束末端姿态(roll/pitch/yaw)
const ikOrientation = document.querySelector('#ik-orientation');
if (ikOrientation) {
  ikOrientation.addEventListener('change', () => {
    persistUiClickState({ ikOrientation: ikOrientation.checked });
    cartesianArms.forEach((_, arm) => scheduleArmSolve(arm));
  });
}

// 四足机器人 IK 姿态约束开关
const ikOrientationQuad = document.querySelector('#ik-orientation-quad');
if (ikOrientationQuad) {
  ikOrientationQuad.addEventListener('change', () => {
    persistUiClickState({ ikOrientationQuad: ikOrientationQuad.checked });
    cartesianArms.forEach((_, arm) => {
      if (['FL', 'FR', 'RL', 'RR'].includes(arm)) scheduleArmSolve(arm);
    });
  });
}

// 注释可见性开关:显示/隐藏关节名称注释连线
document.querySelector('#annotations-toggle').addEventListener('change', (event) => {
  persistUiClickState({ annotations: event.target.checked });
  setAnnotationsVisible(event.target.checked);
});

// 重心可视化开关:质量加权整机 CoM 球 + 地面投影线/环
document.querySelector('#com-toggle').addEventListener('change', (event) => {
  persistUiClickState({ com: event.target.checked });
  setComVisible(event.target.checked);
});

// ── 启动恢复:绑定完成后立即按存储值回放点击状态 ──
// 注释/重心此时模型可能尚未加载,setXVisible 只置模块内标志,
// 模型加载后 initializeJointAnnotations/动画循环会按标志自动显示,无需挂加载回调。
(function restoreUiClickState() {
  const saved = loadUiClickState();
  const annotationsToggle = document.querySelector('#annotations-toggle');
  if (saved.annotations && annotationsToggle) {
    annotationsToggle.checked = true;
    setAnnotationsVisible(true);
  }
  const comToggle = document.querySelector('#com-toggle');
  if (saved.com && comToggle) {
    comToggle.checked = true;
    setComVisible(true);
  }
  if (saved.ikOrientation && ikOrientation) ikOrientation.checked = true;
  if (saved.ikOrientationQuad && ikOrientationQuad) ikOrientationQuad.checked = true;
  // 子页签恢复:直接 .click() 回放,与用户手动点击走同一套 DOM 切换逻辑
  if (saved.armTab) {
    const armBtn = document.querySelector(`[data-arm-tab="${saved.armTab}"]`);
    if (armBtn && !armBtn.classList.contains('active')) armBtn.click();
  }
  if (saved.legTab) {
    const legBtn = document.querySelector(`[data-leg-tab="${saved.legTab}"]`);
    if (legBtn && !legBtn.classList.contains('active')) legBtn.click();
  }
})();

// ── 通知面板交互 ─────────────────────────────────────────
const notifyBtn = document.querySelector('#notify-btn');
const notifyPanel = document.querySelector('#notify-panel');
const notifyClearBtn = document.querySelector('#notify-clear');
if (notifyBtn && notifyPanel) {
  notifyBtn.addEventListener('click', () => {
    const isHidden = notifyPanel.hasAttribute('hidden');
    if (isHidden) {
      notifyPanel.removeAttribute('hidden');
      // 打开面板时清除未读计数
      _notifyUnreadCount = 0;
      _updateNotifyBadge();
    } else {
      notifyPanel.setAttribute('hidden', '');
    }
  });
  // 点击面板外部关闭
  document.addEventListener('click', (e) => {
    if (notifyPanel.hasAttribute('hidden')) return;
    if (notifyPanel.contains(e.target) || notifyBtn.contains(e.target)) return;
    notifyPanel.setAttribute('hidden', '');
  });
}
if (notifyClearBtn) {
  notifyClearBtn.addEventListener('click', () => {
    _notifyHistory.length = 0;
    _notifyUnreadCount = 0;
    _updateNotifyBadge();
    _renderNotifyPanel();
  });
}
// 初始渲染空面板
_renderNotifyPanel();

// ── 移动端控制面板抽屉 ───────────────────────────────────
const panelToggle = document.querySelector('#panel-toggle');
const panelBackdrop = document.querySelector('#panel-backdrop');
const controlPanel = document.querySelector('#control-panel');
function togglePanel(open) {
  const shouldOpen = open ?? !controlPanel.classList.contains('open');
  controlPanel.classList.toggle('open', shouldOpen);
  panelBackdrop.classList.toggle('open', shouldOpen);
  panelToggle.classList.toggle('active', shouldOpen);
  panelToggle.textContent = shouldOpen ? '✕' : '⚙';
}
panelToggle.addEventListener('click', () => togglePanel());
panelBackdrop.addEventListener('click', () => togglePanel(false));

// 视角复位按钮:恢复默认 / 正视 / 侧视 / 底视四种预设视角
document.querySelector('#view-home').addEventListener('click', () => setView(defaultView.camera));
document.querySelector('#view-front').addEventListener('click', () => setView(new THREE.Vector3(4.8, 0, 1.45)));
document.querySelector('#view-side').addEventListener('click', () => setView(new THREE.Vector3(0, -5.2, 1.45)));
document.querySelector('#view-bottom')?.addEventListener('click', () => setView(new THREE.Vector3(0, 0, -3), new THREE.Vector3(0, 0, 0.3)));

/**
 * 键盘底盘控制映射表(WASDQE)。
 * 仅在笛卡尔模式生效;按下持续运动,松开停止。
 * 值为 [vx 前后, vy 横移, omega 旋转]。
 */
const keyboardCommands = {
  w: [0.35, 0, 0], s: [-0.35, 0, 0],
  a: [0, 0.35, 0], d: [0, -0.35, 0], // 左右横移(vy,+Y=左)
  q: [0, 0, 0.8], e: [0, 0, -0.8],   // q/e 原地旋转(ω)
};
window.addEventListener('keydown', (event) => {
  if (activeControlTab !== 'movement' || event.repeat || !keyboardCommands[event.key.toLowerCase()]) return;
  setChassisCommand(...keyboardCommands[event.key.toLowerCase()]);
});
window.addEventListener('keyup', (event) => {
  if (activeControlTab === 'movement' && keyboardCommands[event.key.toLowerCase()]) setChassisCommand(0, 0, 0);
});

// ── 摇杆速度上限滑杆:线速度 m/s + 角速度 rad/s(0~1.5 连续可调,人形/四足共享,每次进入默认 0.5) ──
/** 同步单个 range 的位置与已选段填充比例(--fill 供 WebKit 轨道渐变使用) */
function syncSpeedSlider(slider, value) {
  slider.value = value.toFixed(2);
  const max = Number(slider.max) || JOY_SPEED_MAX;
  const ratio = Math.max(0, Math.min(1, value / max));
  slider.style.setProperty('--fill', `${ratio * 100}%`);
}
/** 同步所有摇杆区的线速度/角速度滑杆位置与数值标签 */
function refreshSpeedLimitUi() {
  const lin = getJoystickMaxLinear();
  const ang = getJoystickMaxAngular();
  document.querySelectorAll('.joy-lin-slider').forEach((s) => syncSpeedSlider(s, lin));
  document.querySelectorAll('.joy-ang-slider').forEach((s) => syncSpeedSlider(s, ang));
  document.querySelectorAll('.joy-lin-val').forEach((o) => { o.textContent = lin.toFixed(2); });
  document.querySelectorAll('.joy-ang-val').forEach((o) => { o.textContent = ang.toFixed(2); });
}
(() => {
  // 固定默认 0.5(不持久化:避免刷新后停留在高限速,驾驶意图应每次显式选择)
  setJoystickMaxLinear(0.5);
  setJoystickMaxAngular(0.5);
  refreshSpeedLimitUi();
  document.querySelectorAll('.joy-lin-slider').forEach((slider) => {
    slider.addEventListener('input', () => {
      setJoystickMaxLinear(Number(slider.value));
      refreshSpeedLimitUi();
    });
  });
  document.querySelectorAll('.joy-ang-slider').forEach((slider) => {
    slider.addEventListener('input', () => {
      setJoystickMaxAngular(Number(slider.value));
      refreshSpeedLimitUi();
    });
  });
})();

// ── Xbox 手柄监听(Gamepad API) ──────────────────────────
/** 左/右摇杆上一次是否有输出(用于检测松开边缘 → 触发对应 onEnd) */
let _gpLeftActive = false;
let _gpRightActive = false;

/**
 * 在 animate 循环中轮询手柄:
 *  左摇杆(X/Y)+ D-pad → 移动(vx/vy);右摇杆 X → ω,右摇杆 Y → 升降 Z。
 * 标准映射: axes[0]/[1]=左摇杆(右+/下+), axes[2]/[3]=右摇杆(右+/下+)
 *          buttons[12-15]=D-pad 上/下/左/右
 */
function pollGamepad() {
  const left = shared.joystickInput;
  const right = shared.joystickZInput;
  if (!left && !right) {
    _gpLeftActive = false; _gpRightActive = false;
    return;
  }
  const pads = navigator.getGamepads?.();
  if (!pads) return;
  const pad = pads[0]; // 只用第一个手柄
  if (!pad) {
    if (_gpLeftActive) { _gpLeftActive = false; left?.onEnd(); }
    if (_gpRightActive) { _gpRightActive = false; right?.onEnd(); }
    return;
  }
  const DZ = 0.15;
  const clamp1 = (v) => Math.max(-1, Math.min(1, v));

  // 左摇杆(死区 0.15,死区外平滑映射)→ 移动 vx/vy
  const lx = pad.axes[0] || 0;
  const ly = pad.axes[1] || 0;
  const mag = Math.sqrt(lx * lx + ly * ly);
  let leftActive = false;
  if (mag > DZ) {
    const scale = (mag - DZ) / (mag * (1 - DZ));
    left?.onMove(
      Math.round(clamp1(lx * scale) * 100) / 100,
      Math.round(clamp1(-ly * scale) * 100) / 100, // Y 取反(上=+)
    );
    leftActive = true;
  } else {
    // D-pad buttons(标准 XInput: 12=上 13=下 14=左 15=右)→ 移动
    let nx = 0, ny = 0;
    if (pad.buttons[12]?.pressed) ny += 1;
    if (pad.buttons[13]?.pressed) ny -= 1;
    if (pad.buttons[14]?.pressed) nx -= 1;
    if (pad.buttons[15]?.pressed) nx += 1;
    if (nx !== 0 || ny !== 0) {
      const r = Math.sqrt(nx * nx + ny * ny);
      left?.onMove(nx / r, ny / r);
      leftActive = true;
    }
  }

  // 右摇杆 X → ω(左推=左转),Y → 升降 Z(上=+)
  const rx = pad.axes[2] || 0;
  const ry = pad.axes[3] || 0;
  const rmag = Math.sqrt(rx * rx + ry * ry);
  let rightActive = false;
  if (rmag > DZ) {
    const scale = (rmag - DZ) / (rmag * (1 - DZ));
    right?.onMove(
      Math.round(clamp1(rx * scale) * 100) / 100,
      Math.round(clamp1(-ry * scale) * 100) / 100,
    );
    rightActive = true;
  }

  // 松开边缘 → 触发对应组的衰减
  if (_gpLeftActive && !leftActive) left?.onEnd();
  if (_gpRightActive && !rightActive) right?.onEnd();
  _gpLeftActive = leftActive;
  _gpRightActive = rightActive;
}

// ── 渲染循环 ─────────────────────────────────────────────

/**
 * 根据画布尺寸调整渲染器与标签层尺寸,并更新相机宽高比。
 * 在初始化与窗口 resize 时调用。
 */
function resize() {
  // 仿真画布未启用时 canvas 为 display:none,rect 为 0;开启后由 enableSim 显式 resize
  if (!simReady || !simEnabled) return;
  const rect = canvas.getBoundingClientRect();
  if (rect.width === 0 || rect.height === 0) return;
  renderer.setSize(rect.width, rect.height, false);
  labelRenderer.setSize(rect.width, rect.height);
  camera.aspect = rect.width / rect.height;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
resize();

/** 上一帧时间戳(performance.now),用于计算帧间 dt */
let lastTime = performance.now();


/* ── IMU:姿态驱动 + 读数刷新 ────────────────────────── */

/**
 * 将 shared.imu 里的姿态(四元数或 RPY)应用到机器人模型根节点,
 * 让真机 IMU 的 roll/pitch/yaw 直接驱动仿真机身摆动。
 * 若 IMU 数据流中断超过 stale 阈值则不应用,避免用"陈旧姿态"误导。
 */
const IMU_STALE_MS = 600;
const _imuTmpQuat = new THREE.Quaternion();
const _imuTmpEuler = new THREE.Euler();
function applyImuAttitudeToRoot() {
  const root = shared.modelRoot;
  if (!root) return;
  const imu = shared.imu;
  if (!imu || imu.msgCount === 0) return;
  const age = performance.now() - (imu.lastMsgAt || 0);
  if (age > IMU_STALE_MS) return;

  const q = imu.quat;
  const qNorm = Math.hypot(q.w, q.x, q.y, q.z);
  if (qNorm > 1e-5) {
    // ROS / IMU convention: body-frame X-forward, Y-left, Z-up, quat wxyz
    // Three: xyzw, coordinate +X right, +Y up, +Z backward (camera convention)
    // 先直接用 quat 按 'ZYX' 顺序应用;真机侧如果 RPY 与四元数一致则肉眼校准即 OK。
    _imuTmpQuat.set(q.x / qNorm, q.y / qNorm, q.z / qNorm, q.w / qNorm);
    try {
      root.quaternion.copy(_imuTmpQuat);
    } catch {
      // fallback:用发送端已算好的 RPY(THREE 默认 'XYZ' 顺序,对应 intrinsic roll-pitch-yaw)
      _imuTmpEuler.set(imu.rpy.roll, imu.rpy.pitch, imu.rpy.yaw, 'XYZ');
      root.quaternion.setFromEuler(_imuTmpEuler);
    }
  } else {
    _imuTmpEuler.set(imu.rpy.roll, imu.rpy.pitch, imu.rpy.yaw, 'XYZ');
    root.quaternion.setFromEuler(_imuTmpEuler);
  }
}

/** 节流:IMU 读数 DOM 刷新(只做显示,姿态应用不节流) */
let _lastImuRenderAt = 0;
const IMU_UI_INTERVAL_MS = 66; // ~15fps
function updateImuReadout() {
  const statusEl = document.getElementById('imu-status');
  const imu = shared.imu;
  if (!statusEl || !imu) return;
  const age = performance.now() - (imu.lastMsgAt || 0);
  const streaming = imu.msgCount > 0 && age < IMU_STALE_MS;
  const stale = imu.msgCount > 0 && age >= IMU_STALE_MS;
  statusEl.classList.toggle('is-streaming', streaming);
  statusEl.classList.toggle('is-warn', stale);
  if (streaming) {
    statusEl.textContent = `数据流 OK · ${imu.msgCount} 帧 · ${(age / 1000).toFixed(1)}s 前`;
  } else if (stale) {
    statusEl.textContent = `IMU 数据流超时(${Math.round(age)}ms)`;
  } else if (imu.msgCount === 0) {
    statusEl.textContent = '等待 IMU 数据...';
  } else {
    statusEl.textContent = '未连接';
  }
}

/**
 * 主渲染循环(每帧执行)。
 * 流程:
 *  1. 计算帧间 dt(上限 0.1s 避免大跳变)
 *  2. 步进底盘控制器(轮子速度/转向)
 *  3. 同步从动夹爪
 *  4. 对每个关节按类型做阻尼插值:
 *     - 速度模式(驱动轮):速度阻尼 + 角度环绕到 [-π, π]
 *     - 位置模式(slide 平移 / hinge 旋转):角度阻尼到 target
 *  5. 更新关节注释布局、底盘位姿读数
 *  6. 更新 OrbitControls 并渲染 WebGL 与 CSS2D 标签层
 *  7. 请求下一帧
 * @param {number} time - requestAnimationFrame 传入的高精度时间戳(ms)
 */
function animate(time) {
  const dt = Math.min((time - lastTime) / 1000, 0.1);
  lastTime = time;
  // 仿真开关:关闭时跳过全部 3D 步进/插值/渲染,但数据采集、曲线、
  // 摇杆衰减、手柄、看板、报警评估等与真机相关的逻辑照常运行。
  // 移动端:仅在「仿真」页签内激活(画布在其它页签的 hidden 容器内,
  // 向隐藏画布渲染纯属耗电);真机指令走 chassisCommandHook,不受影响。
  const simTabActive = !window.__WEB_MOBILE__ || activeControlTab === 'sim';
  const simActive = simEnabled && simReady && simTabActive;
  if (simActive) {
    getQuadChassisController().step(dt);
    jointStates.forEach((state) => {
      if (state.controlMode === 'velocity') {
        // 速度模式:先阻尼插值速度,再积分到角度并环绕归一化
        state.velocity = THREE.MathUtils.damp(state.velocity, state.target, 16, dt);
        state.value = THREE.MathUtils.euclideanModulo(state.value + state.velocity * dt + Math.PI, Math.PI * 2) - Math.PI;
        setJointPose(state, state.value);
        return;
      }
      // 位置模式阻尼率分级:
      //   滑台 0.18(慢启停) / 舵轮转向 10(跟手,95% ≈ 0.3s) / 驱动轮 18(快旋转响应)
      //   其余铰链 2.5(手臂等圆滑过渡)
      const isSteering = state.name?.endsWith('_steering_joint');
      const isWheel = state.name?.endsWith('_wheel_joint');
      const rate = state.type === 'slide' ? 0.18 : isWheel ? 18 : isSteering ? 10 : 2.5;
      state.value = THREE.MathUtils.damp(state.value, state.target, rate, dt);
      setJointPose(state, state.value);
    });
  }
  // 电机数据:每帧采一个样本;只在曲线 tab 显示时才真正渲染画布(渲染节流到 15fps)
  recordSample(time);
  tryRenderCurves(false);
  // IMU 曲线:每帧采一次 15 个关键量(若暂停/超时自动跳过);渲染同样节流到 15fps(独立计数)
  recordImuSample(time);
  tryRenderImuCurves(false);
  // Xbox 手柄轮询(左摇杆/D-pad → onMove/onEnd)
  pollGamepad();
  // 摇杆输出曲线:先执行衰减(松开后逐帧从当前值衰减到 0),再采样记录
  updateJoystickDecay(time);
  recordJoystickSample(time);
  tryRenderJoystickCurves(false);
  // seek bar 状态:在 animate 循环里轻量同步(offset 随实时数据推进时 thumb 位置不变,仅标签更新)
  if (activeControlTab === 'curves') updateSeekBar();
  if (simActive) {
    // IMU:把真机姿态应用到模型根节点(让机身跟随真机摆动)
    applyImuAttitudeToRoot();
  }
  // 数值模式下数值 DOM 读数节流到 ~15fps(与仿真开关无关)
  if (time - _lastImuRenderAt >= IMU_UI_INTERVAL_MS) {
    updateImuReadout();
    _lastImuRenderAt = time;
  }
  if (simActive) {
    enforceGroundContact(dt);
    updateWheelSpokeSides();   // 校正轮子示意条始终在外侧端面
    updateJointAnnotations();
    updateComViz();            // 重心可视化(开关关闭时内部快速返回)
  }
  updateDashboard(); // 数据看板:仅页签可见时刷新(内部节流 ~10fps)
  updateZeroPanel(); // 置零页:实时位置 + 偏零标红(仅页签可见时刷新,内部节流 ~5Hz)
  // 参数报警评估:~5Hz 节流(与连接状态无关,无数据时规则自然不触发)
  if (time - _lastAlarmEvalAt >= 200) {
    evaluateAlarms();
    _lastAlarmEvalAt = time;
  }
  if (simActive) {
    orbit.update();
    renderer.render(scene, camera);
    labelRenderer.render(scene, camera);
  }
  requestAnimationFrame(animate);
}

// ── 录制与复现 ────────────────────────────────────────────
const recStartBtn = document.querySelector('#recorder-start-btn');
const replayStopBtn = document.querySelector('#replay-stop-btn');
const recsClearBtn = document.querySelector('#recordings-clear-btn');
const recordingsListEl = document.querySelector('#recordings-list');
const replaySeekEl    = document.querySelector('#replay-seek');
const replayCurTimeEl = document.querySelector('#replay-cur-time');
const replayTotalTimeEl = document.querySelector('#replay-total-time');
// 保存 .replay-seek-wrap 引用,避免列表重建时(detached) querySelector 找不到
const _replaySeekHost   = document.querySelector('#replay-seek-host');
let   _replaySeekWrap   = _replaySeekHost ? _replaySeekHost.querySelector('.replay-seek-wrap') : null;

/** 视频监控模式:当前选中的录制 ID(独立于 isReplaying,用于驱动拖动条显示) */
let _selectedRecordingId = null;

/** 拖动时临时跳过 range.value 同步,避免 input 事件与我们 set value 打架 */
let _replaySeeking = false;

/* _activeRecordingId / _lastRecStatusRefreshAt / REC_STATUS_REFRESH_MS 声明已在文件顶部
   避免 addStatusListener → applyRosUiState → rosStatusAppendExtra TDZ 报错 */

/** 格式化时长(ms → mm:ss / 秒 / ms) */
function formatDuration(ms) {
  if (!Number.isFinite(ms) || ms <= 0) return '0s';
  const s = Math.floor(ms / 1000);
  if (s < 60) return `${s}s ${Math.floor((ms % 1000) / 10)}`;
  const mm = Math.floor(s / 60), ss = s % 60;
  return `${mm}:${String(ss).padStart(2, '0')}`;
}
function formatDateTime(ts) {
  try { return new Date(ts).toLocaleString('zh-CN', { hour12: false }); }
  catch (_) { return String(ts); }
}

/** 把 mm:ss 补齐两位分钟位(总时长可能超过 1h 这里只显示到分钟,内部用 ms 做 seek) */
function _fmtSeekClock(ms) {
  if (!Number.isFinite(ms) || ms <= 0) return '00:00';
  const s = Math.floor(ms / 1000);
  const mm = Math.floor(s / 60), ss = s % 60;
  return `${String(mm).padStart(2,'0')}:${String(ss).padStart(2,'0')}`;
}

/** 根据 isReplaying() 状态 + getReplayState,把 seek 条 / 时间文本同步到 DOM
 *  视频监控模式:只要 frames 已加载(无论 isReplaying),都能显示 total 与当前位置 */
function applyReplaySeekState(preferredElapsedMs /* 可选,seek 后立刻跳到目标,而不是用 wall clock 重算 */) {
  if (!replaySeekEl) return;
  const rs = getReplayState();
  const total = rs.totalDurationMs || 0;
  const hasFrames = total > 0;
  const elapsed = Number.isFinite(preferredElapsedMs)
    ? Math.max(0, Math.min(total, preferredElapsedMs))
    : (rs.elapsedMs || 0);
  replaySeekEl.max = String(Math.max(0, total));
  if (!_replaySeeking) replaySeekEl.value = String(elapsed);
  replaySeekEl.disabled = !hasFrames;
  if (replayCurTimeEl)   replayCurTimeEl.textContent   = _fmtSeekClock(elapsed);
  if (replayTotalTimeEl) replayTotalTimeEl.textContent = _fmtSeekClock(total);
}

/** 根据 _activeRecordingId / isReplaying() 刷新录制控制按钮禁用态 + 文案。
 *  状态文本与按钮合并:#recorder-start-btn 自身承载状态文本:
 *    - 未录 + 非复现:  ● 开始录制         (灰禁 / 在线则青可点)
 *    - 录制中:          ■ 结束录制 N s    (红可点,每秒刷新秒数)
 *    - 复现中(未录):   ▶ 复现中 N s/M s   (灰禁,显示已复现/总秒数)
 *  录制状态文本仅显示秒(整数),不再含帧数与毫秒位。
 */
function applyRecorderUiState(options = {}) {
  const recording = _activeRecordingId != null && isRecording(_activeRecordingId);
  const replaying = isReplaying();
  // 先同步一次拖动条(无论是否在复现,只要 total>0 就能显示位置)
  applyReplaySeekState(options?.elapsedMs);
  if (recStartBtn) {
    if (recording) {
      // 录制中:已录时长本地计算(= Date.now() - startedAt),同步更新。
      // 之前每次调用都 listRecordings()(IndexedDB 全量读),状态 emit 高频时会造成严重卡顿。
      const dur = Math.max(0, Date.now() - (_activeRecordingStartedAt || Date.now()));
      const secs = Math.floor(dur / 1000);
      recStartBtn.textContent = `■ 结束录制 ${secs}s`;
      recStartBtn.disabled = false;
      recStartBtn.classList.add('ros-active');
      recStartBtn.classList.remove('recorder-idle', 'recorder-replaying');
      recStartBtn.classList.add('recorder-recording');
    } else if (replaying) {
      // 复现中:按钮 disabled,显示已复现/总秒数
      const rs = getReplayState();
      const elapsedSec = Math.floor(rs.elapsedMs / 1000);
      const totalSec = Math.floor(rs.totalDurationMs / 1000);
      recStartBtn.textContent = `▶ 复现中 ${elapsedSec}s/${totalSec}s`;
      recStartBtn.disabled = true;
      recStartBtn.classList.remove('ros-active', 'recorder-recording', 'recorder-idle');
      recStartBtn.classList.add('recorder-replaying');
    } else {
      // 空闲:仅 ROS 在线时可点
      recStartBtn.textContent = '● 开始录制';
      const rosOnline = getStatusSnapshot().connectionStatus === 'online';
      recStartBtn.disabled = !rosOnline || replaying;
      recStartBtn.classList.remove('ros-active', 'recorder-recording', 'recorder-replaying');
      recStartBtn.classList.add('recorder-idle');
    }
  }
  if (replayStopBtn) { replayStopBtn.disabled = !replaying; }
}

/** 渲染 recordings 列表(配置页) */
async function renderRecordingsList() {
  if (!recordingsListEl) return;
  let list = [];
  try { list = await listRecordings(); }
  catch (err) {
    recordingsListEl.innerHTML = `<li class="rec-empty rec-error">加载失败:${err.message || String(err)}</li>`;
    return;
  }
  if (!list.length) {
    recordingsListEl.innerHTML = `<li class="rec-empty">尚无录制记录 — 连接 ROS 后点击「开始录制」即可</li>`;
    return;
  }
  const replayingId = isReplaying() ? getReplayState().recordingId : null;
  const selectedId = _selectedRecordingId;
  const frag = document.createDocumentFragment();
  for (const r of list) {
    const li = document.createElement('li');
    const isAlarmRec = String(r.name || '').startsWith('⚠');
    li.className = 'recordings-row'
      + (isAlarmRec ? ' is-alarm-rec' : '')
      + (replayingId === r.id ? ' is-replaying' : '')
      + (selectedId === r.id ? ' is-selected' : '');
    li.dataset.rid = String(r.id);
    // 列 1:记录名(双击可编辑)
    const nameCol = document.createElement('span');
    nameCol.className = 'rec-name-col';
    const nameLabel = document.createElement('span');
    nameLabel.className = 'rec-name';
    nameLabel.textContent = r.name;
    nameLabel.title = '双击重命名';
    nameLabel.addEventListener('dblclick', async () => {
      const input = document.createElement('input');
      input.className = 'rec-name-input';
      input.type = 'text';
      input.value = r.name;
      const commit = async () => {
        const v = input.value.trim() || '未命名录制';
        try { await renameRecording(r.id, v); } catch (_) {}
        renderRecordingsList();
      };
      input.addEventListener('blur', commit, { once: true });
      input.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') { e.preventDefault(); input.blur(); }
        else if (e.key === 'Escape') { input.value = r.name; input.blur(); }
      });
      nameCol.replaceChild(input, nameLabel);
      input.focus(); input.select();
    });
    nameCol.appendChild(nameLabel);
    if (r.rosUrl) {
      const sub = document.createElement('span');
      sub.className = 'rec-sub';
      sub.textContent = r.rosUrl.replace(/^wss?:\/\//i, '');
      nameCol.appendChild(sub);
    }
    li.appendChild(nameCol);

    // 列 2:模型
    const modelCol = document.createElement('span');
    modelCol.className = 'rec-model-col';
    modelCol.textContent = r.modelId ? `模型 ${r.modelId}` : '—';
    if (r.modelId && String(r.modelId) === String(currentModelId)) {
      modelCol.classList.add('is-current');
      modelCol.title = '与当前模型一致';
    } else if (r.modelId) {
      modelCol.title = `复现时自动切换到模型 ${r.modelId}`;
    }
    li.appendChild(modelCol);

    // 列 3:时间 + 时长
    const metaCol = document.createElement('span');
    metaCol.className = 'rec-meta-col';
    const t1 = document.createElement('span'); t1.className = 'rec-time'; t1.textContent = formatDateTime(r.startedAt); metaCol.appendChild(t1);
    const t2 = document.createElement('span'); t2.className = 'rec-dur';  t2.textContent = `长 ${formatDuration(r.endedAt ? r.durationMs : (Date.now() - r.startedAt))}`; metaCol.appendChild(t2);
    li.appendChild(metaCol);

    // 列 4:帧数
    const framesCol = document.createElement('span');
    framesCol.className = 'rec-frames-col';
    framesCol.textContent = `电机 ${r.motorFrameCount} · IMU ${r.imuFrameCount} · 共 ${r.totalFrames}`;
    li.appendChild(framesCol);

    // 列 5:操作按钮
    const actCol = document.createElement('span');
    actCol.className = 'rec-actions-col';
    const playBtn = document.createElement('button');
    playBtn.className = 'ros-btn rec-btn-play';
    playBtn.type = 'button';
    const isThisReplaying = (replayingId === r.id);
    const isThisSelected = (selectedId === r.id);
    const hasThisFrames = isThisSelected && hasFrames();
    // 根据拖动条位置 + 播放状态决定按钮文字:
    //   正在播放 → 暂停
    //   尾部(100%) → 重播
    //   头部/中间 → 播放
    let playLabel;
    if (isThisReplaying) {
      playLabel = '■ 暂停';
    } else if (hasThisFrames) {
      const rs = getReplayState();
      playLabel = (rs.totalDurationMs > 0 && rs.progress >= 0.999) ? '↻ 重播' : '▶ 播放';
    } else {
      playLabel = '▶ 播放';
    }
    playBtn.textContent = playLabel;
    if (isThisReplaying) playBtn.classList.add('ros-active');
    playBtn.disabled = false;
    playBtn.addEventListener('click', async () => {
      try {
        if (isThisReplaying) {
          stopReplay(false, true); // not silent, keepFrames=true
          applyRecorderUiState();
          renderRecordingsList();
          return;
        }
        if (_activeRecordingId != null) {
          stopRecStatusTimer();
          try { await stopRecording(_activeRecordingId); _activeRecordingId = null; applyRecorderUiState(); await renderRecordingsList(); } catch (_) {}
        }
        if (isReplaying()) stopReplay(false);
        if (r.modelId && String(r.modelId) !== String(currentModelId)) {
          try { await switchModel(String(r.modelId)); }
          catch (err) { console.warn('[replay] 自动切换模型失败,继续复现:', err); }
        }

        // 视频监控模式:如果当前记录已选中(帧已加载),从当前 seek 位置开始复现
        // 但如果是「重播」状态(在尾部),强制从 0 开始
        if (hasThisFrames) {
          const rs = getReplayState();
          const isAtEnd = rs.totalDurationMs > 0 && rs.progress >= 0.999;
          const startMs = isAtEnd ? 0 : getCurrentSeekMs();
          await startReplay(r.id, { startMs });
        } else {
          // 首次点击:先选中记录(加载帧),然后从 0 开始复现
          await selectRecording(r.id);
          await startReplay(r.id, { startMs: 0 });
        }
        _selectedRecordingId = r.id;
        applyRecorderUiState();
        renderRecordingsList();
        _syncReplaySeekSlot();
      } catch (err) {
        console.error('[replay] 启动失败', err);
        if (recStartBtn) {
          recStartBtn.textContent = `复现失败`;
          recStartBtn.disabled = true;
          recStartBtn.classList.remove('recorder-recording', 'recorder-replaying', 'recorder-idle', 'ros-active');
          recStartBtn.classList.add('recorder-error');
          recStartBtn.title = String(err && err.message || err);
        }
      }
    });
    actCol.appendChild(playBtn);
    const delBtn = document.createElement('button');
    delBtn.className = 'ros-btn rec-btn-del ros-active';
    delBtn.type = 'button';
    delBtn.textContent = '删除';
    delBtn.addEventListener('click', async () => {
      const ok = await confirmDialog({
        title: '删除录制记录',
        message: `${r.name}\n${formatDateTime(r.startedAt)} · 共${r.totalFrames}帧`,
        confirmText: '删除', danger: true,
      });
      if (!ok) return;
      try {
        if (isReplaying() && getReplayState().recordingId === r.id) stopReplay(false);
        if (_selectedRecordingId === r.id) {
          clearFrames();
          _selectedRecordingId = null;
        }
        await deleteRecording(r.id);
        purgeAlarmRecordMeta(r.id);
        renderRecordingsList();
        _syncReplaySeekSlot();
      } catch (err) {
        console.error('[rec] 删除失败', err);
      }
    });
    actCol.appendChild(delBtn);
    li.appendChild(actCol);

    // 列 6:本行复现进度拖动条 slot(默认隐藏,仅在选中此行时显示)
    const seekSlot = document.createElement('div');
    seekSlot.className = 'rec-seek-slot';
    seekSlot.dataset.rid = String(r.id);
    seekSlot.hidden = true;
    li.appendChild(seekSlot);

    // 点击行选中(点击按钮/输入框时不触发)
    li.addEventListener('click', async (e) => {
      const tag = (e.target && e.target.tagName) || '';
      if (tag === 'BUTTON' || tag === 'INPUT') return;
      if (e.target.closest('.rec-actions-col')) return;
      await selectRecording(r.id);
      renderRecordingsList();
    });

    frag.appendChild(li);
  }
  // 清空前:若 wrap 在某个 slot 里,先移回 host 防止 detached(引用仍在,不会丢)
  if (_replaySeekWrap && _replaySeekHost && _replaySeekWrap.parentElement !== _replaySeekHost) {
    _replaySeekHost.appendChild(_replaySeekWrap);
  }
  // 清空旧内容
  while (recordingsListEl.firstChild) recordingsListEl.removeChild(recordingsListEl.firstChild);
  recordingsListEl.appendChild(frag);
  // 渲染后,若当前正在复现某行,把拖动条移动到该行的 slot
  _syncReplaySeekSlot();
}

/**
 * 视频监控模式:选中某条录制,展示其拖动条。
 *  - 如果切换了不同记录,清理旧帧
 *  - 如果当前正在复现此记录,不做额外处理(拖动条本来就该显示)
 *  - 如果当前正在复现另一条记录,先停止复现(保留帧供新记录使用)
 *  @param {number|null} rid  要选中的 recordingId,null 表示取消选中
 */
async function selectRecording(rid) {
  const ridNum = rid != null ? Number(rid) : null;

  if (ridNum == null) {
    _selectedRecordingId = null;
    clearFrames();
    _syncReplaySeekSlot();
    applyReplaySeekState(0);
    return;
  }

  // 如果选中的是当前正在复现的记录,直接同步显示
  if (isReplaying() && getReplayState().recordingId === ridNum) {
    _selectedRecordingId = ridNum;
    _syncReplaySeekSlot();
    applyReplaySeekState();
    return;
  }

  // 如果选中的是已加载但未复现的记录(如刚结束复现),保持当前选择
  if (hasFrames() && !isReplaying() && getReplayState().recordingId === ridNum) {
    _selectedRecordingId = ridNum;
    _syncReplaySeekSlot();
    applyReplaySeekState();
    return;
  }

  // 切换到新记录:停掉旧复现(如有),清理旧帧,加载新帧
  if (isReplaying()) {
    stopReplay(true);
  }
  clearFrames();

  _selectedRecordingId = ridNum;
  _syncReplaySeekSlot();

  // 预加载帧(异步,完成后更新 seek bar 的 max)
  try {
    await loadFramesForPreview(ridNum);
    applyReplaySeekState(0);
    renderRecordingsList();
  } catch (err) {
    console.warn('[selectRecording] 预加载帧失败:', err);
    clearFrames();
    _selectedRecordingId = null;
    _syncReplaySeekSlot();
  }
}

/**
 * 把 #replay-seek-wrap(整个 .replay-seek-wrap 子树)移动到:
 *   - 选中某条记录时:选中行的 .rec-seek-slot(并显示 slot)
 *   - 否则:回到 #replay-seek-host(隐藏)
 */
function _syncReplaySeekSlot() {
  if (!_replaySeekHost || !_replaySeekWrap) return;
  const wrap = _replaySeekWrap;
  const rid = _selectedRecordingId;
  let targetSlot = null;
  if (rid != null) {
    targetSlot = document.querySelector(`.rec-seek-slot[data-rid="${rid}"]`);
  }
  if (targetSlot && wrap.parentElement !== targetSlot) {
    targetSlot.appendChild(wrap);
    targetSlot.hidden = false;
  } else if (!targetSlot && wrap.parentElement !== _replaySeekHost) {
    _replaySeekHost.appendChild(wrap);
  }
  document.querySelectorAll('.rec-seek-slot').forEach((slot) => {
    slot.hidden = !(targetSlot && slot === targetSlot);
  });
}

// ── 复现拖动条事件绑定 ──────────────────────────────────

/** 初始化「录制 & 复现」模块:挂事件 + 初始渲染列表 */
async function initRecorderAndReplay() {
  try { await openRecorderDB(); }
  catch (err) {
    console.warn('[recorder] IndexedDB 不可用:', err);
    if (recStartBtn) {
      recStartBtn.textContent = 'IndexedDB 不可用';
      recStartBtn.disabled = true;
      recStartBtn.classList.remove('recorder-recording', 'recorder-replaying', 'recorder-idle', 'ros-active');
      recStartBtn.classList.add('recorder-error');
      recStartBtn.title = String(err && err.message || err);
    }
    [recStartBtn, replayStopBtn, recsClearBtn].forEach((b) => { if (b) b.disabled = true; });
    return;
  }
  await renderRecordingsList();

  if (recStartBtn) {
    recStartBtn.addEventListener('click', async () => {
      // 复现中不可触发录制
      if (isReplaying()) {
        showToast('正在复现中,请先停止复现再录制。', 'warn');
        return;
      }
      // 录制中 → 结束录制;未录 → 启动录制
      if (_activeRecordingId != null && isRecording(_activeRecordingId)) {
        const rid = _activeRecordingId; _activeRecordingId = null;
        // 先停定时器,避免与 stopRecording 的列表刷新竞争
        stopRecStatusTimer();
        try {
          await stopRecording(rid);
        } catch (err) {
          console.error('[rec] 停止录制失败', err);
        } finally {
          applyRecorderUiState();
          await renderRecordingsList();
        }
        return;
      }
      // 启动录制:要求 ROS 在线
      if (getStatusSnapshot().connectionStatus !== 'online') {
        showToast('请先连接 ROS 再开始录制。', 'warn');
        applyRecorderUiState();
        return;
      }
      try {
        const meta = { rosUrl: getRosUrl(), modelId: currentModelId };
        const { recordingId } = await startRecording(meta);
        _activeRecordingId = recordingId;
        _activeRecordingStartedAt = Date.now();
        applyRecorderUiState();
        // 启动录制状态定时刷新:让顶部状态栏秒/帧统计实时更新(否则只在此处刷新一次)
        startRecStatusTimer();
      } catch (err) {
        console.error('[rec] 启动录制失败', err);
        // 错误显示在按钮上(已合并状态胶囊)
        if (recStartBtn) {
          recStartBtn.textContent = '录制启动失败';
          recStartBtn.disabled = true;
          recStartBtn.classList.remove('recorder-recording', 'recorder-replaying', 'recorder-idle', 'ros-active');
          recStartBtn.classList.add('recorder-error');
          recStartBtn.title = String(err && err.message || err);
        }
      }
    });
  }
  if (replayStopBtn) {
    replayStopBtn.addEventListener('click', () => {
      if (!isReplaying()) return;
      stopReplay(false, true); // not silent, keepFrames=true
      applyRecorderUiState();
      renderRecordingsList();
    });
  }
  if (recsClearBtn) {
    recsClearBtn.addEventListener('click', async () => {
      try {
        const list = await listRecordings();
        if (!list.length) { showToast('暂无可清理的记录。'); return; }
        const ok = await confirmDialog({
          title: '清空全部录制',
          message: `共 ${list.length} 条记录,删除后不可恢复。`,
          confirmText: '全部清空', danger: true,
        });
        if (!ok) return;
        if (isReplaying()) stopReplay(false);
        clearFrames();
        _selectedRecordingId = null;
        for (const r of list) {
          await deleteRecording(r.id);
          purgeAlarmRecordMeta(r.id);
        }
        await renderRecordingsList();
        _syncReplaySeekSlot();
        applyRecorderUiState();
      } catch (err) {
        console.error('[rec] 清空失败', err);
      }
    });
  }

  // ════════════════════════════════════════════════════════
  // 复现拖动条:
  //   - 复现中:input 实时 seek(不暂停播放),change 时恢复回写
  //   - 视频监控模式(选中但未复现):input 时 seek-and-inject(暂停在该位置)
  // ════════════════════════════════════════════════════════
  if (replaySeekEl) {
    replaySeekEl.addEventListener('input', (e) => {
      const v = Number(e.target.value) || 0;
      _replaySeeking = true;
      if (isReplaying()) {
        replaySeekTo(v);
      } else if (hasFrames()) {
        seekAndInject(v);
      }
      applyRecorderUiState({ elapsedMs: v });
      applyRosUiState(getStatusSnapshot());
    });
    replaySeekEl.addEventListener('change', (e) => {
      const v = Number(e.target.value) || 0;
      if (isReplaying()) {
        replaySeekTo(v);
      } else if (hasFrames()) {
        seekAndInject(v);
      }
      _replaySeeking = false;
      applyRecorderUiState({ elapsedMs: v });
      applyRosUiState(getStatusSnapshot());
      renderRecordingsList();
    });
  }

  // 录制流管道:连接 motor / imu 监听器,回调时如果在录制就 recordFrame
  addMessageListener((payload) => {
    const tsMs = payload?.at || performance.now();
    if (_activeRecordingId == null) return;
    recordFrame(_activeRecordingId, {
      topic: MOTOR_STATE_TOPIC,
      timestampMs: tsMs,
      data: Array.isArray(payload?.data) ? payload.data : [],
    });
  });
  addImuListener((payload) => {
    const tsMs = payload?.at || performance.now();
    if (_activeRecordingId == null) return;
    recordFrame(_activeRecordingId, {
      topic: IMU_STATE_TOPIC,
      timestampMs: tsMs,
      data: Array.isArray(payload?.data) ? payload.data : [],
    });
  });

  // ════════════════════════════════════════════════════════
  // 机器人通知事件 (/rl_real/notify):显示 toast + 通知历史面板
  // (监听器已在模块顶层注册,这里仅保留注释占位)
  // ════════════════════════════════════════════════════════
  // 报警记录回放请求(alarm.js → 复用录制复现管道,自动切换到报警时的模型)
  window.addEventListener('alarm-replay-request', async (evt) => {
    const id = Number(evt.detail?.id);
    if (!Number.isFinite(id)) return;
    try {
      if (isReplaying()) stopReplay(false);
      if (_activeRecordingId != null) {
        stopRecStatusTimer();
        try { await stopRecording(_activeRecordingId); _activeRecordingId = null; applyRecorderUiState(); } catch (_) {}
      }
      const r = await getRecording(id);
      if (r?.modelId && String(r.modelId) !== String(currentModelId)) {
        try { await switchModel(String(r.modelId)); }
        catch (err) { console.warn('[alarm] 报警回放自动切换模型失败,继续复现:', err); }
      }
      await selectRecording(id);
      await startReplay(id, { startMs: 0 });
      _selectedRecordingId = id;
      applyRecorderUiState();
      renderRecordingsList();
      _syncReplaySeekSlot();
    } catch (err) {
      console.error('[alarm] 报警记录回放失败:', err);
    }
  });
  // 复现事件 → 刷新 UI
  /** 用于 complete 后 200ms 内取消「到点就立刻清 100%」的延迟 id */
  let _replayCompleteClearTimer = 0;
  addReplayListener((evt) => {
    // 若 seek 后 / 进度 100% → 不节流,立即刷新(让 100% 立刻到位,拖动条立刻到位)
    const forceRefresh = evt.seeked || (evt.type === 'progress' && evt.progress >= 1);
    if (evt.type === 'progress' && !forceRefresh) {
      const now = performance.now();
      if (now - _lastRecStatusRefreshAt < REC_STATUS_REFRESH_MS) return;
      _lastRecStatusRefreshAt = now;
    }
    if (evt.type === 'progress') {
      applyRecorderUiState({ elapsedMs: evt.elapsedMs });
      applyRosUiState(getStatusSnapshot());
    } else if (evt.type === 'start') {
      if (_replayCompleteClearTimer) { clearTimeout(_replayCompleteClearTimer); _replayCompleteClearTimer = 0; }
      if (evt.recordingId != null) _selectedRecordingId = evt.recordingId;
      applyRecorderUiState();
      applyRosUiState(getStatusSnapshot());
      renderRecordingsList();
      _syncReplaySeekSlot();
    } else if (evt.type === 'stop') {
      if (_replayCompleteClearTimer) { clearTimeout(_replayCompleteClearTimer); _replayCompleteClearTimer = 0; }
      applyRecorderUiState();
      applyRosUiState(getStatusSnapshot());
      renderRecordingsList();
      _syncReplaySeekSlot();
    } else if (evt.type === 'complete') {
      if (_replayCompleteClearTimer) { clearTimeout(_replayCompleteClearTimer); _replayCompleteClearTimer = 0; }
      applyRecorderUiState({ elapsedMs: evt.elapsedMs });
      applyRosUiState(getStatusSnapshot());
      _replayCompleteClearTimer = setTimeout(() => {
        _replayCompleteClearTimer = 0;
        stopReplay(true /* silent */, true /* keepFrames */);
        // 保持拖动条在结束位置,注入终点帧
        seekAndInject(evt.elapsedMs);
        if (evt.recordingId != null) _selectedRecordingId = evt.recordingId;
        applyRecorderUiState();
        applyRosUiState(getStatusSnapshot());
        renderRecordingsList();
        _syncReplaySeekSlot();
      }, 250);
    } else {
      applyRecorderUiState();
    }
  });
}

// ── 启动 ─────────────────────────────────────────────────

// 初始化录制 & 复现(DB + 列表 + 事件)
initRecorderAndReplay()
  .catch((err) => console.warn('[recorder] init 失败:', err));

// 底盘摇杆提前绑定:仿真画布关闭时,移动页签的摇杆/键盘仍可经
// chassisCommandHook 向真机发布 /rl_real/cmd_vel(createJoystick 幂等,
// 模型加载后 initializeCartesianControl 再绑一次无副作用)。
try {
  setupQuadrupedChassisControl();
  document.querySelector('#movement-quadruped-chassis')?.removeAttribute('hidden');
} catch (err) {
  console.warn('[chassis] 摇杆提前初始化失败:', err);
}

// 仿真开关:工具条按钮 + 关闭态占位层中央按钮,同一对 enable/disable
document.querySelector('#sim-toggle-btn')?.addEventListener('click', () => {
  if (simEnabled) disableSim();
  else enableSim().catch(() => { /* 失败已在 enableSim 内回滚 + 提示 */ });
});
document.querySelector('#sim-enable-btn')?.addEventListener('click', () => {
  enableSim().catch(() => { /* 同上 */ });
});

// 按持久化开关决定是否加载模型(默认关闭 → 不下载 ~40MB 网格、不渲染 3D)。
// 移动端例外:仿真由「仿真」页签联动,启动一律不自动下载(即便上次偏好为开,
// 刷新后内存模型也已失效),进入仿真页后再由占位卡片/秒恢复处理。
applySimUi();
if (!window.__WEB_MOBILE__ && loadSimEnabledPref()) {
  enableSim()
    .catch(() => { /* 失败已提示 */ })
    .finally(() => setControlTab(activeControlTab));
} else {
  document.querySelector('#loading-overlay')?.classList.add('is-hidden');
  setControlTab(activeControlTab);
}
requestAnimationFrame(animate);
