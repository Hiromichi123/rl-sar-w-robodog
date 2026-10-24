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
import { installDiffDriveUi } from './diff-drive-ui.js?v=2026102001';
import { canvas, renderer, labelRenderer, scene, camera, orbit, defaultView, setView, status, controlsElement } from './scene.js?v=2';
import {
  jointStates, visibleControls, cartesianArms, shared,
  clearAllModelState, MODEL_REGISTRY, currentModelId,
} from './state.js?v=1';
import { stopActiveStepperMotion, setJointPose, syncStateInputs } from './joints.js?v=1';
import { loadModel, nextLoadGeneration, updateWheelSpokeSides } from './model.js?v=989';
import { setAnnotationsVisible, updateJointAnnotations, clearStaticAnnotations } from './annotations.js?v=949';
import { setComVisible, updateComViz } from './com-viz.js?v=3';
import { rebuildDashboard, updateDashboard } from './dashboard.js?v=2026102001';
import { setChassisCommand, pushJoystickCommand, applyJoystickLift, getJoystickLift, scheduleArmSolve, syncArmTarget, enforceGroundContact, getQuadChassisController, getJoystickMaxLinear, setJoystickMaxLinear, getJoystickMaxAngular, setJoystickMaxAngular, JOY_SPEED_MAX, setupQuadrupedChassisControl, setCruiseLockLeft, setCruiseLockRight } from './cartesian.js?v=1025';
import {
  addStatusListener, setPositionSource, toggleConnection, setRosUrl, getRosUrl,
  addMessageListener, addImuListener, addNotifyListener, publishNotifyAck,
  addCheckStandListener, addCommandFeedbackListener, publishCheckStand,
  addPostureListener, addCheckStandingListener,
  lastImuMessageAt, imuMessageCount,
  publishBehaviorCommand, setCmdVelRequest, setCmdVelVz, stopCmdVel,
  setCmdVelAxesEnabled, getCmdVelAxesEnabled, getCmdVelRequest,
  MOTOR_STATE_TOPIC, IMU_STATE_TOPIC, getStatusSnapshot,
  motorTelemetry, JOINT_MAPPING,
} from './ros-bridge.js?v=2026102001';
import {
  recordSample, renderCurves, rebuildCurveList, clearAllBuffers,
  setPaused as setCurvesPaused,
  setDisplayMode as setCurvesDisplayMode, getDisplayMode as getCurvesDisplayMode,
  recordImuSample, renderImuCurves, rebuildImuCurveList, clearAllImuBuffers,
  setImuDisplayMode, getImuDisplayMode,
  recordJoystickSample, renderJoystickCurves, rebuildJoystickCurveList, clearAllJoyBuffers,
  getCurvesHistoryOffset, setCurvesHistoryOffset, resetCurvesHistory, getCurvesMaxHistorySec,
} from './curves.js?v=2026102001';
import { initRosDebug, onRosDebugTabShown } from './ros-debug.js?v=2026102001';
import { initAlarm, evaluateAlarms, clearAlarmMarkers, rebuildAlarmEditor } from './alarm.js?v=2026102001';
import { initCameraView, setCameraViewVisible } from './camera-view.js?v=2026102001';
import { initLidarView, setLidarViewVisible } from './lidar-view.js?v=2026102001';
import { initLogView, setLogViewVisible } from './logview.js?v=2026102001';
import { initSysAdmin, setTerminalVisible, setFilesVisible, setSystemVisible, setSyslogVisible, setModelsVisible, setServicesVisible, setProcessesVisible, setNetVisible, setDevVisible, setRcVisible, setAboutVisible } from './sysadmin.js?v=2026102102';
import { initBattery, setBatteryVisible } from './battery.js?v=2026102001';

// 全局 401 拦截:会话 Cookie 过期/服务端更换令牌时,任何 fetch 收到 401
// 都统一回到登录页(文档级导航本身也会被后端 302,这里覆盖接口层失效)
const _origFetch = window.fetch.bind(window);
window.fetch = async (...args) => {
  const resp = await _origFetch(...args);
  if (resp.status === 401) {
    window.location.replace('./login.html');
    return new Promise(() => {});  // 挂起,阻止业务代码继续处理该响应
  }
  return resp;
};

/** 当前激活的控制面板标签:桌面端与移动端统一精简为
 *  移动控制/ROS 指令/置零/数据看板,默认「移动控制」。
 *  刷新后恢复上次选择;旧版本遗留的页签名(rosdebug/curves 等)回退默认。 */
const CONTROL_TAB_STORAGE_KEY = 'web_sim_control_tab_v1';

// 一级分组 ↔ 二级 tab 映射（PC 端两级导航）
const TAB_GROUPS_MAP = {
  control: ['movement', 'ros', 'joint', 'dashboard', 'rc', 'models'],
  system:  ['terminal', 'system', 'syslog', 'services', 'processes', 'net', 'dev', 'files', 'logs', 'battery', 'about'],
};
const TAB_TO_GROUP = Object.fromEntries(
  Object.entries(TAB_GROUPS_MAP).flatMap(([g, tabs]) => tabs.map((t) => [t, g]))
);
const DESKTOP_CONTROL_TABS = ['movement', 'ros', 'joint', 'dashboard', 'logs', 'terminal', 'system', 'models', 'files', 'rc', 'battery'];
const MOBILE_CONTROL_TABS = ['movement', 'sim', 'ros', 'joint', 'dashboard', 'models', 'rc', 'battery'];
function loadPersistedControlTab() {
  try {
    let saved = localStorage.getItem(CONTROL_TAB_STORAGE_KEY);
    // 旧版「系统管理」页签拆分为「终端」+「文件」, 历史选择迁移到终端
    if (saved === 'system') saved = 'terminal';
    if (window.__WEB_MOBILE__) {
      return MOBILE_CONTROL_TABS.includes(saved) ? saved : 'movement';
    }
    return DESKTOP_CONTROL_TABS.includes(saved) ? saved : 'movement';
  } catch {
    return 'movement'; // 隐私模式等 localStorage 不可用时回退默认
  }
}
let activeControlTab = loadPersistedControlTab();

// ════════════════════════════════════════════════════════
// 3D 仿真画布开关(默认关闭)
// 关闭时:不加载模型网格(~40MB OBJ)、不创建渲染帧;ROS 连接、数据看板、
// 曲线、真机遥控(摇杆/键盘 → cmd_vel)均独立可用。
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
      import('./ros-bridge.js?v=2026102001').then((m) => refreshSliderDisabledState(!!m.rosControlActive)).catch(() => {});
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
.ui-modal-danger{background:#e07070;border-color:#e07070;color:#fff}
.ui-modal-input{width:100%;box-sizing:border-box;margin:0 0 16px;padding:8px 10px;border-radius:7px;border:1px solid var(--c-input-border,#2c3a55);background:#0c1422;color:#eef3fa;font-size:13px;font-family:inherit}
.ui-modal-input:focus{outline:none;border-color:var(--c-accent,#4dd0c1)}
.ui-modal-lg{width:min(92vw,560px)}
.ui-modal-textarea{width:100%;box-sizing:border-box;min-height:220px;margin:0 0 16px;padding:9px 11px;border-radius:7px;border:1px solid var(--c-input-border,#2c3a55);background:#0c1422;color:#eef3fa;font-size:12.5px;font-family:ui-monospace,Menlo,Consolas,monospace;line-height:1.55;resize:vertical;tab-size:2}
.ui-modal-textarea:focus{outline:none;border-color:var(--c-accent,#4dd0c1);box-shadow:0 0 0 3px rgba(77,208,193,.15)}
.ui-modal-textarea::placeholder{color:#5f6f89}
.ui-modal-textarea-note{min-height:120px;font-family:inherit;font-size:13px;line-height:1.6;padding:10px 12px}
.ui-modal-hint{font-size:11px;color:#5f6f89;margin:-8px 0 14px}
.ui-modal-log-screen{width:100%;box-sizing:border-box;max-height:60vh;overflow-y:auto;margin:0 0 16px;padding:9px 11px;border-radius:7px;border:1px solid var(--c-input-border,#2c3a55);background:#0d1117;color:#c9d4e0;font-size:11.5px;font-family:ui-monospace,Menlo,Consolas,monospace;line-height:1.5;white-space:pre-wrap;word-break:break-anywhere}
.ui-modal-msg:empty{display:none}`;
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

// 跨模块 toast 通道:battery.js 等独立模块派发 window 事件请求轻提示(避免循环 import)
window.addEventListener('app-toast', (e) => {
  const d = (e && e.detail) || {};
  if (d.message) showToast(d.message, d.type === 'warn' ? 'warn' : 'info');
});

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

/**
 * 自定义信息对话框(仅"确定"按钮),返回 Promise<void>。
 * 用于替代 window.alert。
 */
function alertDialog({ title = '提示', message = '' } = {}) {
  _ensureUiFeedbackDom();
  return new Promise((resolve) => {
    const mask = document.createElement('div');
    mask.className = 'ui-modal-mask';
    mask.innerHTML = `
      <div class="ui-modal" role="alertdialog" aria-modal="true">
        <h3 class="ui-modal-title"></h3>
        <p class="ui-modal-msg"></p>
        <div class="ui-modal-actions">
          <button type="button" class="ui-modal-ok"></button>
        </div>
      </div>`;
    mask.querySelector('.ui-modal-title').textContent = title;
    mask.querySelector('.ui-modal-msg').textContent = message;
    const okBtn = mask.querySelector('.ui-modal-ok');
    okBtn.textContent = '确定';
    document.body.appendChild(mask);
    let settled = false;
    const close = () => {
      if (settled) return;
      settled = true;
      mask.classList.remove('ui-modal-show');
      window.setTimeout(() => mask.remove(), 150);
      resolve();
    };
    mask.querySelector('.ui-modal-ok').addEventListener('click', close);
    requestAnimationFrame(() => {
      mask.classList.add('ui-modal-show');
      okBtn.focus();
    });
  });
}

/**
 * 自定义输入对话框,返回 Promise<string|null>(null=取消)。
 * 用于替代 window.prompt。
 * @param {{title?:string, message?:string, defaultValue?:string, placeholder?:string}} opts
 */
function promptDialog({ title = '请输入', message = '', defaultValue = '', placeholder = '' } = {}) {
  _ensureUiFeedbackDom();
  return new Promise((resolve) => {
    const mask = document.createElement('div');
    mask.className = 'ui-modal-mask';
    mask.innerHTML = `
      <div class="ui-modal" role="dialog" aria-modal="true">
        <h3 class="ui-modal-title"></h3>
        <p class="ui-modal-msg"></p>
        <input type="text" class="ui-modal-input" spellcheck="false" autocomplete="off" />
        <div class="ui-modal-actions">
          <button type="button" class="ui-modal-cancel">取消</button>
          <button type="button" class="ui-modal-ok">确定</button>
        </div>
      </div>`;
    mask.querySelector('.ui-modal-title').textContent = title;
    mask.querySelector('.ui-modal-msg').textContent = message;
    const input = mask.querySelector('.ui-modal-input');
    input.value = defaultValue;
    input.placeholder = placeholder;
    const okBtn = mask.querySelector('.ui-modal-ok');
    document.body.appendChild(mask);
    let settled = false;
    const close = (val) => {
      if (settled) return;
      settled = true;
      mask.classList.remove('ui-modal-show');
      window.setTimeout(() => mask.remove(), 150);
      resolve(val);
    };
    const submit = () => close(input.value);
    mask.querySelector('.ui-modal-cancel').addEventListener('click', () => close(null));
    okBtn.addEventListener('click', submit);
    input.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') { e.preventDefault(); submit(); }
      else if (e.key === 'Escape') close(null);
    });
    mask.addEventListener('click', (e) => { if (e.target === mask) close(null); });
    requestAnimationFrame(() => {
      mask.classList.add('ui-modal-show');
      input.focus();
      input.select();
    });
  });
}

// 暴露到 window,供 sysadmin.js 等子模块复用(避免循环 import)
window.confirmDialog = confirmDialog;
window.alertDialog = alertDialog;
window.promptDialog = promptDialog;
window.ensureUiFeedbackDom = _ensureUiFeedbackDom;

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

// 通知监听器:模块顶层注册,确保页面加载后即可接收通知。
addNotifyListener((payload) => {
  _notifyHistory.push({ ...payload });
  if (_notifyHistory.length > NOTIFY_HISTORY_MAX) _notifyHistory.shift();
  _notifyUnreadCount++;
  _updateNotifyBadge();
  _renderNotifyPanel();
  // probe 类型消息不弹 toast(由探针页签卡片展示);
  // safe_getup 为后端编排进度帧(start/step/done),由摇杆区步骤芯片展示,
  // 成功/失败另有 success/error 类型中文通知,这里不逐帧弹英文 toast
  if (payload.type !== 'probe' && payload.type !== 'safe_getup') {
    const toastType = (payload.type === 'error' || payload.type === 'warning') ? 'warn' : 'info';
    const prefix = _notifyTypeIcon(payload.type);
    showToast(`${prefix} ${payload.content}`, toastType);
  }
  if (payload.type === 'error') console.error(`[notify #${payload.id}] ${payload.content}`);
  // 探针消息分发到页签卡片
  if (payload.type === 'probe') _handleProbeNotify(payload.content);
});

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
      const stateModule = await import('./ros-bridge.js?v=2026102001');
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

// ── 电机探针页签 ─────────────────────────────────────────
const PROBE_MOTOR_NAMES = [
  'FL_hip','FL_thigh','FL_calf','FL_wheel',
  'FR_hip','FR_thigh','FR_calf','FR_wheel',
  'RL_hip','RL_thigh','RL_calf','RL_wheel',
  'RR_hip','RR_thigh','RR_calf','RR_wheel'
];
let _probeGridBuilt = false;

/** 探针网格已合并到置零矩阵;此函数绑定探针工具栏按钮(首次进入页签时调用) */
function _buildProbeGrid() {
  if (_probeGridBuilt) return;
  // 绑定"探针选中"按钮:对选中的电机逐个执行探针
  const probeSelectedBtn = document.getElementById('probe-selected-btn');
  if (probeSelectedBtn && !probeSelectedBtn.dataset.bound) {
    probeSelectedBtn.dataset.bound = '1';
    probeSelectedBtn.addEventListener('click', () => {
      const ids = [...zeroSelectedMotors].sort((a, b) => a - b);
      if (ids.length === 0) { showToast('请先选择电机', 'warn'); return; }
      // 标记选中电机为等待中(内存+格子)
      ids.forEach((id) => {
        probeResults.set(id, 'waiting');
        const rec = zeroMotorCells[id];
        if (rec) {
          rec.cell.classList.remove('is-probe-ok', 'is-probe-fail');
          rec.cell.classList.add('is-probe-running');
          rec.probeStateEl.textContent = '等待中...';
        }
      });
      // 逐个发送 probe_motor 命令(后端串行处理,通过 notify 返回单台结果)
      ids.forEach((id) => publishBehaviorCommand(`probe_motor m${id}`));
    });
  }
  // 绑定分腿探针按钮
  document.querySelectorAll('.probe-leg-btn').forEach((btn) => {
    if (btn.dataset.probeLegBound) return;
    btn.dataset.probeLegBound = '1';
    btn.addEventListener('click', () => {
      const leg = Number(btn.dataset.leg);
      const legName = ['FL','FR','RL','RR'][leg];
      // 标记该腿 4 台为等待中(内存+格子)
      for (let j = 0; j < 4; j++) {
        const id = leg * 4 + j;
        probeResults.set(id, 'waiting');
        const rec = zeroMotorCells[id];
        if (rec) {
          rec.cell.classList.remove('is-probe-ok', 'is-probe-fail');
          rec.cell.classList.add('is-probe-running');
          rec.probeStateEl.textContent = '等待中...';
        }
      }
      const status = document.getElementById('probe-status');
      if (status) { status.textContent = `${legName} 腿探针中...`; status.className = 'probe-status is-running'; }
      publishBehaviorCommand(`probe_leg ${legName}`);
    });
  });
  _probeGridBuilt = true;
}

/** 单台探针 */
function _runSingleProbe(id) {
  const rec = zeroMotorCells[id];
  if (rec) {
    rec.cell.classList.remove('is-probe-ok', 'is-probe-fail');
    rec.cell.classList.add('is-probe-running');
    rec.probeStateEl.textContent = '探针中...';
  }
  publishBehaviorCommand(`probe_motor m${id}`);
}

/** 处理后端 probe 类型 Notify 消息 */
function _handleProbeNotify(content) {
  const parts = content.split(/\s+/);
  const action = parts[0];
  // start: 清空全部格子状态(全部探针模式)
  if (action === 'start') {
    _buildProbeGrid();
    for (let i = 0; i < 16; i++) probeResults.set(i, 'waiting');
    zeroMotorCells.forEach((rec) => {
      rec.cell.classList.remove('is-probe-ok', 'is-probe-fail');
      rec.cell.classList.add('is-probe-running');
      rec.probeStateEl.textContent = '等待中...';
    });
    const status = document.getElementById('probe-status');
    if (status) { status.textContent = '探针进行中...'; status.className = 'probe-status is-running'; }
    return;
  }
  // legstart N: 分腿探针开始,仅标记该腿 4 台为等待中
  if (action === 'legstart') {
    _buildProbeGrid();
    const leg = Number(parts[1]);
    for (let j = 0; j < 4; j++) {
      const id = leg * 4 + j;
      probeResults.set(id, 'waiting');
      const rec = zeroMotorCells[id];
      if (rec) {
        rec.cell.classList.remove('is-probe-ok', 'is-probe-fail');
        rec.cell.classList.add('is-probe-running');
        rec.probeStateEl.textContent = '等待中...';
      }
    }
    const status = document.getElementById('probe-status');
    if (status) { status.textContent = '分腿探针进行中...'; status.className = 'probe-status is-running'; }
    return;
  }
  // legdone N fails: 分腿探针完成
  if (action === 'legdone') {
    const status = document.getElementById('probe-status');
    if (status) { status.textContent = '分腿探针完成'; status.className = 'probe-status is-done'; }
    return;
  }
  // done N: 全部完成
  if (action === 'done') {
    const fails = Number(parts[1]) || 0;
    const status = document.getElementById('probe-status');
    if (status) {
      status.textContent = fails === 0 ? '全部可控 ✅' : `${fails} 台无响应`;
      status.className = 'probe-status is-done';
    }
    return;
  }
  // result m0 FL_hip ok/fail: 单台结果(全部探针模式)
  // single m0 FL_hip start/ok/fail: 单台探针模式
  if (action === 'result' || action === 'single') {
    const idStr = parts[1] || '';
    const id = Number(idStr.replace(/^m/, ''));
    const state = parts[3]; // start / ok / fail
    // 更新内存探针状态
    if (state === 'ok') probeResults.set(id, 'ok');
    else if (state === 'fail') probeResults.set(id, 'fail');
    else if (state === 'start') probeResults.set(id, 'running');
    const rec = zeroMotorCells[id];
    if (!rec) return;
    rec.cell.classList.remove('is-probe-ok', 'is-probe-fail', 'is-probe-running');
    if (state === 'start') {
      rec.cell.classList.add('is-probe-running');
      rec.probeStateEl.textContent = '探针中...';
    } else if (state === 'ok') {
      rec.cell.classList.add('is-probe-ok');
      rec.probeStateEl.textContent = '可控 ✅';
    } else if (state === 'fail') {
      rec.cell.classList.add('is-probe-fail');
      rec.probeStateEl.textContent = '无响应 ❌';
    }
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
  // 一级分组高亮 & 二级 tabs 容器显隐
  const __grp = TAB_TO_GROUP[tab] || null;
  if (__grp) {
    document.querySelectorAll('[data-tab-group]').forEach((b) => {
      const on = b.dataset.tabGroup === __grp;
      b.classList.toggle('active', on);
      b.setAttribute('aria-selected', String(on));
    });
    document.querySelectorAll('.control-tabs[data-group-tabs]').forEach((wrap) => {
      wrap.hidden = wrap.dataset.groupTabs !== __grp;
    });
  }
  document.querySelectorAll('[data-control-tab]').forEach((button) => {
    const active = button.dataset.controlTab === tab;
    button.classList.toggle('active', active);
    button.setAttribute('aria-selected', String(active));
  });
  // 各页签显隐:用可选链避免移动端缺少部分 view 时抛错中断切换
  const q = (sel) => document.querySelector(sel);
  const jointEl = q('#joint-control-view'); if (jointEl) jointEl.hidden = tab !== 'joint';
  const mvEl = q('#movement-control-view'); if (mvEl) mvEl.hidden = tab !== 'movement';
  const cartEl = q('#cartesian-control-view'); if (cartEl) cartEl.hidden = tab !== 'cartesian';
  const curvesEl = q('#curves-control-view'); if (curvesEl) curvesEl.hidden = tab !== 'curves';
  const dashEl = q('#dashboard-control-view'); if (dashEl) dashEl.hidden = tab !== 'dashboard';
  const camEl = q('#camera-control-view'); if (camEl) camEl.hidden = tab !== 'camera';
  const lidarEl = q('#lidar-control-view'); if (lidarEl) lidarEl.hidden = tab !== 'lidar';
  const rosdbgEl = q('#rosdebug-control-view'); if (rosdbgEl) rosdbgEl.hidden = tab !== 'rosdebug';
  const alarmEl = q('#alarm-control-view'); if (alarmEl) alarmEl.hidden = tab !== 'alarm';
  const logsEl = q('#logs-control-view'); if (logsEl) logsEl.hidden = tab !== 'logs';
  const termEl = q('#terminal-control-view'); if (termEl) termEl.hidden = tab !== 'terminal';
  const sysEl = q('#system-control-view'); if (sysEl) sysEl.hidden = tab !== 'system';
  const syslogEl = q('#syslog-control-view'); if (syslogEl) syslogEl.hidden = tab !== 'syslog';
  const svcEl = q('#services-control-view'); if (svcEl) svcEl.hidden = tab !== 'services';
  const psEl = q('#processes-control-view'); if (psEl) psEl.hidden = tab !== 'processes';
  const netEl = q('#net-control-view'); if (netEl) netEl.hidden = tab !== 'net';
  const devEl = q('#dev-control-view'); if (devEl) devEl.hidden = tab !== 'dev';
  const modEl = q('#models-control-view'); if (modEl) modEl.hidden = tab !== 'models';
  const filesEl = q('#files-control-view'); if (filesEl) filesEl.hidden = tab !== 'files';
  const rcEl = q('#rc-control-view'); if (rcEl) rcEl.hidden = tab !== 'rc';
  const batEl = q('#battery-control-view'); if (batEl) batEl.hidden = tab !== 'battery';
  const aboutEl = q('#about-control-view'); if (aboutEl) aboutEl.hidden = tab !== 'about';
  // 移动端独占的仿真页签容器(桌面 HTML 无此节点,可选链跳过)
  const simViewEl = q('#sim-control-view');
  if (simViewEl) simViewEl.hidden = tab !== 'sim';
  // 移动端独占的 ROS 指令页签容器
  const rosViewEl = q('#ros-control-view');
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

    // 切到关节管理页签时构建探针卡片网格(仅首次)
    if (tab === 'joint') _buildProbeGrid();

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
  // 实时日志页签:可见时订阅 journalctl -f,切走退订
  try { setLogViewVisible(tab === 'logs'); } catch { /* 忽略 */ }
  // 终端页签:终端 PTY 随页签可见性保活
  try { setTerminalVisible(tab === 'terminal'); } catch { /* 忽略 */ }
  // 系统页签:服务状态轮询/HR 日志状态随页签可见性开关
  try { setSystemVisible(tab === 'system'); } catch { /* 忽略 */ }
  // 系统日志页签
  try { setSyslogVisible(tab === 'syslog'); } catch { /* 忽略 */ }
  // 服务管理页签
  try { setServicesVisible(tab === 'services'); } catch { /* 忽略 */ }
  // 进程管理页签
  try { setProcessesVisible(tab === 'processes'); } catch { /* 忽略 */ }
  // 网络管理页签(ip addr)
  try { setNetVisible(tab === 'net'); } catch { /* 忽略 */ }
  // 设备管理页签(/dev)
  try { setDevVisible(tab === 'dev'); } catch { /* 忽略 */ }
  // 模型页签: 进入时刷新模型列表
  try { setModelsVisible(tab === 'models'); } catch { /* 忽略 */ }
  // 文件页签: 进入时刷新 FTP 目录
  try { setFilesVisible(tab === 'files'); } catch { /* 忽略 */ }
  // 遥控学习页签: 进入时刷新设备状态/已学码,离开时停止学习
  try { setRcVisible(tab === 'rc'); } catch { /* 忽略 */ }
  // 电池页签: 进入时启动蓝牙轮询,离开时停止(避免与 WiFi 争用射频)
  try { setBatteryVisible(tab === 'battery'); } catch { /* 忽略 */ }
  // 关于页签: 进入时查询主控/Web 版本信息
  try { setAboutVisible(tab === 'about'); } catch { /* 忽略 */ }

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

  // 连接成功时:持久化地址、push 历史、持久化「刷新前是否在线」
  addStatusListener((state) => {
    if (state.connectionStatus === 'online') {
      const u = getRosUrl();
      try { localStorage.setItem('ros_url_last', u); } catch (_) { /* 忽略 */ }
      pushUrlHist(u);
      rosUrlInputEl.value = u;
      // 重连后拉取 safe_getup 当前状态,供刷新/重连后回显
      try { publishBehaviorCommand('safe_getup status'); } catch (_) { /* 忽略 */ }
    }
    try { localStorage.setItem('ros_connected_last', state.connectionStatus === 'online' ? '1' : '0'); } catch (_) { /* 忽略 */ }
  });
  // 页面加载即自动连接 WS:默认同源地址,鉴权靠浏览器自动携带的 jxg_admin cookie,
  // 无需手动填写 token 或点连接按钮。连接失败/断开由指数退避自动重连兜底。
  setTimeout(() => {
    toggleConnection().catch((err) => console.warn('[main] 自动连接失败:', err));
  }, 300);
})();

// 参数报警模块:绑定报警配置页控件并恢复持久化规则
initAlarm();


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
  // 「置零选中电机」也带 behavior-pill 类,上面的统一联动会覆盖其选中态;
  // 这里按「在线 && 已选」重算一次(零页未初始化时跳过)
  if (zeroPanelReady) refreshZeroSelectionUi();
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
/** 置零页 DOM 初始化完成标志:addStatusListener 首次回调早于置零页 const 初始化(TDZ),需门控 */
let zeroPanelReady = false;
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
  rosStatusEl.textContent = text;
  rosStatusEl.title = lastError || fullText || text;

  // 连接中/在线 → 禁用地址输入框;离线/错误 → 可改地址
  const locked = s === 'online' || s === 'connecting';
  setRosInputsDisabled(locked);

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
}

rosConnectBtn?.addEventListener('click', () => {
  toggleConnection().catch((err) => console.warn('[main] ROS 连接失败:', err));
});
rosPosSourceEl?.addEventListener('change', () => {
  setPositionSource(rosPosSourceEl.value);
});
addStatusListener(applyRosUiState);

// ── 真机命令:行为按钮 + 底盘 cmd_vel 路由 ────────────────
// ── Twist 读数(数值 + 中心零点方向条):所有 cmd_vel 出口统一走 updateTwistReadout ──
/** 方向条渲染:中心为零点,正值向右填(accent 青),负值向左填(橙);宽度=|val|/scale 的半条比例 */
function setTwistGauge(id, val, scale) {
  const el = document.getElementById(id);
  if (!el) return;
  const v = Math.max(-1, Math.min(1, scale > 0 ? val / scale : 0));
  el.style.width = (Math.abs(v) * 50).toFixed(1) + '%';
  if (v >= 0) { el.style.left = '50%'; el.style.right = 'auto'; }
  else { el.style.left = 'auto'; el.style.right = '50%'; }
  el.classList.toggle('is-neg', v < 0);
}
/** 方向条量程:跟随当前摇杆速度上限(满偏=满条),滑杆缺失时回退 1.5 */
function twistGaugeScale(selector, fallback) {
  const s = document.querySelector(selector);
  const v = s ? parseFloat(s.value) : NaN;
  return Number.isFinite(v) && v > 0.05 ? v : fallback;
}
/** 刷新 Twist 栏 4 个数值 + 4 条方向条(读 getCmdVelRequest,方向禁用的轴已置 0) */
function updateTwistReadout() {
  const out = getCmdVelRequest();
  const lin = twistGaugeScale('.joy-lin-slider', 1.5);
  const ang = twistGaugeScale('.joy-ang-slider', 1.5);
  const linX = document.getElementById('twist-lin-x'); if (linX) linX.value = out.vx.toFixed(2);
  const linY = document.getElementById('twist-lin-y'); if (linY) linY.value = out.vy.toFixed(2);
  const linZ = document.getElementById('twist-lin-z'); if (linZ) linZ.value = (out.vz || 0).toFixed(2);
  const angZ = document.getElementById('twist-ang-z'); if (angZ) angZ.value = out.omega.toFixed(2);
  setTwistGauge('twist-gauge-x', out.vx, lin);
  setTwistGauge('twist-gauge-y', out.vy, lin);
  setTwistGauge('twist-gauge-z', out.vz || 0, lin);
  setTwistGauge('twist-gauge-w', out.omega, ang);
}

/**
 * 底盘命令钩子:cartesian.js 里的四足 chassis 所有 vx/vy/omega 命令入口
 * 都会回调到这里。ROS 在线时把它转发为 /rl_real/cmd_vel 持续发布(20Hz keep-alive),离线则忽略。
 */
shared.chassisCommandHook = (vx, vy, omega) => {
  // 即便 ROS 未连接也允许 setCmdVelRequest(内部会 return false,副作用很小),
  // 这样 UI 按钮/键盘不需要额外判断,所有输入路径保持一致。
  // 第 4 参不传 → 保持当前垂直速度(vz 由独立滑块控制,摇杆/旋转不清掉它)。
  setCmdVelRequest(vx, vy, omega);
  // 显示实际输出值(方向禁用后该方向为 0),与后端回显保持一致
  updateTwistReadout();
};

// ── 垂直速度(linear.z):vz 滑块已删除,仅在底盘复位时清零 ROS vz 通道 ──
/** 复位/停止时把 ROS vz 请求清零 */
function resetCmdVelVz() {
  setCmdVelVz(0);
  updateTwistReadout();
}
// 底盘复位按钮(cartesian.js 已有自己的复位逻辑,这里追加清 vz)
document.getElementById('base-reset')?.addEventListener('click', resetCmdVelVz);
document.getElementById('quad-base-reset')?.addEventListener('click', resetCmdVelVz);

// ── 摇杆锁定(定速巡航):左右两杆各自独立开关,运行时态不持久化(刷新/重连默认关) ──
const cruiseLockBtn = document.getElementById('joy-cruise-lock');      // 左杆:vx/vy 速度锁定
const cruiseLockZBtn = document.getElementById('joy-cruise-lock-z');   // 右杆:ω/Z 转向高度锁定
function syncCruiseLockBtn(btn, on, label) {
  if (!btn) return;
  btn.classList.toggle('is-active', on);
  btn.setAttribute('aria-pressed', String(on));
  const txt = btn.querySelector('.joy-lock-text');
  if (txt) txt.textContent = on ? `${label}：开` : `${label}：关`;
}
function applyCruiseLockLeft(on) {
  setCruiseLockLeft(on);      // cartesian.js:锁当前 vx/vy / 关闭即归零停车
  syncCruiseLockBtn(cruiseLockBtn, on, '速度锁定');
  syncLockSliders();          // 锁定期间冻结线速度上限滑杆
  updateTwistReadout();       // 关锁归零/开锁锁速,读数立即反映
}
function applyCruiseLockRight(on) {
  setCruiseLockRight(on);     // cartesian.js:锁当前 ω/Z / 关闭即转向归零、Z 回站姿
  syncCruiseLockBtn(cruiseLockZBtn, on, '转向锁定');
  syncLockSliders();          // 锁定期间冻结角速度上限滑杆
  updateTwistReadout();
}
/** 锁定期冻结速度上限滑杆:左杆锁 vx/vy → 线速度滑杆禁用,右杆锁 ω → 角速度滑杆禁用。
 * 否则锁定巡航中拖滑杆会直接改变锁定输出的上限(锁定值=摇杆量×上限),造成速度跳变。 */
function syncLockSliders() {
  const lockL = !!shared.joystick.cruiseLockedLeft;
  const lockR = !!shared.joystick.cruiseLockedRight;
  document.querySelectorAll('.joy-lin-slider').forEach((s) => {
    if (!('origTitle' in s.dataset)) s.dataset.origTitle = s.title || '';
    s.disabled = lockL;
    s.title = lockL ? '速度锁定中:先关闭「速度锁定」再调整线速度上限' : s.dataset.origTitle;
  });
  document.querySelectorAll('.joy-ang-slider').forEach((s) => {
    if (!('origTitle' in s.dataset)) s.dataset.origTitle = s.title || '';
    s.disabled = lockR;
    s.title = lockR ? '转向锁定中:先关闭「转向锁定」再调整角速度上限' : s.dataset.origTitle;
  });
}
syncLockSliders();  // 初始同步(默认两杆都未锁,滑杆可用)
cruiseLockBtn?.addEventListener('click', () => applyCruiseLockLeft(!shared.joystick.cruiseLockedLeft));
cruiseLockZBtn?.addEventListener('click', () => applyCruiseLockRight(!shared.joystick.cruiseLockedRight));
// 点底盘复位等同停车:同步解除两个锁定,避免"指令已清零但锁定仍挂着"的状态错配
document.getElementById('quad-base-reset')?.addEventListener('click', () => { applyCruiseLockLeft(false); applyCruiseLockRight(false); });
document.getElementById('base-reset')?.addEventListener('click', () => { applyCruiseLockLeft(false); applyCruiseLockRight(false); });
// 方向输出启用开关:整张轴卡片即开关(卡片内隐藏 checkbox),默认 x 和 ω 启用
document.querySelectorAll('.twist-v input[type="checkbox"]').forEach((cb) => {
  const axis = cb.dataset.axis;
  if (!axis) return;
  // 同步初始状态(确保 UI 勾选态与 ros-bridge 默认值一致)
  const initial = getCmdVelAxesEnabled();
  cb.checked = !!initial[axis];
  cb.addEventListener('change', () => {
    setCmdVelAxesEnabled(axis, cb.checked);
    updateTwistReadout();  // 禁用方向立即归零显示
  });
});
/** 行为按钮点击 → 对应 String burst 发布 3 次 */
// 「进入强化学习」特殊处理:先做站姿校验,未站稳直接拒绝(后端 locomotion 门禁同样会拦截)
document.querySelectorAll('.behavior-pill[data-behavior="locomotion"]').forEach((btn) => {
  btn.addEventListener('click', async (ev) => {
    // 拦截默认发布(通用处理器在同元素上后注册,用 stopImmediatePropagation 阻止)
    ev.stopImmediatePropagation();
    if (btn.disabled) return;
    const online = (typeof getStatusSnapshot === 'function')
        && getStatusSnapshot().connectionStatus === 'online';
    if (!online) { showToast('ROS 未连接,无法进入强化学习', 'warn'); return; }
    // 快捷路径:当前已是 locomotion 且 settled → 直接发(重发语义)
    if (lastPosture && lastPosture.posture === 'locomotion' && lastPosture.settled) {
      publishBehaviorCommand('locomotion');
      return;
    }
    btn.disabled = true;
    btn.classList.add('is-pressed');
    showToast('进入强化学习前:正在执行站姿校验…', 'info');
    const r = await requestCheckStanding(2500);
    btn.classList.remove('is-pressed');
    if (r === null) {
      showToast('站姿校验超时(未收到结果),已取消进入强化学习', 'warn');
      btn.disabled = false;
      return;
    }
    if (!r.at_stand) {
      // 站姿校验未通过 → 直接拒绝,不再提供"强制进入"选项(后端 locomotion 门禁也会兜底拦截)
      const detail = r.reason || `joint[${r.worst_joint}] err=${(r.worst_err ?? NaN).toFixed?.(3)}rad`;
      showToast(`站姿校验未通过,已拒绝进入强化学习:${detail}`, 'warn');
      btn.disabled = false;
      return;
    }
    showToast('✅ 站姿校验通过,进入强化学习', 'info');
    publishBehaviorCommand('locomotion');
    setTimeout(() => { btn.disabled = false; }, 1200);
  }, true); // capture: 先于下方通用处理器执行
});
// 置零类事件命令: 单发;可靠置零后端四腿并行+逐台确认,16 台正常 ~0.3s、
// 含重发最坏 ~1s,冷却 3s 仅为防连点(结果以 zero_result 回执 toast 为准)
// 最近一帧后端姿态/门禁广播(8Hz,与「门禁」胶囊同源):供行为按钮前置检测使用,
// 避免本地缓存与后端真实门禁状态错位
let lastPostureFrame = null;
addPostureListener((p) => { lastPostureFrame = p; });

const ZERO_CMD_COOLDOWN_MS = { zero: 3000 };
const ZERO_CMD_LABEL = { zero: '电机置零' };
document.querySelectorAll('.behavior-pill').forEach((btn) => {
  btn.addEventListener('click', () => {
    const cmd = btn.dataset.behavior;
    if (!cmd) return;
    // 起立(getup)前置检测:探针门禁。
    // 优先用后端 8Hz 姿态帧的实时门禁字段(probe_done/probe_ready)——与「门禁」胶囊
    // 及后端 getup 入口完全同源;仅旧后端不广播该字段时才回退本地 probeResults。
    // 之前只用本地结果:页面刷新/重连后本地记录清空而后端探针状态仍在,
    // 出现"胶囊显示探针通过、点击起立却提示需探针"的状态错位。
    if (cmd === 'getup') {
      // 卧姿校验门禁(与探针同语义):优先读后端 8Hz 姿态帧 lie_done/lie_ready,
      // 与「卧姿」胶囊及后端 getup 入口完全同源;旧后端无 lie_done 时跳过预检,
      // 由后端实时门禁兜底。
      if (lastPostureFrame && typeof lastPostureFrame.lie_done === 'boolean') {
        const lieOk = lastPostureFrame.lie_done && lastPostureFrame.lie_ready === true;
        if (!lieOk) {
          showToast(lastPostureFrame.lie_done === false
            ? '起立前需先执行「卧姿校验」(趴下后记录作废,每次起立前需重新校验)'
            : '卧姿校验未通过:有腿关节偏离零位超限,请先置零再重新校验', 'warn');
          return;
        }
      }
      let probeOk = null;   // null = 后端未广播实时门禁(旧后端),走本地回退
      let liveDone = null;
      if (lastPostureFrame && typeof lastPostureFrame.probe_done === 'boolean') {
        liveDone = lastPostureFrame.probe_done;
        const liveReady = lastPostureFrame.probe_ready === true;
        probeOk = liveDone && liveReady;
      }
      if (probeOk === null) {
        const failed = [];
        const untested = [];
        for (let i = 0; i < 16; i++) {
          const st = probeResults.get(i);
          if (st === 'fail') failed.push(i);
          else if (st !== 'ok') untested.push(i);
        }
        if (failed.length > 0 || untested.length > 0) {
          const parts = [];
          if (untested.length > 0) parts.push(`${untested.length} 台未检测`);
          if (failed.length > 0) parts.push(`${failed.length} 台无响应`);
          showToast(`起立前需全部电机探针通过(${parts.join('、')}),请先在「关节管理」执行探针`, 'warn');
          return;
        }
      } else if (!probeOk) {
        showToast(liveDone === false
          ? '起立前需先执行电机可控性探针,请先在「关节管理」执行探针'
          : '探针门禁未通过:存在无响应电机,请先在「关节管理」重新探针确认', 'warn');
        return;
      }
    }
    const cooldown = ZERO_CMD_COOLDOWN_MS[cmd] || 0;
    if (cooldown && btn.disabled) return;
    const ok = publishBehaviorCommand(cmd);
    console.debug(`[main] 点击行为按钮 cmd=${cmd} accepted=${ok}`);
    if (!ok) return;
    // 趴下即作废整条安全起立流程的视觉进度:立即清芯片(若流程运行中被趴下
    // 打断,busy 锁留给后端 done fail 帧复位,芯片会先红后由 posture 回趴兜底清空)
    if (cmd === 'getdown') {
      clearSafeGetupSteps();
      if (!safeGetupBusy) resetSafeGetupUi();
    }
    if (cooldown) {
      showToast(`${ZERO_CMD_LABEL[cmd]}执行中，约 1 秒，结果见回执提示`, 'info');
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
  // 可靠置零结构化回执:逐台真实结果(后端经反馈确认,非"命令已发"),
  // 交置零页处理并抑制原始 JSON toast
  if (typeof text === 'string' && text.startsWith('zero_result ')) {
    handleZeroResult(text.slice('zero_result '.length));
    return;
  }
  // 安全起立流程进行中:safe_getup 系列回执(accepted/rejected/内部进度)由流程自身
  // 与后端 success/error Notify 提示,抑制通用英文应答 toast
  if (safeGetupBusy && typeof text === 'string' && /safe_getup/.test(text)) {
    return;
  }
  // 将后端英文应答翻译为中文展示
  let display = text;
  let type = 'info';
  // 已知的 getup 拒绝原因(后端英文→中文)
  const rejectReasonMap = {
    'motors not probed': '电机未执行可控性探针',
    'some motors not controllable': '存在不可控电机',
    'IMU no data': 'IMU 无数据',
    'not at standing pose': '未处于站立姿态',
    'joint feedback invalid': '关节反馈无效',
    'read default_dof_pos failed': '读取站立期望角失败',
  };
  if (/command accepted/i.test(text)) {
    display = '命令已接受: ' + text.replace(/command accepted:?\s*/i, '');
  } else if (/command rejected/i.test(text)) {
    let reason = text.replace(/command rejected:?\s*/i, '');
    // 尝试匹配已知原因并翻译
    for (const [en, zh] of Object.entries(rejectReasonMap)) {
      if (reason.includes(en)) { reason = reason.replace(en, zh); break; }
    }
    display = '命令被拒绝: ' + reason;
    type = 'warn';
  } else if (/unknown command/i.test(text)) {
    display = '未知命令: ' + text.replace(/unknown command:?\s*/i, '');
    type = 'warn';
  } else if (/not ready/i.test(text)) {
    display = '系统未就绪: ' + text.replace(/not ready:?\s*/i, '');
    type = 'warn';
  } else {
    const warn = /reject|unknown|not ready|fail/i.test(text);
    type = warn ? 'warn' : 'info';
  }
  showToast(`机器人应答:${display}`, type);
  // 兼容旧后端: 单电机置零 accepted 文本回执
  const m = /zero_motor\s+(\d{1,2})/i.exec(text);
  if (m && /accepted/i.test(text)) markMotorZeroed(Number(m[1]));
});

// ── 姿态显示:后端 8Hz posture 帧 → 「当前姿态」胶囊 ──
const POSTURE_LABEL = {
  lying:       { text: '趴下',       cls: 'posture-lying' },
  standing:    { text: '站立',       cls: 'posture-standing' },
  locomotion:  { text: '强化学习',   cls: 'posture-locomotion' },
  diff_drive:  { text: '四轮差速',   cls: 'posture-standing' },
  transition:  { text: '切换中…',    cls: 'posture-transition' },
  unknown:     { text: '— 未知 —',   cls: 'posture-unknown' },
};
/** 最近一次姿态帧(供 RL 进入检测读取) */
let lastPosture = null;
installDiffDriveUi({ addPosture: addPostureListener, addStatus: addStatusListener,
  send: publishBehaviorCommand,
  stop: () => { stopCmdVel(); updateTwistReadout(); } });
function renderPosture(p) {
  lastPosture = p;
  const pills = document.querySelectorAll('.posture-pill');
  if (pills.length === 0) return;
  const info = POSTURE_LABEL[p.posture] || POSTURE_LABEL.unknown;
  const title = `FSM: ${p.fsm || 'unknown'} · settled=${p.settled} · motors_ready=${p.motors_ready} · can_ready=${p.can_ready === null ? '未知' : p.can_ready}`;
  const fsmShort = (p.fsm || '').replace(/^RLFSMState/, '');
  pills.forEach((pill) => {
    if (pill.textContent !== info.text) pill.textContent = info.text;
    pill.className = 'posture-pill ' + info.cls;
    pill.title = title;
    const canEl = renderCanPill(pill, p.can_ready);
    const gateEl = renderGatePill(canEl, p);
    const lieEl = renderLiePill(gateEl, p);
    renderProbePill(lieEl, p);
  });
  document.querySelectorAll('.posture-fsm').forEach((fsmEl) => {
    if (fsmEl.textContent !== fsmShort) fsmEl.textContent = fsmShort;
  });
}

/**
 * 在姿态胶囊旁渲染 CAN 实时状态胶囊(随姿态帧 8Hz)。
 * true=16 电机反馈全部新鲜(绿);false=存在超时电机,动作指令会被后端拒绝(红);
 * null=后端不广播该字段(旧版本),隐藏。每个 posture-pill 只注入一次。
 */
function renderCanPill(pill, canReady) {
  let can = pill.nextElementSibling;
  if (!can || !can.classList.contains('can-pill')) {
    can = document.createElement('span');
    can.className = 'can-pill';
    pill.insertAdjacentElement('afterend', can);
  }
  if (canReady === null || canReady === undefined) { can.hidden = true; return can; }
  can.hidden = false;
  if (canReady) {
    can.textContent = 'CAN 正常';
    can.className = 'can-pill is-ok';
    can.title = '16 个电机反馈均在 500ms 内';
  } else {
    can.textContent = 'CAN 断线';
    can.className = 'can-pill is-bad';
    can.title = '有电机反馈超过 500ms 未更新：起立/行走/趴下指令将被拒绝';
  }
  return can;
}

/**
 * 姿态胶囊旁的三枚门禁胶囊(随姿态帧 8Hz 实时刷新,判定与后端命令入口同源):
 *   站姿门禁 → 进强化学习:12 腿关节在站立期望 ±0.30rad 内且静止(stand_ready);
 *   卧姿门禁 → 起立:「卧姿校验」手动结果,与探针同语义(lie_done/lie_ready,
 *              趴下即作废转灰,不跨趴下周期);
 *   探针门禁 → 起立:16 台电机可控性探针全部通过(probe_ready/probe_done)。
 * 字段缺失(旧后端)时隐藏;lie_done 缺失但 lie_ready 存在时回退旧实时语义。
 */
function ensurePill(prevEl, token, fullCls) {
  let el = prevEl?.nextElementSibling;
  if (!el || !el.classList.contains(token)) {
    el = document.createElement('span');
    el.className = fullCls || token;
    prevEl.insertAdjacentElement('afterend', el);
  }
  return el;
}

function renderGatePill(canEl, p) {
  if (!canEl) return null;
  const gate = ensurePill(canEl, 'gate-pill');
  const ready = p.stand_ready;
  if (ready === null || ready === undefined) { gate.hidden = true; return gate; }
  gate.hidden = false;
  if (ready) {
    gate.textContent = '站姿 通过';
    gate.className = 'gate-pill is-ok';
    gate.title = '站姿门禁(进强化学习):当前姿态满足条件(站立期望 ±0.30rad 内且关节静止)';
  } else {
    gate.textContent = '站姿 未通过';
    gate.className = 'gate-pill is-bad';
    const err = (typeof p.stand_err === 'number' && p.stand_err >= 0)
      ? `;最差偏差 ${p.stand_err.toFixed(3)}rad(joint${p.stand_joint})` : '';
    gate.title = `站姿门禁未通过,拒绝进入强化学习:${p.stand_reason || '未处于站立姿态'}${err}`;
  }
  return gate;
}

function renderLiePill(gateEl, p) {
  if (!gateEl) return null;
  const lie = ensurePill(gateEl, 'lie-pill', 'gate-pill lie-pill');
  // 新语义(后端广播 lie_done):与探针同三态——未检测(灰)/通过(绿)/未通过(红)。
  // 结果仅来自「卧姿校验」按钮,趴下即作废转灰,不跨趴下周期。
  if (typeof p.lie_done === 'boolean') {
    lie.hidden = false;
    if (!p.lie_done) {
      lie.textContent = '卧姿 未检测';
      lie.className = 'gate-pill lie-pill is-idle';
      lie.title = '卧姿门禁(起立):本次趴下周期尚未执行「卧姿校验」,拒绝起立。请先执行「卧姿校验」';
    } else if (p.lie_ready) {
      lie.textContent = '卧姿 通过';
      lie.className = 'gate-pill lie-pill is-ok';
      lie.title = '卧姿门禁(起立):卧姿校验通过,腿关节均在零位邻域(|q|≤1.0rad),允许起立。趴下后记录作废,需重新校验';
    } else {
      lie.textContent = '卧姿 未通过';
      lie.className = 'gate-pill lie-pill is-bad';
      lie.title = '卧姿门禁未通过:卧姿校验时有腿关节偏离零位超过 1.0rad(或 CAN 断线),拒绝起立。请先置零再重新校验';
    }
    return lie;
  }
  // 旧后端回退:无 lie_done 但有 lie_ready(实时判定语义)
  const ready = p.lie_ready;
  if (ready === null || ready === undefined) { lie.hidden = true; return lie; }
  lie.hidden = false;
  if (ready) {
    lie.textContent = '卧姿 通过';
    lie.className = 'gate-pill lie-pill is-ok';
    lie.title = '卧姿门禁(起立):腿关节均在零位邻域(|q|≤1.0rad),允许起立';
  } else {
    lie.textContent = '卧姿 未通过';
    lie.className = 'gate-pill lie-pill is-bad';
    lie.title = '卧姿门禁未通过:有腿关节偏离零位超过 1.0rad,拒绝起立(请先置零)。站立时该门禁天然不适用,照实显示未通过';
  }
  return lie;
}

function renderProbePill(lieEl, p) {
  if (!lieEl) return;
  const probe = ensurePill(lieEl, 'probe-pill', 'gate-pill probe-pill');
  const done = p.probe_done;
  const ready = p.probe_ready;
  if (done === null || done === undefined) { probe.hidden = true; return; }
  probe.hidden = false;
  if (!done) {
    probe.textContent = '探针 未检测';
    probe.className = 'gate-pill probe-pill is-idle';
    probe.title = '探针门禁:本次运行尚未执行电机可控性探针,拒绝起立。请先在「关节管理」执行探针';
  } else if (ready) {
    probe.textContent = '探针 通过';
    probe.className = 'gate-pill probe-pill is-ok';
    probe.title = '探针门禁(起立):16 台电机可控性探针全部通过';
  } else {
    probe.textContent = '探针 未通过';
    probe.className = 'gate-pill probe-pill is-bad';
    probe.title = '探针门禁未通过:存在无响应电机,拒绝起立。请在「关节管理」重新探针确认';
  }
}
addPostureListener(renderPosture);

// ── 进入 RL 前站立检测:点击「进入 RL」时先校验,未站稳则确认/拒绝 ──
/** 等待一次 check_standing_result 的 Promise(带超时) */
function requestCheckStanding(timeoutMs = 2500) {
  return new Promise((resolve) => {
    let done = false;
    const timer = setTimeout(() => { if (!done) { done = true; off(); resolve(null); } }, timeoutMs);
    const off = addCheckStandingListener((r) => {
      if (done) return;
      done = true; clearTimeout(timer); off(); resolve(r);
    });
    // 通过 command 通道触发后端 HandleCheckStanding
    publishBehaviorCommand('check_standing');
  });
}

// ── 置零页签:4 腿 × 4 关节电机选择矩阵(多选) + 实时位置标红 + 单/多电机置零 ──
const ZERO_MOTOR_LEGS = ['FL', 'FR', 'RL', 'RR'];
const ZERO_MOTOR_JOINTS = ['髋', '大腿', '小腿', '足轮'];
/** 位置偏差超过该值(rad)视为「不在零位」,格子标红 */
const ZERO_POS_TOLERANCE_RAD = 0.1;
const zeroMotorGrid = document.getElementById('zero-motor-grid');
const zeroMotorBtn = document.getElementById('zero-motor-btn');
const zeroSelectRedBtn = document.getElementById('zero-select-red-btn');
/** 已选电机 motor_id 集合(多选) */
const zeroSelectedMotors = new Set();
/** 电机格元素表:{ cell, posEl, motorId, shortLabel } */
const zeroMotorCells = [];
/** 已成功置零的电机集合(仅本会话有效;反馈帧驱动,见 addCommandFeedbackListener) */
const zeroedMotors = new Set();
/** 探针结果内存状态:motor_id → 'ok'|'fail'|'running'|'waiting'|null(未检测) */
const probeResults = new Map();

/** 重置本地探针状态:趴下后后端作废探针记录(RLFSMStatePassive::Enter 调
 *  invalidateProbeResults,posture 帧 probe_done 翻转为 false),内存 Map 与
 *  探针卡片须同步恢复"未检测",避免"胶囊灰、卡片仍可控 ✅"的新错位。 */
function resetLocalProbeResults() {
  for (let i = 0; i < 16; i++) {
    probeResults.delete(i);
    const rec = zeroMotorCells[i];
    if (!rec) continue;
    rec.cell.classList.remove('is-probe-ok', 'is-probe-fail', 'is-probe-running');
    rec.probeStateEl.textContent = '未检测';
  }
  const status = document.getElementById('probe-status');
  if (status) {
    status.textContent = '探针记录已失效(趴下后需重新探针)';
    status.className = 'probe-status';
  }
}

// 监听 posture 帧 probe_done 的 true→false 翻转(=后端趴下作废探针记录的瞬间),
// 仅在翻转时重置:探针进行中 valid 本就逐步写入,不受影响
let lastProbeDone = null;
addPostureListener((p) => {
  if (typeof p.probe_done !== 'boolean') return;
  if (lastProbeDone === true && p.probe_done === false) resetLocalProbeResults();
  lastProbeDone = p.probe_done;
});

// 回到「已趴下稳态」即清空安全起立步骤芯片——与后端 Passive::Enter 作废物语义对齐,
// 覆盖任何趴下入口(摇杆区/快捷栏/其他 ROS 客户端/手柄)。流程运行中(busy)起点本就
// 是趴姿(lie/probe 阶段),绝不能清;仅在芯片存在非 idle 态时操作,避免每 8Hz 刷 DOM。
addPostureListener((p) => {
  if (safeGetupBusy || !safeGetupStepsDirty) return;
  if (p && p.posture === 'lying' && p.settled) clearSafeGetupSteps();
});
/** 标记某电机置零成功:格子保持绿色选中框,再次点击可选择/取消不影响该标记 */
function markMotorZeroed(motorId) {
  if (!Number.isInteger(motorId) || motorId < 0 || motorId >= 16) return;
  zeroedMotors.add(motorId);
  const rec = zeroMotorCells[motorId];
  if (rec) {
    rec.cell.classList.add('zeroed');
    if (rec.zeroStateEl) rec.zeroStateEl.textContent = '置零 ✅';
  }
}

/** 标记电机置零失败(撤销成功状态,恢复"未置零") */
function markMotorNotZeroed(motorId) {
  if (!Number.isInteger(motorId) || motorId < 0 || motorId >= 16) return;
  zeroedMotors.delete(motorId);
  const rec = zeroMotorCells[motorId];
  if (rec) {
    rec.cell.classList.remove('zeroed');
    if (rec.zeroStateEl) rec.zeroStateEl.textContent = '未置零';
  }
}

/** 置零命令进行中时间戳(0=空闲);收到 zero_result 或兜底超时后清零 */
let zeroPendingAt = 0;
let zeroPendingTimer = null;

/** 结束置零等待态:解锁按钮(由 refreshZeroSelectionUi 按选择集最终决定可用态) */
function finishZeroPending() {
  zeroPendingAt = 0;
  if (zeroPendingTimer) { clearTimeout(zeroPendingTimer); zeroPendingTimer = null; }
  if (zeroMotorBtn) zeroMotorBtn.classList.remove('is-pressed');
}

/** 处理后端 zero_result JSON:真实成功/失败列表驱动格子与下一轮重试选择集 */
function handleZeroResult(jsonStr) {
  let r;
  try {
    r = JSON.parse(jsonStr);
  } catch (e) {
    showToast('置零回执解析失败,请按格子红绿状态人工确认', 'warn');
    finishZeroPending();
    refreshZeroSelectionUi();
    return;
  }
  const zeroed = Array.isArray(r.zeroed) ? r.zeroed.filter(Number.isInteger) : [];
  const failed = Array.isArray(r.failed) ? r.failed.filter(Number.isInteger) : [];

  zeroed.forEach((id) => markMotorZeroed(id));
  failed.forEach((id) => {
    // 重发后仍失败:撤销历史成功标记,等待下一轮重试
    markMotorNotZeroed(id);
  });
  // 选择集收敛:成功的移除(已完成),失败的保留/勾上——直接再点一次即只重试失败台,
  // 不必重新"全选标红"。这是消除"置零要反复点很多遍"的关键交互。
  zeroed.forEach((id) => zeroSelectedMotors.delete(id));
  failed.forEach((id) => {
    zeroSelectedMotors.add(id);
    zeroMotorCells[id]?.cell.classList.add('selected');
  });
  zeroMotorCells.forEach(({ cell }) => cell.classList.toggle('selected', zeroSelectedMotors.has(cell.dataset.motorId * 1)));

  if (failed.length === 0) {
    showToast(`置零成功:${zeroed.length} 个电机已反馈确认`, 'info');
  } else {
    showToast(`置零完成:成功 ${zeroed.length} 个,失败 ${failed.length} 个(#${failed.join(', #')}),已自动勾选,直接再点即可重试`, 'warn');
  }
  finishZeroPending();
  refreshZeroSelectionUi();
}

/** 读取某 motor_id 的实时位置(rad);无遥测时返回 undefined */
function zeroMotorPos(motorId) {
  const jointName = JOINT_MAPPING[currentModelId]?.[motorId];
  if (!jointName) return undefined;
  const pos = motorTelemetry.get(jointName)?.realPos;
  return Number.isFinite(pos) ? pos : undefined;
}

/** 刷新置零按钮可用态。
 *  可用条件 = ROS 在线 且 已选至少 1 个电机(离线/未选均禁用);
 *  applyRosUiState 的在线状态联动也会回调本函数,避免统一禁用逻辑覆盖选中态。 */
function refreshZeroSelectionUi() {
  if (!zeroMotorBtn) return;
  const n = zeroSelectedMotors.size;
  const online = (typeof getStatusSnapshot === 'function')
      && getStatusSnapshot().connectionStatus === 'online';
  // "探针选中"按钮与选中集联动(有选中即启用,无需 ROS 在线也可点)
  const probeSelectedBtn = document.getElementById('probe-selected-btn');
  if (probeSelectedBtn) probeSelectedBtn.disabled = n === 0;
  if (n === 0) {
    zeroMotorBtn.disabled = true;
    zeroMotorBtn.textContent = '置零选中';
    return;
  }
  zeroMotorBtn.disabled = !online;
  zeroMotorBtn.textContent = `置零选中 (${n})`;
}

if (zeroMotorGrid && zeroMotorBtn) {
  // 生成 16 电机选择矩阵:motor_id = 腿*4 + 关节(与后端 motor_map_ 顺序一致)
  // 合并置零选择 + 探针状态/按钮,一个格子同时承载两个功能
  ZERO_MOTOR_LEGS.forEach((leg, legIdx) => {
    ZERO_MOTOR_JOINTS.forEach((joint, jointIdx) => {
      const motorId = legIdx * 4 + jointIdx;
      const cell = document.createElement('div');
      cell.className = 'zero-motor-cell';
      cell.dataset.motorId = String(motorId);
      cell.title = `motor_id=${motorId}(ttycan${legIdx + 1} / CAN ID ${4 - jointIdx})`;
      // 名称行
      const nameEl = document.createElement('span');
      nameEl.className = 'zero-name';
      nameEl.textContent = `${leg}·${joint}`;
      const idEl = document.createElement('span');
      idEl.className = 'zero-mid';
      idEl.textContent = `m${motorId}`;
      // 位置行(置零用)
      const posEl = document.createElement('span');
      posEl.className = 'zero-pos';
      posEl.textContent = '—';
      // 探针行:探针状态 + 置零状态(无单台按钮,统一用工具栏"探针选中")
      const probeRow = document.createElement('div');
      probeRow.className = 'zero-probe-row';
      const probeStateEl = document.createElement('span');
      probeStateEl.className = 'zero-probe-state';
      probeStateEl.textContent = '未检测';
      const zeroStateEl = document.createElement('span');
      zeroStateEl.className = 'zero-zero-state';
      zeroStateEl.textContent = '未置零';
      probeRow.append(probeStateEl, zeroStateEl);
      cell.append(nameEl, idEl, posEl, probeRow);
      // 点击格子 = 选择(共享选择集,用于置零/探针)
      cell.addEventListener('click', () => {
        if (zeroSelectedMotors.has(motorId)) zeroSelectedMotors.delete(motorId);
        else zeroSelectedMotors.add(motorId);
        cell.classList.toggle('selected', zeroSelectedMotors.has(motorId));
        refreshZeroSelectionUi();
      });
      zeroMotorCells[motorId] = { cell, posEl, probeStateEl, probeBtn: null, zeroStateEl, motorId, shortLabel: `${leg}·${joint}` };
      // 从内存状态恢复探针结果(页面重建/切换页签后保持)
      const savedProbe = probeResults.get(motorId);
      if (savedProbe === 'ok') {
        cell.classList.add('is-probe-ok');
        probeStateEl.textContent = '可控 ✅';
      } else if (savedProbe === 'fail') {
        cell.classList.add('is-probe-fail');
        probeStateEl.textContent = '无响应 ❌';
      } else if (savedProbe === 'running' || savedProbe === 'waiting') {
        cell.classList.add('is-probe-running');
        probeStateEl.textContent = savedProbe === 'running' ? '探针中...' : '等待中...';
      }
      // 从内存状态恢复置零结果
      if (zeroedMotors.has(motorId)) {
        cell.classList.add('zeroed');
        zeroStateEl.textContent = '置零 ✅';
      }
      zeroMotorGrid.appendChild(cell);
    });
  });

  // 置零选中电机:一条 zero_motors 批量命令(后端四腿并行+逐台反馈确认+自动重发,
  // 16 台 ~0.3s);按钮等待 zero_result 回执解锁,失败台自动保留勾选可一键重试
  zeroMotorBtn.addEventListener('click', async () => {
    if (zeroSelectedMotors.size === 0 || zeroMotorBtn.disabled || zeroPendingAt) return;
    const ids = [...zeroSelectedMotors].sort((a, b) => a - b);
    const labels = ids.map((id) => zeroMotorCells[id]?.shortLabel || `#${id}`).join('、');
    const ok = await confirmDialog({
      title: `确认置零 ${ids.length} 个电机?`,
      message: `${labels}\n\n这些电机的当前位置将被写为零点!请确认机器人处于趴下/被动态(Passive)。`,
      confirmText: '置零', danger: true,
    });
    if (!ok) return;
    const sent = publishBehaviorCommand(`zero_motors ${ids.join(',')}`);
    if (!sent) {
      showToast('ROS 未连接,无法置零', 'warn');
      return;
    }
    showToast(`正在置零 ${ids.length} 个电机…`, 'info');
    zeroPendingAt = performance.now();
    zeroMotorBtn.disabled = true;
    zeroMotorBtn.classList.add('is-pressed');
    // 兜底:后端极端情况下无回执(WS 断链等),12s 后自动解锁(正常往返 <1s)
    if (zeroPendingTimer) clearTimeout(zeroPendingTimer);
    zeroPendingTimer = setTimeout(() => {
      if (zeroPendingAt) {
        finishZeroPending();
        refreshZeroSelectionUi();
        showToast('置零应答超时,请按格子红绿状态确认', 'warn');
      }
    }, 12000);
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
  zeroPanelReady = true;
}

/** 置零页实时刷新:位置数值 + 偏零标红(仅页签可见时,~5Hz 节流,由动画循环调用) */
let zeroPanelLastRefreshAt = 0;
function updateZeroPanel() {
  const view = document.getElementById('joint-control-view');
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

// ── 「卧姿校验」按钮(趴姿近零位检测,决定能否 getup) —— 移动控制快捷栏 + ROS 指令页 ──
const checkStandBtns = document.querySelectorAll('.check-stand-btn');
checkStandBtns.forEach((checkStandBtn) => {
  checkStandBtn.addEventListener('click', () => {
    const ok = publishCheckStand();
    if (ok) {
      showToast('正在执行卧姿校验…', 'info');
      checkStandBtn.classList.add('is-pressed');
      setTimeout(() => checkStandBtn.classList.remove('is-pressed'), 600);
    } else {
      showToast('ROS 未连接,无法校验', 'warn');
    }
  });
});
// 卧姿校验结果 → toast + console(结果即起立门禁记录,趴下后作废需重新校验)
addCheckStandListener((result) => {
  if (result.ready) {
    showToast('✅ 卧姿校验通过:电机零位达标,IMU 正常,已解锁起立(趴下后需重新校验)', 'info');
  } else {
    showToast(`⚠ 卧姿校验未通过:${result.reason}`, 'warn');
    console.warn('[check-stand] 卧姿校验未通过:', result.reason);
  }
});

// ── 「站姿校验」按钮(进入强化学习前的站姿校验,check_standing) —— 移动控制快捷栏 + ROS 指令页 ──
// 只校验、不触发任何动作;与点「进入 RL」时自动触发的校验走同一后端/结果通道
const checkStandingBtns = document.querySelectorAll('.check-standing-btn');
checkStandingBtns.forEach((checkStandingBtn) => {
  checkStandingBtn.addEventListener('click', async () => {
    if (checkStandingBtn.disabled) return;
    const online = (typeof getStatusSnapshot === 'function')
        && getStatusSnapshot().connectionStatus === 'online';
    if (!online) { showToast('ROS 未连接,无法校验', 'warn'); return; }
    checkStandingBtn.disabled = true;
    checkStandingBtn.classList.add('is-pressed');
    showToast('正在执行站姿校验(12 关节站姿+静止)…', 'info');
    const r = await requestCheckStanding(2500);
    checkStandingBtn.classList.remove('is-pressed');
    checkStandingBtn.disabled = false;
    if (r === null) {
      showToast('站姿校验超时(未收到结果)', 'warn');
    } else if (r.at_stand) {
      const errTxt = Number.isFinite(r.worst_err) ? `,最大偏差 ${r.worst_err.toFixed(3)}rad` : '';
      showToast(`✅ 站姿校验通过:关节就位、IMU 正常${errTxt},可以进入强化学习`, 'info');
    } else {
      showToast(`⚠ 站姿校验未通过:${r.reason || '未站在期望站姿'}`, 'warn');
      console.warn('[check-standing] 站姿校验未通过:', r);
    }
  });
});

// ── 摇杆区「安全起立」一键流程(后端编排,前端只发一条命令) ─────────────
// 命令: publishBehaviorCommand('safe_getup rl')
// 后端状态机依次执行(每个操作之间固定停顿 1000ms):
//   卧姿校验 →(1s)→ 探针 →(1s)→ 起立 →(1s)→ 站姿确认 →(1s)→ 进入强化学习
// 进度帧 notify(type="safe_getup"):
//   "start[ rl]" / "step <lie|probe|getup|stand|rl> <run|ok|fail|skip>"
//   / "done ok" / "done fail <step> <reason>"
// 成功/失败的中文提示由后端 success/error Notify 统一承担。
const SAFE_GETUP_STEPS = ['lie', 'probe', 'getup', 'stand', 'rl'];
const SAFE_GETUP_RUN_TEXT = {
  lie: '① 卧姿校验中…',
  probe: '② 探针检测中…',
  getup: '③ 起立中…',
  stand: '④ 站姿确认中…',
  rl: '⑤ 进入强化学习中…',
};
// 后端拒绝原因(英文)→ 中文;CAN 类原因后端本身就是中文,直接透传
const SAFE_GETUP_REJECT_ZH = [
  ['safe getup already running', '安全起立流程已在运行中,请勿重复触发'],
  ['already in locomotion, no need to get up', '当前已在强化学习行走状态,无需安全起立'],
  ['rl model is loading, please wait', '强化学习模型加载中,请稍候再试'],
  ['robot is getting down, wait until settled', '机器人正在趴下,请等动作完成后再试'],
  ['robot is getting up, wait until settled', '机器人正在起立,请等站稳后再试'],
  ['fsm not ready', '机器人状态机未就绪,请稍后再试'],
];
// 纯 UI 兜底:后端各阶段超时合计约 75s + 4 个 1s 间隔;85s 没收到 done
// 仅解锁按钮显示(后端流程仍按自身状态机继续/收尾,互不影响)
const SAFE_GETUP_UI_TIMEOUT_MS = 85000;
let safeGetupBusy = false;
let safeGetupBtn = null;
let safeGetupUiTimer = null;

/** 更新安全起立按钮内分段进度条状态:idle|run|ok|fail|skip */
let safeGetupStepsDirty = false;  // 分段是否存在任一非 idle 视觉态
function setSafeGetupStep(name, state) {
  document.querySelectorAll(`.safe-getup-track .sg-seg[data-step="${name}"]`).forEach((el) => {
    el.classList.remove('is-run', 'is-ok', 'is-fail', 'is-skip');
    if (state && state !== 'idle') el.classList.add('is-' + state);
  });
  if (state && state !== 'idle') safeGetupStepsDirty = true;
}

/**
 * 清空全部分段(回到未执行态)。趴下即作废:后端 Passive::Enter 已同步作废
 * 卧姿校验/探针门禁(posture 帧 lie_done/probe_done 回落,门禁胶囊转灰),
 * 按钮上的进度不能继续挂着全绿/全红,否则与真实门禁状态错位。
 */
function clearSafeGetupSteps() {
  SAFE_GETUP_STEPS.forEach((n) => setSafeGetupStep(n, 'idle'));
  safeGetupStepsDirty = false;
}

/** 解锁按钮/趴下键/文案,清 UI 兜底定时器(done 帧或超时都会调) */
function resetSafeGetupUi() {
  safeGetupBusy = false;
  safeGetupBtn = null;
  if (safeGetupUiTimer) { clearTimeout(safeGetupUiTimer); safeGetupUiTimer = null; }
  document.querySelectorAll('.safe-getup-btn').forEach((b) => {
    b.disabled = false;
    b.classList.remove('is-running');
    const l = b.querySelector('.safe-getup-label');
    if (l) l.textContent = '安全起立';
  });
  document.querySelectorAll('.safe-getdown-btn').forEach((b) => { b.disabled = false; });
}

// 进度帧驱动芯片/按钮文案(模块级常驻监听;页面中途载入也能从下一帧续显)
addNotifyListener((payload) => {
  if (!payload || payload.type !== 'safe_getup') return;
  const parts = String(payload.content || '').split(/\s+/);
  if (parts[0] === 'status') {
    // 页面刷新/重连后的状态回显: status active | status idle
    const isActive = parts[1] === 'active';
    if (isActive) {
      // 正在运行:锁定按钮,等待后续 step 帧更新进度
      safeGetupBusy = true;
      if (safeGetupBtn) {
        safeGetupBtn.disabled = true;
        safeGetupBtn.classList.add('is-running');
        const l = safeGetupBtn.querySelector('.safe-getup-label');
        if (l) l.textContent = '安全起立进行中…';
      }
      document.querySelectorAll('.safe-getdown-btn').forEach((b) => { b.disabled = true; });
    } else {
      // 未运行:解锁按钮(若步骤全 idle 则完全复位)
      safeGetupBusy = false;
      if (safeGetupBtn) {
        safeGetupBtn.disabled = false;
        safeGetupBtn.classList.remove('is-running');
        const l = safeGetupBtn.querySelector('.safe-getup-label');
        if (l) l.textContent = '安全起立';
      }
      document.querySelectorAll('.safe-getdown-btn').forEach((b) => { b.disabled = false; });
    }
  } else if (parts[0] === 'step' && parts[1] && parts[2]) {
    setSafeGetupStep(parts[1], parts[2]);
    if (parts[2] === 'run' && safeGetupBtn) {
      const l = safeGetupBtn.querySelector('.safe-getup-label');
      if (l && SAFE_GETUP_RUN_TEXT[parts[1]]) l.textContent = SAFE_GETUP_RUN_TEXT[parts[1]];
    }
  } else if (parts[0] === 'done') {
    // done ok / done fail:fail 芯片已由先行的 step fail 帧置好;
    // 成功/失败 toast 由后端 success/error Notify 弹出,这里只收 UI
    resetSafeGetupUi();
  }
});

/** 把 safe_getup 的英文拒绝原因翻成中文(匹配不到则原样返回) */
function translateSafeGetupReject(text) {
  let reason = text.replace(/^command rejected:?\s*safe_getup,?\s*/i, '');
  for (const [en, zh] of SAFE_GETUP_REJECT_ZH) {
    if (reason.includes(en)) return zh;
  }
  return reason || '后端拒绝了安全起立';
}

async function runSafeGetup(btn) {
  if (safeGetupBusy) return;
  const online = (typeof getStatusSnapshot === 'function')
      && getStatusSnapshot().connectionStatus === 'online';
  if (!online) { showToast('ROS 未连接,无法执行安全起立', 'warn'); return; }

  if (lastPostureFrame && lastPostureFrame.posture === 'locomotion') {
    showToast('当前已处于强化学习行走状态,无需安全起立(如需重走流程请先退出强化学习并趴下)', 'warn');
    return;
  }

  SAFE_GETUP_STEPS.forEach((n) => setSafeGetupStep(n, 'idle'));
  safeGetupStepsDirty = false;  // 已主动清空,等待新一轮 step 帧重新标脏
  safeGetupBusy = true;
  safeGetupBtn = btn;
  btn.disabled = true;
  btn.classList.add('is-running');
  const labelEl = btn.querySelector('.safe-getup-label');
  if (labelEl) labelEl.textContent = '安全起立启动中…';
  document.querySelectorAll('.safe-getdown-btn').forEach((b) => { b.disabled = true; });

  // 等后端受理(notify "start")或拒绝(feedback "command rejected: safe_getup, …");
  // 3s 无任何应答只解锁 UI(可能 WS 抖动,不能把按钮永久锁死)
  const ack = await new Promise((resolve) => {
    let settled = false;
    const offN = addNotifyListener((p) => {
      if (!settled && p && p.type === 'safe_getup'
          && String(p.content || '').startsWith('start')) {
        settled = true; cleanup(); resolve('start');
      }
    });
    const offF = addCommandFeedbackListener((text) => {
      if (!settled && typeof text === 'string'
          && /^command rejected:\s*safe_getup\b/i.test(text)) {
        settled = true; cleanup(); resolve(text);
      }
    });
    const timer = setTimeout(() => { settled = true; cleanup(); resolve(null); }, 3000);
    function cleanup() { clearTimeout(timer); offN(); offF(); }
    // 监听已先注册,再发这唯一一条命令(ros-bridge 对 safe_getup 走 single-shot)
    if (!publishBehaviorCommand('safe_getup rl')) {
      settled = true; cleanup(); resolve('sendfail');
    }
  });

  if (ack !== 'start') {
    resetSafeGetupUi();
    if (ack === null) {
      showToast('安全起立未收到后端受理确认,请检查连接后重试', 'warn');
    } else if (ack === 'sendfail') {
      showToast('安全起立指令发送失败(ROS 未连接),请重试', 'warn');
    } else {
      showToast(`⚠ ${translateSafeGetupReject(ack)}`, 'warn');
    }
    return;
  }

  // 已受理:后续只跟进度帧走;兜底定时器防止帧丢失导致按钮永久禁用
  if (safeGetupUiTimer) clearTimeout(safeGetupUiTimer);
  safeGetupUiTimer = setTimeout(() => {
    if (!safeGetupBusy) return;
    resetSafeGetupUi();
    showToast('安全起立进度同步超时,已重置按钮显示(机器人仍按后端流程执行)', 'warn');
  }, SAFE_GETUP_UI_TIMEOUT_MS);
}

document.querySelectorAll('.safe-getup-btn').forEach((btn) => {
  btn.addEventListener('click', () => { runSafeGetup(btn); });
});

// 摇杆区「趴下」按钮:与快捷栏 getdown 同命令,放在安全起立旁边便于一键收回;
// 安全起立流程运行中禁用,避免流程中打断
document.querySelectorAll('.safe-getdown-btn').forEach((btn) => {
  btn.addEventListener('click', () => {
    if (safeGetupBusy || btn.disabled) return;
    const online = (typeof getStatusSnapshot === 'function')
        && getStatusSnapshot().connectionStatus === 'online';
    if (!online) { showToast('ROS 未连接,无法趴下', 'warn'); return; }
    if (publishBehaviorCommand('getdown')) {
      // 趴下即作废安全起立进度:芯片立即清空、按钮解锁(与快捷栏趴下入口同源)
      clearSafeGetupSteps();
      resetSafeGetupUi();
      showToast('正在趴下…', 'info');
      btn.classList.add('is-pressed');
      setTimeout(() => btn.classList.remove('is-pressed'), 600);
    }
  });
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
// 一级分组按钮事件：切到分组的第一个 tab
document.querySelectorAll('[data-tab-group]').forEach((grpBtn) => {
  grpBtn.addEventListener('click', () => {
    const g = grpBtn.dataset.tabGroup;
    const firstTab = TAB_GROUPS_MAP[g]?.[0];
    if (firstTab) setControlTab(firstTab);
  });
});
// 相机页签(先于首次 setControlTab 初始化,使持久化页签为 camera 时可见性立即生效)
initCameraView();
// Livox 雷达页签同理
initLidarView();
// 实时日志页签 + 系统管理页签(终端/WebFTP/电源/HR 日志开关)
initLogView();
initSysAdmin();
initBattery();

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
    // 左杆速度锁定中:vx/vy 保持锁定值,不参与衰减
    if (!joy.cruiseLockedLeft) {
      const t = 1 - Math.min(1, (nowMs - joy.decayLeftStartAt) / dur);
      joy.vx = r2((joy.decayLeftVx || 0) * t);
      joy.vy = r2((joy.decayLeftVy || 0) * t);
      active = true;
      if (t <= 0) { joy.vx = 0; joy.vy = 0; joy.decayLeftStartAt = 0; }
    } else {
      joy.decayLeftStartAt = 0;
    }
  }
  if (joy.decayRightStartAt) {
    const t = 1 - Math.min(1, (nowMs - joy.decayRightStartAt) / dur);
    // 右杆转向/高度锁定中:ω 与 Z 都保持锁定值,不参与衰减
    if (!joy.cruiseLockedRight) joy.omega = r2((joy.decayRightOmega || 0) * t);
    if (!joy.cruiseLockedRight) applyJoystickLift(r2((joy.decayRightLift || 0) * t));
    active = true;
    if (t <= 0) {
      if (!joy.cruiseLockedRight) { joy.omega = 0; applyJoystickLift(0); }
      joy.decayRightStartAt = 0;
    }
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
// 安全网:窗口失焦(alt-tab/切屏)时 keyup 会丢失,keep-alive 会持续发布按下的速度,
// 机器人将在无人操作时自己走动。失焦立即归零(尾帧逻辑由 ros-bridge 处理)。
window.addEventListener('blur', () => {
  if (activeControlTab === 'movement') setChassisCommand(0, 0, 0);
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
      updateTwistReadout();  // 量程变了,方向条按新上限重新缩放
    });
  });
  document.querySelectorAll('.joy-ang-slider').forEach((slider) => {
    slider.addEventListener('input', () => {
      setJoystickMaxAngular(Number(slider.value));
      refreshSpeedLimitUi();
      updateTwistReadout();
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
