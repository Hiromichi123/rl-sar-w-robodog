/**
 * 实时程序日志页签(logview)
 *
 * 复用 ros-bridge 的共享 WebSocket,协议:
 *   页面 → 服务器: {"type":"log_subscribe"}                 纯实时订阅(不回放历史)
 *                  {"type":"log_unsubscribe"}
 *                  {"type":"log_history","before":<epoch>,"n":200}  向上翻页时按需拉更早历史
 *   服务器 → 页面: {"type":"log","time":<epoch秒>,"data":"一行"}    实时帧(time=0 表示无时间)
 *                  {"type":"log_history","n":N,"lines":[{time,data},...]}  历史响应(更旧在前)
 *                  {"type":"log_status","data":"..."}
 *
 * 能力: 实时跟随(不预载历史)/向上滚动到顶部自动加载更早日志/暂停/清屏/
 *       关键词过滤/自动滚动/行数上限(5000 行,超出丢头部)。
 * 仅在页签可见且 WS 在线时订阅,切走自动退订(服务端按引用计数关闭 journalctl -f)。
 */
import {
  publishRaw, addRawMessageListener, addStatusListener,
} from './ros-bridge.js?v=2026102001';

const MAX_LINES = 5000;
const HISTORY_BATCH = 200;    // 每次向上翻页拉取的行数
const NEAR_BOTTOM_PX = 30;    // 距底部多少像素内视为"正在跟随"

// 版本标记:验证浏览器是否加载到最新模块(缓存排查用)
window.__LOGVIEW_V = 7;

let lastScrollTop = 0;        // scroll 事件去抖:区分用户上翻与程序化滚动

const state = {
  visible: false,
  subscribed: false,
  paused: false,
  autoscroll: true,
  filter: '',
  lines: [],            // {time, text, el}
  online: false,
  inited: false,
  historyLoading: false,
  noMoreHistory: false,
};

let els = {};

function fmtTime(ts) {
  if (!(ts > 0)) return '--:--:--';
  const d = new Date(ts * 1000);
  const p = (x) => String(x).padStart(2, '0');
  return `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`;
}

function renderLine(li) {
  const div = document.createElement('div');
  div.className = 'logview-line';
  const t = document.createElement('span');
  t.className = 'logview-time';
  t.textContent = fmtTime(li.time);
  const m = document.createElement('span');
  m.className = 'logview-text';
  m.textContent = li.text;
  div.append(t, m);
  li.el = div;
  return div;
}

function updateCount() {
  els.count.textContent = `${els.screen.childElementCount}/${state.lines.length} 行`;
}

/** 全量重建(清屏/改过滤词时);高频追加走 appendLine 增量路径 */
function rebuildScreen() {
  const frag = document.createDocumentFragment();
  for (const li of state.lines) {
    if (!li.el) li.el = renderLine(li);
    const show = !state.filter || li.text.includes(state.filter);
    if (li.el.hidden !== !show) li.el.hidden = !show;
    if (show) frag.appendChild(li.el);
  }
  els.screen.textContent = '';
  els.screen.appendChild(frag);
  updateCount();
  if (state.autoscroll) els.screen.scrollTop = els.screen.scrollHeight;
}

function isNearBottom() {
  return els.screen.scrollTop + els.screen.clientHeight >= els.screen.scrollHeight - NEAR_BOTTOM_PX;
}

function appendLine(time, text) {
  // 暂停跟随: 仍缓存(便于恢复后回看),只是不渲染
  const li = { time, text, el: null };
  state.lines.push(li);
  if (state.lines.length > MAX_LINES) {
    const drop = state.lines.splice(0, state.lines.length - MAX_LINES);
    drop.forEach((d) => { if (d.el && d.el.parentNode) d.el.parentNode.removeChild(d.el); });
  }
  const show = !state.filter || text.includes(state.filter);
  if (!state.paused && show) {
    // 在 DOM 变更前判断用户是否正看着底部(内容不足一屏时恒为 true)
    const wasNearBottom = isNearBottom();
    li.el = renderLine(li);
    els.screen.appendChild(li.el);
    // DOM 节点也设上限,避免长时间挂载巨量节点
    while (els.screen.childElementCount > MAX_LINES) els.screen.removeChild(els.screen.firstChild);
    if (state.autoscroll && wasNearBottom) els.screen.scrollTop = els.screen.scrollHeight;
  }
  updateCount();
}

/** 向上翻页: 拉取比当前首行更早的历史,前插;用户本来就在顶部,插完后仍留在顶部看更旧日志 */
function prependHistory(lines) {
  if (!lines.length) return;
  const initialLoad = !!state._initialLoad;
  const wasTop = els.screen.scrollTop <= 2;
  const frag = document.createDocumentFragment();
  const items = [];
  for (const h of lines) {
    const li = { time: h.time, text: h.data, el: null };
    li.el = renderLine(li);
    const show = !state.filter || li.text.includes(state.filter);
    li.el.hidden = !show;
    if (show) frag.appendChild(li.el);
    items.push(li);
  }
  state.lines = items.concat(state.lines);
  // 超上限时从尾部(最新侧)丢弃,保住用户正在回看的历史区
  if (state.lines.length > MAX_LINES) {
    const drop = state.lines.splice(MAX_LINES);
    drop.forEach((d) => { if (d.el && d.el.parentNode) d.el.parentNode.removeChild(d.el); });
  }
  els.screen.insertBefore(frag, els.screen.firstChild);
  // 程序化滚动:屏蔽随后的 scroll 事件,防止连续触发历史加载
  state._suppressScroll = true;
  if (initialLoad) {
    // 初始加载:滚到底部(最新日志),符合"打开后默认在最后一行"
    els.screen.scrollTop = els.screen.scrollHeight;
    state._initialLoad = false;
  } else if (wasTop) {
    els.screen.scrollTop = 0;
  }
  lastScrollTop = els.screen.scrollTop;
  updateCount();
}

function requestHistory() {
  if (state.historyLoading || state.noMoreHistory || !state.subscribed) return;
  // 找到首条带时间的行作为 before 基准
  const first = state.lines.find((li) => li.time > 0);
  if (!first) return;
  state.historyLoading = true;
  els.status.textContent = '加载更早日志…';
  publishRaw({ type: 'log_history', before: first.time, n: HISTORY_BATCH });
}

/** 订阅后立即拉取最近一批历史日志(before=当前时间),填充到实时流前面 */
const INITIAL_HISTORY_N = 500;
function requestInitialHistory() {
  if (state.historyLoading || !state.subscribed) return;
  state.historyLoading = true;
  state.noMoreHistory = false;
  state._initialLoad = true;   // 标记本次为初始加载,prependHistory 后滚到底部
  els.status.textContent = '加载历史日志…';
  // before 用当前时间,拉取当前时刻之前的最近 N 行
  publishRaw({ type: 'log_history', before: Date.now() / 1000, n: INITIAL_HISTORY_N });
}

function sendSubscribe() {
  const ok = publishRaw({ type: 'log_subscribe' });
  state.subscribed = ok;
  if (ok) {
    els.status.textContent = '已订阅 · 加载历史中…';
    els.status.className = 'logview-status is-on';
    // 订阅实时流后,立即拉取最近一批历史日志,让用户看到"之前 + 实时"的完整日志
    setTimeout(requestInitialHistory, 50);
  }
  return ok;
}

function sendUnsubscribe() {
  if (!state.subscribed) return;
  publishRaw({ type: 'log_unsubscribe' });
  state.subscribed = false;
  els.status.textContent = '已停止订阅';
  els.status.className = 'logview-status';
}

function refreshDesired() {
  const want = state.visible && state.online;
  if (want && !state.subscribed) sendSubscribe();
  else if (!want && state.subscribed) sendUnsubscribe();
}

export function setLogViewVisible(visible) {
  state.visible = !!visible;
  if (!state.inited) return;
  refreshDesired();
}

export function initLogView() {
  if (state.inited) return;
  els.screen = document.querySelector('#logview-screen');
  if (!els.screen) return;  // 页面无此容器(非目标页)
  state.inited = true;
  els.status = document.querySelector('#logview-status');
  els.count = document.querySelector('#logview-count');
  els.btnPause = document.querySelector('#logview-pause');
  els.btnClear = document.querySelector('#logview-clear');
  els.btnSave = document.querySelector('#logview-save');
  els.chkAuto = document.querySelector('#logview-autoscroll');
  els.inputFilter = document.querySelector('#logview-filter');
  els.btnReconnect = document.querySelector('#logview-reconnect');

  addRawMessageListener((frame) => {
    if (!state.visible) return;
    if (frame.type === 'log' && typeof frame.data === 'string') {
      appendLine(Number(frame.time) || 0, frame.data);
    } else if (frame.type === 'log_history' && Array.isArray(frame.lines)) {
      state.historyLoading = false;
      if (frame.lines.length === 0) {
        state.noMoreHistory = true;
        els.status.textContent = '已是最早日志';
      } else {
        prependHistory(frame.lines);
        els.status.textContent = '已订阅 · 实时跟随';
      }
    } else if (frame.type === 'log_status' && typeof frame.data === 'string') {
      if (els.status) {
        els.status.textContent = frame.data;
        els.status.className = 'logview-status';
      }
    }
  });

  addStatusListener((s) => {
    const wasOnline = state.online;
    state.online = s.connectionStatus === 'online';
    if (!state.online) {
      state.subscribed = false;
      state.historyLoading = false;   // 防离线期间历史响应丢失后卡死加载态
      if (els.status) { els.status.textContent = 'WS 离线'; els.status.className = 'logview-status'; }
    } else if (!wasOnline) {
      refreshDesired();   // 重连成功后自动重新订阅
    }
  });

  // 滚动到顶部 → 懒加载更早历史;
  // 用户向上滚离底部 → 自动暂停跟随(否则新日志会把回看视口顶走),滚回底部恢复跟随
  els.screen.addEventListener('scroll', () => {
    const st = els.screen.scrollTop;
    if (state._suppressScroll) { state._suppressScroll = false; lastScrollTop = st; return; }
    const nearBottom = st + els.screen.clientHeight >= els.screen.scrollHeight - NEAR_BOTTOM_PX;
    if (state.autoscroll !== nearBottom) {
      state.autoscroll = nearBottom;
      if (els.chkAuto) els.chkAuto.checked = nearBottom;
    }
    // 仅在"向上滚动且触顶"时加载历史;程序化 scrollTop=0 已被 _suppressScroll 屏蔽
    if (st < lastScrollTop && st <= 2) requestHistory();
    lastScrollTop = st;
  }, { passive: true });

  els.btnPause?.addEventListener('click', () => {
    state.paused = !state.paused;
    els.btnPause.textContent = state.paused ? '▶ 继续' : '⏸ 暂停';
    els.btnPause.classList.toggle('is-pressed', state.paused);
    if (!state.paused) rebuildScreen();
  });

  els.btnClear?.addEventListener('click', () => {
    state.lines.length = 0;
    state.noMoreHistory = false;
    els.screen.textContent = '';
    els.count.textContent = '0 行';
  });

  els.chkAuto?.addEventListener('change', () => {
    state.autoscroll = !!els.chkAuto.checked;
    if (state.autoscroll) els.screen.scrollTop = els.screen.scrollHeight;
  });

  els.inputFilter?.addEventListener('input', () => {
    state.filter = els.inputFilter.value.trim();
    rebuildScreen();
  });

  /** 保存日志: 将当前所有日志行导出为 txt 文件 */
  els.btnSave?.addEventListener('click', () => {
    if (!state.lines.length) {
      els.status.textContent = '无日志可保存';
      return;
    }
    const now = new Date();
    const stamp = `${now.getFullYear()}${String(now.getMonth() + 1).padStart(2, '0')}${String(now.getDate()).padStart(2, '0')}_${String(now.getHours()).padStart(2, '0')}${String(now.getMinutes()).padStart(2, '0')}${String(now.getSeconds()).padStart(2, '0')}`;
    const lines = state.lines.map((li) => `${fmtTime(li.time)}  ${li.text}`);
    const blob = new Blob([lines.join('\n')], { type: 'text/plain;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `main_controller_log_${stamp}.txt`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    els.status.textContent = `已保存 ${state.lines.length} 行日志`;
  });

  els.btnReconnect?.addEventListener('click', () => {
    if (state.online) { sendUnsubscribe(); setTimeout(sendSubscribe, 100); }
  });
}
