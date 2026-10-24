/**
 * 电池页签(battery):通过 web_server 的蓝牙 BLE 线程读取 BMS 数据。
 *
 * 数据来源:WS 出站帧
 *   {"type":"bms_status","data":{online,msg}}
 *   {"type":"bms_data","data":{online,msg,ts,soc,soh,voltage,current,remain_ah,
 *                              fcc_ah,dc_ah,cycles,mos,t1,t2,t3,alarm,pack,
 *                              cells,fw,cell_mv:[...]}}
 * 控制:蓝牙线程后台常驻连接;进入页签仅发 "bms info" 拉取一次最新快照。
 *       「蓝牙」开关按钮下发 "bms on" / "bms off" 启停后端 BLE 轮询。
 */
import { publishRaw, addRawMessageListener, addStatusListener } from './ros-bridge.js?v=2026102001';

const state = {
  inited: false,
  visible: false,
  wsOnline: false,
  online: false,
  power: true,  // 蓝牙开关状态(后端 bms 线程是否启用)
  data: null,
  statusText: '未连接',
};

const $ = (id) => document.getElementById(id);

// 充放静判定:协议口径正=充电/负=放电,±0.3A 死区滤零漂
const IDLE_A = 0.3;
function chargeState(current) {
  const i = Number(current);
  if (!Number.isFinite(i)) return null;
  if (i >= IDLE_A) return { key: 'chg', label: '充电中' };
  if (i <= -IDLE_A) return { key: 'dsg', label: '放电中' };
  return { key: 'idle', label: '静止' };
}

function num(v, d = 1) {
  return v === null || v === undefined || Number.isNaN(Number(v))
    ? '--'
    : Number(v).toFixed(d);
}

function metricCards(d) {
  const c = [];
  const push = (label, val, unit, cls) => c.push({ label, val, unit: unit || '', cls: cls || '' });
  push('SOC', d.soc >= 0 ? String(d.soc) : '--', '%', 'bat-hl');
  push('SOH', d.soh >= 0 ? String(d.soh) : '--', '%');
  push('总电压', num(d.voltage, 2), 'V', 'bat-hl');
  push('总电流', (d.current >= 0 ? '+' : '') + num(d.current, 2), 'A');
  const cs = chargeState(d.current);
  push('工作状态', cs ? cs.label : '--', '', cs && cs.key === 'chg' ? 'bat-hl' : '');
  push('剩余容量', num(d.remain_ah, 2), 'Ah');
  push('循环次数', d.cycles >= 0 ? String(d.cycles) : '--', '次');
  push('满充容量', num(d.fcc_ah, 2), 'Ah');
  push('设计容量', num(d.dc_ah, 2), 'Ah');
  push('MOS 温度', num(d.mos, 1), '°C');
  push('电池温度1', num(d.t1, 1), '°C');
  push('电池温度2', num(d.t2, 1), '°C');
  push('电池温度3', num(d.t3, 1), '°C');
  const alarm = Number(d.alarm || 0);
  push('报警状态', alarm ? '0x' + alarm.toString(16).toUpperCase() : '正常',
       alarm ? '异常' : '', alarm ? 'bat-warn' : '');
  push('固件版本', d.fw || '--', '');
  return c;
}

function renderMetrics() {
  const box = $('battery-metrics');
  if (!box) return;
  const d = state.data;
  if (!d) {
    box.innerHTML = '<div class="bat-empty">暂无数据</div>';
    return;
  }
  box.innerHTML = metricCards(d)
    .map(
      (m) => `<div class="bat-card ${m.cls}"><div class="bat-label">${m.label}</div>
        <div class="bat-value">${m.val}<span class="bat-unit">${m.unit}</span></div></div>`
    )
    .join('');
}

function renderCells() {
  const box = $('battery-cells');
  const sum = $('battery-cell-summary');
  if (!box) return;
  const d = state.data;
  const mv = (d && d.cell_mv) || [];
  if (!mv.length) {
    box.innerHTML = '<div class="bat-empty">暂无单体电压</div>';
    if (sum) sum.textContent = '';
    return;
  }
  const mx = Math.max(...mv);
  const mn = Math.min(...mv);
  const spread = mx - mn;
  box.innerHTML = mv
    .map((v, i) => {
      // 条形为绝对电量刻度(单体工作区间 3.0~4.2V):健康电池压差仅几 mV,
      // 若按"相对压差"缩放(最低格 12% / 最高格 100%),几乎等压的格子条长会差数倍,
      // 造成电量悬殊的错觉;压差信息改由 is-max/is-min 配色 + 摘要行数值表达
      const pct = Math.max(0, Math.min(100, ((v - 3000) / 1200) * 100));
      let cls = '';
      if (v === mx) cls = 'is-max';
      else if (v === mn) cls = 'is-min';
      return `<div class="bat-cell ${cls}"><div class="bat-cell-head">
          <span class="bat-cell-idx">#${i + 1}</span>
          <span class="bat-cell-mv">${v}<i>mV</i></span></div>
        <div class="bat-cell-bar"><span style="width:${pct.toFixed(1)}%"></span></div></div>`;
    })
    .join('');
  if (sum) sum.textContent = `最高 ${mx} mV · 最低 ${mn} mV · 压差 ${spread} mV`;
}

function renderStatus() {
  const pill = $('battery-status');
  if (pill) {
    const ok = state.online;
    pill.textContent = state.statusText || (ok ? '已连接' : '未连接');
    pill.className = 'ros-status-pill ' + (ok ? 'ros-online' : 'ros-offline');
  }
  const btn = $('battery-power');
  if (btn) {
    btn.textContent = state.power ? '蓝牙:开' : '蓝牙:关';
    btn.classList.toggle('is-off', !state.power);
  }
}

// 后端状态文本 → 蓝牙开关状态("已停止" 表示 bms off)
function powerFromMsg(msg) {
  return typeof msg !== 'string' ? state.power : !msg.includes('已停止');
}

// 低电量提醒:≤20% 触发一次 toast;回升到 25% 以上重新布防(回差防抖)。
// BMS 断连/重连不解除布防,避免低电量时反复重连导致重复弹窗。
const LOW_SOC_WARN = 20;
const LOW_SOC_REARM = 25;
let lowSocWarned = false;
function lowSocNotify(soc) {
  window.dispatchEvent(new CustomEvent('app-toast', {
    detail: { message: `电池电量低（剩余 ${soc}%），请及时充电`, type: 'warn' },
  }));
}

// 移动控制页姿态栏的电量徽标(充放静状态已融合:充电显 ⚡、放电/静止显 🔋,
// 由全局 bms 帧驱动,不依赖电池页签可见;悬停 title 看完整充放静状态)
function updateMoveSoc(soc, online, current) {
  const el = $('move-batt-soc');
  const ok = online && typeof soc === 'number' && soc >= 0;
  const cs = online ? chargeState(current) : null;
  const icon = cs && cs.key === 'chg' ? '⚡' : '🔋';
  if (el) {
    el.textContent = ok ? `${icon} ${soc}%` : '🔋 --%';
    el.className = 'move-batt-soc' + (ok
      ? (soc <= 20 ? ' is-low' : soc <= 50 ? ' is-mid' : ' is-high')
      : ' is-na');
    el.title = ok && cs ? `BMS 剩余电量 ${soc}% · ${cs.label}` : 'BMS 剩余电量';
  }
  // 低电量单次提醒(跨阈值触发,20→25 回差解除)
  if (ok) {
    if (soc <= LOW_SOC_WARN && !lowSocWarned) {
      lowSocWarned = true;
      lowSocNotify(soc);
    } else if (soc >= LOW_SOC_REARM && lowSocWarned) {
      lowSocWarned = false;
    }
  }
}

function renderAll() {
  renderStatus();
  renderMetrics();
  renderCells();
}

function requestInfo() {
  publishRaw({ type: 'command', data: 'bms info' });
}

export function setBatteryVisible(visible) {
  state.visible = !!visible;
  if (!state.inited) return;
  if (visible && state.wsOnline) {
    // 进入页签:蓝牙后台常驻,这里只主动拉取一次最新快照
    requestInfo();
  }
  // 离开页签不做任何操作:连接在后台保持,再次进入立即可用
}

export function initBattery() {
  if (state.inited) return;
  if (!$('battery-control-view')) return;
  state.inited = true;

  $('battery-refresh')?.addEventListener('click', () => {
    if (state.wsOnline) publishRaw({ type: 'command', data: 'bms info' });
  });

  // 蓝牙开关:开启/关闭后端 BLE 轮询线程(命令 bms on / bms off)
  $('battery-power')?.addEventListener('click', () => {
    if (!state.wsOnline) return;
    const next = !state.power;
    publishRaw({ type: 'command', data: next ? 'bms on' : 'bms off' });
    state.power = next;
    if (!next) {
      state.online = false;
      state.data = null;
      state.statusText = '已停止';
    } else {
      state.statusText = '正在连接…';
    }
    renderAll();
  });

  // 低电量阈值:应用 / 查询
  const thrSoc = $('bms-thr-soc');
  const thrVolt = $('bms-thr-voltage');
  const thrInterval = $('bms-thr-interval');
  const thrStatus = $('battery-threshold-status');
  $('bms-thr-apply')?.addEventListener('click', () => {
    if (!state.wsOnline) return;
    const soc = parseFloat(thrSoc.value);
    const volt = parseFloat(thrVolt.value);
    const iv = parseInt(thrInterval.value, 10);
    if (!Number.isFinite(soc) || soc < 0 || soc > 100) {
      if (thrStatus) thrStatus.textContent = 'SOC 阈值需在 0~100 之间';
      return;
    }
    if (!Number.isFinite(volt) || volt <= 0 || volt > 60) {
      if (thrStatus) thrStatus.textContent = '电压阈值需在 0~60V 之间';
      return;
    }
    if (!Number.isFinite(iv) || iv < 1 || iv > 3600) {
      if (thrStatus) thrStatus.textContent = '提示周期需在 1~3600 秒之间';
      return;
    }
    publishRaw({ type: 'command', data: `bms_threshold soc ${soc}` });
    setTimeout(() => publishRaw({ type: 'command', data: `bms_threshold voltage ${volt}` }), 200);
    setTimeout(() => publishRaw({ type: 'command', data: `bms_threshold interval ${iv}` }), 400);
    if (thrStatus) thrStatus.textContent = `已下发: SOC<${soc}% / 电压<${volt}V / 周期${iv}秒`;
  });
  $('bms-thr-query')?.addEventListener('click', () => {
    if (state.wsOnline) publishRaw({ type: 'command', data: 'bms_threshold' });
  });

  addRawMessageListener((frame) => {
    if (frame.type === 'bms_data' && frame.data) {
      state.online = !!frame.data.online;
      state.power = powerFromMsg(frame.data.msg);
      state.statusText = frame.data.msg || (state.online ? '已连接' : '未连接');
      state.data = state.online ? frame.data : null;  // 离线(含 bms off)不显示陈旧值
      updateMoveSoc(frame.data.soc, state.online, frame.data.current);
      renderAll();
    } else if (frame.type === 'bms_status' && frame.data) {
      state.online = !!frame.data.online;
      state.power = powerFromMsg(frame.data.msg);
      state.statusText = frame.data.msg || '';
      if (!state.online) state.data = null;
      updateMoveSoc(null, false);
      renderAll();
    } else if (frame.type === 'feedback' && typeof frame.data === 'string') {
      // 处理 bms_threshold 反馈,更新阈值显示
      const t = frame.data;
      if (t.startsWith('bms_threshold')) {
        // 格式: "bms_threshold soc=15% voltage=20V interval=10s"
        const socM = t.match(/soc=([\d.]+)/);
        const voltM = t.match(/voltage=([\d.]+)/);
        const ivM = t.match(/interval=([\d.]+)/);
        if (socM && thrSoc) thrSoc.value = socM[1];
        if (voltM && thrVolt) thrVolt.value = voltM[1];
        if (ivM && thrInterval) thrInterval.value = ivM[1];
        if (thrStatus) thrStatus.textContent = t;
      }
    }
  });

  addStatusListener((s) => {
    const was = state.wsOnline;
    state.wsOnline = s.connectionStatus === 'online';
    // WS 重连后,若页签可见则拉取一次最新快照(蓝牙连接在后台常驻)
    if (state.wsOnline && !was && state.visible) requestInfo();
    renderStatus();
  });

  renderAll();
}
