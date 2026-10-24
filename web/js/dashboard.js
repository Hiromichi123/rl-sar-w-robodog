/**
 * 数据看板模块:以纯数字形式一页展示电机与 IMU 数据。
 *
 *  - 电机区:按 JOINT_MAPPING(真机电机全量映射)逐行展示
 *    实际 / 目标 / 偏差 / 速度 四列数字;模型无映射时退回滑块控制列表;
 *  - IMU 区:姿态(Roll/Pitch/Yaw,°)、四元数、加速度(m/s²)、
 *    角速度(rad/s)、位置 NED(m)、机体系速度(m/s)共 19 项数值芯片;
 *  - 数据直接读 state.js 的 jointStates 与 shared.imu,
 *    仅在页签可见时以 ~10fps 节流刷新,不参与渲染循环热路径。
 *
 * 页签切换或模型切换后由 rebuildDashboard() 重建行/芯片
 * (内部按关节名签名自动检测变化)。
 */
import { jointStates, visibleControls, currentModelId, shared } from './state.js?v=1';
import { labelForJoint } from './joints.js?v=1';
import { JOINT_MAPPING, motorTelemetry } from './ros-bridge.js?v=2026102001';
import { isParamAlarmed } from './alarm.js?v=2026102001';

/** 刷新节流间隔(ms),约 10fps */
const REFRESH_INTERVAL_MS = 100;

/** 数据新鲜度阈值(ms):超过该时间未收到新数据即标红 */
const STALE_AFTER_MS = 1000;

/** 电机温度色阶阈值(°C):≥60 预警(橙),≥75 过热(红) */
const MOTOR_TEMP_WARN = 60;
const MOTOR_TEMP_DANGER = 75;

/** 电机反馈年龄色阶阈值(ms):与后端断线检测一致,≥500 预警(橙),≥1000 断线(红) */
const MOTOR_AGE_WARN_MS = 500;
const MOTOR_AGE_DANGER_MS = 1000;

/** 数值格短标签:PC 表头已含同名列,此处写入 data-label 供移动端卡片式
 *  布局(表头隐藏后)用 CSS ::before 回显字段名。 */
const CELL_LABELS = {
  targetPos: '目标位置',
  realPos: '实际位置',
  targetVel: '目标速度',
  realVel: '实际速度',
  targetTau: '目标力矩',
  realTau: '实际力矩',
  kp: 'kp 刚度',
  kd: 'kd 阻尼',
  temp: '温度 °C',
  age: '更新',
};

// ── DOM 引用 ─────────────────────────────────────────────
const viewEl = document.querySelector('#dashboard-control-view');
const motorGridEl = document.querySelector('#dashboard-motor-grid');
const imuGridEl = document.querySelector('#dashboard-imu-grid');
// 视图切换三件套仅移动端(mobile.html)存在;PC 端为 null → 相关逻辑整体跳过
const dashBarEl = document.querySelector('#dash-view-bar');
const dashChipsEl = document.querySelector('#dash-field-chips');
const dashFieldGridEl = document.querySelector('#dashboard-field-grid');
const dashScrollEl = document.querySelector('#dashboard-control-view .dash-scroll');

// ── 电机行缓存 ───────────────────────────────────────────
/** @type {{signature: string, rows: Array<{name: string, rowEl: HTMLElement, cells: Object<string, HTMLElement>}>}} */
const motor = { signature: '', rows: [] };

// ── 看板视图(按字段 / 按电机)─────────────────────────────
// 按字段:4×4 网格(行=FL/FR/RL/RR,列=髋/大腿/小腿/轮),一次只显示一个指标的
//   全 16 台电机值 —— 解决卡片式布局把同一字段拆散到各卡片、无法逐列比对「相同项」
//   的问题;排布与「关节管理」的置零矩阵同构。
// 按电机:每台一张卡片,看单机全貌(原卡片布局保持不变)。
const DASH_VIEW_KEY = 'jxg_dash_view';
const DASH_FIELD_KEY = 'jxg_dash_field';
/** 指标顺序 = 表格列顺序;文案复用 CELL_LABELS */
const FIELD_ORDER = ['targetPos', 'realPos', 'targetVel', 'realVel', 'targetTau', 'realTau', 'kp', 'kd', 'temp', 'age'];
/** 网格列:关节类型 → 表头短名 */
const FIELD_JOINTS = [['hip', '髋'], ['thigh', '大腿'], ['calf', '小腿'], ['foot', '轮']];
/** 行值格 → 字段网格值格需同步的状态类(色阶/报警/等待) */
const FIELD_SYNC_FLAGS = ['is-warn', 'is-danger', 'is-waiting', 'is-alarmed'];

/** @type {{view: 'field'|'motor', field: string, cells: Map<string, HTMLElement>, signature: string}} */
const dashView = { view: 'field', field: 'realPos', cells: new Map(), signature: '' };
let dashViewBound = false;

/** 视图切换 / 指标选择持久化(存储不可用则仅本次会话生效) */
function persistDashView() {
  try {
    localStorage.setItem(DASH_VIEW_KEY, dashView.view);
    localStorage.setItem(DASH_FIELD_KEY, dashView.field);
  } catch { /* 隐私模式/存储禁用:忽略 */ }
}

/** 绑定切换与指标按钮(仅一次) */
function initDashViewOnce() {
  if (dashViewBound || !dashBarEl) return;
  dashViewBound = true;
  try {
    if (localStorage.getItem(DASH_VIEW_KEY) === 'motor') dashView.view = 'motor';
    const f = localStorage.getItem(DASH_FIELD_KEY);
    if (f && FIELD_ORDER.includes(f)) dashView.field = f;
  } catch { /* 读取失败:保持默认 */ }
  dashBarEl.addEventListener('click', (e) => {
    const viewBtn = e.target.closest('[data-dash-view]');
    if (viewBtn) {
      dashView.view = viewBtn.dataset.dashView === 'motor' ? 'motor' : 'field';
      persistDashView();
      applyDashView();
      refreshNow();
      return;
    }
    const chip = e.target.closest('[data-dash-field]');
    if (chip) {
      dashView.field = chip.dataset.dashField;
      persistDashView();
      applyDashView();
      refreshNow();
    }
  });
}

/** 四足模型(16 台电机可归到 4 腿 × 4 关节)才提供「按字段」网格 */
function fieldModeAvailable() {
  return motor.rows.length > 0 && motor.rows.every((r) => r.rowEl.dataset.leg && r.rowEl.dataset.joint);
}

/** 指标 chips:按 FIELD_ORDER 生成(值只在字段视图下有意义,故随视图显隐) */
function buildFieldChips() {
  if (!dashChipsEl || dashChipsEl.childElementCount) return;
  FIELD_ORDER.forEach((f) => {
    const btn = el('button', 'dash-field-chip', CELL_LABELS[f] || f);
    btn.type = 'button';
    btn.dataset.dashField = f;
    dashChipsEl.appendChild(btn);
  });
}

/** 应用当前视图:切换两套 DOM 的显隐与按钮高亮 */
function applyDashView() {
  if (!dashBarEl) return;
  const fieldOn = dashView.view === 'field';
  if (dashFieldGridEl) dashFieldGridEl.hidden = !fieldOn;
  if (dashScrollEl) dashScrollEl.hidden = fieldOn;
  if (dashChipsEl) dashChipsEl.hidden = !fieldOn;
  dashBarEl.querySelectorAll('[data-dash-view]').forEach((b) => {
    b.classList.toggle('is-active', b.dataset.dashView === dashView.view);
  });
  if (dashChipsEl) {
    dashChipsEl.querySelectorAll('[data-dash-field]').forEach((b) => {
      b.classList.toggle('is-active', b.dataset.dashField === dashView.field);
    });
  }
}

/** 构建 4×4 字段网格:表头 4 关节 + 每腿一行;值格按电机名缓存,刷新时与行值格同步 */
function buildFieldGrid() {
  if (!dashFieldGridEl) return;
  const legs = [];
  const slot = new Map();   // `腿|关节` → 电机名
  motor.rows.forEach((r) => {
    const leg = r.rowEl.dataset.leg;
    const joint = r.rowEl.dataset.joint;
    if (!leg || !joint) return;
    if (!legs.includes(leg)) legs.push(leg);
    slot.set(`${leg}|${joint}`, r.name);
  });
  const signature = `${legs.join(',')}|${motor.signature}`;
  if (signature === dashView.signature) return;
  dashView.signature = signature;
  dashView.cells.clear();
  dashFieldGridEl.textContent = '';
  dashFieldGridEl.appendChild(el('div', 'dash-field-corner'));          // 左上角空位
  FIELD_JOINTS.forEach(([, short]) => dashFieldGridEl.appendChild(el('div', 'dash-field-colhead', short)));
  legs.forEach((leg) => {
    dashFieldGridEl.appendChild(el('div', 'dash-field-rowhead', leg));
    FIELD_JOINTS.forEach(([joint]) => {
      const name = slot.get(`${leg}|${joint}`);
      const cell = el('div', 'dash-field-cell', '—');
      if (!name) {
        cell.classList.add('is-empty');                                // 该模型缺这一格
      } else {
        cell.title = name;
        if (joint === 'foot') cell.classList.add('is-wheel');          // 足端轮(轮子)虚线区分
        dashView.cells.set(name, cell);
      }
      dashFieldGridEl.appendChild(cell);
    });
  });
}

// ── 电机表筛选(按腿 / 按关节,仅四足模型,均支持多选) ─────
/** 当前筛选:每组一个已选值集合;空集合 = 不限制(等同"全部")。
 *  两组条件取交集(腿选 FL/FR + 关节选髋 → 显示 FL、FR 的髋共 2 行) */
const motorFilter = { leg: new Set(), joint: new Set() };
const LEG_FILTERS = [
  { value: 'FL', label: '左前 FL' },
  { value: 'FR', label: '右前 FR' },
  { value: 'RL', label: '左后 RL' },
  { value: 'RR', label: '右后 RR' },
];
const JOINT_FILTERS = [
  { value: 'hip', label: '髋' },
  { value: 'thigh', label: '大腿' },
  { value: 'calf', label: '小腿' },
  { value: 'foot', label: '足端轮' },
];
/** 懒创建筛选条并插到表头上方;visible=false 时隐藏(人形等非四足模型) */
function buildFilterBar(visible) {
  let bar = document.querySelector('#dashboard-motor-filter');
  if (!bar) {
    bar = el('div', 'dash-filter-bar');
    bar.id = 'dashboard-motor-filter';
    const mkGroup = (key, labelText, options) => {
      const group = el('div', 'dash-filter-group');
      group.dataset.filterKey = key;
      const label = el('span', 'dash-filter-label', labelText);
      group.appendChild(label);
      // 「全部」:清空该组多选;组内无选中时高亮
      const allBtn = el('button', 'dash-filter-btn dash-filter-all');
      allBtn.type = 'button';
      allBtn.dataset.value = 'all';
      allBtn.textContent = '全部';
      allBtn.title = '显示全部(清除本组多选)';
      allBtn.classList.add('is-active');
      allBtn.addEventListener('click', () => {
        motorFilter[key].clear();
        syncGroupButtons(group, key, options);
        applyMotorFilters();
      });
      group.appendChild(allBtn);
      // 具体值:点击切换选中,可多选
      options.forEach((opt) => {
        const btn = el('button', 'dash-filter-btn');
        btn.type = 'button';
        btn.dataset.value = opt.value;
        btn.textContent = opt.label;
        btn.title = `点击多选/取消(已选 ${opt.label})`;
        btn.addEventListener('click', () => {
          const set = motorFilter[key];
          if (set.has(opt.value)) set.delete(opt.value);
          else set.add(opt.value);
          // 全选等价于不限制:归一化为空集合,让「全部」高亮
          if (set.size === options.length) set.clear();
          syncGroupButtons(group, key, options);
          applyMotorFilters();
        });
        group.appendChild(btn);
      });
      bar.appendChild(group);
    };
    mkGroup('leg', '按腿', LEG_FILTERS);
    mkGroup('joint', '按关节', JOINT_FILTERS);
    // 插在表头(.dashboard-motor-head)与数据网格之间
    motorGridEl.parentElement.insertBefore(bar, motorGridEl);
  }
  bar.hidden = !visible;
}
/** 按集合状态刷新一组按钮的高亮:空集合 → 仅「全部」高亮 */
function syncGroupButtons(group, key, options) {
  const set = motorFilter[key];
  const allActive = set.size === 0;
  group.querySelector('.dash-filter-all')?.classList.toggle('is-active', allActive);
  options.forEach((opt) => {
    const btn = group.querySelector(`.dash-filter-btn[data-value="${opt.value}"]`);
    btn?.classList.toggle('is-active', set.has(opt.value));
  });
}
/** 按当前筛选条件显示/隐藏电机行,并重排斑马纹(隐藏行不参与计数) */
function applyMotorFilters() {
  let visibleOrdinal = 0;
  motor.rows.forEach(({ rowEl }) => {
    let show = true;
    if (rowEl.dataset.leg) {
      const legOk = motorFilter.leg.size === 0 || motorFilter.leg.has(rowEl.dataset.leg);
      const jointOk = motorFilter.joint.size === 0 || motorFilter.joint.has(rowEl.dataset.joint);
      show = legOk && jointOk;
    }
    rowEl.style.display = show ? '' : 'none';
    if (show) rowEl.classList.toggle('dash-row-alt', (visibleOrdinal++ % 2) === 1);
  });
}

// ── IMU 芯片定义 ─────────────────────────────────────────
const RAD2DEG = 180 / Math.PI;

/** IMU 数值芯片定义:group 分组标题 + 字段列表(alarmId 对应报警模块 IMU 字段,用于报警标红) */
const IMU_DEFS = [
  {
    group: '姿态 (°)',
    fields: [
      { label: 'Roll',  alarmId: 'rpy.roll',  get: (imu) => imu.rpy.roll * RAD2DEG },
      { label: 'Pitch', alarmId: 'rpy.pitch', get: (imu) => imu.rpy.pitch * RAD2DEG },
      { label: 'Yaw',   alarmId: 'rpy.yaw',   get: (imu) => imu.rpy.yaw * RAD2DEG },
    ],
  },
  {
    group: '四元数',
    fields: [
      { label: 'W', get: (imu) => imu.quat.w },
      { label: 'X', get: (imu) => imu.quat.x },
      { label: 'Y', get: (imu) => imu.quat.y },
      { label: 'Z', get: (imu) => imu.quat.z },
    ],
  },
  {
    group: '加速度 (m/s²)',
    fields: [
      { label: 'X', alarmId: 'acc.x', get: (imu) => imu.acc.x },
      { label: 'Y', alarmId: 'acc.y', get: (imu) => imu.acc.y },
      { label: 'Z', alarmId: 'acc.z', get: (imu) => imu.acc.z },
    ],
  },
  {
    group: '角速度 (rad/s)',
    fields: [
      { label: 'X', alarmId: 'gyro.x', get: (imu) => imu.gyro.x },
      { label: 'Y', alarmId: 'gyro.y', get: (imu) => imu.gyro.y },
      { label: 'Z', alarmId: 'gyro.z', get: (imu) => imu.gyro.z },
    ],
  },
  {
    group: '位置 NED (m)',
    fields: [
      { label: 'N', alarmId: 'pos.north', get: (imu) => imu.pos.north },
      { label: 'E', alarmId: 'pos.east',  get: (imu) => imu.pos.east },
      { label: 'D', alarmId: 'pos.down',  get: (imu) => imu.pos.down },
    ],
  },
  {
    group: '速度 机体系 (m/s)',
    fields: [
      { label: 'X', alarmId: 'vel.x', get: (imu) => imu.vel.x },
      { label: 'Y', alarmId: 'vel.y', get: (imu) => imu.vel.y },
      { label: 'Z', alarmId: 'vel.z', get: (imu) => imu.vel.z },
    ],
  },
];

/** @type {Array<{get: Function, el: HTMLElement}>} IMU 芯片值元素缓存 */
const imuCells = [];

// ── 构建 ─────────────────────────────────────────────────

/** 创建带类名的元素 */
function el(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
}

/** 当前模型的电机名列表:优先 JOINT_MAPPING(真机电机全量),无映射退回滑块列表 */
function getMotorNames() {
  const mapped = JOINT_MAPPING[String(currentModelId)];
  if (Array.isArray(mapped) && mapped.length > 0) return mapped;
  return visibleControls.map((s) => s.name);
}

/** CAN ID 参考清单:仅四足模型 2 提供(配置序号 0-15);其他模型无清单显示 */
const CAN_ID_LIST = {
  '2': true,
};

/** 组内序号(0=髋 1=大腿 2=小腿 3=足端轮)→ 实际 CAN ID:
 *  髋=4, 大腿=3, 小腿=2, 足端轮=1 */
function canIdForIndex(index) {
  return 4 - (index % 4);
}
/** 关节名 → 腿(FL/FR/RL/RR)/关节类型(hip/thigh/calf/foot),非四足命名返回 null */
function classifyQuadJoint(name) {
  const m = /^(FL|FR|RL|RR)_(hip|thigh|calf|foot)_/i.exec(name);
  if (!m) return null;
  return { leg: m[1].toUpperCase(), joint: m[2].toLowerCase() };
}

/** 按当前模型重建电机行(IMU 芯片只在首次构建) */
function rebuildMotorRows() {
  const names = getMotorNames();
  const signature = names.join(',');
  if (signature === motor.signature) return;
  motor.signature = signature;
  motor.rows = [];
  motorGridEl.textContent = '';
  const CELLS = ['targetPos', 'realPos', 'targetVel', 'realVel', 'targetTau', 'realTau', 'kp', 'kd', 'temp', 'age'];
  const hasCanList = !!CAN_ID_LIST[String(currentModelId)];
  names.forEach((name, index) => {
    const row = el('div', 'dash-motor-row');
    // 序号即电机块号,与 /rl_real/motor_state 的 9/10/11 字段块下标一致
    const idx = el('span', 'dash-motor-idx', String(index));
    const nameEl = el('span', 'dash-motor-name', labelForJoint(name));
    // CAN ID(仅四足模型 2):每组 4 个关节,髋=4/大腿=3/小腿=2/足端轮=1;MasterID 11~14
    const canEl = el('span', 'dash-motor-can', hasCanList ? `CAN:${String(canIdForIndex(index)).padStart(2, '0')}` : '—');
    if (hasCanList) {
      canEl.title = `MasterID ${11 + Math.floor(index / 4)} · 髋关节=4 大腿=3 小腿=2 足端轮=1`;
    } else {
      canEl.title = '该模型无 CAN ID 参考清单';
    }
    row.append(idx, nameEl, canEl);
    // 四足关节:打标供「按腿/按关节」筛选
    const cls = classifyQuadJoint(name);
    if (cls) {
      row.dataset.leg = cls.leg;
      row.dataset.joint = cls.joint;
    }
    const cells = {};
    CELLS.forEach((field) => {
      const cell = el('span', `dash-motor-val dash-m-${field}`, '—');
      cell.dataset.label = CELL_LABELS[field] || field;
      row.appendChild(cell);
      cells[field] = cell;
    });
    motorGridEl.appendChild(row);
    motor.rows.push({ name, rowEl: row, cells });
  });
  // 筛选条:仅四足(可分类)模型显示
  const quadRows = motor.rows.filter((r) => r.rowEl.dataset.leg);
  buildFilterBar(quadRows.length === motor.rows.length && motor.rows.length > 0);
  applyMotorFilters();

  // 视图切换(仅移动端有此 UI;PC 端 dashBarEl 为 null 直接跳过):
  // 四足模型提供「按字段」4×4 网格;非四足无 4×4 语义 → 强制卡片视图
  if (dashBarEl) {
    initDashViewOnce();
    const fieldOk = fieldModeAvailable();
    dashBarEl.hidden = !fieldOk;
    if (fieldOk) {
      buildFieldChips();
      buildFieldGrid();
    } else {
      dashView.view = 'motor';   // 临时降级,不写回持久化值(换回四足模型即恢复用户选择)
      dashView.cells.clear();
      dashView.signature = '';
    }
    applyDashView();
  }
}

/** 构建 IMU 数值芯片(仅一次;模型切换不影响 IMU) */
function buildImuChipsOnce() {
  if (imuCells.length) return;
  imuGridEl.textContent = '';
  IMU_DEFS.forEach((def) => {
    imuGridEl.appendChild(el('div', 'dash-imu-group', def.group));
    // 每组独占一行,按组内数量均分整行(3 项的组每格更宽,不再留空框)
    const count = Math.min(4, Math.max(1, def.fields.length));
    const row = el('div', `dash-imu-row imu-cols-${count}`);
    def.fields.forEach((field) => {
      const chip = el('div', 'dash-imu-chip');
      chip.appendChild(el('span', 'dash-imu-label', field.label));
      const val = el('span', 'dash-imu-val', '—');
      chip.appendChild(val);
      row.appendChild(chip);
      imuCells.push({ get: field.get, el: val, alarmId: field.alarmId || null });
    });
    imuGridEl.appendChild(row);
  });
}

/** 页签显示时调用:确保行/芯片与当前模型一致 */
export function rebuildDashboard() {
  buildImuChipsOnce();
  rebuildMotorRows();
  refreshNow();
}

// ── 刷新 ─────────────────────────────────────────────────

/** 数字格式化:非有限值显示 — */
function fmt(value, digits = 3) {
  return Number.isFinite(value) ? value.toFixed(digits) : '—';
}

/** 立即刷新一次所有数值 */
function refreshNow() {
  const now = performance.now();
  const imu = shared.imu;
  // 电机行:优先原始遥测(与真机打印一致);无遥测回退仿真 jointStates
  motor.rows.forEach((row) => {
    const tel = motorTelemetry.get(row.name);
    const st = jointStates.get(row.name);
    const get = (field) => (tel ? tel[field] : undefined);
    // 反馈年龄(后端打戳)提前取出,同时用于年龄列着色与整行断线判定
    const rawAge = get('ageMs');
    // 整行变红有两种情形:
    //  ① 帧级:整条 motor_state 停发超过 1s(websocket/后端异常)——全部行红
    //  ② 电机级:帧照常到达,但该台反馈年龄 ≥1s(单路 CAN 断线只冻 4 台)——仅断线台行红。
    //    ageMs<0(启动后从未收到反馈)维持「等待」灰,不触发行红,避免上电瞬间 16 行齐红。
    const telStale = !!tel && (typeof tel.at !== 'number' || now - tel.at > STALE_AFTER_MS);
    const motorAgeDead = !!tel && typeof rawAge === 'number' && rawAge >= MOTOR_AGE_DANGER_MS;
    const rowStale = telStale || motorAgeDead;
    row.cells.targetPos.textContent = fmt(get('targetPos') ?? st?.target);
    row.cells.realPos.textContent   = fmt(get('realPos') ?? st?.value);
    row.cells.targetVel.textContent = fmt(get('targetVel'));
    row.cells.realVel.textContent   = fmt(get('realVel') ?? st?.velocity);
    row.cells.targetTau.textContent = fmt(get('targetTau'));
    row.cells.realTau.textContent   = fmt(get('realTau'));
    row.cells.kp.textContent        = fmt(get('kp'));
    row.cells.kd.textContent        = fmt(get('kd'));
    const tempVal = get('temp');
    row.cells.temp.textContent      = fmt(tempVal, 1);
    // 温度色阶:60°C 预警 / 75°C 过热(不依赖用户是否配置了温度报警规则)
    row.cells.temp.classList.toggle('is-warn', Number.isFinite(tempVal) && tempVal >= MOTOR_TEMP_WARN && tempVal < MOTOR_TEMP_DANGER);
    row.cells.temp.classList.toggle('is-danger', Number.isFinite(tempVal) && tempVal >= MOTOR_TEMP_DANGER);
    // 更新时间(反馈年龄):直接显示后端打戳的 ageMs(真实反馈年龄)。
    // 不再叠加 (now - tel.at):motor_state 发布间隔 125ms(8Hz),叠加会让显示值
    // 在 0~125ms 间振荡,掩盖真实反馈延迟(<2ms)。无遥测/旧协议显示 —;
    // 从未收到该电机显示「等待」(灰);≥0.5s 橙、≥1s 红。
    const ageCell = row.cells.age;
    ageCell.classList.remove('is-warn', 'is-danger', 'is-waiting');
    if (!tel || typeof rawAge !== 'number') {
      ageCell.textContent = '—';
    } else if (rawAge < 0) {
      ageCell.textContent = '等待';
      ageCell.classList.add('is-waiting');
      ageCell.title = '已收到遥测帧,但后端从未解析出该电机的有效反馈';
    } else {
      const ageNow = rawAge;
      if (ageNow >= 1000) ageCell.textContent = `${(ageNow / 1000).toFixed(1)}s`;
      else ageCell.textContent = `${Math.round(ageNow)}ms`;
      ageCell.title = `距上次有效电机反馈 ${Math.round(ageNow)} ms(≥500ms 预警,≥1000ms 视为断线)`;
      ageCell.classList.toggle('is-warn', ageNow >= MOTOR_AGE_WARN_MS && ageNow < MOTOR_AGE_DANGER_MS);
      ageCell.classList.toggle('is-danger', ageNow >= MOTOR_AGE_DANGER_MS);
    }
    // 报警标红:该关节该参数正处于报警状态
    for (const field of ['targetPos', 'realPos', 'targetVel', 'realVel', 'targetTau', 'realTau', 'temp']) {
      const cell = row.cells[field];
      cell.classList.toggle('is-alarmed', isParamAlarmed('motor', field, row.name));
    }
    // 冻结标红:整行(序号/名称/CAN/全部数值)变红——
    // 帧停发(全红)或该台反馈年龄≥1s 视为断线(仅对应行红)
    row.rowEl.classList.toggle('is-stale', rowStale);
    // 按字段视图:把当前指标值格的文案与色阶同步到 4×4 网格对应格,
    // 复用同一份阈值判定,无需为网格再实现一套
    const srcCell = row.cells[dashView.field];
    const dstCell = dashView.cells.get(row.name);
    if (srcCell && dstCell) {
      dstCell.textContent = srcCell.textContent;
      for (const flag of FIELD_SYNC_FLAGS) dstCell.classList.toggle(flag, srcCell.classList.contains(flag));
      dstCell.classList.toggle('is-stale', rowStale);
    }
  });
  // IMU 芯片
  // 新鲜度:lastMsgAt 无效或距上次消息超过 1s 即整芯片标红
  const imuStale = !imu || typeof imu.lastMsgAt !== 'number' || now - imu.lastMsgAt > STALE_AFTER_MS;
  imuCells.forEach((cell) => {
    cell.el.textContent = fmt(cell.get(imu));
    if (cell.alarmId) cell.el.classList.toggle('is-alarmed', isParamAlarmed('imu', cell.alarmId));
    // chip 为数值元素的父节点,class 加在芯片上使标签+数值一起变红
    cell.el.parentElement?.classList.toggle('is-stale', imuStale);
  });
}

/** 动画循环调用:仅页签可见时按 ~10fps 节流刷新 */
let lastRefreshAt = 0;
export function updateDashboard() {
  if (!viewEl || viewEl.hidden) return;
  const now = performance.now();
  if (now - lastRefreshAt < REFRESH_INTERVAL_MS) return;
  lastRefreshAt = now;
  // 模型切换后行数/关节名变化时自动重建
  rebuildMotorRows();
  refreshNow();
}
