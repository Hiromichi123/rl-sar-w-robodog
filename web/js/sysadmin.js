/**
 * 终端页签(terminal) + 系统页签(system) + 文件页签(files) + 模型页签(models)
 *
 * 终端页: 仅 WebSSH(xterm.js, root shell)
 * 系统页: 机器人服务(rl-real-jxg) + 电源(重启/关机) + 高频日志开关 + 管理令牌
 * 文件页: WebFTP(/api/fs/ list/download/upload/mkdir/delete/read/stat/copy/move)
 *         在线文本编辑、复制/剪切/粘贴、上传/下载、详情(文件属性)、多选批量删除/下载
 * 模型页: 策略模型管理(/api/model/ list/upload/download/delete/activate/mkdir)
 *         枚举 POLICY_DIR/<robot>/<submodel>/,可上传 .onnx/.pt、切换激活模型
 *         (改写 config.yaml 的 model_name;激活后需在「系统」页重启服务生效)
 *
 * WebSSH 保活:
 *   - 服务端 20s 发 WS ping, ~75s 无响应才清理;
 *   - 浏览器侧对意外 ssh_exit/WS 重连自动恢复会话(用户主动关闭除外),
 *     合盖休眠唤醒后也能自愈, 不要求手点"重连终端"。
 */
import {
  publishRaw, addRawMessageListener, addStatusListener,
  setRosUrl, getRosUrl, toggleConnection,
} from './ros-bridge.js?v=2026102001';

const TOKEN_KEY = 'jxg_admin_token';

const state = {
  inited: false,
  termVisible: false,
  sysVisible: false,
  filesVisible: false,
  modelsVisible: false,
  online: false,
  // 终端
  term: null,
  fitAddon: null,
  termWanted: false,   // 用户是否希望保持会话(主动关闭=false 时不自动重连)
  termReady: false,
  resizeObs: null,
  termRetryTimer: null,
  // FTP
  cwd: '/home/jinjiao',
  ftpLoaded: false,
  clip: null,          // 剪贴板: {path,name,dir,mode:'copy'|'cut'}
  // 机器人服务(systemd)
  svcPollTimer: null,
  svcBusy: false,
  // 系统监控
  sysInfoTimer: null,
  svcListEditor: null,  // {unit, textarea}
  servicesVisible: false,
  syslogVisible: false,
  processesVisible: false,
  // 服务列表分页/搜索
  svcAllUnits: [],
  svcPage: 1,
  svcPageSize: 10,
  svcSearch: '',
  svcEnableState: {},  // unit -> bool(是否已自启),前端维护
  // 系统日志
  syslogLines: [],
  syslogFilter: '',
  // 进程列表分页/搜索
  psAllProcs: [],
  psPage: 1,
  psPageSize: 10,
  psSearch: '',
  // 网络管理(ip addr)
  netVisible: false,
  netRows: [],
  netSearch: '',
  // 设备管理器(/dev)
  devVisible: false,
  devRows: [],
  devSearch: '',
  // 遥控学习(CH340 串口捕获/克隆)
  rcVisible: false,
  rcLearning: false,
  rcDev: null,        // rc_info 数据(设备/学习状态)
  rcEntries: [],      // 已学码列表
  rcSelectedHex: '',  // 当前选中的待保存捕获 hex
};

// 代码编辑器(CodeMirror) + 左侧文件树的运行时状态
const editorState = {
  cm: null,          // CodeMirror 实例(懒加载就绪前为 null)
  ta: null,          // CodeMirror 就绪前承载内容的普通文本框
  seq: 0,            // 打开序号:换文件/重开时使旧的后台升级作废
  path: '', name: '',
  origText: '',      // 打开时内容(用于 dirty 判定)
  dirty: false,
  truncated: false,  // 原文件超 2MiB 被截断
  tree: new Map(),   // 目录路径 -> {open, loaded, loading, entries}
  selected: '',      // 树中选中文件路径
  prevOkPath: '',    // 最近一次成功打开的文件(二进制/读取失败时回退)
  prevOkName: '',
  expectReopen: false, // 本次打开来自文件树:失败时回退到上一文件而非关闭
};

let els = {};

function token() {
  // 仅从 localStorage 读取手动保存的令牌,不设默认值(安全:不硬编码)。
  // WS 鉴权正常依赖后端下发的 jxg_admin cookie,此处 localStorage 值仅作手动覆盖用。
  return (localStorage.getItem(TOKEN_KEY) || '').trim();
}

function apiUrl(params) {
  const usp = new URLSearchParams(params);
  const t = token();
  if (t) usp.set('token', t);
  return `/api/fs/?${usp.toString()}`;
}

// 模型管理 API 共用:构造 /api/model/ URL
function modelUrl(params) {
  const usp = new URLSearchParams(params);
  const t = token();
  if (t) usp.set('token', t);
  return `/api/model/?${usp.toString()}`;
}

function humanSize(n) {
  if (n < 1024) return `${n} B`;
  if (n < 1024 * 1024) return `${(n / 1024).toFixed(1)} KB`;
  if (n < 1024 * 1024 * 1024) return `${(n / 1024 / 1024).toFixed(1)} MB`;
  return `${(n / 1024 / 1024 / 1024).toFixed(2)} GB`;
}

function humanTime(ts) {
  try {
    const d = new Date(Number(ts) * 1000);
    const p = (x) => String(x).padStart(2, '0');
    return `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())} ${p(d.getHours())}:${p(d.getMinutes())}`;
  } catch { return ''; }
}

function humanTimeSec(ts) {
  try {
    const d = new Date(Number(ts) * 1000);
    const p = (x) => String(x).padStart(2, '0');
    return `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())} ${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`;
  } catch { return ''; }
}

function joinPath(dir, name) {
  return dir.replace(/\/+$/, '') + '/' + name;
}
function dirName(p) {
  const d = p.replace(/\/+$/, '').replace(/\/[^/]*$/, '');
  return d || '/';
}

// ── 文件类型(按扩展名):媒体可预览,其余按文本尝试编辑 ──
const IMG_EXTS = new Set(['png', 'jpg', 'jpeg', 'gif', 'webp', 'bmp', 'svg', 'ico', 'avif']);
const AUD_EXTS = new Set(['mp3', 'wav', 'ogg', 'flac', 'm4a', 'aac', 'opus']);
const VID_EXTS = new Set(['mp4', 'webm', 'mkv', 'avi', 'mov', 'm4v']);
function fileKind(name) {
  const i = name.lastIndexOf('.');
  const ext = i >= 0 ? name.slice(i + 1).toLowerCase() : '';
  if (IMG_EXTS.has(ext)) return 'image';
  if (AUD_EXTS.has(ext)) return 'audio';
  if (VID_EXTS.has(ext)) return 'video';
  return 'text';
}
// 「打开」路由:媒体→预览,其他→文本编辑器(二进制由编辑器内拒绝)
function ftpOpenAny(path, name) {
  if (fileKind(name) === 'text') ftpOpenEditor(path, name);
  else ftpOpenPreview(path, name, fileKind(name));
}
// 触发浏览器下载(供单击名称/下载按钮/预览弹窗共用)
function ftpDownload(path, name) {
  const a = document.createElement('a');
  a.href = apiUrl({ op: 'download', path });
  a.download = name;
  document.body.appendChild(a); a.click(); a.remove();
}

// ─────────────────────────────────────────────
//  WebSSH 终端
// ─────────────────────────────────────────────
function termSendStart() {
  if (!state.term) return;
  // 由 fit 后的真实行列数发起
  let cols = 100, rows = 30;
  try {
    state.fitAddon.fit();
    cols = state.term.cols; rows = state.term.rows;
  } catch { /* 容器未布局时用默认值 */ }
  publishRaw({ type: 'ssh_start', cols, rows });
  els.termStatus.textContent = '正在打开终端…';
  els.termStatus.className = 'sys-term-status is-busy';
}

function termEnsure() {
  if (state.term) return;
  const TerminalCtor = window.Terminal;
  const FitCtor = window.FitAddon && window.FitAddon.FitAddon;
  if (!TerminalCtor || !FitCtor) {
    els.termStatus.textContent = 'xterm 资源未加载';
    return;
  }
  state.term = new TerminalCtor({
    fontSize: 13,
    fontFamily: 'Menlo, Consolas, "DejaVu Sans Mono", monospace',
    theme: { background: '#101418', foreground: '#d7e0e8', cursor: '#4da3ff' },
    cursorBlink: true,
    convertEol: true,
  });
  state.fitAddon = new FitCtor();
  state.term.loadAddon(state.fitAddon);
  state.term.open(els.termMount);
  state.term.onData((d) => {
    if (state.termReady) publishRaw({ type: 'ssh_data', data: d });
  });
  // 容器尺寸变化 → fit + 通知 PTY
  state.resizeObs = new ResizeObserver(() => {
    if (!state.termReady || !state.termVisible) return;
    try {
      state.fitAddon.fit();
      publishRaw({ type: 'ssh_resize', cols: state.term.cols, rows: state.term.rows });
    } catch { /* 忽略布局抖动 */ }
  });
  state.resizeObs.observe(els.termMount);
}

function termScheduleRetry(delayMs) {
  // 自动重连去重: 同一时刻只保留一个重试定时器
  if (state.termRetryTimer) return;
  state.termRetryTimer = setTimeout(() => {
    state.termRetryTimer = null;
    if (state.termWanted && state.termVisible && state.online && !state.termReady) {
      termEnsure();
      termSendStart();
    }
  }, delayMs);
}

function termConnect() {
  termEnsure();
  state.termWanted = true;
  if (state.online) {
    termSendStart();
  } else {
    els.termStatus.textContent = 'WS 离线,连接后自动打开';
  }
  els.termBtn.textContent = '关闭终端';
}

function termDisconnect() {
  // 用户主动关闭: 通知服务端释放 PTY, 且不再自动重连
  if (state.termRetryTimer) { clearTimeout(state.termRetryTimer); state.termRetryTimer = null; }
  if (state.termReady) publishRaw({ type: 'ssh_stop' });
  state.termWanted = false;
  state.termReady = false;
  els.termBtn.textContent = '打开终端';
  els.termStatus.textContent = '未连接';
  els.termStatus.className = 'sys-term-status';
}

// ─────────────────────────────────────────────
//  WebFTP
// ─────────────────────────────────────────────
// 多选集合: 存当前目录下选中的文件名(目录切换时清空)
state.selSet = new Set();
// 分页: 每页条数(桌面10/手机5,手机屏一屏装不下10行) + 当前页码
const FTP_PAGE_SIZE = (typeof window !== 'undefined' && window.__WEB_MOBILE__) ? 5 : 10;
state.ftpPage = 0;

async function ftpList(path) {
  // 移动版没有「系统文件」页签,#sys-ftp-status 不存在,直接跳过避免空引用
  if (!els.ftpStatus) return false;
  els.ftpStatus.textContent = '加载中…';
  try {
    const resp = await fetch(apiUrl({ op: 'list', path }), { cache: 'no-store' });
    const j = await resp.json();
    if (!j.ok) throw new Error(j.error || `HTTP ${resp.status}`);
    state.cwd = j.path;
    if (els.ftpUp) els.ftpUp.disabled = (!j.path || j.path === '/');
    // 进入新目录时清空搜索词 + 多选集合 + 页码
    state.ftpQuery = '';
    state.ftpPage = 0;
    if (els.ftpSearch && els.ftpSearch.value) els.ftpSearch.value = '';
    state.selSet.clear();
    renderSelBar();
    renderFtpCrumb(j.path);
    const n = renderFtpRows(j.path, j.entries || []);
    els.ftpStatus.textContent = `${n} 项`;
    state.ftpLoaded = true;
    return true;
  } catch (err) {
    els.ftpStatus.textContent = `列表失败: ${err.message}`;
    return false;
  }
}

function ftpActBtn(label, cls, handler) {
  const b = document.createElement('button');
  b.type = 'button';
  b.className = `ftp-act-btn ${cls}`;
  b.textContent = label;
  b.addEventListener('click', (ev) => { ev.preventDefault(); ev.stopPropagation(); handler(); });
  return b;
}

function renderFtpCrumb(cwd) {
  const box = els.ftpCrumb;
  if (!box) return;
  // 「↩」返回上级:根目录时禁用
  if (els.ftpUp) els.ftpUp.disabled = (cwd === '/' || cwd === '');
  box.textContent = '';
  const segs = cwd.split('/').filter(Boolean);
  // 根目录:保留一个斜杠 "/"(不可点,仅作根标识);点击面包屑空白仍可内联编辑,编辑框里为 "/"
  // 非根目录:以 "/seg1/seg2" 逐级显示
  let acc = '';
  for (const seg of segs) {
    acc += '/' + seg;
    const segPath = acc; // 捕获当前段路径,避免闭包共享 acc 变量
    const sep = document.createElement('span');
    sep.className = 'ftp-crumb-sep';
    sep.textContent = '/';
    const a = document.createElement('a');
    a.href = '#'; a.className = 'ftp-crumb-seg';
    a.textContent = seg;
    a.title = segPath;
    a.addEventListener('click', (ev) => { ev.preventDefault(); ftpList(segPath); });
    box.append(sep, a);
  }
  if (segs.length === 0) {
    const rootSep = document.createElement('span');
    rootSep.className = 'ftp-crumb-sep ftp-crumb-root';
    rootSep.textContent = '/';
    rootSep.title = '根目录 /';
    box.appendChild(rootSep);
  }
}

// ── 面包屑内联路径编辑 ─────────────────────────────────────
let crumbCommitFn = null; // 「→」按钮点击时提交当前编辑

// 面包屑空白处点击/「→」按钮 → 内联路径编辑:Enter 或「→」提交跳转,Esc 取消,失焦提交
function crumbEnterPathEdit() {
  const box = els.ftpCrumb;
  if (!box || box.dataset.editing === '1') return;
  box.dataset.editing = '1';
  const input = document.createElement('input');
  input.type = 'text';
  input.className = 'ftp-crumb-input';
  input.value = state.cwd;
  input.spellcheck = false;
  box.replaceChildren(input);
  input.focus(); input.select();
  const done = (commit) => {
    if (!input.isConnected) return; // 防 Enter/失焦重复执行
    const v = input.value.trim();
    delete box.dataset.editing;
    crumbCommitFn = null;
    // 提交跳转:成功时 ftpList 内部重渲面包屑;失败则回滚——清除输入框、
    // 退出编辑态、恢复原路径面包屑(不停留在输入失败路径的状态)
    if (commit && v && v !== state.cwd) {
      ftpList(v).then((ok) => { if (!ok) renderFtpCrumb(state.cwd); });
      return;
    }
    renderFtpCrumb(state.cwd);
  };
  crumbCommitFn = () => done(true);
  input.addEventListener('keydown', (ev) => {
    if (ev.key === 'Enter') { ev.preventDefault(); done(true); }
    else if (ev.key === 'Escape') { done(false); }
  });
  input.addEventListener('blur', () => done(true));
}

// 模糊匹配:大小写不敏感的子序列匹配(如 "fbr" 命中 "foobar")
function fuzzyMatch(query, text) {
  const q = query.toLowerCase();
  const s = String(text).toLowerCase();
  if (!q) return true;
  let i = 0;
  for (let k = 0; k < s.length && i < q.length; k++) {
    if (s[k] === q[i]) i++;
  }
  return i === q.length;
}

// 渲染分页数字(通用): 首尾页 + 当前页前后各2页(共5个过渡码) + 省略号
// 参数均为 1 基: totalPages=总页数, current=当前页
function renderPageNums(box, totalPages, current) {
  if (!box) return;
  box.textContent = '';
  const frag = document.createDocumentFragment();
  const addNum = (p) => {
    const b = document.createElement('button');
    b.type = 'button';
    b.className = 'ros-btn sys-ftp-page-num' + (p === current ? ' is-active' : '');
    b.dataset.page = String(p);
    b.textContent = String(p);
    if (p === current) { b.disabled = true; b.setAttribute('aria-current', 'page'); }
    frag.appendChild(b);
  };
  const addEllipsis = () => {
    const s = document.createElement('span');
    s.className = 'sys-ftp-page-ellipsis';
    s.textContent = '…';
    frag.appendChild(s);
  };
  // 必显: 首页/末页; 窗口: 当前页 ±2
  const set = new Set([1, totalPages]);
  for (let p = current - 2; p <= current + 2; p += 1) {
    if (p >= 1 && p <= totalPages) set.add(p);
  }
  const pages = Array.from(set).sort((a, b) => a - b);
  let prev = 0;
  for (const p of pages) {
    if (p - prev > 1) addEllipsis();
    addNum(p);
    prev = p;
  }
  box.appendChild(frag);
}

/** WebFTP 分页数字(固定渲染到 FTP 页码容器) */
function renderFtpPageNums(totalPages, current) {
  renderPageNums(els.ftpPageNums, totalPages, current);
}

// 渲染文件列表(过滤「..」与当前搜索词,支持分页),返回实际显示行数
function renderFtpRows(cwd, entries) {
  const tb = els.ftpBody;
  tb.textContent = '';
  const list = (entries || []).filter((e) => e.name && e.name !== '..' && e.name !== '.');
  state.ftpEntries = list;
  state.ftpListPath = cwd;
  const q = (state.ftpQuery || '').trim();
  const shown = q ? list.filter((e) => fuzzyMatch(q, e.name)) : list;
  // 分页
  const total = shown.length;
  const maxPage = Math.max(0, Math.ceil(total / FTP_PAGE_SIZE) - 1);
  if (state.ftpPage > maxPage) state.ftpPage = maxPage;
  const pageStart = state.ftpPage * FTP_PAGE_SIZE;
  const pageEnd = Math.min(total, pageStart + FTP_PAGE_SIZE);
  const pageItems = shown.slice(pageStart, pageEnd);
  // 全选复选框状态: 同步表头(仅当前页)
  if (els.ftpSelAll) {
    const selectedInPage = pageItems.filter((e) => state.selSet.has(e.name)).length;
    els.ftpSelAll.checked = pageItems.length > 0 && selectedInPage === pageItems.length;
    els.ftpSelAll.indeterminate = selectedInPage > 0 && selectedInPage < pageItems.length;
  }
  // 分页控件: 始终显示(不足一页也显示),空目录同样显示
  if (els.ftpPager) {
    els.ftpPager.hidden = false;
    els.ftpPagePrev.disabled = state.ftpPage <= 0;
    els.ftpPageNext.disabled = state.ftpPage >= maxPage;
  }
  // 中间过渡页码: 首尾页 + 当前页前后各2页(共5个) + 省略号(参数均为1基)
  renderFtpPageNums(maxPage + 1, state.ftpPage + 1);
  if (!pageItems.length) {
    const tr = document.createElement('tr');
    const td = document.createElement('td');
    td.colSpan = 5;
    td.className = 'sys-ftp-empty';
    td.textContent = q ? `没有匹配「${q}」的文件或文件夹` : '(空目录)';
    tr.appendChild(td);
    tb.appendChild(tr);
    return 0;
  }
  for (const e of pageItems) {
    const tr = document.createElement('tr');
    const full = joinPath(cwd, e.name);
    // 多选框
    const tdSel = document.createElement('td');
    tdSel.className = 'sys-ftp-sel';
    const cb = document.createElement('input');
    cb.type = 'checkbox';
    cb.className = 'sys-ftp-sel-cb';
    cb.checked = state.selSet.has(e.name);
    cb.title = '选中此项';
    cb.addEventListener('click', (ev) => ev.stopPropagation());
    cb.addEventListener('change', () => {
      if (cb.checked) state.selSet.add(e.name);
      else state.selSet.delete(e.name);
      renderSelBar();
      // 同步表头全选框(不整表重渲以保留输入焦点)
      if (els.ftpSelAll) {
        const all = els.ftpBody.querySelectorAll('.sys-ftp-sel-cb');
        const on = els.ftpBody.querySelectorAll('.sys-ftp-sel-cb:checked');
        els.ftpSelAll.checked = all.length > 0 && on.length === all.length;
        els.ftpSelAll.indeterminate = on.length > 0 && on.length < all.length;
      }
    });
    tdSel.appendChild(cb);
    tr.appendChild(tdSel);
    // 名称: 不换行,超长截断隐藏;鼠标悬停/键盘聚焦时横向滚动显示全名
    const tdName = document.createElement('td');
    const mkNameLink = (href) => {
      const a = document.createElement('a');
      a.href = href;
      a.className = 'sys-ftp-name' + (e.dir ? ' is-dir' : '');
      const label = document.createElement('span');
      label.className = 'sys-ftp-name-label';
      label.textContent = (e.dir ? '📁 ' : '📄 ') + e.name;
      a.appendChild(label);
      a.title = e.name;
      const roll = () => {
        const overflow = label.scrollWidth - a.clientWidth;
        if (overflow > 0) {
          label.style.transitionDuration = `${Math.min(2.2, 0.3 + overflow / 80).toFixed(2)}s`;
          label.style.transform = `translateX(${-overflow}px)`;
        }
      };
      const reset = () => { label.style.transitionDuration = '.25s'; label.style.transform = 'translateX(0)'; };
      a.addEventListener('mouseenter', roll);
      a.addEventListener('mouseleave', reset);
      a.addEventListener('focus', roll);
      a.addEventListener('blur', reset);
      return a;
    };
    if (e.dir) {
      const a = mkNameLink('#');
      a.addEventListener('click', (ev) => { ev.preventDefault(); ftpList(full); });
      tdName.appendChild(a);
    } else {
      const a = mkNameLink(apiUrl({ op: 'download', path: full }));
      a.title = '单击下载,双击打开';
      a.download = e.name;
      // 单击=下载,双击=打开:短延时消歧,双击到达时取消待发的下载
      let clickTimer = null;
      a.addEventListener('click', (ev) => {
        ev.preventDefault();
        if (clickTimer) return;
        clickTimer = setTimeout(() => { clickTimer = null; ftpDownload(full, e.name); }, 260);
      });
      a.addEventListener('dblclick', (ev) => {
        ev.preventDefault();
        ev.stopPropagation(); // 阻止冒泡到行双击,避免重复打开
        if (clickTimer) { clearTimeout(clickTimer); clickTimer = null; }
        ftpOpenAny(full, e.name);
      });
      tdName.appendChild(a);
    }
    tr.appendChild(tdName);
    // 行双击 = 打开(目录进入,文件按类型打开)
    tr.addEventListener('dblclick', () => {
      if (e.dir) ftpList(full);
      else ftpOpenAny(full, e.name);
    });
    // 大小
    const tdSize = document.createElement('td');
    tdSize.className = 'sys-ftp-size';
    tdSize.textContent = e.dir ? '—' : humanSize(Number(e.size) || 0);
    tr.appendChild(tdSize);
    // 时间
    const tdTime = document.createElement('td');
    tdTime.className = 'sys-ftp-time';
    tdTime.textContent = humanTime(Number(e.mtime) || 0);
    tr.appendChild(tdTime);
    // 操作: 编辑(文本)|查看(媒体) + 属性 直出;其余收入「更多」下拉(重命名/复制/剪切/下载/删除)
    const tdAct = document.createElement('td');
    tdAct.className = 'sys-ftp-act';
    if (!e.dir) {
      if (fileKind(e.name) === 'text') tdAct.appendChild(ftpActBtn('编辑', 'act-edit', () => ftpOpenEditor(full, e.name)));
      else tdAct.appendChild(ftpActBtn('查看', 'act-view', () => ftpOpenAny(full, e.name)));
    }
    tdAct.appendChild(ftpActBtn('属性', 'act-info', () => ftpOpenDetails(full, e.name, e.dir)));
    tdAct.appendChild(ftpMoreBtn([
      { label: '重命名', cls: 'act-ren', fn: () => ftpRename(full, e.name, !!e.dir) },
      { label: '复制', cls: 'act-copy', fn: () => ftpSetClip('copy', full, e.name, !!e.dir) },
      { label: '剪切', cls: 'act-cut', fn: () => ftpSetClip('cut', full, e.name, !!e.dir) },
      ...(!e.dir ? [{ label: '下载', cls: 'act-dl', fn: () => ftpDownload(full, e.name) }] : []),
      { label: '删除', cls: 'act-del', fn: () => ftpDelete(full, e.dir, e.name) },
    ]));
    tr.appendChild(tdAct);
    tb.appendChild(tr);
  }
  return shown.length;
}

// 当前展开的「更多」菜单(全局唯一,展开新菜单时关闭旧的)
let ftpOpenMoreMenu = null;
function ftpCloseMoreMenu() {
  if (ftpOpenMoreMenu) {
    ftpOpenMoreMenu.classList.remove('is-open');
    const btn = ftpOpenMoreMenu.querySelector('.ftp-act-more-btn');
    if (btn) btn.setAttribute('aria-expanded', 'false');
    ftpOpenMoreMenu = null;
  }
}
// 「更多」下拉: 直出按钮(编辑/查看)之外的属性类操作收纳于此
function ftpMoreBtn(items) {
  const wrap = document.createElement('div');
  wrap.className = 'ftp-act-more';
  const btn = document.createElement('button');
  btn.type = 'button';
  btn.className = 'ftp-act-btn act-more-btn';
  btn.setAttribute('aria-haspopup', 'menu');
  btn.setAttribute('aria-expanded', 'false');
  btn.textContent = '更多 ▾';
  const menu = document.createElement('div');
  menu.className = 'ftp-act-menu';
  menu.setAttribute('role', 'menu');
  for (const it of items) {
    const mi = document.createElement('button');
    mi.type = 'button';
    mi.className = `ftp-act-menu-item ${it.cls || ''}`;
    mi.setAttribute('role', 'menuitem');
    mi.textContent = it.label;
    mi.addEventListener('click', (ev) => {
      ev.preventDefault(); ev.stopPropagation();
      ftpCloseMoreMenu();
      it.fn();
    });
    menu.appendChild(mi);
  }
  btn.addEventListener('click', (ev) => {
    ev.preventDefault(); ev.stopPropagation();
    const isOpen = wrap.classList.contains('is-open');
    ftpCloseMoreMenu();
    ftpCloseCtxMenu();
    if (!isOpen) {
      wrap.classList.add('is-open');
      btn.setAttribute('aria-expanded', 'true');
      ftpOpenMoreMenu = wrap;
      // 底部空间不足则向上展开(相对最近的可滚动表格容器判断)
      menu.classList.remove('is-up');
      const wrapBox = wrap.closest('.sys-ftp-tablewrap, .sys-svclist-wrap');
      const mRect = menu.getBoundingClientRect();
      const limit = wrapBox ? wrapBox.getBoundingClientRect().bottom : window.innerHeight;
      if (mRect.bottom > limit - 4) menu.classList.add('is-up');
    }
  });
  wrap.append(btn, menu);
  return wrap;
}

// ── 文件列表空白区右键菜单 ───────────────────────────────────
let ftpCtxMenuEl = null;
function ftpCloseCtxMenu() {
  if (ftpCtxMenuEl) {
    ftpCtxMenuEl.classList.remove('is-open');
    ftpCtxMenuEl.style.left = '-9999px';
  }
}
function ftpCtxMenuEnsure() {
  if (ftpCtxMenuEl) return ftpCtxMenuEl;
  const m = document.createElement('div');
  m.className = 'ftp-ctx-menu';
  m.setAttribute('role', 'menu');
  m.style.left = '-9999px';
  const mkItem = (act, label, title) => {
    const b = document.createElement('button');
    b.type = 'button';
    b.className = 'ftp-ctx-item';
    b.dataset.act = act;
    b.setAttribute('role', 'menuitem');
    b.textContent = label;
    if (title) b.title = title;
    b.addEventListener('click', (ev) => {
      ev.preventDefault(); ev.stopPropagation();
      if (b.disabled) return;
      ftpCloseCtxMenu();
      if (act === 'mkfile') ftpMkfile();
      else if (act === 'mkdir') ftpMkdir();
      else if (act === 'paste') ftpPaste();
      else if (act === 'upload') els.ftpUpInput && els.ftpUpInput.click();
    });
    return b;
  };
  const mkSep = () => { const s = document.createElement('div'); s.className = 'ftp-ctx-sep'; return s; };
  m.append(
    mkItem('mkfile', '📄 新建文件'),
    mkItem('mkdir', '📁 新建文件夹'),
    mkSep(),
    mkItem('paste', '粘贴', ''),
    mkItem('upload', '⬆ 文件上传'),
  );
  // 菜单内部的 mousedown 不冒泡,避免触发 document 关闭后又被 contextmenu 重开的竞态
  m.addEventListener('mousedown', (ev) => ev.stopPropagation());
  m.addEventListener('contextmenu', (ev) => { ev.preventDefault(); ev.stopPropagation(); });
  document.body.appendChild(m);
  ftpCtxMenuEl = m;
  return m;
}
function ftpShowCtxMenu(clientX, clientY) {
  const m = ftpCtxMenuEnsure();
  // 粘贴项:无剪贴板时禁用并提示来源
  const pasteItem = m.querySelector('[data-act="paste"]');
  if (state.clip) {
    pasteItem.disabled = false;
    pasteItem.textContent = `粘贴(${state.clip.mode === 'copy' ? '复制' : '剪切'}: ${state.clip.name})`;
  } else {
    pasteItem.disabled = true;
    pasteItem.textContent = '粘贴';
  }
  // 先显示再测量,做视口边界修正
  m.classList.add('is-open');
  m.style.left = '-9999px'; m.style.top = '0';
  const r = m.getBoundingClientRect();
  let x = Math.min(clientX, window.innerWidth - r.width - 6);
  let y = Math.min(clientY, window.innerHeight - r.height - 6);
  x = Math.max(4, x); y = Math.max(4, y);
  m.style.left = `${x}px`;
  m.style.top = `${y}px`;
}

// 递归收集目录下所有文件与子目录(BFS);目录按路径长度降序排列,便于自底向上删除空目录
async function collectDirTree(rootPath, onProgress) {
  const files = [];
  const dirs = [];
  const stack = [rootPath];
  while (stack.length) {
    const dir = stack.pop();
    const resp = await fetch(apiUrl({ op: 'list', path: dir }), { cache: 'no-store' });
    const j = await resp.json();
    if (!j.ok) throw new Error(j.error || `扫描失败: ${dir}`);
    for (const e of j.entries || []) {
      if (e.name === '..' || e.name === '.') continue;
      const full = joinPath(dir, e.name);
      if (e.dir) { dirs.push(full); stack.push(full); }
      else files.push(full);
    }
    if (onProgress) onProgress(files.length, dirs.length);
  }
  dirs.sort((a, b) => b.length - a.length);
  return { files, dirs };
}

// 删除进度弹窗:显示当前删除项、进度条、百分比与计数,支持取消
function showDeleteProgressModal({ title }) {
  window.ensureUiFeedbackDom && window.ensureUiFeedbackDom();
  const mask = document.createElement('div');
  mask.className = 'ui-modal-mask ui-modal-show';
  mask.innerHTML = `
    <div class="ui-modal ui-modal-lg" role="dialog" aria-modal="true">
      <h3 class="ui-modal-title"></h3>
      <p class="ui-modal-msg" id="del-prog-current" style="margin-bottom:10px;font-family:ui-monospace,Menlo,Consolas,monospace;font-size:12px"></p>
      <div style="width:100%;height:8px;background:rgba(128,140,160,.2);border-radius:4px;overflow:hidden;margin-bottom:10px">
        <div id="del-prog-bar" style="height:100%;width:0%;background:var(--c-accent,#4da3ff);transition:width .15s;border-radius:4px"></div>
      </div>
      <p class="ui-modal-msg" id="del-prog-count" style="margin-bottom:16px;font-size:12px"></p>
      <div class="ui-modal-actions">
        <button type="button" class="ui-modal-cancel" id="del-prog-cancel">取消</button>
      </div>
    </div>`;
  mask.querySelector('.ui-modal-title').textContent = title;
  document.body.appendChild(mask);
  let cancelled = false;
  mask.querySelector('#del-prog-cancel').addEventListener('click', () => { cancelled = true; });
  return {
    update(done, total, current) {
      const pct = total > 0 ? Math.round(done / total * 100) : 0;
      mask.querySelector('#del-prog-bar').style.width = pct + '%';
      mask.querySelector('#del-prog-count').textContent = `${done} / ${total}  (${pct}%)`;
      if (current != null) {
        const el = mask.querySelector('#del-prog-current');
        el.textContent = current;
        el.title = current;
      }
    },
    isCancelled() { return cancelled; },
    close() {
      mask.classList.remove('ui-modal-show');
      setTimeout(() => mask.remove(), 150);
    },
  };
}

// 调用后端 delete 接口删除单个路径(文件或空目录)
async function fsDeleteOne(path, recursive = false) {
  const params = { op: 'delete', path };
  if (recursive) params.recursive = '1';
  const resp = await fetch(apiUrl(params), { cache: 'no-store' });
  const j = await resp.json();
  if (!j.ok) throw new Error(j.error || `HTTP ${resp.status}`);
}

// 递归删除目录:先扫描,再逐个删文件,最后清理空目录;每步通过 onStep(done,total,current) 回调;
// isCancelled() 返回 true 时中止(已删除的文件不会回滚)
async function deleteDirRecursive(path, name, onStep, isCancelled) {
  // 阶段1:扫描(实时显示已发现文件数)
  if (onStep) onStep(0, 0, `正在扫描目录: ${name}…`);
  const { files, dirs } = await collectDirTree(path, (fCnt, dCnt) => {
    if (onStep) onStep(0, 0, `正在扫描… 已发现 ${fCnt} 个文件, ${dCnt} 个子目录`);
  });
  const total = files.length + dirs.length + 1;
  if (total === 0) return { files: 0, dirs: 0, cancelled: false };
  let done = 0;

  // 阶段2:逐个删文件
  for (const f of files) {
    if (isCancelled && isCancelled()) break;
    await fsDeleteOne(f);
    done++;
    if (onStep) onStep(done, total, f);
  }
  // 阶段3:清理子目录
  if (!isCancelled || !isCancelled()) {
    for (const d of dirs) {
      if (isCancelled && isCancelled()) break;
      try { await fsDeleteOne(d, true); } catch { /* ignore */ }
      done++;
      if (onStep) onStep(done, total, d);
    }
  }
  // 阶段4:删根目录(仅在未取消时)
  if (!isCancelled || !isCancelled()) {
    await fsDeleteOne(path, true);
    done++;
    if (onStep) onStep(done, total, path);
  }
  return { files: files.length, dirs: dirs.length, cancelled: isCancelled ? isCancelled() : false };
}

async function ftpDelete(path, isDir, name) {
  const tip = isDir
    ? `确认删除目录及其全部内容?\n${path}`
    : `确认删除文件?\n${path}`;
  const ok = await window.confirmDialog({ title: '确认删除', message: tip, confirmText: '删除', danger: true });
  if (!ok) return;

  // 单文件:一次请求即可,状态栏提示
  if (!isDir) {
    els.ftpStatus.textContent = `删除中: ${name}…`;
    try {
      await fsDeleteOne(path);
      els.ftpStatus.textContent = `已删除: ${name}`;
      state.selSet.delete(name);
      if (state.clip && state.clip.path === path) ftpClearClip();
      ftpList(state.cwd);
    } catch (err) {
      els.ftpStatus.textContent = `删除失败: ${err.message}`;
    }
    return;
  }

  // 目录:前端驱动增量删除,显示进度与当前删除项
  const prog = showDeleteProgressModal({ title: `删除目录: ${name}` });
  try {
    const r = await deleteDirRecursive(path, name, (done, total, current) => {
      prog.update(done, total, current);
    }, () => prog.isCancelled());
    if (r.cancelled) {
      els.ftpStatus.textContent = `已取消删除: ${name} (部分内容已删除)`;
    } else {
      els.ftpStatus.textContent = `已删除目录: ${name} (${r.files} 个文件, ${r.dirs} 个子目录)`;
    }
  } catch (err) {
    els.ftpStatus.textContent = `删除失败: ${err.message}`;
  } finally {
    prog.close();
    state.selSet.delete(name);
    if (state.clip && state.clip.path === path) ftpClearClip();
    ftpList(state.cwd);
  }
}

// ── 重命名(同目录 move) ──────────────────────────────────────
async function ftpRename(path, name, isDir) {
  const input = await window.promptDialog({
    title: '重命名',
    message: `请输入新名称(${isDir ? '目录' : '文件'}):`,
    defaultValue: name,
  });
  if (input === null) return;
  const newName = input.trim();
  if (!newName || newName === '.' || newName === '..' || newName.includes('/') || newName.includes('\\')) {
    els.ftpStatus.textContent = '名称无效';
    return;
  }
  if (newName === name) return; // 未改名,直接关闭
  els.ftpStatus.textContent = `重命名中: ${name} → ${newName}…`;
  try {
    const resp = await fetch(apiUrl({ op: 'move', src: path, dir: dirName(path), name: newName }), { cache: 'no-store' });
    const j = await resp.json();
    if (!j.ok) throw new Error(j.error || `HTTP ${resp.status}`);
    els.ftpStatus.textContent = `已重命名: ${name} → ${newName}`;
    // 清理与旧名相关的选中态/剪贴板
    state.selSet.delete(name);
    if (state.clip && state.clip.path === path) ftpClearClip();
    ftpList(state.cwd);
  } catch (err) {
    els.ftpStatus.textContent = `重命名失败: ${err.message}`;
  }
}

// ── 多选批量操作 ─────────────────────────────────────────────
// 批量下载/删除按钮直接内嵌工具条(文件上传之后),仅在有选中项时显示
function renderSelBar() {
  const has = state.selSet.size > 0;
  if (els.ftpSelDl) els.ftpSelDl.hidden = !has;
  if (els.ftpSelDel) els.ftpSelDel.hidden = !has;
}

// 批量删除:逐个处理,目录走递归删除(带内部进度),整体显示当前项与计数
async function ftpBatchDelete() {
  const names = Array.from(state.selSet);
  if (!names.length) return;
  const ok = await window.confirmDialog({
    title: '批量删除',
    message: `确认删除选中的 ${names.length} 项?\n` + names.slice(0, 8).join('\n') + (names.length > 8 ? `\n… 等共 ${names.length} 项` : '') + '\n目录将被递归删除,不可恢复!',
    confirmText: '删除', danger: true,
  });
  if (!ok) return;
  const total = names.length;
  const prog = showDeleteProgressModal({ title: `批量删除 (${total} 项)` });
  let done = 0;
  try {
    for (const name of names) {
      if (prog.isCancelled()) break;
      const isDir = (state.ftpEntries || []).find((e) => e.name === name)?.dir;
      const path = joinPath(state.cwd, name);
      if (isDir) {
        // 目录:复用递归删除,内部进度前缀显示当前项
        await deleteDirRecursive(path, name, (_subDone, _subTotal, current) => {
          prog.update(done, total, `[${done + 1}/${total}] ${name}  →  ${current}`);
        }, () => prog.isCancelled());
      } else {
        prog.update(done, total, `删除文件: ${name}`);
        await fsDeleteOne(path);
      }
      state.selSet.delete(name);
      done++;
      prog.update(done, total, `✓ ${name}`);
    }
    if (prog.isCancelled()) {
      els.ftpStatus.textContent = `已取消 (已删除 ${done}/${total})`;
    } else {
      els.ftpStatus.textContent = `已删除 ${done} 项`;
    }
  } catch (err) {
    els.ftpStatus.textContent = `删除失败: ${err.message} (已删 ${done}/${total})`;
  } finally {
    prog.close();
    ftpList(state.cwd);
  }
}

// 批量下载: 仅对文件(目录跳过); 顺序触发浏览器下载
function ftpBatchDownload() {
  const names = Array.from(state.selSet);
  const files = names.filter((n) => !(state.ftpEntries || []).find((e) => e.name === n)?.dir);
  if (!files.length) {
    els.ftpStatus.textContent = '所选不含文件(目录不支持批量下载)';
    return;
  }
  files.forEach((name, i) => {
    // 错开 200ms,避免浏览器多任务弹窗堆积
    setTimeout(() => ftpDownload(joinPath(state.cwd, name), name), i * 200);
  });
  els.ftpStatus.textContent = `已触发 ${files.length} 个文件下载`;
}

// 全选/全不选(基于当前显示条目,搜索过滤后仅作用于显示的项)
function ftpSelToggleAll() {
  const all = els.ftpBody.querySelectorAll('.sys-ftp-sel-cb');
  if (!all.length) return;
  const anyOff = Array.from(all).some((cb) => !cb.checked);
  all.forEach((cb) => {
    cb.checked = anyOff;
    // 通过 tr 找回 entry 名:cb 所在 tr 的第 2 个 td 中的链接文本
    const tr = cb.closest('tr');
    const nameEl = tr && tr.querySelector('.sys-ftp-name');
    if (!nameEl) return;
    // nameEl.textContent 前缀为 "📁 " / "📄 " 等 emoji + 空格,去掉前缀
    const raw = nameEl.textContent.replace(/^[^ ]+ /, '');
    if (anyOff) state.selSet.add(raw);
    else state.selSet.delete(raw);
  });
  renderSelBar();
  if (els.ftpSelAll) { els.ftpSelAll.checked = anyOff; els.ftpSelAll.indeterminate = false; }
}

async function ftpUploadFiles(files) {
  const list = Array.from(files || []);
  if (!list.length) return;
  for (const file of list) {
    els.ftpStatus.textContent = `上传中: ${file.name} (${humanSize(file.size)})…`;
    try {
      const usp = new URLSearchParams({ op: 'upload', dir: state.cwd });
      const t = token(); if (t) usp.set('token', t);
      const resp = await fetch(`/api/fs/?${usp.toString()}`, {
        method: 'PUT',
        headers: { 'X-Filename': encodeURIComponent(file.name) },
        body: file,
      });
      const j = await resp.json().catch(() => ({}));
      if (!resp.ok || j.ok === false) throw new Error(j.error || `HTTP ${resp.status}`);
      els.ftpStatus.textContent = `上传完成: ${file.name}`;
    } catch (err) {
      els.ftpStatus.textContent = `上传失败: ${file.name}: ${err.message}`;
      await window.alertDialog({ title: '上传失败', message: `${file.name}\n${err.message}` });
      break;
    }
  }
  ftpList(state.cwd);
}

async function ftpMkdir() {
  const name = await window.promptDialog({ title: '新建目录', message: '在当前目录下创建', defaultValue: 'new_dir' });
  if (!name) return;
  const path = joinPath(state.cwd, name);
  try {
    const resp = await fetch(apiUrl({ op: 'mkdir', path }), { cache: 'no-store' });
    const j = await resp.json();
    if (!j.ok) throw new Error(j.error || `HTTP ${resp.status}`);
    ftpList(state.cwd);
  } catch (err) {
    els.ftpStatus.textContent = `建目录失败: ${err.message}`;
  }
}

// 新建文件: 用 op=upload 上传空 body 创建空文件,然后直接进入编辑器
async function ftpMkfile() {
  const name = await window.promptDialog({ title: '新建文件', message: '在当前目录下创建文本文件', defaultValue: 'new.txt' });
  if (!name) return;
  if (name.includes('/')) {
    els.ftpStatus.textContent = '文件名不能包含 /';
    return;
  }
  const path = joinPath(state.cwd, name);
  try {
    const usp = new URLSearchParams({ op: 'upload', dir: state.cwd });
    const t = token(); if (t) usp.set('token', t);
    const resp = await fetch(`/api/fs/?${usp.toString()}`, {
      method: 'PUT',
      headers: { 'X-Filename': encodeURIComponent(name) },
      body: new Blob([], { type: 'application/octet-stream' }),
    });
    const j = await resp.json();
    if (!j.ok) throw new Error(j.error || `HTTP ${resp.status}`);
    els.ftpStatus.textContent = `已创建: ${name}`;
    await ftpList(state.cwd);
    // 直接进入编辑器
    ftpOpenEditor(path, name);
  } catch (err) {
    els.ftpStatus.textContent = `新建文件失败: ${err.message}`;
  }
}

// ── 剪贴板: 复制 / 剪切 / 粘贴 ────────────────────────────────
function ftpSetClip(mode, path, name, isDir) {
  state.clip = { mode, path, name, dir: isDir };
  els.ftpStatus.textContent = (mode === 'copy' ? '已复制' : '已剪切') + `: ${name}`;
  ftpRenderClip();
}

function ftpClearClip() {
  state.clip = null;
  ftpRenderClip();
}

function ftpRenderClip() {
  const bar = els.ftpClip;
  const btn = els.ftpPaste;
  if (!state.clip) {
    if (bar) bar.hidden = true;
    if (btn) { btn.disabled = true; btn.textContent = '粘贴'; btn.classList.remove('is-cut', 'is-copy'); }
    return;
  }
  const icon = state.clip.mode === 'copy' ? '📋 复制' : '✂ 剪切';
  if (btn) {
    btn.disabled = false;
    btn.textContent = `粘贴(${state.clip.mode === 'copy' ? '复制' : '剪切'})`;
    btn.classList.toggle('is-cut', state.clip.mode === 'cut');
    btn.classList.toggle('is-copy', state.clip.mode === 'copy');
  }
  if (bar) {
    bar.hidden = false;
    bar.innerHTML = '';
    const txt = document.createElement('span');
    txt.textContent = `${icon}:${state.clip.dir ? '📁' : '📄'} ${state.clip.name}`;
    const clear = document.createElement('button');
    clear.type = 'button';
    clear.className = 'ftp-clip-clear';
    clear.textContent = '清除';
    clear.addEventListener('click', ftpClearClip);
    bar.append(txt, clear);
  }
}

async function ftpPaste() {
  const clip = state.clip;
  if (!clip) return;
  let name = clip.name;
  // 源就在当前目录: 复制必改名; 剪切=同目录改名
  if (dirName(clip.path) === state.cwd.replace(/\/+$/, '')) {
    const input = await window.promptDialog({
      title: clip.mode === 'copy' ? '粘贴到同一目录' : '同目录剪切(重命名)',
      message: '请输入新名称:',
      defaultValue: clip.name,
    });
    if (input === null) return;
    name = input.trim();
    if (!name || name === '.' || name === '..' || name.includes('/')) {
      els.ftpStatus.textContent = '名称无效';
      return;
    }
  }
  const op = clip.mode === 'copy' ? 'copy' : 'move';
  els.ftpStatus.textContent = `${clip.mode === 'copy' ? '复制' : '移动'}中: ${clip.name} → ${state.cwd}${name !== clip.name ? '/' + name : ''}…`;
  try {
    const resp = await fetch(apiUrl({ op, src: clip.path, dir: state.cwd, name }), { cache: 'no-store' });
    const j = await resp.json();
    if (!j.ok) throw new Error(j.error || `HTTP ${resp.status}`);
    els.ftpStatus.textContent = `${clip.mode === 'copy' ? '复制' : '移动'}完成: ${name}`;
    // 粘贴成功后即清除剪贴板姿态(复制/剪切均如此),避免残留待粘贴状态
    ftpClearClip();
    ftpList(state.cwd);
  } catch (err) {
    els.ftpStatus.textContent = `粘贴失败: ${err.message}`;
  }
}

// ── 通用模态(编辑/详情共用) ─────────────────────────────────
function ftpModalEnsure() {
  if (els.modal) return els.modal;
  const ov = document.createElement('div');
  ov.className = 'ftp-modal-overlay';
  ov.hidden = true;
  const box = document.createElement('div');
  box.className = 'ftp-modal';
  const head = document.createElement('div');
  head.className = 'ftp-modal-head';
  const title = document.createElement('span');
  title.className = 'ftp-modal-title';
  const close = document.createElement('button');
  close.type = 'button';
  close.className = 'ftp-modal-x';
  close.textContent = '×';
  close.title = '关闭';
  head.append(title, close);
  const body = document.createElement('div');
  body.className = 'ftp-modal-body';
  const foot = document.createElement('div');
  foot.className = 'ftp-modal-foot';
  box.append(head, body, foot);
  ov.appendChild(box);
  document.body.appendChild(ov);
  ov.addEventListener('click', (ev) => { if (ev.target === ov) ftpModalClose(); });
  els.modal = ov; els.modalBox = box;
  els.modalTitle = title; els.modalBody = body; els.modalFoot = foot;
  close.addEventListener('click', ftpModalClose);
  return ov;
}

// 卸载 CodeMirror 实例(打开其他模态内容或关闭时调用)
function editorTeardown() {
  if (editorState.cm) {
    try { editorState.cm.toTextArea(); } catch { /* DOM 已清除时忽略 */ }
    editorState.cm = null;
  }
  editorState.path = ''; editorState.name = '';
  editorState.origText = ''; editorState.dirty = false; editorState.truncated = false;
}

function ftpModalOpen(title) {
  editorTeardown();   // 新内容(详情/预览/另一文件)接管模态,先释放旧编辑器
  ftpModalEnsure();
  els.modalBox.classList.remove('ftp-modal-editor');
  els.modalTitle.textContent = title;
  els.modalBody.textContent = '';
  els.modalFoot.textContent = '';
  els.modal.hidden = false;
}

function ftpModalClose() {
  if (!els.modal) return;
  editorTeardown();
  els.modalBox.classList.remove('ftp-modal-editor');
  els.modal.hidden = true;
  // 停止可能正在播放的预览媒体,避免关闭后声音继续
  els.modalBody.querySelectorAll('audio,video').forEach((m) => {
    try { m.pause(); m.removeAttribute('src'); m.load(); } catch { /* 忽略 */ }
  });
}

// ── 文件详情(属性) ─────────────────────────────────────────
const FTYPE_TEXT = {
  file: '普通文件', dir: '目录', symlink: '符号链接',
  'char-dev': '字符设备', 'block-dev': '块设备', fifo: '管道(FIFO)', socket: '套接字', other: '其他',
};

async function ftpOpenDetails(path, name, isDir) {
  ftpModalOpen(`详情:${name}`);
  els.modalBody.textContent = '读取属性中…';
  try {
    const resp = await fetch(apiUrl({ op: 'stat', path }), { cache: 'no-store' });
    const j = await resp.json();
    if (!j.ok) throw new Error(j.error || `HTTP ${resp.status}`);
    const dl = document.createElement('dl');
    dl.className = 'ftp-props';
    const row = (k, v) => {
      const dt = document.createElement('dt'); dt.textContent = k;
      const dd = document.createElement('dd'); dd.textContent = String(v);
      dl.append(dt, dd);
    };
    row('名称', j.name);
    row('完整路径', j.path);
    row('类型', FTYPE_TEXT[j.type] || j.type);
    if (j.symlink) row('链接目标', j.target || '(悬空链接)');
    row('大小', isDir ? `${humanSize(Number(j.size) || 0)} (目录项合计)` : humanSize(Number(j.size) || 0));
    row('权限', `${j.perm} (${j.mode})`);
    row('属主', `${j.owner} (uid=${j.uid})`);
    row('属组', `${j.group} (gid=${j.gid})`);
    row('修改时间', humanTimeSec(j.mtime));
    row('访问时间', humanTimeSec(j.atime));
    row('属性变更', humanTimeSec(j.ctime));
    els.modalBody.textContent = '';
    els.modalBody.appendChild(dl);
    const closeBtn = document.createElement('button');
    closeBtn.type = 'button'; closeBtn.className = 'ros-btn'; closeBtn.textContent = '关闭';
    closeBtn.addEventListener('click', ftpModalClose);
    els.modalFoot.appendChild(closeBtn);
  } catch (err) {
    els.modalBody.textContent = `属性读取失败: ${err.message}`;
  }
}

// ── 媒体预览(图片/音频/视频,直接以 download 流作为 src,浏览器按内容嗅探解码) ──
function ftpOpenPreview(path, name, kind) {
  const KIND_TEXT = { image: '图片', audio: '音频', video: '视频' };
  ftpModalOpen(`${KIND_TEXT[kind] || '预览'}:${name}`);
  const url = apiUrl({ op: 'download', path });
  let media;
  if (kind === 'image') {
    media = document.createElement('img');
    media.className = 'ftp-preview-img';
    media.alt = name;
  } else if (kind === 'audio') {
    media = document.createElement('audio');
    media.controls = true;
    media.className = 'ftp-preview-audio';
  } else {
    media = document.createElement('video');
    media.controls = true;
    media.className = 'ftp-preview-video';
  }
  media.src = url;
  els.modalBody.textContent = '';
  els.modalBody.appendChild(media);
  const dlBtn = document.createElement('button');
  dlBtn.type = 'button'; dlBtn.className = 'ros-btn'; dlBtn.textContent = '下载';
  dlBtn.addEventListener('click', () => ftpDownload(path, name));
  const closeBtn = document.createElement('button');
  closeBtn.type = 'button'; closeBtn.className = 'ros-btn'; closeBtn.textContent = '关闭';
  closeBtn.addEventListener('click', ftpModalClose);
  els.modalFoot.append(dlBtn, closeBtn);
}

// ─────────────────────────────────────────────────────────────
//  在线文本编辑: CodeMirror 5(本地离线资源,首次编辑时按需加载)
//  + 左侧树状文件浏览器(懒加载展开,点文件直接打开)
// ─────────────────────────────────────────────────────────────
const CM_BASE = './vendor/codemirror/';
const CM_CSS = [
  'codemirror.min.css',
  'theme/dracula.min.css',
  'addon/fold/foldgutter.min.css',
  'addon/search/dialog-local.css?v=1001',
  'addon/search/matchesonscrollbar.min.css',
];
// 懒加载分层:
//  ① 核心(打开文件前必须): 编辑器本体 + simple 基类 + meta 模式查找表 + 常用编辑插件
//  ② 语言模式: 按所打开文件的扩展名按需加载(见 CM_MODE_FILES)
//  ③ 折叠/搜索插件: 编辑器渲染后利用空闲时间加载,不阻塞打开
const CM_CORE_JS = [
  'codemirror.min.js',
  'addon/mode/simple.min.js',
  'mode/meta.min.js',
  'addon/edit/matchbrackets.min.js',
  'addon/edit/closebrackets.min.js',
  'addon/selection/active-line.min.js',
  'addon/comment/comment.min.js',
];
// 模式名 → 需要顺序加载的脚本(htmlmixed 依赖 xml/css/javascript)
const CM_MODE_FILES = {
  xml: ['mode/xml/xml.min.js'],
  css: ['mode/css/css.min.js'],
  javascript: ['mode/javascript/javascript.min.js'],
  htmlmixed: [
    'mode/xml/xml.min.js', 'mode/css/css.min.js',
    'mode/javascript/javascript.min.js', 'mode/htmlmixed/htmlmixed.min.js',
  ],
  python: ['mode/python/python.min.js'],
  clike: ['mode/clike/clike.min.js'],
  shell: ['mode/shell/shell.min.js'],
  yaml: ['mode/yaml/yaml.min.js'],
  markdown: ['mode/markdown/markdown.min.js'],
  nginx: ['mode/nginx/nginx.min.js'],
  properties: ['mode/properties/properties.min.js'],
  dockerfile: ['mode/dockerfile/dockerfile.min.js'],
  sql: ['mode/sql/sql.min.js'],
};
// 编辑器渲染后空闲加载的增强插件(折叠系列 + 搜索系列)
const CM_IDLE_ADDONS = [
  'addon/fold/foldcode.min.js',
  'addon/fold/foldgutter.min.js',
  'addon/fold/brace-fold.min.js',
  'addon/fold/indent-fold.min.js',
  'addon/fold/xml-fold.min.js',
  'addon/fold/markdown-fold.min.js',
  'addon/search/searchcursor.min.js',
  'addon/search/dialog.js?v=1001',
  'addon/search/search.min.js',
  'addon/scroll/annotatescrollbar.min.js',
  'addon/search/matchesonscrollbar.min.js',
];
let cmLoader = null;
const cmModeLoader = {};   // 各模式脚本的加载 Promise(去重并发)

function cmInjectCss(href) {
  if (document.querySelector(`link[data-cm-href="${href}"]`)) return;
  const l = document.createElement('link');
  l.rel = 'stylesheet';
  l.href = CM_BASE + href;
  l.dataset.cmHref = href;
  document.head.appendChild(l);
}

function cmLoadScript(src) {
  return new Promise((resolve, reject) => {
    const s = document.createElement('script');
    s.src = src;
    s.async = false;
    s.onload = () => resolve();
    s.onerror = () => reject(new Error(`CodeMirror 资源加载失败: ${src}`));
    document.head.appendChild(s);
  });
}

function loadCodeMirror() {
  if (cmLoader) return cmLoader;
  cmLoader = (async () => {
    CM_CSS.forEach(cmInjectCss);
    for (const js of CM_CORE_JS) await cmLoadScript(CM_BASE + js);
    if (!window.CodeMirror) throw new Error('CodeMirror 初始化失败');
    return window.CodeMirror;
  })();
  cmLoader.catch(() => { cmLoader = null; });  // 加载失败后允许重试
  return cmLoader;
}

// 按需加载某个语言模式(同模式并发打开只加载一次)
function cmEnsureMode(CM, modeName) {
  if (!modeName || (CM.modes && CM.modes[modeName])) return Promise.resolve();
  if (cmModeLoader[modeName]) return cmModeLoader[modeName];
  const files = CM_MODE_FILES[modeName];
  if (!files) return Promise.reject(new Error(`未知模式: ${modeName}`));
  cmModeLoader[modeName] = (async () => {
    for (const f of files) await cmLoadScript(CM_BASE + f);
  })();
  cmModeLoader[modeName].catch(() => { delete cmModeLoader[modeName]; });
  return cmModeLoader[modeName];
}

// 不检查模式是否已加载,仅按文件名推断想要的 {mode, mime}
function cmWantedMode(CM, fileName) {
  const manual = {
    service: { mime: 'text/properties', mode: 'properties' },
    socket: { mime: 'text/properties', mode: 'properties' },
    timer: { mime: 'text/properties', mode: 'properties' },
    ini: { mime: 'text/properties', mode: 'properties' },
    conf: { mime: 'text/properties', mode: 'properties' },
  };
  const ext = (fileName.split('.').pop() || '').toLowerCase();
  if (manual[ext]) return manual[ext];
  try {
    const info = CM.findModeByFileName(fileName);
    if (info && info.mime && CM_MODE_FILES[info.mode]) return { mime: info.mime, mode: info.mode };
  } catch { /* meta 异常时忽略 */ }
  return null;
}

// 编辑器渲染后空闲加载折叠/搜索插件并热升级当前实例
function cmIdleUpgrade(cm) {
  const ric = window.requestIdleCallback || ((fn) => window.setTimeout(fn, 400));
  ric(async () => {
    try {
      for (const f of CM_IDLE_ADDONS) await cmLoadScript(CM_BASE + f);
      if (!cm || !cm.getDoc) return;             // 已被关闭/替换
      cm.setOption('foldGutter', true);
      cm.setOption('gutters', ['CodeMirror-linenumbers', 'CodeMirror-foldgutter']);
      cm.refresh();
    } catch { /* 增强插件加载失败不影响编辑 */ }
  });
}

function editorUpdateTitle() {
  els.modalTitle.textContent = `编辑:${editorState.name}${editorState.dirty ? ' ●' : ''}`;
}

// 文件读取中的圆圈加载占位(参考模型加载 loading-spinner)
function editorShowReadingSpinner() {
  els.modalBody.textContent = '';
  const wrap = document.createElement('div');
  wrap.className = 'ftp-editor-loading';
  const ring = document.createElement('div');
  ring.className = 'ftp-editor-spinner';
  const txt = document.createElement('div');
  txt.className = 'ftp-editor-loading-text';
  txt.textContent = '文件读取中…';
  wrap.append(ring, txt);
  els.modalBody.appendChild(wrap);
}

// opts.noTree: 不显示左侧文件树(模型管理编辑配置时使用)
// 立即读出文件并用普通文本框显示内容;CodeMirror 在后台懒加载,就绪后无感升级
async function ftpOpenEditor(path, name, opts = {}) {
  ftpModalOpen(`编辑:${name}`);
  els.modalBox.classList.add('ftp-modal-editor');
  // 圆圈懒加载(复用模型加载 spinner 风格),文件读完前展示
  editorShowReadingSpinner();
  const seq = ++editorState.seq;
  editorState.cm = null;
  editorState.ta = null;

  let j;
  try {
    const resp = await fetch(apiUrl({ op: 'read', path }), { cache: 'no-store' });
    j = await resp.json();
    if (!j.ok) throw new Error(j.error || `HTTP ${resp.status}`);
  } catch (err) {
    await editorOpenFailed(`文件读取失败: ${err.message}`);
    return;
  }
  if (j.binary) {
    await editorOpenFailed(`「${name}」是二进制文件,不能在代码编辑器中编辑(请在文件列表中下载/查看)。`);
    return;
  }

  // ① 立即渲染:文件树(可选) + 承载内容的普通文本框,打开即可读可写
  editorBuildRaw(path, name, j, { noTree: !!opts.noTree });

  // ② 后台懒加载 CodeMirror;加载失败就保持普通文本框,保存不受影响
  let CM;
  try {
    CM = await loadCodeMirror();
  } catch {
    editorSetBadge('⚠ 语法高亮加载失败,已使用兼容模式');
    return;
  }
  if (editorState.seq !== seq || els.modal.hidden) return;  // 已换文件/被关闭
  await editorUpgrade(CM, seq);
}



// 文件无法在编辑器中打开(二进制/读取失败):
// 从文件树切换来的 → 提示后自动回到上一个文件,文件树不消失;
// 从文件列表直接打开的 → 提示 + 关闭按钮。
async function editorOpenFailed(message) {
  const fromTree = editorState.expectReopen;
  const back = fromTree ? { p: editorState.prevOkPath, n: editorState.prevOkName } : null;
  editorState.expectReopen = false;
  await window.alertDialog({ title: '无法编辑', message });
  if (back && back.p) {
    ftpOpenEditor(back.p, back.n);
  } else {
    els.modalBox.classList.remove('ftp-modal-editor');
    els.modalBody.textContent = '';
    els.modalBody.textContent = message;
    const closeBtn = document.createElement('button');
    closeBtn.type = 'button'; closeBtn.className = 'ros-btn'; closeBtn.textContent = '关闭';
    closeBtn.addEventListener('click', ftpModalClose);
    els.modalFoot.appendChild(closeBtn);
  }
}

// ① 立即构建「文件树 + 普通文本框」:文件一读完就显示,无任何等待
function editorBuildRaw(path, name, j, { noTree = false } = {}) {
  // ── 左侧文件树(noTree 时不创建) ──
  let aside = null, resizer = null, treeScroll = null;
  if (!noTree) {
    aside = document.createElement('aside');
    aside.className = 'ftp-tree';
    const treeBar = document.createElement('div');
    treeBar.className = 'ftp-tree-bar';
    const treeTitle = document.createElement('span');
    treeTitle.className = 'ftp-tree-title';
    treeTitle.textContent = '📁 文件系统';
    treeBar.appendChild(treeTitle);
    treeScroll = document.createElement('div');
    treeScroll.className = 'ftp-tree-scroll';
    aside.append(treeBar, treeScroll);

    resizer = document.createElement('div');
    resizer.className = 'ftp-tree-resizer';
    resizer.title = '拖拽调整文件树宽度';
  }

  // ── 右侧编辑区 ──
  const main = document.createElement('div');
  main.className = 'ftp-edit-main';
  const meta = document.createElement('div');
  meta.className = 'ftp-edit-meta';
  const pathText = document.createElement('span');
  pathText.className = 'ftp-edit-crumb';
  pathText.textContent = path;
  const sizeInfo = document.createElement('span');
  sizeInfo.className = 'ftp-edit-size';
  sizeInfo.textContent = ` · ${humanSize(Number(j.size) || 0)}` +
    (j.truncated ? ' · ⚠ 超过 2 MiB,保存将截断' : '');
  const badge = document.createElement('span');
  badge.className = 'ftp-edit-badge';
  badge.textContent = '◌ 语法高亮加载中…';
  meta.append(pathText, sizeInfo, badge);
  const cmWrap = document.createElement('div');
  cmWrap.className = 'ftp-cm-wrap';
  const ta = document.createElement('textarea');
  ta.className = 'ftp-cm-raw';
  ta.spellcheck = false;
  ta.value = j.content || '';
  cmWrap.appendChild(ta);
  main.append(meta, cmWrap);

  const layout = document.createElement('div');
  layout.className = 'ftp-edit-layout' + (noTree ? ' is-notree' : '');
  if (!noTree) layout.append(aside, resizer);
  layout.appendChild(main);
  els.modalBody.textContent = '';
  els.modalBody.appendChild(layout);

  editorState.cm = null;
  editorState.ta = ta;
  editorState.path = path;
  editorState.name = name;
  editorState.origText = ta.value;
  editorState.dirty = false;
  editorState.truncated = !!j.truncated;
  editorState.expectReopen = false;
  editorState.prevOkPath = path;
  editorState.prevOkName = name;

  // 普通文本框阶段:dirty 跟踪 + Ctrl+S 保存(之后由 CodeMirror 接管)
  ta.addEventListener('input', () => {
    const d = ta.value !== editorState.origText;
    if (d !== editorState.dirty) { editorState.dirty = d; editorUpdateTitle(); }
  });
  ta.addEventListener('keydown', (e) => {
    if ((e.ctrlKey || e.metaKey) && (e.key === 's' || e.key === 'S')) {
      e.preventDefault(); editorSave();
    }
  });
  setTimeout(() => ta.focus(), 0);

  // ── 底部按钮 ──
  const cancelBtn = document.createElement('button');
  cancelBtn.type = 'button'; cancelBtn.className = 'ros-btn'; cancelBtn.textContent = '取消';
  cancelBtn.addEventListener('click', ftpModalClose);
  const saveBtn = document.createElement('button');
  saveBtn.type = 'button'; saveBtn.className = 'ros-btn ftp-save-btn'; saveBtn.textContent = '保存 (Ctrl+S)';
  saveBtn.addEventListener('click', () => editorSave());
  els.modalFoot.append(cancelBtn, saveBtn);

  // ── 文件树 + 调宽(noTree 时跳过) ──
  if (noTree) {
    els.treeScroll = null;
  } else {
    els.treeScroll = treeScroll;
    setupTreeResizer(resizer, aside);
    treeInit(path);
  }
}

// 顶部小徽标(语法高亮加载中/失败提示)
function editorSetBadge(text) {
  const badge = els.modalBody.querySelector('.ftp-edit-badge');
  if (!badge) return;
  badge.textContent = text;
  badge.classList.add('is-fail');
}

// ② CodeMirror 就绪:把普通文本框无感升级为代码编辑器(保留已输入内容)
async function editorUpgrade(CM, seq) {
  const ta = editorState.ta;
  if (!ta) return;
  const hadFocus = (document.activeElement === ta);

  const cm = CM.fromTextArea(ta, {
    mode: undefined,                    // 语法模式随后按需加载并热设置
    theme: 'dracula',
    lineNumbers: true,
    lineWrapping: false,
    styleActiveLine: true,
    matchBrackets: true,
    autoCloseBrackets: true,
    gutters: ['CodeMirror-linenumbers'],
    indentUnit: 2, tabSize: 2, indentWithTabs: false,
    extraKeys: {
      'Ctrl-S': () => editorSave(),
      'Cmd-S': () => editorSave(),
      'Ctrl-/': 'toggleComment',
      'Cmd-/': 'toggleComment',
    },
  });
  editorState.cm = cm;
  cm.on('change', () => {
    const d = cm.getValue() !== editorState.origText;
    if (d !== editorState.dirty) { editorState.dirty = d; editorUpdateTitle(); }
  });
  setTimeout(() => { cm.refresh(); if (hadFocus) cm.focus(); }, 60);
  cmIdleUpgrade(cm);   // 折叠/搜索插件继续空闲加载

  // 语法模式脚本按文件类型懒加载,就绪后热切换高亮
  const wanted = cmWantedMode(CM, editorState.name);
  if (wanted) {
    (async () => {
      try { await cmEnsureMode(CM, wanted.mode); } catch { return; }
      if (editorState.seq !== seq || !editorState.cm) return;
      cm.setOption('mode', wanted.mime);
    })();
  }

  // 加载中徽标移除,meta 补上快捷键提示
  const badge = els.modalBody.querySelector('.ftp-edit-badge');
  if (badge) badge.remove();
  const sizeInfo = els.modalBody.querySelector('.ftp-edit-size');
  if (sizeInfo && !editorState.truncated) {
    sizeInfo.textContent = sizeInfo.textContent + ' · Ctrl+F 查找 · Ctrl+Shift+F 替换';
  }
}

async function editorSave() {
  // CodeMirror 未就绪时从普通文本框取值(保存功能不依赖高亮加载完成)
  if (!editorState.cm && !editorState.ta) return;
  const getText = () => (editorState.cm ? editorState.cm.getValue() : editorState.ta.value);
  const { path, name, truncated } = editorState;
  if (truncated) {
    const c = await window.confirmDialog({
      title: '文件过大', message: '原文件超过 2 MiB,保存将覆盖为当前编辑内容(超出部分丢失)!确认保存?',
      confirmText: '确认保存', danger: true,
    });
    if (!c) return;
  }
  // 服务文件保存走 svc save 命令
  if (path && path.startsWith('/etc/systemd/system/')) {
    const saveBtn = els.modalFoot.querySelector('.ftp-save-btn');
    if (saveBtn) { saveBtn.disabled = true; saveBtn.textContent = '保存中…'; }
    try {
      publishRaw({ type: 'command', data: `svc save ${name}\n${getText()}` });
      editorState.origText = getText();
      editorState.dirty = false;
      ftpModalClose();
    } catch (err) {
      if (saveBtn) { saveBtn.disabled = false; saveBtn.textContent = '保存 (Ctrl+S)'; }
    }
    return;
  }
  const saveBtn = els.modalFoot.querySelector('.ftp-save-btn');
  if (saveBtn) { saveBtn.disabled = true; saveBtn.textContent = '保存中…'; }
  try {
    const usp = new URLSearchParams({ op: 'upload', dir: dirName(path) });
    const t = token(); if (t) usp.set('token', t);
    const r = await fetch(`/api/fs/?${usp.toString()}`, {
      method: 'PUT',
      headers: { 'X-Filename': encodeURIComponent(name) },
      body: new Blob([getText()], { type: 'application/octet-stream' }),
    });
    const jj = await r.json().catch(() => ({}));
    if (!r.ok || jj.ok === false) throw new Error(jj.error || `HTTP ${r.status}`);
    editorState.origText = getText();
    editorState.dirty = false;
    els.ftpStatus.textContent = `已保存: ${name}`;
    ftpModalClose();
    ftpList(state.cwd);
    // 编辑的是模型目录下文件(如 config.yaml)时,同步刷新模型卡片激活标签
    if (modelState.data && modelState.data.policy_dir &&
        path.startsWith(modelState.data.policy_dir + '/')) modelRefresh();
  } catch (err) {
    await window.alertDialog({ title: '保存失败', message: err.message });
    if (saveBtn) { saveBtn.disabled = false; saveBtn.textContent = '保存 (Ctrl+S)'; }
  }
}

// ── 文件树(数据懒加载;目录展开状态跨切文件保留) ─────────────
function treeNodeGet(p) {
  let node = editorState.tree.get(p);
  if (!node) {
    node = { open: false, loaded: false, loading: false, entries: [] };
    editorState.tree.set(p, node);
  }
  return node;
}

async function treeEnsure(dirPath) {
  const node = treeNodeGet(dirPath);
  if (node.loaded || node.loading) return node;
  node.loading = true;
  try {
    const resp = await fetch(apiUrl({ op: 'list', path: dirPath }), { cache: 'no-store' });
    const j = await resp.json();
    if (!j.ok) throw new Error(j.error || 'list failed');
    node.entries = (j.entries || [])
      .filter((e) => e.name !== '..')
      .map((e) => ({ name: e.name, dir: !!e.dir }));
    node.entries.sort((a, b) => (a.dir !== b.dir
      ? (a.dir ? -1 : 1)
      : a.name.localeCompare(b.name)));
    node.loaded = true;
  } finally {
    node.loading = false;
  }
  return node;
}

// 以 / 为根,展开并选中到 filePath 所在目录(各级并行预取,失败目录保持折叠)
async function treeInit(filePath) {
  editorState.selected = filePath;
  const parts = filePath.split('/').filter(Boolean);
  const dirs = [];
  let acc = '';
  for (let i = 0; i < parts.length - 1; i++) { acc += '/' + parts[i]; dirs.push(acc); }
  treeNodeGet('/').open = true;
  await treeEnsure('/').catch(() => {});
  for (const d of dirs) {
    treeNodeGet(d).open = true;
    await treeEnsure(d).catch(() => {});
  }
  renderTree();
  const sel = els.treeScroll && els.treeScroll.querySelector('.ftp-tree-row.is-selected');
  if (sel) sel.scrollIntoView({ block: 'nearest' });
}

function treeToggle(dirPath) {
  const node = treeNodeGet(dirPath);
  if (!node.open) {
    node.open = true;
    if (!node.loaded && !node.loading) {
      renderTree();   // 先显示"加载中",完成后再渲一次
      treeEnsure(dirPath).then(renderTree).catch(() => { node.open = false; renderTree(); });
      return;
    }
  } else {
    node.open = false;
  }
  renderTree();
}

function renderTree() {
  if (!els.treeScroll) return;
  els.treeScroll.textContent = '';
  els.treeScroll.appendChild(treeRenderDir('/'));
}

function treeRenderDir(dirPath) {
  const node = treeNodeGet(dirPath);
  const wrap = document.createElement('div');
  const label = dirPath === '/' ? '/' : dirPath.split('/').filter(Boolean).pop();
  wrap.appendChild(treeDirRow(dirPath, label, node));
  if (node.open) {
    const kids = document.createElement('div');
    kids.className = 'ftp-tree-children';
    if (node.loading) {
      const hint = document.createElement('div');
      hint.className = 'ftp-tree-hint';
      hint.textContent = '加载中…';
      kids.appendChild(hint);
    } else {
      for (const e of node.entries) {
        const full = joinPath(dirPath, e.name);
        if (e.dir) kids.appendChild(treeRenderDir(full));
        else kids.appendChild(treeFileRow(full, e.name));
      }
    }
    wrap.appendChild(kids);
  }
  return wrap;
}

function treeDirRow(dirPath, label, node) {
  const row = document.createElement('div');
  row.className = 'ftp-tree-row' + (editorState.selected === dirPath ? ' is-selected' : '');
  const chev = document.createElement('span');
  chev.className = 'ftp-tree-chev';
  chev.textContent = node.loading ? '…' : (node.open ? '▼' : '▶');
  const ico = document.createElement('span');
  ico.className = 'ftp-tree-ico';
  ico.textContent = '📁';
  const nm = document.createElement('span');
  nm.className = 'ftp-tree-name';
  nm.textContent = label;
  nm.title = dirPath;
  row.append(chev, ico, nm);
  row.addEventListener('click', () => treeToggle(dirPath));
  return row;
}

function treeFileIcon(fileName) {
  const k = fileKind(fileName);
  if (k === 'image') return '🖼';
  if (k === 'audio') return '🎵';
  if (k === 'video') return '🎬';
  return '📄';
}

function treeFileRow(filePath, fileName) {
  const row = document.createElement('div');
  row.className = 'ftp-tree-row' + (editorState.selected === filePath ? ' is-selected' : '');
  const chev = document.createElement('span');
  chev.className = 'ftp-tree-chev';   // 占位,与目录行的箭头对齐
  const ico = document.createElement('span');
  ico.className = 'ftp-tree-ico';
  ico.textContent = treeFileIcon(fileName);
  const nm = document.createElement('span');
  nm.className = 'ftp-tree-name';
  nm.textContent = fileName;
  nm.title = filePath;
  row.append(chev, ico, nm);
  row.addEventListener('click', () => treeOpenFile(filePath, fileName));
  return row;
}

async function treeOpenFile(filePath, fileName) {
  if (filePath === editorState.path) return;
  // 非文本文件(图片/音频/视频/未知二进制):不离开编辑界面,仅提示不打开
  if (fileKind(fileName) !== 'text') {
    await window.alertDialog({
      title: '不支持在代码编辑器中打开',
      message: `「${fileName}」不是文本文件。\n如需查看或下载,请在「文件」列表中操作。`,
    });
    return;
  }
  if (editorState.dirty) {
    const ok = await window.confirmDialog({
      title: '未保存的修改',
      message: `「${editorState.name}」有未保存修改,切换到「${fileName}」将丢弃这些修改。`,
      confirmText: '放弃并切换', danger: true,
    });
    if (!ok) return;
  }
  editorState.selected = filePath;
  editorState.expectReopen = true;   // 新文件若为二进制/读取失败,回退到上一文件
  renderTree();
  await ftpOpenEditor(filePath, fileName);  // 树展开状态在 editorState.tree 中保留
}

// 文件树宽度拖拽(160~460px)
function setupTreeResizer(bar, aside) {
  bar.addEventListener('mousedown', (e) => {
    e.preventDefault();
    const startX = e.clientX;
    const startW = aside.getBoundingClientRect().width;
    bar.classList.add('is-drag');
    const move = (ev) => {
      const w = Math.max(160, Math.min(460, startW + (ev.clientX - startX)));
      aside.style.flex = `0 0 ${w}px`;
      aside.style.width = `${w}px`;
    };
    const up = () => {
      bar.classList.remove('is-drag');
      document.removeEventListener('mousemove', move);
      document.removeEventListener('mouseup', up);
      document.body.style.cursor = '';
      document.body.style.userSelect = '';
    };
    document.addEventListener('mousemove', move);
    document.addEventListener('mouseup', up);
    document.body.style.cursor = 'col-resize';
    document.body.style.userSelect = 'none';
  });
}

// ─────────────────────────────────────────────
//  模型管理(/api/model/)
// ─────────────────────────────────────────────
const modelState = {
  data: null,         // 最近一次 list 返回的完整数据
  loaded: false,
  page: 0,            // 当前模型卡片页码(0 基)
  search: '',         // 搜索关键字(小写匹配模型配置名/文件名)
};

/** 模型管理每页卡片数(卡片较高,移动端更少) */
const MODEL_PAGE_SIZE = (typeof window !== 'undefined' && window.__WEB_MOBILE__) ? 2 : 3;

/** 模型配置目录的"最近时间":目录内所有文件 mtime 的最大值(秒),无文件为 0 */
function modelLatestMtime(m) {
  let t = 0;
  for (const f of (m.files || [])) {
    const v = Number(f.mtime) || 0;
    if (v > t) t = v;
  }
  return t;
}

async function modelRefresh() {
  if (els.modelStatus) els.modelStatus.textContent = '加载中…';
  try {
    const resp = await fetch(modelUrl({ op: 'list' }), { cache: 'no-store' });
    const j = await resp.json();
    if (!j.ok) throw new Error(j.error || `HTTP ${resp.status}`);
    modelState.data = j;
    modelState.loaded = true;
    renderModels(j);
    const cfgCount = (j.robots || []).reduce((n, r) => n + r.models.length, 0);
    const fileCount = (j.robots || []).reduce(
      (n, r) => n + r.models.reduce(
        (k, m) => k + (m.files ? m.files.filter((f) => f.name !== 'config.yaml').length : 0), 0), 0);
    if (els.modelStatus) els.modelStatus.textContent =
      `${j.policy_dir} · ${cfgCount} 个模型配置 · ${fileCount} 个文件`;
  } catch (err) {
    if (els.modelStatus) els.modelStatus.textContent = `列表失败: ${err.message}`;
  }
}

function modelBtn(label, cls, handler) {
  const b = document.createElement('button');
  b.type = 'button';
  b.className = `ftp-act-btn ${cls}`;
  b.textContent = label;
  b.addEventListener('click', (ev) => { ev.preventDefault(); ev.stopPropagation(); handler(); });
  return b;
}

function renderModels(data) {
  const box = els.modelBody;
  if (!box) return;
  box.textContent = '';

  /** 空态:隐藏分页条 */
  const showEmpty = (text) => {
    const p = document.createElement('div');
    p.className = 'sys-ftp-empty';
    p.textContent = text;
    box.appendChild(p);
    if (els.modelPager) els.modelPager.hidden = true;
  };

  if (!data.robots || !data.robots.length) {
    showEmpty('(未找到 JXGw 模型目录:policy/JXGw)');
    return;
  }

  // v5: 仅保留 JXGw,展平 robots 层级 —— 每个模型配置目录直接是一张卡片
  let cards = [];
  for (const robot of data.robots) {
    for (const m of robot.models) cards.push({ robot: robot.name, m });
  }
  if (!cards.length) {
    showEmpty('(JXGw 下暂无模型配置,点击上方「＋ 新建模型配置」创建)');
    return;
  }

  // 搜索过滤:命中模型配置目录名或其中任一文件名(大小写不敏感)
  const q = (modelState.search || '').trim().toLowerCase();
  if (q) {
    cards = cards.filter(({ m }) =>
      m.name.toLowerCase().includes(q) ||
      (m.files || []).some((f) => f.name.toLowerCase().includes(q)));
    if (!cards.length) {
      showEmpty(`(无匹配「${modelState.search.trim()}」的模型配置)`);
      return;
    }
  }

  // 按时间排序:目录内最新文件 mtime 降序(最新在前),并列按名称字典序
  cards.sort((a, b) =>
    (modelLatestMtime(b.m) - modelLatestMtime(a.m)) || a.m.name.localeCompare(b.m.name));

  // 分页:每页 MODEL_PAGE_SIZE 张卡片
  const total = cards.length;
  const maxPage = Math.max(0, Math.ceil(total / MODEL_PAGE_SIZE) - 1);
  if (modelState.page > maxPage) modelState.page = maxPage;
  const start = modelState.page * MODEL_PAGE_SIZE;
  const pageCards = cards.slice(start, Math.min(total, start + MODEL_PAGE_SIZE));
  for (const { robot, m } of pageCards) {
    box.appendChild(buildModelCard(robot, m, data.policy_dir));
  }
  // 分页控件:与文件页一致,不足一页也显示
  if (els.modelPager) {
    els.modelPager.hidden = false;
    els.modelPagePrev.disabled = modelState.page <= 0;
    els.modelPageNext.disabled = modelState.page >= maxPage;
  }
  renderPageNums(els.modelPageNums, maxPage + 1, modelState.page + 1);
}

// ---- 模型文件备注: 存放在模型配置目录下的 .notes.json (列表接口天然不显示) ----
const MODEL_NOTES_FILE = '.notes.json';

async function modelReadText(robot, model, file) {
  const resp = await fetch(modelUrl({ op: 'download', robot, model, file }), { cache: 'no-store' });
  if (resp.status === 404) return null;
  if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
  return await resp.text();
}

async function modelWriteText(robot, model, file, text) {
  const resp = await fetch(modelUrl({ op: 'upload', robot, model }), {
    method: 'PUT',
    headers: { 'X-Filename': encodeURIComponent(file) },
    body: text,
  });
  const j = await resp.json().catch(() => ({}));
  if (!resp.ok || j.ok === false) throw new Error(j.error || `HTTP ${resp.status}`);
  return j;
}

// 多行文本编辑弹窗(复用 ui-modal 样式框架)
// variant: 'note' = 普通字体的紧凑备注框; 默认 = 等宽代码框
function textAreaDialog({ title = '编辑', message = '', value = '', placeholder = '',
                          okText = '保存', variant = '', hint = '' } = {}) {
  if (window.ensureUiFeedbackDom) window.ensureUiFeedbackDom();
  return new Promise((resolve) => {
    const mask = document.createElement('div');
    mask.className = 'ui-modal-mask';
    const taCls = 'ui-modal-textarea' + (variant === 'note' ? ' ui-modal-textarea-note' : '');
    mask.innerHTML = `
      <div class="ui-modal ui-modal-lg" role="dialog" aria-modal="true">
        <h3 class="ui-modal-title"></h3>
        <p class="ui-modal-msg" hidden></p>
        <textarea class="${taCls}" spellcheck="false"></textarea>
        <div class="ui-modal-hint" hidden></div>
        <div class="ui-modal-actions">
          <button type="button" class="ui-modal-cancel">取消</button>
          <button type="button" class="ui-modal-ok"></button>
        </div>
      </div>`;
    mask.querySelector('.ui-modal-title').textContent = title;
    const msgEl = mask.querySelector('.ui-modal-msg');
    if (message) { msgEl.textContent = message; msgEl.hidden = false; }
    const hintEl = mask.querySelector('.ui-modal-hint');
    if (hint) { hintEl.textContent = hint; hintEl.hidden = false; }
    const ta = mask.querySelector('.ui-modal-textarea');
    ta.value = value;
    ta.placeholder = placeholder;
    mask.querySelector('.ui-modal-ok').textContent = okText;
    document.body.appendChild(mask);
    let settled = false;
    const close = (val) => {
      if (settled) return;
      settled = true;
      mask.classList.remove('ui-modal-show');
      window.setTimeout(() => mask.remove(), 150);
      resolve(val);
    };
    mask.querySelector('.ui-modal-cancel').addEventListener('click', () => close(null));
    mask.querySelector('.ui-modal-ok').addEventListener('click', () => close(ta.value));
    ta.addEventListener('keydown', (e) => {
      if (e.key === 'Escape') { e.preventDefault(); close(null); }
      else if (e.key === 'Enter' && (e.ctrlKey || e.metaKey)) { e.preventDefault(); close(ta.value); }
    });
    mask.addEventListener('click', (e) => { if (e.target === mask) close(null); });
    requestAnimationFrame(() => { mask.classList.add('ui-modal-show'); ta.focus(); });
  });
}

// 单个模型配置目录卡片(原 robot 分组头已去除; config.yaml 行隐藏, 改为「编辑配置」按钮)
function buildModelCard(robot, m, policyDir) {
      const wrap = document.createElement('div');
      wrap.className = 'model-card-wrap';
      let notes = {};

      // 保存某个模型文件的备注(空串则移除; 全部清空则删除 .notes.json)
      const saveNote = async (fileName, text) => {
        const v = (text || '').trim();
        if (v) notes[fileName] = v; else delete notes[fileName];
        try {
          if (Object.keys(notes).length) {
            await modelWriteText(robot, m.name, MODEL_NOTES_FILE, JSON.stringify(notes, null, 2));
          } else {
            const resp = await fetch(
              modelUrl({ op: 'delete', robot, model: m.name, file: MODEL_NOTES_FILE }),
              { cache: 'no-store' });
            if (!resp.ok && resp.status !== 404) throw new Error(`HTTP ${resp.status}`);
          }
          draw();
        } catch (err) {
          await window.alertDialog({ title: '备注保存失败', message: err.message });
        }
      };

      // 编辑 config.yaml: 复用文件管理的代码编辑器(CodeMirror),不显示左侧文件树
      const openConfigEditor = () => {
        const pdir = (modelState.data && modelState.data.policy_dir) || '';
        if (!pdir) { window.alertDialog({ title: '无法打开', message: '策略路径未知' }); return; }
        ftpOpenEditor(`${pdir}/${robot}/${m.name}/config.yaml`, 'config.yaml', { noTree: true });
      };

      function draw() {
      const sub = document.createElement('div');
      sub.className = 'model-sub model-config';
      const subHead = document.createElement('div');
      subHead.className = 'model-sub-head';
      const subTitle = document.createElement('span');
      subTitle.className = 'model-sub-title';
      subTitle.textContent = `📂 ${m.name}`;
      // 目录最近修改时间(与卡片时间排序依据一致,便于核对顺序)
      const mt = modelLatestMtime(m);
      if (mt > 0) {
        const timeEl = document.createElement('span');
        timeEl.className = 'model-time';
        timeEl.textContent = humanTime(mt);
        timeEl.title = '目录内最新文件修改时间';
        subTitle.appendChild(timeEl);
      }
      const activeTag = document.createElement('span');
      activeTag.className = 'model-active-tag';
      activeTag.textContent = m.has_config
        ? (m.active ? `当前激活:${m.active}` : 'config.yaml 中无 model_name')
        : '缺 config.yaml';
      if (m.active) activeTag.classList.add('is-on');
      // 编辑配置按钮(位于上传按钮之前)
      const cfgBtn = document.createElement('button');
      cfgBtn.type = 'button';
      cfgBtn.className = 'model-cfg-edit-btn';
      cfgBtn.textContent = '✏ 编辑配置';
      cfgBtn.addEventListener('click', (ev) => { ev.preventDefault(); ev.stopPropagation(); openConfigEditor(); });
      // 上传按钮
      const upLabel = document.createElement('label');
      upLabel.className = 'model-up-label';
      upLabel.textContent = '⬆ 上传模型';
      const upInput = document.createElement('input');
      upInput.type = 'file';
      upInput.hidden = true;
      upInput.accept = '.onnx,.pt,.pth,.jit,.ckpt';
      upInput.addEventListener('change', async () => {
        const f = upInput.files && upInput.files[0];
        upInput.value = '';
        if (!f) return;
        await modelUpload(robot, m.name, f);
      });
      upLabel.appendChild(upInput);
      // 删除整个配置目录
      const delBtn = modelBtn('🗑 删除目录', 'act-del', async () => {
        const ok = await window.confirmDialog({
          title: '删除模型配置',
          message: `确认删除整个目录?\n${policyDir}/${robot}/${m.name}\n该目录下所有文件(含 config.yaml)都将被删除!`,
          confirmText: '删除', danger: true,
        });
        if (!ok) return;
        try {
          const resp = await fetch(modelUrl({ op: 'delete', robot, model: m.name }), { cache: 'no-store' });
          const j = await resp.json();
          if (!j.ok) throw new Error(j.error || `HTTP ${resp.status}`);
          modelRefresh();
        } catch (err) {
          await window.alertDialog({ title: '删除失败', message: err.message });
        }
      });
      subHead.append(subTitle, activeTag, cfgBtn, upLabel, delBtn);
      sub.appendChild(subHead);

      // 文件表(config.yaml 不再显示为行)
      const modelFiles = (m.files || []).filter((f) => f.name !== 'config.yaml');
      if (modelFiles.length) {
        const tbl = document.createElement('table');
        tbl.className = 'sys-ftp-table model-file-table';
        const thead = document.createElement('thead');
        thead.innerHTML = '<tr><th>模型文件</th><th>大小</th><th>修改时间</th><th>操作</th></tr>';
        tbl.appendChild(thead);
        const tbody = document.createElement('tbody');
        for (const f of modelFiles) {
          const tr = document.createElement('tr');
          const isActive = (f.name === m.active);
          if (isActive) tr.classList.add('is-active');
          const tdN = document.createElement('td');
          const nm = document.createElement('span');
          nm.className = 'model-file-name';
          nm.textContent = '🧠 ' + f.name;
          if (isActive) {
            const tag = document.createElement('span');
            tag.className = 'model-cur-tag';
            tag.textContent = '  ✓ 当前';
            nm.appendChild(tag);
          }
          tdN.appendChild(nm);
          if (notes[f.name]) {
            const noteEl = document.createElement('div');
            noteEl.className = 'model-note-text';
            noteEl.textContent = `📝 ${notes[f.name]}`;
            noteEl.title = notes[f.name];
            tdN.appendChild(noteEl);
          }
          const tdS = document.createElement('td');
          tdS.className = 'sys-ftp-size';
          tdS.textContent = humanSize(Number(f.size) || 0);
          const tdT = document.createElement('td');
          tdT.className = 'sys-ftp-time';
          tdT.textContent = humanTime(Number(f.mtime) || 0);
          const tdA = document.createElement('td');
          tdA.className = 'sys-ftp-act';
          // 备注
          tdA.appendChild(modelBtn('📝 备注', 'act-note', async () => {
            const val = await textAreaDialog({
              title: `模型备注 · ${f.name}`,
              message: `${robot}/${m.name}/${f.name}`,
              value: notes[f.name] || '',
              placeholder: '输入该模型的备注,清空保存则删除备注',
              okText: '保存备注',
              variant: 'note',
              hint: 'Esc 取消 · Ctrl+Enter 快速保存',
            });
            if (val == null) return;
            await saveNote(f.name, val);
          }));
          if (!isActive && m.has_config) {
            tdA.appendChild(modelBtn('启用', 'act-edit', async () => {
              const ok = await window.confirmDialog({
                title: '切换激活模型',
                message: `将 ${robot}/${m.name}/config.yaml 中的 model_name 改为:\n${f.name}\n\n无需重启服务:先趴下(退出强化学习),再重新进入强化学习即热加载新模型。`,
                confirmText: '启用',
              });
              if (!ok) return;
              try {
                const resp = await fetch(modelUrl({ op: 'activate', robot, model: m.name, file: f.name }), { cache: 'no-store' });
                const j = await resp.json();
                if (!j.ok) throw new Error(j.error || `HTTP ${resp.status}`);
                await window.alertDialog({ title: '已启用', message: j.hint || '已写入 config.yaml;趴下后再进入强化学习即加载新模型。' });
                modelRefresh();
              } catch (err) {
                await window.alertDialog({ title: '启用失败', message: err.message });
              }
            }));
          }
          tdA.appendChild(modelBtn('下载', 'act-dl', () => {
            const a = document.createElement('a');
            a.href = modelUrl({ op: 'download', robot, model: m.name, file: f.name });
            a.download = f.name;
            document.body.appendChild(a); a.click(); a.remove();
          }));
          tdA.appendChild(modelBtn('删除', 'act-del', async () => {
            const extra = isActive ? '\n⚠ 这是当前激活的模型文件,删除后服务下次启动将加载失败!' : '';
            const ok = await window.confirmDialog({
              title: '删除模型文件',
              message: `确认删除?\n${robot}/${m.name}/${f.name}${extra}`,
              confirmText: '删除', danger: true,
            });
            if (!ok) return;
            try {
              const resp = await fetch(modelUrl({ op: 'delete', robot, model: m.name, file: f.name }), { cache: 'no-store' });
              const j = await resp.json();
              if (!j.ok) throw new Error(j.error || `HTTP ${resp.status}`);
              modelRefresh();
            } catch (err) {
              await window.alertDialog({ title: '删除失败', message: err.message });
            }
          }));
          tr.append(tdN, tdS, tdT, tdA);
          tbody.appendChild(tr);
        }
        tbl.appendChild(tbody);
        sub.appendChild(tbl);
      } else {
        const nof = document.createElement('div');
        nof.className = 'sys-ftp-empty';
        nof.textContent = '(无模型文件,点击上方「⬆ 上传模型」添加)';
        sub.appendChild(nof);
      }
      wrap.textContent = '';
      wrap.appendChild(sub);
      }

      draw();
      // 异步加载备注,到位后重绘
      modelReadText(robot, m.name, MODEL_NOTES_FILE)
        .then((txt) => { if (txt != null) { try { notes = JSON.parse(txt); } catch { /* ignore */ } draw(); } })
        .catch(() => { /* 备注读取失败不影响主界面 */ });
      return wrap;
}

async function modelUpload(robot, sub, file) {
  if (els.modelStatus) els.modelStatus.textContent = `上传中: ${file.name} (${humanSize(file.size)})…`;
  try {
    const resp = await fetch(modelUrl({ op: 'upload', robot, model: sub }), {
      method: 'PUT',
      headers: { 'X-Filename': encodeURIComponent(file.name) },
      body: file,
    });
    const j = await resp.json().catch(() => ({}));
    if (!resp.ok || j.ok === false) throw new Error(j.error || `HTTP ${resp.status}`);
    if (els.modelStatus) els.modelStatus.textContent = `上传完成: ${file.name}`;
    modelRefresh();
  } catch (err) {
    if (els.modelStatus) els.modelStatus.textContent = `上传失败: ${err.message}`;
    await window.alertDialog({ title: '上传失败', message: `${file.name}\n${err.message}` });
  }
}

// ─────────────────────────────────────────────
//  电源 / HR 日志开关
// ─────────────────────────────────────────────
async function sendCommand(data) {
  if (!publishRaw({ type: 'command', data })) {
    await window.alertDialog({ title: '提示', message: 'WS 未连接,无法发送' });
    return false;
  }
  return true;
}

function setHrUi(on, text) {
  if (els.hrBtn) {
    els.hrBtn.textContent = on ? '⏸ 关闭高频记录' : '▶ 开启高频记录';
    els.hrBtn.classList.toggle('is-on', !!on);
  }
  if (els.hrStatus && text) els.hrStatus.textContent = text;
}

// ─────────────────────────────────────────────
//  机器人服务管理(等价 scripts/service.sh)
// ─────────────────────────────────────────────
async function svcSend(op) {
  return sendCommand(`svc ${op}`);
}

// 页签可见且在线时每 3s 轮询;不可见/离线时停
function svcPollStart() {
  if (state.svcPollTimer) return;
  // 首帧直接走原始通道(页面刚加载可能尚未连上,避免"WS 未连接"弹框)
  publishRaw({ type: 'command', data: 'svc status' });
  state.svcPollTimer = setInterval(() => {
    if (state.sysVisible && state.online && !state.svcBusy) svcSend('status');
  }, 3000);
}

function svcPollStop() {
  if (state.svcPollTimer) { clearInterval(state.svcPollTimer); state.svcPollTimer = null; }
}

// ── 系统监控轮询 ──
function sysInfoPollStart() {
  if (state.sysInfoTimer) return;
  publishRaw({ type: 'command', data: 'sysinfo' });
  state.sysInfoTimer = setInterval(() => {
    if (state.sysVisible && state.online) publishRaw({ type: 'command', data: 'sysinfo' });
  }, 3000);
}
function sysInfoPollStop() {
  if (state.sysInfoTimer) { clearInterval(state.sysInfoTimer); state.sysInfoTimer = null; }
}

function fmtUptime(s) {
  s = Math.floor(Number(s) || 0);
  const d = Math.floor(s / 86400); s %= 86400;
  const h = Math.floor(s / 3600); s %= 3600;
  const m = Math.floor(s / 60); s %= 60;
  if (d > 0) return `${d}天 ${h}时 ${m}分`;
  if (h > 0) return `${h}时 ${m}分 ${s}秒`;
  return `${m}分 ${s}秒`;
}

function renderSysInfo(d) {
  const cpu = Number(d.cpu_pct) || 0;
  const memTotal = Number(d.mem_total_kb) || 0;
  const memUsed = Number(d.mem_used_kb) || 0;
  const swapTotal = Number(d.swap_total_kb) || 0;
  const swapUsed = Number(d.swap_used_kb) || 0;
  const memPct = memTotal > 0 ? (memUsed / memTotal * 100) : 0;
  const swapPct = swapTotal > 0 ? (swapUsed / swapTotal * 100) : 0;

  const el = (id) => document.getElementById(id);
  if (el('sys-cpu-pct')) el('sys-cpu-pct').textContent = cpu.toFixed(1) + '%';
  if (el('sys-cpu-bar')) el('sys-cpu-bar').style.width = Math.min(cpu, 100) + '%';
  if (el('sys-mem-info')) el('sys-mem-info').textContent = `${(memUsed/1024/1024).toFixed(1)}/${(memTotal/1024/1024).toFixed(1)} GB`;
  if (el('sys-mem-bar')) el('sys-mem-bar').style.width = memPct.toFixed(0) + '%';
  if (el('sys-swap-info')) el('sys-swap-info').textContent = swapTotal > 0 ? `${(swapUsed/1024/1024).toFixed(1)}/${(swapTotal/1024/1024).toFixed(1)} GB` : '无';
  if (el('sys-swap-bar')) el('sys-swap-bar').style.width = swapPct.toFixed(0) + '%';
  if (el('sys-tasks')) el('sys-tasks').textContent = d.tasks || 0;
  if (el('sys-load')) el('sys-load').textContent = `${Number(d.load1).toFixed(2)} / ${Number(d.load5).toFixed(2)} / ${Number(d.load15).toFixed(2)}`;
  if (el('sys-uptime')) el('sys-uptime').textContent = fmtUptime(d.uptime_s);
  // 磁盘占用
  const disksEl = el('sys-disks');
  if (disksEl) {
    const disks = Array.isArray(d.disks) ? d.disks : [];
    if (disks.length === 0) {
      disksEl.innerHTML = '<div class="sys-disk-row"><span class="sys-disk-mount">无</span></div>';
    } else {
      disksEl.innerHTML = disks.map(dk => {
        const total = Number(dk.total_kb) || 0;
        const used = Number(dk.used_kb) || 0;
        const pct = total > 0 ? (used / total * 100) : 0;
        const cls = pct >= 90 ? 'danger' : (pct >= 75 ? 'warn' : '');
        const totalGb = (total / 1024 / 1024).toFixed(1);
        const usedGb = (used / 1024 / 1024).toFixed(1);
        return `<div class="sys-disk-row">
          <span class="sys-disk-mount" title="${dk.fs || ''}">${dk.mount}</span>
          <div class="sys-disk-bar"><div class="sys-disk-bar-fill ${cls}" style="width:${pct.toFixed(0)}%"></div></div>
          <span class="sys-disk-info">${usedGb}/${totalGb} GB (${pct.toFixed(0)}%)</span>
        </div>`;
      }).join('');
    }
  }
  if (el('sys-monitor-status')) { el('sys-monitor-status').textContent = '正常'; el('sys-monitor-status').className = 'sys-ftp-status is-on'; }
}

// ── 服务列表(带分页+搜索) ──
function renderSvcList(units) {
  state.svcAllUnits = units || [];
  renderSvcPage();
}

function renderSvcPage() {
  const body = document.getElementById('sys-svclist-body');
  if (!body) return;
  const q = state.svcSearch.toLowerCase();
  const filtered = q
    ? state.svcAllUnits.filter(u =>
        (u.unit || '').toLowerCase().includes(q) ||
        (u.desc || '').toLowerCase().includes(q))
    : state.svcAllUnits;
  const total = filtered.length;
  const totalPages = Math.max(1, Math.ceil(total / state.svcPageSize));
  if (state.svcPage > totalPages) state.svcPage = totalPages;
  const start = (state.svcPage - 1) * state.svcPageSize;
  const pageItems = filtered.slice(start, start + state.svcPageSize);

  body.innerHTML = '';
  pageItems.forEach(u => {
    const tr = document.createElement('tr');
    const activeCls = u.active === 'active' ? 'state-active' : (u.active === 'failed' ? 'state-failed' : 'state-inactive');
    const isActive = u.active === 'active';
    // 启停合并: active 显示"停止",否则"启动"
    const powerOp = isActive ? 'stop' : 'start';
    const powerText = isActive ? '停止' : '启动';
    // 自启/禁启合并: 前端维护 enable 切换状态,初始默认未自启
    if (state.svcEnableState[u.unit] === undefined) state.svcEnableState[u.unit] = false;
    const enabled = state.svcEnableState[u.unit];
    const autoOp = enabled ? 'disable' : 'enable';
    const autoText = enabled ? '禁启' : '自启';
    // systemd 状态英文→中文映射
    const svcLoadMap = { loaded: '已加载', not_found: '未找到', masked: '已屏蔽', bad: '错误' };
    const svcActiveMap = { active: '运行中', inactive: '未运行', activating: '启动中', deactivating: '停止中', failed: '失败', reloading: '重载中' };
    const svcSubMap = {
      running: '运行中', dead: '已停止', exited: '已退出', failed: '失败',
      start: '启动中', stop: '停止中', stop_sigterm: '终止中', stop_sigkill: '强杀中',
      auto_restart: '自动重启', waiting: '等待中', condition: '条件检查',
    };
    const loadTxt = svcLoadMap[u.load] || u.load;
    const activeTxt = svcActiveMap[u.active] || u.active;
    const subTxt = svcSubMap[u.sub] || u.sub;
    tr.innerHTML = `
      <td>${u.unit}</td>
      <td>${loadTxt}</td>
      <td class="${activeCls}">${activeTxt}</td>
      <td>${subTxt}</td>
      <td class="cmd-cell" title="${u.desc || ''}">${u.desc || ''}</td>
      <td class="sys-ftp-act"></td>`;
    const actCell = tr.querySelector('td:last-child');
    // 直出:停止/启动、重启、自启/禁启
    actCell.appendChild(ftpActBtn(powerText, 'act-power', () => publishRaw({ type: 'command', data: `svc ${powerOp} ${u.unit}` })));
    actCell.appendChild(ftpActBtn('重启', 'act-restart', () => publishRaw({ type: 'command', data: `svc restart ${u.unit}` })));
    actCell.appendChild(ftpActBtn(autoText, 'act-auto', () => {
      publishRaw({ type: 'command', data: `svc ${autoOp} ${u.unit}` });
      state.svcEnableState[u.unit] = (autoOp === 'enable');
      renderSvcPage();
    }));
    // 更多下拉:日志、编辑、删除
    actCell.appendChild(ftpMoreBtn([
      { label: '日志', cls: 'act-log', fn: () => publishRaw({ type: 'command', data: `svc log ${u.unit}` }) },
      { label: '编辑', cls: 'act-edit', fn: () => publishRaw({ type: 'command', data: `svc file ${u.unit}` }) },
      { label: '删除', cls: 'act-del', fn: async () => {
        const ok = await window.confirmDialog({ title: '删除服务', message: `确定删除服务 ${u.unit} ?`, confirmText: '删除', danger: true });
        if (ok) publishRaw({ type: 'command', data: `svc delete ${u.unit}` });
      } },
    ]));
    body.appendChild(tr);
  });
  if (pageItems.length === 0) {
    body.innerHTML = '<tr><td colspan="6" style="text-align:center;color:var(--c-mute,#8a99a8)">无匹配服务</td></tr>';
  }
  renderPager('sys-svclist', state.svcPage, totalPages, total);
}

// 服务编辑:复用文件管理的 CodeMirror 编辑器
async function openSvcEditor(unit, content) {
  const path = '/etc/systemd/system/' + unit;
  ftpModalOpen(`编辑服务: ${unit}`);
  els.modalBox.classList.add('ftp-modal-editor');
  const seq = ++editorState.seq;
  editorState.cm = null;
  editorState.ta = null;
  editorBuildRaw(path, unit, { content: content || '', size: (content || '').length }, { noTree: true });
  // 后台懒加载 CodeMirror
  let CM;
  try {
    CM = await loadCodeMirror();
  } catch {
    editorSetBadge('⚠ 语法高亮加载失败,已使用兼容模式');
    return;
  }
  if (editorState.seq !== seq || els.modal.hidden) return;
  await editorUpgrade(CM, seq);
}

function showSvcLog(unit, lines) {
  window.ensureUiFeedbackDom && window.ensureUiFeedbackDom();
  const existing = document.getElementById('svc-log-modal');
  if (existing) existing.remove();
  const mask = document.createElement('div');
  mask.className = 'ui-modal-mask';
  mask.id = 'svc-log-modal';
  const esc = (s) => String(s).replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
  const content = lines.length > 0
    ? lines.map(l => `<div class="logview-line">${esc(l)}</div>`).join('')
    : '<div style="color:var(--c-mute,#8a99a8);text-align:center;padding:20px">暂无日志</div>';
  mask.innerHTML = `
    <div class="ui-modal ui-modal-lg" role="dialog" aria-modal="true">
      <h3 class="ui-modal-title">服务日志</h3>
      <p class="ui-modal-msg"></p>
      <div class="ui-modal-log-screen">${content}</div>
      <div class="ui-modal-actions">
        <button type="button" class="ui-modal-ok">关闭</button>
      </div>
    </div>`;
  mask.querySelector('.ui-modal-msg').textContent = unit;
  document.body.appendChild(mask);
  let settled = false;
  const close = () => {
    if (settled) return;
    settled = true;
    mask.classList.remove('ui-modal-show');
    setTimeout(() => mask.remove(), 150);
  };
  mask.addEventListener('click', (e) => { if (e.target === mask) close(); });
  mask.querySelector('.ui-modal-ok').addEventListener('click', close);
  requestAnimationFrame(() => {
    mask.classList.add('ui-modal-show');
    const screen = mask.querySelector('.ui-modal-log-screen');
    if (screen) screen.scrollTop = screen.scrollHeight;
  });
}

// ── 进程列表(带分页+搜索) ──
function renderPsList(procs) {
  state.psAllProcs = procs || [];
  renderPsPage();
}

function renderPsPage() {
  const body = document.getElementById('sys-ps-body');
  if (!body) return;
  const q = state.psSearch.toLowerCase();
  const filtered = q
    ? state.psAllProcs.filter(p =>
        String(p.pid).includes(q) ||
        (p.comm || '').toLowerCase().includes(q) ||
        (p.cmd || '').toLowerCase().includes(q))
    : state.psAllProcs;
  const total = filtered.length;
  const totalPages = Math.max(1, Math.ceil(total / state.psPageSize));
  if (state.psPage > totalPages) state.psPage = totalPages;
  const start = (state.psPage - 1) * state.psPageSize;
  const pageItems = filtered.slice(start, start + state.psPageSize);

  body.innerHTML = '';
  pageItems.forEach(p => {
    const tr = document.createElement('tr');
    const stateCls = `state-${p.state}`;
    // 进程状态码→中文
    const procStateMap = {
      R: '运行', S: '睡眠', D: '不可中断', T: '停止', t: '跟踪停止',
      Z: '僵尸', X: '死亡', I: '空闲', P: '等待',
    };
    const stateTxt = procStateMap[p.state] || p.state;
    tr.innerHTML = `
      <td class="pid-cell" data-pid="${p.pid}" title="点击查看完整命令行">${p.pid}</td>
      <td>${p.ppid}</td>
      <td>${p.comm}</td>
      <td class="${stateCls}">${stateTxt}</td>
      <td>${humanSize((p.rss_kb || 0) * 1024)}</td>
      <td>${Number(p.cpu_s).toFixed(1)}</td>
      <td class="cmd-cell" title="${p.cmd || ''}">${p.cmd || ''}</td>
      <td class="sys-ftp-act"></td>`;
    const actCell = tr.querySelector('td:last-child');
    actCell.appendChild(ftpActBtn('结束', '', async () => {
      const ok = await window.confirmDialog({ title: '结束进程', message: `确定结束进程 ${p.pid} ?`, confirmText: '结束' });
      if (ok) publishRaw({ type: 'command', data: `ps kill ${p.pid}` });
    }));
    actCell.appendChild(ftpActBtn('强杀', 'act-del', async () => {
      const ok = await window.confirmDialog({ title: '强制结束进程', message: `确定强制结束进程 ${p.pid} ?`, confirmText: '强杀', danger: true });
      if (ok) publishRaw({ type: 'command', data: `ps kill9 ${p.pid}` });
    }));
    body.appendChild(tr);
  });
  if (pageItems.length === 0) {
    body.innerHTML = '<tr><td colspan="8" style="text-align:center;color:var(--c-mute,#8a99a8)">无匹配进程</td></tr>';
  }
  body.querySelectorAll('.pid-cell').forEach(td => {
    td.addEventListener('click', () => {
      const pid = td.dataset.pid;
      const cmd = td.parentElement.querySelector('.cmd-cell').title;
      window.alertDialog({ title: `进程 ${pid}`, message: `命令行: ${cmd}` });
    });
  });
  renderPager('sys-ps', state.psPage, totalPages, total);
}

// ── 通用分页渲染 ──
// ── 系统日志(本次启动 journalctl -b) ──
function renderSyslog(lines) {
  state.syslogLines = Array.isArray(lines) ? lines : [];
  drawSyslog();
}
function drawSyslog() {
  const screen = document.getElementById('sys-syslog-screen');
  const countEl = document.getElementById('sys-syslog-count');
  const statusEl = document.getElementById('sys-syslog-status');
  if (!screen) return;
  const q = state.syslogFilter.toLowerCase();
  const filtered = q ? state.syslogLines.filter(l => l.toLowerCase().includes(q)) : state.syslogLines;
  // 最多渲染 3000 行避免 DOM 过大
  const shown = filtered.slice(-3000);
  const esc = (s) => s.replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
  screen.innerHTML = shown.map(l => `<div class="logview-line">${esc(l)}</div>`).join('');
  if (countEl) countEl.textContent = `${filtered.length} 行${filtered.length !== state.syslogLines.length ? ` (共 ${state.syslogLines.length})` : ''}`;
  if (statusEl) { statusEl.textContent = state.syslogLines.length > 0 ? '已加载' : '无数据'; statusEl.className = 'sys-ftp-status is-on'; }
  const auto = document.getElementById('sys-syslog-autoscroll');
  if (auto && auto.checked) screen.scrollTop = screen.scrollHeight;
}

// ── 网络管理(ip addr 等价结构化展示) ──
const NET_TYPE_TEXT = { 1: '以太网', 772: '回环', 280: 'CAN', 801: '无线', 776: '隧道', 778: '隧道', 65534: '虚拟' };

// ── 网络配置(netcfg): 多IP/掩码/网关/DNS, netplan 持久化 ──
function addNetCfgIpRow(addr = '', prefix = 24) {
  const wrap = document.getElementById('netcfg-ip-rows');
  if (!wrap) return;
  const row = document.createElement('div');
  row.className = 'netcfg-ip-row';
  row.style.cssText = 'display:flex;gap:6px;align-items:center';
  const ip = document.createElement('input');
  ip.className = 'netcfg-ip';
  ip.type = 'text';
  ip.placeholder = '192.168.1.50';
  ip.value = addr;
  ip.autocomplete = 'off';
  ip.style.cssText = 'width:190px;padding:4px 8px';
  const sel = document.createElement('select');
  sel.className = 'netcfg-prefix';
  sel.style.cssText = 'width:90px;padding:4px 4px';
  for (const p of [8, 16, 20, 22, 24, 25, 26, 27, 28, 30, 32]) {
    const o = document.createElement('option');
    o.value = p;
    o.textContent = '/' + p;
    if (p === prefix) o.selected = true;
    sel.appendChild(o);
  }
  const del = document.createElement('button');
  del.type = 'button';
  del.className = 'ros-btn';
  del.textContent = '✕';
  del.title = '删除此 IP';
  del.style.cssText = 'font-size:12px;padding:2px 8px';
  del.addEventListener('click', () => row.remove());
  row.append(ip, sel, del);
  wrap.appendChild(row);
}

function renderNetConfig(data) {
  const modeEl = document.getElementById('netcfg-mode');
  const ifaceEl = document.getElementById('netcfg-iface');
  const gwEl = document.getElementById('netcfg-gw');
  const dnsEl = document.getElementById('netcfg-dns');
  const rows = document.getElementById('netcfg-ip-rows');
  const routeRows = document.getElementById('netcfg-route-rows');
  const policyRows = document.getElementById('netcfg-policy-rows');
  if (!ifaceEl) return; // 移动端无此面板
  const prev = ifaceEl.value;
  ifaceEl.innerHTML = '';
  for (const it of (data.ifaces || [])) {
    const o = document.createElement('option');
    o.value = it.name;
    const ips = (it.ips || []).map((x) => `${x.addr}/${x.prefix}`).join(', ');
    o.textContent = it.name + (ips ? ` (${ips})` : ' (无 IPv4)');
    ifaceEl.appendChild(o);
  }
  if (prev && [...ifaceEl.options].some((o) => o.value === prev)) ifaceEl.value = prev;
  if (data.gateway && data.gateway.ip && gwEl) gwEl.value = data.gateway.ip;
  if (data.dns && data.dns.length && dnsEl) dnsEl.value = data.dns.join(',');
  if (rows) {
    rows.innerHTML = '';
    const cur = (data.ifaces || []).find((x) => x.name === ifaceEl.value);
    const ips = (cur && cur.ips) || [];
    if (ips.length) ips.forEach((x) => addNetCfgIpRow(x.addr, x.prefix));
    else addNetCfgIpRow();
  }
  if (routeRows) {
    routeRows.innerHTML = '';
    (data.routes || []).forEach((r) => addNetCfgRouteRow(r.to, r.via || '', r.table || ''));
  }
  if (policyRows) {
    policyRows.innerHTML = '';
    (data.rules || []).forEach((p) => addNetCfgPolicyRow(p.from, p.table || ''));
  }
  if (modeEl) {
    modeEl.textContent = data.override ? '静态接管中(存在覆盖文件)' : 'DHCP 模式(无覆盖文件)';
  }
}

function addNetCfgRouteRow(to = '', via = '', table = '') {
  const wrap = document.getElementById('netcfg-route-rows');
  if (!wrap) return;
  const row = document.createElement('div');
  row.className = 'netcfg-route-row';
  row.style.cssText = 'display:flex;gap:4px;align-items:center';
  row.innerHTML = `
    <input class="netcfg-route-to" value="${to}" placeholder="目标 CIDR 或 default" style="flex:2;min-width:120px;font-size:11px;padding:1px 6px">
    <input class="netcfg-route-via" value="${via}" placeholder="via 下一跳 IP" style="flex:2;min-width:100px;font-size:11px;padding:1px 6px">
    <input class="netcfg-route-table" value="${table}" placeholder="table" style="width:60px;font-size:11px;padding:1px 6px">
    <button type="button" class="ros-btn" style="font-size:10px;padding:1px 6px" onclick="this.parentElement.remove()">✕</button>
  `;
  wrap.appendChild(row);
}

function addNetCfgPolicyRow(from = '', table = '') {
  const wrap = document.getElementById('netcfg-policy-rows');
  if (!wrap) return;
  const row = document.createElement('div');
  row.className = 'netcfg-policy-row';
  row.style.cssText = 'display:flex;gap:4px;align-items:center';
  row.innerHTML = `
    <input class="netcfg-policy-from" value="${from}" placeholder="源 CIDR" style="flex:2;min-width:140px;font-size:11px;padding:1px 6px">
    <input class="netcfg-policy-table" value="${table}" placeholder="table 路由表号" style="flex:1;min-width:100px;font-size:11px;padding:1px 6px">
    <button type="button" class="ros-btn" style="font-size:10px;padding:1px 6px" onclick="this.parentElement.remove()">✕</button>
  `;
  wrap.appendChild(row);
}

function renderNetConfigResult(data) {
  const m = document.getElementById('netcfg-mode');
  if (m) {
    const first = String(data.message || '').split('\n')[0];
    m.textContent = (data.ok ? '✓ ' : '✗ ') + first.slice(0, 64);
  }
}

function collectNetCfg() {
  const iface = document.getElementById('netcfg-iface')?.value || '';
  const gw = (document.getElementById('netcfg-gw')?.value || '').trim();
  const dns = (document.getElementById('netcfg-dns')?.value || '').trim();
  const lines = [`iface ${iface}`];
  let count = 0;
  document.querySelectorAll('#netcfg-ip-rows .netcfg-ip-row').forEach((row) => {
    const a = row.querySelector('.netcfg-ip')?.value.trim();
    const p = row.querySelector('.netcfg-prefix')?.value || '24';
    if (a) { lines.push(`ip ${a} ${p}`); count++; }
  });
  document.querySelectorAll('#netcfg-route-rows .netcfg-route-row').forEach((row) => {
    const to = row.querySelector('.netcfg-route-to')?.value.trim();
    const via = row.querySelector('.netcfg-route-via')?.value.trim();
    const table = row.querySelector('.netcfg-route-table')?.value.trim();
    if (to && via) {
      let ln = `route ${to} ${via}`;
      if (table) ln += ` table ${table}`;
      lines.push(ln);
    }
  });
  document.querySelectorAll('#netcfg-policy-rows .netcfg-policy-row').forEach((row) => {
    const from = row.querySelector('.netcfg-policy-from')?.value.trim();
    const table = row.querySelector('.netcfg-policy-table')?.value.trim();
    if (from && table) lines.push(`policy ${from} ${table}`);
  });
  if (gw) lines.push(`gw ${gw}`);
  if (dns) lines.push(`dns ${dns}`);
  return { lines, iface, count };
}

function applyNetCfg() {
  const { lines, iface, count } = collectNetCfg();
  if (!iface) return;
  if (!count) { alert('请至少添加一个 IP'); return; }
  const ipList = lines.filter((l) => l.startsWith('ip ')).map((l) => l.slice(3)).join(', ');
  const gwLine = lines.find((l) => l.startsWith('gw ')) || '(不设默认路由)';
  const routeLines = lines.filter((l) => l.startsWith('route '));
  const policyLines = lines.filter((l) => l.startsWith('policy '));
  let routeBlock = '';
  if (routeLines.length) routeBlock = `\n静态路由:\n  ${routeLines.join('\n  ')}`;
  let policyBlock = '';
  if (policyLines.length) policyBlock = `\n策略路由:\n  ${policyLines.join('\n  ')}`;
  const ok = confirm(
    `确认应用静态网络配置?\n\n接口: ${iface}\nIP: ${ipList}\n网关: ${gwLine}${routeBlock}${policyBlock}\n\n` +
    '将写入 /etc/netplan/90-jxg-net.yaml 并执行 netplan try(120s 后自动回滚,重启持久)。\n' +
    '若修改的是当前正在使用的管理地址,连接会瞬时断开(刷新重连即可)。\n' +
    '误配可点「恢复 DHCP」一键回滚。'
  );
  if (ok) sendCommand('netcfg apply\n' + lines.join('\n'));
}

function restoreNetCfg() {
  const ok = confirm('确认恢复 DHCP?\n将删除静态覆盖文件(留有 .bak 备份)并执行 netplan try(120s 自动回滚)。');
  if (ok) sendCommand('netcfg restore');
}

function renderNetList(ifaces) {
  state.netRows = Array.isArray(ifaces) ? ifaces : [];
  const statusEl = document.getElementById('sys-net-status');
  if (statusEl) { statusEl.textContent = state.netRows.length > 0 ? '已加载' : '无数据'; statusEl.className = 'sys-ftp-status is-on'; }
  renderNetRows();
}

function renderNetRows() {
  const body = document.getElementById('sys-net-body');
  if (!body) return;
  const q = state.netSearch.trim().toLowerCase();
  const rows = q ? state.netRows.filter(n => {
    const hay = [n.name, n.mac, n.operstate, ...(Array.isArray(n.v4) ? n.v4 : []), ...(Array.isArray(n.v6) ? n.v6 : [])].join(' ').toLowerCase();
    return hay.includes(q);
  }) : state.netRows;
  const humanBytes = (x) => {
    x = Number(x) || 0;
    if (x < 1024) return `${x} B`;
    if (x < 1048576) return `${(x / 1024).toFixed(1)} K`;
    if (x < 1073741824) return `${(x / 1048576).toFixed(1)} M`;
    return `${(x / 1073741824).toFixed(2)} G`;
  };
  body.innerHTML = rows.map(n => {
    const stRaw = !n.up ? 'DOWN' : (n.running ? 'UP' : 'NO-CARRIER');
    const stMap = { UP: '已连接', DOWN: '已断开', 'NO-CARRIER': '无载波' };
    const st = stMap[stRaw] || stRaw;
    const stCls = stRaw === 'UP' ? 'state-active' : (stRaw === 'DOWN' ? 'state-inactive' : 'state-failed');
    const type = NET_TYPE_TEXT[Number(n.type)] || (Number(n.loopback) ? '回环' : `type ${n.type}`);
    const v4 = (Array.isArray(n.v4) && n.v4.length) ? n.v4.join(', ') : '-';
    const v6 = (Array.isArray(n.v6) && n.v6.length) ? n.v6.join(', ') : '-';
    const rx = humanBytes(n.rx) + (Number(n.rx_err) ? ` (错误包 ${n.rx_err})` : '');
    const tx = humanBytes(n.tx) + (Number(n.tx_err) ? ` (错误包 ${n.tx_err})` : '');
    return `<tr>
      <td title="${n.operstate || ''}">${n.name}</td>
      <td>${type}</td>
      <td class="${stCls}">${st}</td>
      <td class="cmd-cell" title="${v4}">${v4}</td>
      <td class="cmd-cell" title="${v6}">${v6}</td>
      <td title="${n.mac || ''}">${n.mac || '-'}</td>
      <td>${n.mtu || '-'}</td>
      <td title="收 ${rx} / 发 ${tx}">${rx} / ${tx}</td>
    </tr>`;
  }).join('');
  if (rows.length === 0) {
    body.innerHTML = '<tr><td colspan="8" style="text-align:center;color:var(--c-mute,#8a99a8)">无匹配接口</td></tr>';
  }
}

// ip addr 原始输出弹窗
function showNetRawModal(lines) {
  window.ensureUiFeedbackDom && window.ensureUiFeedbackDom();
  const existing = document.getElementById('net-raw-modal');
  if (existing) existing.remove();
  const mask = document.createElement('div');
  mask.className = 'ui-modal-mask';
  mask.id = 'net-raw-modal';
  const esc = (s) => String(s).replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
  const content = lines.length > 0
    ? lines.map(l => `<div class="logview-line">${esc(l)}</div>`).join('')
    : '<div style="color:var(--c-mute,#8a99a8);text-align:center;padding:20px">无输出</div>';
  mask.innerHTML = `
    <div class="ui-modal ui-modal-lg" role="dialog" aria-modal="true">
      <h3 class="ui-modal-title">ip addr 原始输出</h3>
      <p class="ui-modal-msg"></p>
      <div class="ui-modal-log-screen">${content}</div>
      <div class="ui-modal-actions">
        <button type="button" class="ui-modal-ok">关闭</button>
      </div>
    </div>`;
  document.body.appendChild(mask);
  let settled = false;
  const close = () => {
    if (settled) return;
    settled = true;
    mask.classList.remove('ui-modal-show');
    setTimeout(() => mask.remove(), 150);
  };
  mask.addEventListener('click', (e) => { if (e.target === mask) close(); });
  mask.querySelector('.ui-modal-ok').addEventListener('click', close);
  requestAnimationFrame(() => {
    mask.classList.add('ui-modal-show');
    const screen = mask.querySelector('.ui-modal-log-screen');
    if (screen) screen.scrollTop = 0;
  });
}

// ── 设备管理器(/dev 顶层枚举) ──
const DEV_TYPE_TEXT = { char: '字符设备', block: '块设备', dir: '目录', link: '符号链接', fifo: '管道', sock: '套接字', file: '文件' };

function devCategory(name) {
  if (/^can\d+$/.test(name)) return 'CAN';
  if (/^tty(USB|ACM|HS|AMA|WL|MAX)/.test(name) || name === 'tty') return '终端/串口';
  if (/^ttyS\d+$/.test(name)) return '串口';
  if (/^(sd[a-z]+|nvme\d+(n\d+)?(p\d+)?|vd[a-z]+|mmcblk\d+(p\d+)?|hd[a-z]+|sr\d+)$/.test(name)) return '磁盘';
  if (/^loop\d+$/.test(name)) return '回环设备';
  if (/^video\d+$/.test(name)) return '摄像头';
  if (/^(null|zero|full|random|urandom)$/.test(name)) return '虚拟设备';
  if (/^i2c-\d+$/.test(name)) return 'I2C';
  if (/^hidraw\d+$/.test(name)) return 'HID';
  if (/^ptmx$|^pts$/.test(name)) return '伪终端';
  return '';
}

function renderDevList(devs) {
  state.devRows = Array.isArray(devs) ? devs : [];
  const statusEl = document.getElementById('sys-dev-status');
  if (statusEl) { statusEl.textContent = state.devRows.length > 0 ? `${state.devRows.length} 项` : '无数据'; statusEl.className = 'sys-ftp-status is-on'; }
  renderDevRows();
}

function renderDevRows() {
  const body = document.getElementById('sys-dev-body');
  if (!body) return;
  const q = state.devSearch.trim().toLowerCase();
  const rows = q ? state.devRows.filter(d => `${d.name} ${d.type} ${d.link || ''}`.toLowerCase().includes(q)) : state.devRows;
  body.innerHTML = rows.map(d => {
    const isDev = d.type === 'char' || d.type === 'block';
    const mm = isDev ? `${d.major}:${d.minor}` : '-';
    const cat = devCategory(d.name) || '-';
    const size = (d.type === 'file' || d.type === 'dir') ? humanSize(Number(d.size) || 0) : '-';
    return `<tr>
      <td title="/dev/${d.name}">${d.name}</td>
      <td>${DEV_TYPE_TEXT[d.type] || d.type}</td>
      <td>${cat}</td>
      <td>${mm}</td>
      <td>${size}</td>
      <td class="cmd-cell" title="${d.link || ''}">${d.link || '-'}</td>
    </tr>`;
  }).join('');
  if (rows.length === 0) {
    body.innerHTML = '<tr><td colspan="6" style="text-align:center;color:var(--c-mute,#8a99a8)">无匹配设备</td></tr>';
  }
}

function renderPager(prefix, page, totalPages, total) {
  const pager = document.getElementById(`${prefix}-pager`);
  if (!pager) return;
  const nums = document.getElementById(`${prefix}-page-nums`);
  const prev = document.getElementById(`${prefix}-page-prev`);
  const next = document.getElementById(`${prefix}-page-next`);
  if (!nums) return;
  prev.disabled = page <= 1;
  next.disabled = page >= totalPages;
  // 页码: 当前页前后各2页 + 首页/末页
  const pages = new Set([1, totalPages]);
  for (let i = page - 2; i <= page + 2; i++) {
    if (i >= 1 && i <= totalPages) pages.add(i);
  }
  const sorted = [...pages].sort((a, b) => a - b);
  let html = '';
  let last = 0;
  sorted.forEach(p => {
    if (p - last > 1) html += '<span style="color:var(--c-mute,#8a99a8)">…</span>';
    html += `<button class="ros-btn ${p === page ? 'active' : ''}" data-page="${p}" style="padding:2px 8px;font-size:12px">${p}</button>`;
    last = p;
  });
  nums.innerHTML = html + `<span style="color:var(--c-mute,#8a99a8);margin-left:8px">共 ${total} 条</span>`;
  nums.querySelectorAll('button[data-page]').forEach(b => {
    b.addEventListener('click', () => {
      if (prefix === 'sys-svclist') { state.svcPage = Number(b.dataset.page); renderSvcPage(); }
      else { state.psPage = Number(b.dataset.page); renderPsPage(); }
    });
  });
}

function renderSvcStatus(d) {
  state.svcBusy = !!d.busy;
  const active = d.active || 'unknown';
  const enabled = d.enabled || 'unknown';
  const activeOn = active === 'activating' || active === 'active';
  const activeText = { active: '运行中', inactive: '已停止', activating: '启动中…', deactivating: '停止中…', failed: '失败', unknown: '未知' }[active] || active;
  const enabledText = { enabled: '已开启', disabled: '已关闭', 'static': '静态', 'indirect': '间接开启', unknown: '未知' }[enabled] || enabled;

  els.svcStatus.textContent = d.busy ? `执行中:${d.op}…` : activeText;
  els.svcStatus.className = 'sys-ftp-status ' + (activeOn ? 'is-on' : (active === 'failed' ? 'is-err' : 'is-warn'));
  els.svcMetrics.innerHTML =
    `运行状态:<b class="${activeOn ? 'svc-val-on' : 'svc-val-off'}">${activeText}</b> &nbsp; ` +
    `开机自启:<b>${enabledText}</b> &nbsp; PID:<b>${d.pid || '-'}</b> &nbsp; ` +
    `FIFO 线程:<b class="${d.pid && d.fifo >= d.fifo_total ? 'svc-val-on' : (d.pid ? 'svc-val-warn' : '')}">${d.pid ? d.fifo + '/' + d.fifo_total : '-'}</b> &nbsp; ` +
    `cgroup:<b>${d.cgroup || '-'}</b>`;

  // 按钮可用性: 变更操作进行中全部禁用
  for (const b of [els.svcStart, els.svcStop, els.svcRestart, els.svcEnable, els.svcDisable]) {
    if (b) b.disabled = !!d.busy;
  }
  // 按当前状态强调/禁用无意义按钮
  if (els.svcStart) els.svcStart.disabled = !!(d.busy || activeOn);
  if (els.svcStop) els.svcStop.disabled = !!(d.busy || !d.pid);
  if (els.svcEnable) els.svcEnable.disabled = !!(d.busy || enabled === 'enabled');
  if (els.svcDisable) els.svcDisable.disabled = !!(d.busy || enabled === 'disabled');

  if (d.op && d.op !== 'status') {
    if (d.ok && !d.busy) {
      els.svcMsg.textContent = `✓ ${d.op} 完成`;
      els.svcMsg.className = 'sys-svc-msg is-ok';
    } else if (!d.ok) {
      els.svcMsg.textContent = `✗ ${d.op}: ${d.error || '失败'}`;
      els.svcMsg.className = 'sys-svc-msg is-err';
    } else {
      els.svcMsg.textContent = `正在执行 ${d.op} …(重启约需 10~20 秒,期间遥测中断,本页面连接不断)`;
      els.svcMsg.className = 'sys-svc-msg is-busy';
    }
  }
  if (!d.installed) {
    els.svcMsg.textContent = '服务未安装(/etc/systemd/system/rl-real-jxg.service 不存在)';
    els.svcMsg.className = 'sys-svc-msg is-err';
  }
}

// ─────────────────────────────────────────────
//  页签可见性(main.js 驱动)
// ─────────────────────────────────────────────
export function setTerminalVisible(visible) {
  state.termVisible = !!visible;
  if (!state.inited) return;
  // 终端: 进入页签即自动打开(无需手点「打开终端」);切走关闭 PTY(释放资源)
  if (visible) {
    if (!state.termReady && !state.termWanted) {
      termConnect();
    } else if (state.termWanted && state.online && !state.termReady) {
      termSendStart();
    }
    setTimeout(() => { try { state.fitAddon && state.fitAddon.fit(); } catch { /* */ } }, 50);
  } else {
    if (state.termReady) {
      publishRaw({ type: 'ssh_stop' });
      state.termReady = false;
    }
  }
}

export function setSystemVisible(visible) {
  state.sysVisible = !!visible;
  if (!state.inited) return;
  // 系统页: 可见时查 HR 日志状态并轮询服务;不可见时停轮询
  if (visible) {
    publishRaw({ type: 'command', data: 'log status' });
    if (state.online) svcPollStart();
    sysInfoPollStart();
  } else {
    svcPollStop();
    sysInfoPollStop();
  }
}

// 服务管理页签
export function setServicesVisible(visible) {
  state.servicesVisible = !!visible;
  if (!state.inited) return;
  if (visible && state.online) {
    publishRaw({ type: 'command', data: 'svc list' });
  }
}

// 系统日志页签
export function setSyslogVisible(visible) {
  state.syslogVisible = !!visible;
  if (!state.inited) return;
  if (visible && state.online && state.syslogLines.length === 0) {
    // 首次进入自动加载日志
    const n = document.getElementById('sys-syslog-lines')?.value || '2000';
    publishRaw({ type: 'command', data: `syslog ${n}` });
  }
}

// 进程管理页签
export function setProcessesVisible(visible) {
  state.processesVisible = !!visible;
  if (!state.inited) return;
  if (visible && state.online) {
    publishRaw({ type: 'command', data: 'ps list' });
  }
}

// 网络管理页签(ip addr)
export function setNetVisible(visible) {
  state.netVisible = !!visible;
  if (!state.inited) return;
  if (visible && state.online) {
    publishRaw({ type: 'command', data: 'net list' });
  }
}

// 设备管理页签(/dev)
export function setDevVisible(visible) {
  state.devVisible = !!visible;
  if (!state.inited) return;
  if (visible && state.online) {
    publishRaw({ type: 'command', data: 'dev list' });
  }
}

export function setFilesVisible(visible) {
  state.filesVisible = !!visible;
  if (!state.inited) return;
  // 首次进入文件页或重新进入时刷新目录(控制程序重启后也能立即恢复视图)
  if (visible && (!state.ftpLoaded || state.online)) ftpList(state.cwd);
}

export function setModelsVisible(visible) {
  state.modelsVisible = !!visible;
  if (!state.inited) return;
  // 首次进入或回到模型页时刷新(激活状态可能已被外部改动)
  if (visible) modelRefresh();
}

// ─────────────────────────────────────────────
//  关于页签: 主控(WS 命令查询) + Web 服务(/api/version) 版本信息
// ─────────────────────────────────────────────
let aboutBusy = false;
let aboutRlBuild = '';   // 最近一次拿到的主控构建号
let aboutWebBuild = '';  // 最近一次拿到的 Web 构建号

function aboutSet(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent = text || '--';
}

function aboutSetPill(id, text, cls) {
  const el = document.getElementById(id);
  if (!el) return;
  el.textContent = text || '--';
  el.classList.toggle('is-on', cls === 'on');
  el.classList.toggle('is-off', cls === 'off');
}

/** 双端构建一致性徽章:同一次编译显示绿色,不一致提示橙色 */
function aboutUpdateMatch() {
  const el = document.getElementById('about-build-match');
  if (!el) return;
  if (!aboutRlBuild || !aboutWebBuild) { el.hidden = true; return; }
  const same = aboutRlBuild === aboutWebBuild;
  el.hidden = false;
  el.textContent = same ? '✓ 同一次编译' : '⚠ 构建不一致';
  el.classList.toggle('is-same', same);
  el.classList.toggle('is-diff', !same);
}

async function aboutRefresh() {
  if (aboutBusy) return;
  aboutBusy = true;
  try {
    // ① Web 服务自身版本(HTTP,与页面同源 → 必为当前提供页面的进程)
    try {
      const resp = await fetch('/api/version', { cache: 'no-store' });
      const j = await resp.json();
      if (j && j.ok) {
        aboutSetPill('about-web-proc', j.proc, 'on');
        aboutSet('about-web-version', 'v' + (j.version || '?'));
        aboutSet('about-web-build', j.build);
        aboutSet('about-web-built', j.built);
        aboutSet('about-web-cwd', j.cwd);
        aboutWebBuild = j.build || '';
        aboutUpdateMatch();
      }
    } catch { /* web 信息失败保持 -- */ }

    // ② 主控程序版本(WS 命令,feedback 一次性返回)
    if (state.online) {
      aboutSetPill('about-rl-status', '查询中…', '');
      publishRaw({ type: 'command', data: 'version' });
    } else {
      aboutSetPill('about-rl-status', '离线', 'off');
      aboutRlBuild = '';
      aboutUpdateMatch();
      ['about-rl-version', 'about-rl-build', 'about-rl-built', 'about-rl-cwd']
        .forEach((id) => aboutSet(id, '--'));
    }
  } finally {
    aboutBusy = false;
  }
}

export function setAboutVisible(visible) {
  if (visible) aboutRefresh();
}

function bindAboutPanel() {
  document.getElementById('about-refresh')?.addEventListener('click', () => aboutRefresh());
  // 主控 version 命令的应答: "version rl_real_JXG v4.0.0 build <n> built <ts> cwd <dir>"
  addRawMessageListener((frame) => {
    if (frame.type !== 'feedback' || typeof frame.data !== 'string') return;
    const s = frame.data;
    if (!s.startsWith('version rl_real_JXG')) return;
    const m = s.match(/^version rl_real_JXG v(\S+) build (\S+) built (.+) cwd (.+)$/);
    aboutSetPill('about-rl-status', '在线', 'on');
    if (m) {
      aboutSet('about-rl-version', 'v' + m[1]);
      aboutSet('about-rl-build', m[2]);
      aboutSet('about-rl-built', m[3]);
      aboutSet('about-rl-cwd', m[4]);
      aboutRlBuild = m[2];
      aboutUpdateMatch();
    }
  });
}

// ─────────────────────────────────────────────
//  遥控学习 / 克隆(CH340 串口,后端 web_server 独占)
// ─────────────────────────────────────────────
function rcResult(ok, msg) {
  const el = els.rcResult;
  if (el) {
    el.hidden = false;
    el.textContent = (ok ? '✔ ' : '✖ ') + (msg || '');
    el.style.color = ok ? '#7ddc8e' : '#ff7a7a';
  }
  if (!ok && els.rcResult) els.rcResult.scrollIntoView?.({ block: 'nearest' });
}

function renderRcInfo(d) {
  state.rcDev = d || null;
  state.rcLearning = !!(d && d.learning);
  if (els.rcDevStatus) {
    els.rcDevStatus.textContent = d && d.available ? '设备可用' : '设备不可用';
    els.rcDevStatus.className = 'ros-status-pill ' + (d && d.available ? 'ros-online' : 'ros-offline');
  }
  if (els.rcDevPath) {
    const p = d ? (d.dev || '') : '';
    let hint;
    if (d && !d.available) hint = d.error || '未检测到串口';
    else if (d) hint = `9600 8N1 · ${d.count || 0} 条已学`;
    else hint = '';
    els.rcDevPath.textContent = p ? `${p} · ${hint}` : hint;
  }
  if (els.rcLearnState) {
    els.rcLearnState.textContent = state.rcLearning ? '学习中…' : '未学习';
    els.rcLearnState.className = 'ros-status-pill ' + (state.rcLearning ? 'ros-online' : '');
  }
  if (els.rcLearnBtn) els.rcLearnBtn.textContent = state.rcLearning ? '停止学习' : '学习';
}

function renderRcList(entries) {
  state.rcEntries = Array.isArray(entries) ? entries : [];
  if (els.rcListCount) els.rcListCount.textContent = `${state.rcEntries.length} 条`;
  const body = els.rcTableBody;
  if (!body) return;
  if (state.rcEntries.length === 0) {
    body.innerHTML = '<tr><td colspan="5" style="color:#8fa3b8">暂无已学码</td></tr>';
    return;
  }
  const esc = (s) => String(s == null ? '' : s).replace(/[&<>"]/g,
    (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
  body.innerHTML = state.rcEntries.map((e) => `
    <tr data-rc-id="${e.id}">
      <td>${e.id}</td>
      <td>${esc(e.name)}</td>
      <td><input type="text" class="sys-ftp-path rc-hex-input" data-id="${e.id}" data-orig="${esc(e.hex)}" value="${esc(e.hex)}" style="font-family:monospace;max-width:120px;padding:1px 4px" spellcheck="false" /><button type="button" class="ros-btn rc-hex-commit" data-id="${e.id}" style="font-size:12px;padding:2px 8px">保存</button></td>
      <td style="font-size:11px">${esc(e.time || '')}</td>
      <td>
        <button type="button" class="ros-btn rc-act-send" data-id="${e.id}" style="font-size:12px;padding:2px 8px">发送</button>
        <button type="button" class="ros-btn rc-act-rename" data-id="${e.id}" style="font-size:12px;padding:2px 8px">重命名</button>
        <button type="button" class="ros-btn rc-act-del" data-id="${e.id}" style="font-size:12px;padding:2px 8px">删除</button>
      </td>
    </tr>`).join('');
}

// 校验并规范化 hex:去分隔符后须恰好 6 个十六进制字符,非法返回 null
function rcNormalizeHex(raw) {
  const s = String(raw == null ? '' : raw).toUpperCase().replace(/[\s,:\-_.]+/g, '');
  if (!/^[0-9A-F]{6}$/.test(s)) return null;
  return `${s.slice(0, 2)} ${s.slice(2, 4)} ${s.slice(4, 6)}`;
}

// 设置「待保存 hex」(同步选中状态与可编辑输入框)
function rcSetPendingHex(hex) {
  state.rcSelectedHex = hex || '';
  if (els.rcPendingHex) els.rcPendingHex.value = state.rcSelectedHex;
  // 名称留空时以 hex 数值作为默认名称(用户可再改)
  if (els.rcNameInput && state.rcSelectedHex && !els.rcNameInput.value.trim()) {
    els.rcNameInput.value = state.rcSelectedHex;
  }
}

// 捕获帧到达:有效 3 字节帧自动填入「待保存 hex」,作为「发送测试 / 保存为」的目标
function addRcCapture(frame) {
  if (frame && !frame.raw && frame.hex) rcSetPendingHex(frame.hex);
}

function rcRefreshAll() {
  publishRaw({ type: 'command', data: 'rc info' });
  publishRaw({ type: 'command', data: 'rc list' });
}

// 电机温度告警阈值面板:应用 / 查询 / 反馈解析(仅提醒,不干预控制)
function bindTempThresholdPanel() {
  const val = document.getElementById('temp-thr-value');
  const iv = document.getElementById('temp-thr-interval');
  const status = document.getElementById('temp-threshold-status');
  if (!val) return;
  document.getElementById('temp-thr-apply')?.addEventListener('click', () => {
    if (!state.online) return;
    const t = parseFloat(val.value);
    const sec = parseInt(iv.value, 10);
    if (!Number.isFinite(t) || t < 30 || t > 120) {
      if (status) status.textContent = '温度阈值需在 30~120°C 之间';
      return;
    }
    if (!Number.isFinite(sec) || sec < 1 || sec > 3600) {
      if (status) status.textContent = '提示周期需在 1~3600 秒之间';
      return;
    }
    publishRaw({ type: 'command', data: `temp_threshold temp ${t}` });
    setTimeout(() => publishRaw({ type: 'command', data: `temp_threshold interval ${sec}` }), 200);
    if (status) status.textContent = `已下发: 滑动中值>${t}°C / 周期${sec}秒`;
  });
  document.getElementById('temp-thr-query')?.addEventListener('click', () => {
    if (state.online) publishRaw({ type: 'command', data: 'temp_threshold' });
  });
  addRawMessageListener((frame) => {
    if (frame.type === 'feedback' && typeof frame.data === 'string') {
      const s = frame.data;
      if (s.startsWith('temp_threshold')) {
        // 格式: "temp_threshold temp=75°C interval=30s"(或单项 temp= / interval=)
        const tm = s.match(/temp=([\d.]+)/);
        const im = s.match(/interval=([\d.]+)/);
        if (tm) val.value = tm[1];
        if (im) iv.value = im[1];
        if (status) status.textContent = s;
      }
    }
  });
}

function bindRcPanel() {
  if (!els.rcRefresh) return;
  els.rcRefresh.addEventListener('click', () => rcRefreshAll());
  els.rcLearnBtn?.addEventListener('click', () => {
    publishRaw({ type: 'command', data: state.rcLearning ? 'rc learn off' : 'rc learn on' });
  });
  els.rcSaveBtn?.addEventListener('click', () => {
    // 「待保存 hex」可手工编辑:非空时校验后随名称发送(留空则由后端落盘最近一条捕获)
    let extra = '';
    const pendRaw = (els.rcPendingHex && els.rcPendingHex.value || '').trim();
    if (pendRaw) {
      const norm = rcNormalizeHex(pendRaw);
      if (!norm) {
        rcSetPendingHex(state.rcSelectedHex);  // 回退为上次有效值
        rcResult(false, `待保存 hex 非法:「${pendRaw}」需恰好 3 字节(如 50 3C 7A)`);
        return;
      }
      rcSetPendingHex(norm);
      extra = ` ${norm}`;
    }
    // 名称留空时默认用 hex 数值(既无待保存 hex 又未填名称才要求输入)
    let name = (els.rcNameInput && els.rcNameInput.value || '').trim();
    if (!name) name = extra.trim();
    if (!name) { rcResult(false, '请先填写名称,或捕获/输入待保存 hex'); return; }
    const safeName = /\s/.test(name) ? `"${name}"` : name;
    publishRaw({ type: 'command', data: `rc save ${safeName}${extra}` });
  });
  // 清除新增框内容:清空名称与待保存 hex,便于重新录制
  els.rcClearBtn?.addEventListener('click', () => {
    if (els.rcNameInput) els.rcNameInput.value = '';
    rcSetPendingHex('');
  });
  // 发送测试:用「待保存 hex」当前值发一帧,验证捕获到的码是否正确
  els.rcTestBtn?.addEventListener('click', () => {
    const pendRaw = (els.rcPendingHex && els.rcPendingHex.value || '').trim();
    if (!pendRaw) { rcResult(false, '没有可发送的数据,请先开启学习捕获或手动填写 hex'); return; }
    const norm = rcNormalizeHex(pendRaw);
    if (!norm) { rcResult(false, `hex 非法:「${pendRaw}」需恰好 3 字节(如 50 3C 7A)`); return; }
    rcSetPendingHex(norm);
    publishRaw({ type: 'command', data: `rc raw ${norm}` });
  });
  // 已学码列表:事件委托(保存 hex/发送/重命名/删除)
  els.rcTableBody?.addEventListener('click', async (ev) => {
    const btn = ev.target.closest('button');
    if (!btn) return;
    const id = Number(btn.dataset.id);
    if (btn.classList.contains('rc-hex-commit')) {
      const inp = els.rcTableBody.querySelector(`input.rc-hex-input[data-id="${id}"]`);
      if (inp) commitHexEdit(inp);
    } else if (btn.classList.contains('rc-act-send')) {
      publishRaw({ type: 'command', data: `rc send ${id}` });
    } else if (btn.classList.contains('rc-act-rename')) {
      const cur = state.rcEntries.find((e) => e.id === id);
      const name = await window.promptDialog({
        title: '重命名', message: '输入新的名称', defaultValue: (cur && cur.name) || '',
      });
      if (name == null) return;
      const n = String(name).trim();
      if (!n) { rcResult(false, '名称不能为空'); return; }
      publishRaw({ type: 'command', data: `rc rename ${id} ${n}` });
    } else if (btn.classList.contains('rc-act-del')) {
      const cur = state.rcEntries.find((e) => e.id === id);
      const ok = await window.confirmDialog({
        title: '确认删除', message: `删除已学码「${(cur && cur.name) || id}」?`,
        confirmText: '删除', danger: true,
      });
      if (ok) publishRaw({ type: 'command', data: `rc del ${id}` });
    }
  });
  // 已学码 HEX 就地编辑:回车或行内「保存」按钮提交;非法输入回退原值并提示
  const commitHexEdit = (input) => {
    const orig = input.dataset.orig != null ? input.dataset.orig : '';
    const val = input.value.trim();
    if (val === orig) return;
    const id = Number(input.dataset.id);
    const norm = rcNormalizeHex(val);
    if (!norm) {
      input.value = orig;                 // 回退为原值
      rcResult(false, `hex 非法:「${val}」需恰好 3 字节(如 50 3C 7A)`);
      return;
    }
    input.dataset.orig = norm;
    input.value = norm;
    publishRaw({ type: 'command', data: `rc hex ${id} ${norm}` });
  };
  els.rcTableBody?.addEventListener('keydown', (ev) => {
    const inp = ev.target.closest('input.rc-hex-input');
    if (!inp) return;
    if (ev.key === 'Enter') { ev.preventDefault(); commitHexEdit(inp); inp.blur(); }
    else if (ev.key === 'Escape') { inp.value = inp.dataset.orig || ''; inp.blur(); }
  });
}

export function setRcVisible(visible) {
  state.rcVisible = !!visible;
  if (!state.inited) return;
  if (visible) {
    // 进入页签:已连接则刷新设备状态与列表;离线不提示(重连后由状态监听自动刷新)
    if (state.online) rcRefreshAll();
  } else if (state.rcLearning) {
    // 离开页签自动停止学习,避免持续推送
    publishRaw({ type: 'command', data: 'rc learn off' });
  }
}

export function initSysAdmin() {
  if (state.inited) return;
  els.termMount = document.querySelector('#sys-term-mount');
  if (!els.termMount) return;
  state.inited = true;
  els.termBtn = document.querySelector('#sys-term-btn');
  els.termStatus = document.querySelector('#sys-term-status');
  els.ftpBtn = document.querySelector('#sys-ftp-go');
  els.ftpUp = document.querySelector('#sys-ftp-up');
  els.ftpBody = document.querySelector('#sys-ftp-body');
  els.ftpStatus = document.querySelector('#sys-ftp-status');
  els.ftpUpInput = document.querySelector('#sys-ftp-upfile');
  els.ftpMkdirBtn = document.querySelector('#sys-ftp-mkdir');
  els.ftpMkfileBtn = document.querySelector('#sys-ftp-mkfile');
  els.ftpRefresh = document.querySelector('#sys-ftp-refresh');
  els.ftpSearch = document.querySelector('#sys-ftp-search');
  els.ftpPaste = document.querySelector('#sys-ftp-paste');
  // els.ftpClip 元素已移除,ftpRenderClip() 内有 null 保护
  els.ftpCrumb = document.querySelector('#sys-ftp-crumb');
  els.rebootBtn = document.querySelector('#sys-reboot-btn');
  els.poweroffBtn = document.querySelector('#sys-poweroff-btn');
  els.hrBtn = document.querySelector('#sys-hrlog-btn');
  els.hrStatus = document.querySelector('#sys-hrlog-status');
  els.svcStatus = document.querySelector('#sys-svc-status');
  els.svcMetrics = document.querySelector('#sys-svc-metrics');
  els.svcMsg = document.querySelector('#sys-svc-msg');
  els.svcStart = document.querySelector('#sys-svc-start');
  els.svcStop = document.querySelector('#sys-svc-stop');
  els.svcRestart = document.querySelector('#sys-svc-restart');
  els.svcEnable = document.querySelector('#sys-svc-enable');
  els.svcDisable = document.querySelector('#sys-svc-disable');
  els.svcRefresh = document.querySelector('#sys-svc-refresh');
  els.tokenInput = document.querySelector('#sys-admin-token');
  els.tokenApply = document.querySelector('#sys-admin-token-apply');
  els.logoutBtn = document.querySelector('#sys-logout-btn');
  // 多选(批量下载/删除按钮内嵌工具条)
  els.ftpSelAll = document.querySelector('#sys-ftp-selall');
  els.ftpSelDl = document.querySelector('#sys-ftp-sel-dl');
  els.ftpSelDel = document.querySelector('#sys-ftp-sel-del');
  // 分页
  els.ftpPager = document.querySelector('#sys-ftp-pager');
  els.ftpPageNums = document.querySelector('#sys-ftp-page-nums');
  els.ftpPagePrev = document.querySelector('#sys-ftp-page-prev');
  els.ftpPageNext = document.querySelector('#sys-ftp-page-next');
  // 模型管理
  els.modelBody = document.querySelector('#sys-model-body');
  els.modelStatus = document.querySelector('#sys-model-status');
  els.modelRefresh = document.querySelector('#sys-model-refresh');
  els.modelNew = document.querySelector('#sys-model-new');
  els.modelSearch = document.querySelector('#sys-model-search');
  els.modelPager = document.querySelector('#sys-model-pager');
  els.modelPageNums = document.querySelector('#sys-model-page-nums');
  els.modelPagePrev = document.querySelector('#sys-model-page-prev');
  els.modelPageNext = document.querySelector('#sys-model-page-next');
  // 遥控学习
  els.rcRefresh = document.querySelector('#rc-refresh');
  els.rcDevStatus = document.querySelector('#rc-dev-status');
  els.rcDevPath = document.querySelector('#rc-dev-path');
  els.rcLearnBtn = document.querySelector('#rc-learn-btn');
  els.rcLearnState = document.querySelector('#rc-learn-state');
  els.rcNameInput = document.querySelector('#rc-name-input');
  els.rcPendingHex = document.querySelector('#rc-pending-hex');
  els.rcTestBtn = document.querySelector('#rc-test-btn');
  els.rcSaveBtn = document.querySelector('#rc-save-btn');
  els.rcClearBtn = document.querySelector('#rc-clear-btn');
  els.rcListCount = document.querySelector('#rc-list-count');
  els.rcTableBody = document.querySelector('#rc-table-body');
  els.rcResult = document.querySelector('#rc-result');
  bindRcPanel();

  // ── 报警阈值(低电量已移至本页;电机温度告警阈值) ──
  bindTempThresholdPanel();

  // ── 关于页签(版本信息) ──
  bindAboutPanel();

  // ── 终端按钮(开/关 切换) ──
  els.termBtn?.addEventListener('click', () => {
    if (!state.termWanted) termConnect();
    else termDisconnect();
  });

  // ── 服务端帧 ──
  addRawMessageListener((frame) => {
    if (frame.type === 'ssh_ready') {
      state.termReady = true;
      if (state.termRetryTimer) { clearTimeout(state.termRetryTimer); state.termRetryTimer = null; }
      termEnsure();
      els.termStatus.textContent = '已连接(root shell)';
      els.termStatus.className = 'sys-term-status is-on';
    } else if (frame.type === 'ssh_data') {
      if (state.term && typeof frame.data === 'string') state.term.write(frame.data);
    } else if (frame.type === 'ssh_exit') {
      const wasReady = state.termReady;
      state.termReady = false;
      if (state.term && (wasReady || state.termWanted)) {
        state.term.write(`\r\n\x1b[33m[终端会话结束] ${frame.data || ''}\x1b[0m\r\n`);
      }
      // 非用户主动停止(stopped by client)且页面仍需要会话 → 1s 后自动重连。
      // 覆盖: 长时间空闲/合盖唤醒后服务端清理 PTY、服务重启等场景。
      const userStopped = typeof frame.data === 'string' && frame.data.startsWith('stopped by client');
      if (state.termWanted && !userStopped && state.termVisible && state.online) {
        els.termStatus.textContent = '会话中断,1s 后自动重连…';
        els.termStatus.className = 'sys-term-status is-busy';
        if (state.term) state.term.write('\r\n\x1b[33m[连接中断,正在自动重连…]\x1b[0m\r\n');
        termScheduleRetry(1000);
      } else if (!state.termWanted) {
        els.termStatus.textContent = '未连接';
        els.termStatus.className = 'sys-term-status';
      } else {
        els.termStatus.textContent = '会话已结束(离线,恢复后自动重连)';
        els.termStatus.className = 'sys-term-status';
      }
    } else if (frame.type === 'feedback' && typeof frame.data === 'string') {
      const t = frame.data;
      if (t.startsWith('hr_logger:')) {
        setHrUi(!t.includes('off') && !t.includes('not compiled'), t);
      }
    } else if (frame.type === 'svc_status' && frame.data) {
      renderSvcStatus(frame.data);
    } else if (frame.type === 'sys_info' && frame.data) {
      renderSysInfo(frame.data);
    } else if (frame.type === 'svc_list' && frame.data) {
      renderSvcList(frame.data.units || []);
    } else if (frame.type === 'svc_file' && frame.data) {
      openSvcEditor(frame.data.unit, frame.data.content || '');
    } else if (frame.type === 'svc_log' && frame.data) {
      showSvcLog(frame.data.unit, frame.data.lines || []);
    } else if (frame.type === 'ps_list' && frame.data) {
      renderPsList(frame.data.procs || []);
    } else if (frame.type === 'syslog' && frame.data) {
      renderSyslog(frame.data.lines || []);
    } else if (frame.type === 'net_list' && frame.data) {
      renderNetList(frame.data.ifaces || []);
    } else if (frame.type === 'net_raw' && frame.data) {
      showNetRawModal(frame.data.lines || []);
    } else if (frame.type === 'net_config' && frame.data) {
      renderNetConfig(frame.data);
    } else if (frame.type === 'net_config_result' && frame.data) {
      renderNetConfigResult(frame.data);
    } else if (frame.type === 'dev_list' && frame.data) {
      renderDevList(frame.data.devs || []);
    } else if (frame.type === 'rc_info' && frame.data) {
      renderRcInfo(frame.data);
    } else if (frame.type === 'rc_list' && frame.data) {
      renderRcList(frame.data.entries || []);
    } else if (frame.type === 'rc_capture') {
      addRcCapture(frame);
    } else if (frame.type === 'rc_result') {
      rcResult(!!frame.ok, frame.msg || '');
    }
  });

  addStatusListener((s) => {
    const was = state.online;
    state.online = s.connectionStatus === 'online';
    if (state.online && !was) {
      // 重连后按需恢复终端会话
      if (state.termWanted && state.termVisible) termSendStart();
      // 系统页可见时恢复 HR 状态与服务轮询
      if (state.sysVisible) {
        publishRaw({ type: 'command', data: 'log status' });
        svcPollStart();
      }
      if (state.filesVisible) ftpList(state.cwd);
      // 遥控学习页可见时刷新设备状态与已学码
      if (state.rcVisible) rcRefreshAll();
    }
    if (!state.online) {
      svcPollStop();
      state.termReady = false;
      if (state.termWanted) {
        els.termStatus.textContent = 'WS 离线,恢复后自动重连';
        els.termStatus.className = 'sys-term-status is-busy';
      }
    }
  });

  // 页面从后台/休眠恢复时: 若仍需要会话且未就绪, 立即补一次 start
  document.addEventListener('visibilitychange', () => {
    if (!document.hidden && state.termVisible && state.termWanted &&
        state.online && !state.termReady) {
      termEnsure();
      termScheduleRetry(200);
    }
  });

  // ── FTP ──
  // 「↩」返回上一级目录
  els.ftpUp?.addEventListener('click', () => { if (state.cwd !== '/') ftpList(dirName(state.cwd)); });
  // 「→」按钮:非编辑态=进入路径编辑;编辑态=提交跳转(mousedown 阻止输入框失焦抢先提交)
  els.ftpBtn?.addEventListener('mousedown', (ev) => ev.preventDefault());
  els.ftpBtn?.addEventListener('click', () => {
    if (crumbCommitFn) crumbCommitFn();
    else crumbEnterPathEdit();
  });
  els.ftpRefresh?.addEventListener('click', () => ftpList(state.cwd));

  // 文件列表模糊搜索:仅过滤当前目录已加载条目,Esc 清空
  els.ftpSearch?.addEventListener('input', () => {
    if (!state.ftpLoaded) return;
    state.ftpQuery = els.ftpSearch.value;
    state.ftpPage = 0;  // 搜索词变化回到第 1 页
    const n = renderFtpRows(state.ftpListPath || state.cwd, state.ftpEntries || []);
    const total = (state.ftpEntries || []).length;
    els.ftpStatus.textContent = state.ftpQuery.trim()
      ? `匹配 ${n}/${total} 项`
      : `${total} 项`;
  });
  els.ftpSearch?.addEventListener('keydown', (ev) => {
    if (ev.key === 'Escape') {
      els.ftpSearch.value = '';
      els.ftpSearch.dispatchEvent(new Event('input'));
    }
  });
  els.ftpPaste?.addEventListener('click', ftpPaste);
  // 文件上传: 仅保留「选择文件上传」一个入口(label 点击触发隐藏 input,选完即传)
  els.ftpUpInput?.addEventListener('change', () => {
    const fs2 = els.ftpUpInput.files;
    if (fs2 && fs2.length) ftpUploadFiles(fs2);
    // 重置 value,允许用户连续选择同一文件
    els.ftpUpInput.value = '';
  });
  // 多选: 表头全选 + 批量操作
  els.ftpSelAll?.addEventListener('change', ftpSelToggleAll);
  els.ftpSelDel?.addEventListener('click', ftpBatchDelete);
  els.ftpSelDl?.addEventListener('click', ftpBatchDownload);
  els.ftpMkdirBtn?.addEventListener('click', ftpMkdir);
  els.ftpMkfileBtn?.addEventListener('click', ftpMkfile);
  // 「更多」菜单 / 空白区右键菜单:点击外部、Esc、滚动时关闭(监听只注册一次)
  if (!window.__ftpMoreDismissBound) {
    window.__ftpMoreDismissBound = true;
    document.addEventListener('click', (ev) => {
      if (ftpOpenMoreMenu && !ftpOpenMoreMenu.contains(ev.target)) ftpCloseMoreMenu();
      if (ftpCtxMenuEl && ftpCtxMenuEl.classList.contains('is-open') && !ftpCtxMenuEl.contains(ev.target)) ftpCloseCtxMenu();
    });
    document.addEventListener('keydown', (ev) => {
      if (ev.key !== 'Escape') return;
      if (ftpCtxMenuEl && ftpCtxMenuEl.classList.contains('is-open')) ftpCloseCtxMenu();
      else if (ftpOpenMoreMenu) ftpCloseMoreMenu();
    });
    // 滚动文件/服务列表时收起,避免菜单悬空错位
    document.addEventListener('scroll', (ev) => {
      if (!ev.target.closest || !ev.target.closest('.sys-ftp-tablewrap, .sys-svclist-wrap')) return;
      if (ftpOpenMoreMenu) ftpCloseMoreMenu();
      if (ftpCtxMenuEl && ftpCtxMenuEl.classList.contains('is-open')) ftpCloseCtxMenu();
    }, true);
  }
  // 文件列表空白区右键菜单:仅在非行/非表头的空白处弹出(行上的右键走浏览器默认)
  const ftpTableWrap = els.ftpBody && els.ftpBody.closest('.sys-ftp-tablewrap');
  if (ftpTableWrap) {
    ftpTableWrap.addEventListener('contextmenu', (ev) => {
      const onRow = ev.target.closest && ev.target.closest('tr');
      if (onRow) return;                 // 点在表头行或数据行上:不拦截
      ev.preventDefault();
      ftpCloseMoreMenu();               // 互斥:收起「更多」下拉
      ftpShowCtxMenu(ev.clientX, ev.clientY);
    });
  }
  // 分页: 上一页/下一页
  els.ftpPagePrev?.addEventListener('click', () => {
    if (state.ftpPage > 0) { state.ftpPage -= 1; renderFtpRows(state.ftpListPath || state.cwd, state.ftpEntries || []); }
  });
  els.ftpPageNext?.addEventListener('click', () => {
    state.ftpPage += 1; renderFtpRows(state.ftpListPath || state.cwd, state.ftpEntries || []);
  });
  // 中间页码: 事件委托(节点每次渲染重建,无需逐个绑定),data-page 为 1 基
  els.ftpPageNums?.addEventListener('click', (ev) => {
    const b = ev.target.closest('.sys-ftp-page-num');
    if (!b) return;
    const p = Number(b.dataset.page) - 1;
    if (p >= 0 && p !== state.ftpPage) {
      state.ftpPage = p;
      renderFtpRows(state.ftpListPath || state.cwd, state.ftpEntries || []);
    }
  });
  // 面包屑:点击某段跳转;点击末尾空白处进入内联路径编辑模式
  els.ftpCrumb?.addEventListener('click', (ev) => {
    if (ev.target === els.ftpCrumb) crumbEnterPathEdit();
  });
  ftpRenderClip();

  // ── 电源(双重确认,危险操作) ──
  els.rebootBtn?.addEventListener('click', async () => {
    const c1 = await window.confirmDialog({ title: '重启系统', message: '⚠ 确认重启机器人整机系统?\n重启期间 Web 连接将断开约 1 分钟。', confirmText: '继续', danger: true });
    if (!c1) return;
    const c2 = await window.confirmDialog({ title: '再次确认', message: '现在立即【重启】?', confirmText: '重启', danger: true });
    if (!c2) return;
    if (await sendCommand('sys_reboot')) {
      await window.alertDialog({ title: '已执行', message: '重启指令已发出,页面将失联,请等待约 1 分钟后刷新。' });
    }
  });
  els.poweroffBtn?.addEventListener('click', async () => {
    const c1 = await window.confirmDialog({ title: '关机', message: '⚠ 确认【关机】机器人整机系统?\n关机后需现场重新上电开机!', confirmText: '继续', danger: true });
    if (!c1) return;
    const c2 = await window.confirmDialog({ title: '再次确认', message: '现在立即【关机】?此操作不可远程恢复!', confirmText: '关机', danger: true });
    if (!c2) return;
    if (await sendCommand('sys_poweroff')) {
      await window.alertDialog({ title: '已执行', message: '关机指令已发出,机器人即将下电。' });
    }
  });

  // ── HR 高频日志开关 ──
  els.hrBtn?.addEventListener('click', async () => {
    const turningOff = els.hrBtn.classList.contains('is-on');
    if (await sendCommand(turningOff ? 'log off' : 'log on')) {
      setHrUi(!turningOff, turningOff ? '正在关闭…' : '正在开启…');
    }
  });

  // ── CAN 初始化(「ROS 指令」页签): 重跑 can_setup.sh 免重启恢复接口 ──
  // 按钮无 data-behavior,不走 behavior-pill 白名单;直接经 sendCommand 发 can_recover。
  // 带 .behavior-pill 类,ROS 离线时由 main.js 统一置灰。PC/移动端共用同一 id。
  document.getElementById('ros-can-init')?.addEventListener('click', async () => {
    const ok = await window.confirmDialog({
      title: 'CAN 初始化',
      message: '重跑 can_setup.sh:按 USB 序列号重命名 can0~4 并重新 UP。\n初始化期间 CAN 通信会瞬断约 1~2 秒,请确认机器人处于安全姿态(趴姿/吊装)。\n\n确认执行?',
      confirmText: '执行', danger: true,
    });
    if (!ok) return;
    await sendCommand('can_recover');
  });

  // ── 机器人服务管理(stop/restart/disable 需确认) ──
  els.svcRefresh?.addEventListener('click', () => { svcSend('status'); });
  els.svcStart?.addEventListener('click', async () => {
    if (await svcSend('start')) els.svcMsg.textContent = '正在启动服务…';
  });
  els.svcStop?.addEventListener('click', async () => {
    const ok = await window.confirmDialog({
      title: '停止服务',
      message: '确认【停止】rl-real-jxg 控制程序?\n停止后机器人不再响应遥控/行为指令(当前趴姿 Passive 不会动电机),Web 页面不受影响,可用「启动」恢复。',
      confirmText: '停止', danger: true,
    });
    if (!ok) return;
    if (await svcSend('stop')) {
      els.svcMsg.textContent = '正在停止服务…';
      els.svcMsg.className = 'sys-svc-msg is-busy';
    }
  });
  els.svcRestart?.addEventListener('click', async () => {
    const ok = await window.confirmDialog({
      title: '重启服务',
      message: '确认【重启】rl-real-jxg 控制程序?\n优雅停机约需 10~20 秒,期间遥测/指令中断,本 Web 页面与终端连接保持不断。',
      confirmText: '重启', danger: true,
    });
    if (!ok) return;
    if (await svcSend('restart')) {
      els.svcMsg.textContent = '正在重启服务(约 10~20 秒)…';
      els.svcMsg.className = 'sys-svc-msg is-busy';
    }
  });
  els.svcEnable?.addEventListener('click', () => { svcSend('enable'); });
  els.svcDisable?.addEventListener('click', async () => {
    const ok = await window.confirmDialog({
      title: '关闭开机自启',
      message: '确认【关闭开机自启】?\n关闭后系统重启将不会自动拉起机器人控制程序(不影响当前运行)。',
      confirmText: '关闭自启',
    });
    if (!ok) return;
    svcSend('disable');
  });

  // ── 服务列表管理 ──
  document.getElementById('sys-svclist-refresh')?.addEventListener('click', () => {
    publishRaw({ type: 'command', data: 'svc list' });
  });
  document.getElementById('sys-svclist-search')?.addEventListener('input', (e) => {
    state.svcSearch = e.target.value;
    state.svcPage = 1;
    renderSvcPage();
  });
  document.getElementById('sys-svclist-new')?.addEventListener('click', () => {
    const name = document.getElementById('sys-svclist-new-name').value.trim();
    if (!name) { window.alertDialog({ title: '提示', message: '请输入服务名' }); return; }
    const unit = name.endsWith('.service') ? name : name + '.service';
    const defaultContent = `[Unit]\nDescription=${unit}\nAfter=network.target\n\n[Service]\nType=simple\nExecStart=/usr/bin/${unit.replace('.service','')}\nRestart=on-failure\n\n[Install]\nWantedBy=multi-user.target\n`;
    openSvcEditor(unit, defaultContent);
  });
  document.getElementById('sys-svclist-page-prev')?.addEventListener('click', () => {
    if (state.svcPage > 1) { state.svcPage--; renderSvcPage(); }
  });
  document.getElementById('sys-svclist-page-next')?.addEventListener('click', () => {
    state.svcPage++; renderSvcPage();
  });
  document.getElementById('sys-svclist-page-size')?.addEventListener('change', (e) => {
    state.svcPageSize = Number(e.target.value) || 10;
    state.svcPage = 1;
    renderSvcPage();
  });

  // ── 进程管理 ──
  document.getElementById('sys-ps-refresh')?.addEventListener('click', () => {
    publishRaw({ type: 'command', data: 'ps list' });
  });
  document.getElementById('sys-ps-search')?.addEventListener('input', (e) => {
    state.psSearch = e.target.value;
    state.psPage = 1;
    renderPsPage();
  });
  document.getElementById('sys-ps-start')?.addEventListener('click', () => {
    const cmd = document.getElementById('sys-ps-start-cmd').value.trim();
    if (!cmd) { window.alertDialog({ title: '提示', message: '请输入启动命令' }); return; }
    publishRaw({ type: 'command', data: `ps start ${cmd}` });
  });
  document.getElementById('sys-ps-page-prev')?.addEventListener('click', () => {
    if (state.psPage > 1) { state.psPage--; renderPsPage(); }
  });
  document.getElementById('sys-ps-page-next')?.addEventListener('click', () => {
    state.psPage++; renderPsPage();
  });
  document.getElementById('sys-ps-page-size')?.addEventListener('change', (e) => {
    state.psPageSize = Number(e.target.value) || 10;
    state.psPage = 1;
    renderPsPage();
  });

  // ── 网络管理 ──
  document.getElementById('sys-net-refresh')?.addEventListener('click', () => {
    const st = document.getElementById('sys-net-status');
    if (st) { st.textContent = '加载中…'; st.className = 'sys-ftp-status'; }
    publishRaw({ type: 'command', data: 'net list' });
  });
  document.getElementById('sys-net-raw')?.addEventListener('click', () => {
    publishRaw({ type: 'command', data: 'net raw' });
  });
  document.getElementById('sys-net-search')?.addEventListener('input', (e) => {
    state.netSearch = e.target.value;
    renderNetRows();
  });

  // ── 网络配置(netcfg) ──
  document.getElementById('netcfg-load')?.addEventListener('click', () => {
    const m = document.getElementById('netcfg-mode');
    if (m) { m.textContent = '读取中…'; m.className = 'sys-ftp-status'; }
    sendCommand('netcfg get');
  });
  document.getElementById('netcfg-add-route')?.addEventListener('click', () => addNetCfgRouteRow());
  document.getElementById('netcfg-add-policy')?.addEventListener('click', () => addNetCfgPolicyRow());


  // ── 设备管理器 ──
  document.getElementById('sys-dev-refresh')?.addEventListener('click', () => {
    publishRaw({ type: 'command', data: 'dev list' });
  });
  document.getElementById('sys-dev-search')?.addEventListener('input', (e) => {
    state.devSearch = e.target.value;
    renderDevRows();
  });

  // ── 系统日志 ──
  document.getElementById('sys-syslog-load')?.addEventListener('click', () => {
    const n = document.getElementById('sys-syslog-lines')?.value || '2000';
    const st = document.getElementById('sys-syslog-status');
    if (st) { st.textContent = '加载中…'; st.className = 'sys-ftp-status'; }
    publishRaw({ type: 'command', data: `syslog ${n}` });
  });
  document.getElementById('sys-syslog-clear')?.addEventListener('click', () => {
    state.syslogLines = [];
    drawSyslog();
  });
  document.getElementById('sys-syslog-filter')?.addEventListener('input', (e) => {
    state.syslogFilter = e.target.value;
    drawSyslog();
  });
  document.getElementById('sys-syslog-autoscroll')?.addEventListener('change', (e) => {
    if (e.target.checked) {
      const screen = document.getElementById('sys-syslog-screen');
      if (screen) screen.scrollTop = screen.scrollHeight;
    }
  });

  // ── 管理令牌 ──
  if (els.tokenInput) els.tokenInput.value = token();
  els.tokenApply?.addEventListener('click', async () => {
    const t = (els.tokenInput.value || '').trim();
    localStorage.setItem(TOKEN_KEY, t);
    // 重写共享 WS 地址的 query token 并重连
    try {
      const u = new URL(getRosUrl());
      if (t) u.searchParams.set('token', t); else u.searchParams.delete('token');
      setRosUrl(u.toString());
      if (state.online) {
        await toggleConnection();       // 断开
        await toggleConnection();       // 用新地址重连
      }
      await window.alertDialog({ title: '令牌已保存', message: '令牌已保存' + (state.online ? ',WS 已重连。' : ',将在下次连接时生效。') });
    } catch (err) {
      await window.alertDialog({ title: '令牌应用失败', message: err.message });
    }
  });

  // ── 退出登录: 清会话 Cookie + 本地令牌,回登录页 ──
  els.logoutBtn?.addEventListener('click', async () => {
    const ok = await window.confirmDialog({
      title: '退出登录',
      message: '确认退出登录并返回登录页?',
      confirmText: '退出',
      danger: true,
    });
    if (!ok) return;
    try { await fetch('./api/logout', { method: 'POST' }); } catch (e) { /* 继续本地清理 */ }
    try { localStorage.removeItem(TOKEN_KEY); } catch (e) {}
    window.location.replace('./login.html');
  });

  // 模型管理: 手动刷新
  els.modelRefresh?.addEventListener('click', () => modelRefresh());
  // 模型搜索:输入即过滤(复用已拉取的列表数据,不重复请求),重置到第一页
  els.modelSearch?.addEventListener('input', () => {
    modelState.search = els.modelSearch.value || '';
    modelState.page = 0;
    if (modelState.data) renderModels(modelState.data);
  });
  els.modelSearch?.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') {
      els.modelSearch.value = '';
      modelState.search = '';
      modelState.page = 0;
      if (modelState.data) renderModels(modelState.data);
    }
  });

  // 模型分页: 直接用已加载数据重渲染,无需重新请求
  els.modelPagePrev?.addEventListener('click', () => {
    if (modelState.page > 0) { modelState.page -= 1; renderModels(modelState.data); }
  });
  els.modelPageNext?.addEventListener('click', () => {
    modelState.page += 1; renderModels(modelState.data);   // 超界时内部会 clamp
  });
  els.modelPageNums?.addEventListener('click', (ev) => {
    const b = ev.target.closest('.sys-ftp-page-num');
    if (!b) return;
    const p = Number(b.dataset.page) - 1;
    if (p >= 0 && p !== modelState.page) {
      modelState.page = p; renderModels(modelState.data);
    }
  });

  // v5: 顶部「新建模型配置」— 机器人固定 JXGw
  els.modelNew?.addEventListener('click', async () => {
    const robot = modelState.data?.robots?.[0]?.name || 'JXGw';
    const name = await window.promptDialog({
      title: '新建模型配置',
      message: '请输入模型配置目录名(字母/数字/._-):',
      defaultValue: 'new_model',
    });
    if (!name) return;
    try {
      const resp = await fetch(modelUrl({ op: 'mkdir', robot, model: name }), { cache: 'no-store' });
      const j = await resp.json();
      if (!j.ok) throw new Error(j.error || `HTTP ${resp.status}`);
      // 跳到最后一页,使新建的卡片可见
      modelState.page = Number.MAX_SAFE_INTEGER;
      modelRefresh();
    } catch (err) {
      await window.alertDialog({ title: '创建失败', message: err.message });
    }
  });

  // 初始加载家目录列表;HR 日志状态与服务轮询仅在系统页激活时启动
  ftpList(state.cwd);
  if (state.sysVisible) {
    publishRaw({ type: 'command', data: 'log status' });
    if (state.online) svcPollStart();
  }
  if (state.modelsVisible) modelRefresh();
}
