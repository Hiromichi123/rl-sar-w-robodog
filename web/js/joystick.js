/**
 * 虚拟摇杆组件。
 *
 * - 圆形底座 + 可拖动旋钮,指针(鼠标/触摸)按下即可把旋钮吸到按点,
 *   拖动时旋钮被限制在底座半径内,松开自动回中。
 * - 回调输出归一化偏移: nx 右为 +1、ny 上为 +1,范围 [-1, 1]。
 * - createJoystick 可重复调用(模型切换会重复初始化):
 *   DOM 与事件只绑定一次,后续调用仅替换回调。
 * - 触控兼容:优先 Pointer Events;无 PointerEvent 的旧内核(部分国产
 *   Android WebView)退回 Touch Events,保证「有些手机摇杆点不动」不再出现。
 */

/** 创建/更新一个摇杆实例
 * @param {HTMLElement} container 摇杆容器(.joystick)
 * @param {{ onMove?: (nx: number, ny: number) => void|false, onEnd?: () => void|boolean }} handlers
 *   onMove 返回 false 表示拒绝本次移动(旋钮不跟随、速度不变);用于速度锁定时冻结位置。
 *   onEnd  返回 true  表示保持旋钮当前位置(不回中);用于速度锁定时松手不复位。
 */
export function createJoystick(container, { onMove, onEnd } = {}) {
  if (!container) return;
  // 旋钮元素:复用已有,避免重复创建
  let knob = container.querySelector('.joystick-knob');
  if (!knob) {
    knob = document.createElement('div');
    knob.className = 'joystick-knob';
    container.appendChild(knob);
  }
  // 回调挂在容器上:事件闭包每次从容器读取,保证重复调用只替换回调
  container._joystickHandlers = { onMove, onEnd };

  if (container.dataset.joyBound) return;
  container.dataset.joyBound = '1';

  /** 旋钮中心最大偏移(px)= 底座半径 - 旋钮半径 */
  const maxOffset = () => Math.max(1, container.clientWidth / 2 - knob.offsetWidth / 2);
  const clamp = (v) => {
    const m = maxOffset();
    return Math.min(m, Math.max(-m, v));
  };
  const setKnob = (dx, dy) => {
    knob.style.transform = `translate(-50%, -50%) translate(${dx}px, ${dy}px)`;
  };
  // 外部复位:解锁速度锁定/复位底盘时把旋钮拉回中心
  container._joystickReset = () => setKnob(0, 0);
  // 外部按归一化值定位旋钮:用于锁定时把旋钮对齐到锁定速度(松手后衰减再锁定的场景)
  container._joystickSetNorm = (nx, ny) => {
    // 限幅 [-1,1]:传入陈旧/超量程值(如速度换算比>1)时旋钮也不得越出底座半径
    const cx = Math.min(1, Math.max(-1, Number(nx) || 0));
    const cy = Math.min(1, Math.max(-1, Number(ny) || 0));
    const m = maxOffset();
    setKnob(cx * m, -cy * m);
  };

  // ── 抖动抑制 ──────────────────────────────────────
  const DEADZONE = 0.06;       // 径向死区:中心附近忽略(消除回中/微颤)
  const SMOOTH_ALPHA = 0.6;    // 低通滤波系数:越小越平滑,越大越跟手(0.6 ≈ 50ms 跟手)
  let smoothNx = 0, smoothNy = 0; // 平滑状态(每次拖拽开始时归零)

  /** 由像素偏移归一化 → 死区 → 平滑 → 截断 → 回调
   *  返回 onMove 的返回值:false 表示调用方拒绝本次移动(旋钮不更新) */
  const emit = (dx, dy) => {
    const m = maxOffset();
    let nx = Math.min(1, Math.max(-1, dx / m));
    let ny = Math.min(1, Math.max(-1, -dy / m));
    // 径向死区:模长 < DEADZONE 时输出 0,否则平滑映射 [DEADZONE,1]→[0,1]
    const r = Math.sqrt(nx * nx + ny * ny);
    if (r < DEADZONE) {
      nx = 0; ny = 0;
    } else {
      const scale = (r - DEADZONE) / (r * (1 - DEADZONE));
      nx *= scale; ny *= scale;
    }
    // 指数平滑(低通滤波)抑制高频抖动
    smoothNx += (nx - smoothNx) * SMOOTH_ALPHA;
    smoothNy += (ny - smoothNy) * SMOOTH_ALPHA;
    // 截断到 2 位小数,消除亚厘米级量化噪声
    return container._joystickHandlers?.onMove?.(
      Math.round(smoothNx * 100) / 100,
      Math.round(smoothNy * 100) / 100,
    );
  };

  // ── 一次拖拽会话:Pointer / Touch 两套事件共用同一套状态 ──────────────
  let dragging = false, dragCx = 0, dragCy = 0;
  const sessionMove = (clientX, clientY) => {
    const dx = clamp(clientX - dragCx);
    const dy = clamp(clientY - dragCy);
    // 若 onMove 拒绝(如速度锁定),则不更新旋钮位置
    if (emit(dx, dy) !== false) setKnob(dx, dy);
  };
  const sessionStart = (clientX, clientY) => {
    container.classList.add('dragging');
    dragging = true;
    // 重置平滑状态:每次按下从零开始,避免上次残留
    smoothNx = 0; smoothNy = 0;
    const rect = container.getBoundingClientRect();
    dragCx = rect.left + rect.width / 2;
    dragCy = rect.top + rect.height / 2;
    sessionMove(clientX, clientY);
  };
  const sessionEnd = () => {
    if (!dragging) return;
    dragging = false;
    container.classList.remove('dragging');
    // onEnd 返回 true 表示保持旋钮位置(速度锁定:松手不回中)
    const keep = container._joystickHandlers?.onEnd?.();
    if (keep !== true) setKnob(0, 0);
  };

  if (window.PointerEvent) {
    container.addEventListener('pointerdown', (event) => {
      // 只对鼠标校验左键:触摸/笔在部分浏览器的 pointerdown 上 button 为 -1,
      // 一并校验会让这些机型的摇杆完全无响应(「有些手机摇杆点不动」)
      if (event.pointerType === 'mouse' && event.button !== 0) return;
      event.preventDefault();
      try { container.setPointerCapture(event.pointerId); } catch { /* 忽略已释放的指针 */ }
      sessionStart(event.clientX, event.clientY);
      const onMoveEv = (e) => sessionMove(e.clientX, e.clientY);
      const onUp = (e) => {
        try { container.releasePointerCapture(e.pointerId); } catch { /* 忽略已释放的指针 */ }
        sessionEnd();
        container.removeEventListener('pointermove', onMoveEv);
        container.removeEventListener('pointerup', onUp);
        container.removeEventListener('pointercancel', onUp);
      };
      container.addEventListener('pointermove', onMoveEv);
      container.addEventListener('pointerup', onUp);
      container.addEventListener('pointercancel', onUp);
    });
  } else {
    // 兜底:无 Pointer Events 的旧内核退回 Touch Events
    container.addEventListener('touchstart', (event) => {
      if (event.touches.length !== 1) return;   // 多指手势交还浏览器
      event.preventDefault();
      const t = event.touches[0];
      sessionStart(t.clientX, t.clientY);
    }, { passive: false });
    container.addEventListener('touchmove', (event) => {
      if (!dragging || event.touches.length !== 1) return;
      event.preventDefault();
      const t = event.touches[0];
      sessionMove(t.clientX, t.clientY);
    }, { passive: false });
    container.addEventListener('touchend', sessionEnd);
    container.addEventListener('touchcancel', sessionEnd);
  }

  // 拦截右键菜单:拖拽中误触右键不会弹出浏览器上下文菜单
  container.addEventListener('contextmenu', (e) => e.preventDefault());
}
