/**
 * 虚拟摇杆组件。
 *
 * - 圆形底座 + 可拖动旋钮,指针(鼠标/触摸)按下即可把旋钮吸到按点,
 *   拖动时旋钮被限制在底座半径内,松开自动回中。
 * - 回调输出归一化偏移: nx 右为 +1、ny 上为 +1,范围 [-1, 1]。
 * - createJoystick 可重复调用(模型切换会重复初始化):
 *   DOM 与事件只绑定一次,后续调用仅替换回调。
 */

/** 创建/更新一个摇杆实例
 * @param {HTMLElement} container 摇杆容器(.joystick)
 * @param {{ onMove?: (nx: number, ny: number) => void, onEnd?: () => void }} handlers
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

  // ── 抖动抑制 ──────────────────────────────────────
  const DEADZONE = 0.06;       // 径向死区:中心附近忽略(消除回中/微颤)
  const SMOOTH_ALPHA = 0.6;    // 低通滤波系数:越小越平滑,越大越跟手(0.6 ≈ 50ms 跟手)
  let smoothNx = 0, smoothNy = 0; // 平滑状态(每次拖拽开始时归零)

  /** 由像素偏移归一化 → 死区 → 平滑 → 截断 → 回调 */
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
    container._joystickHandlers?.onMove?.(
      Math.round(smoothNx * 100) / 100,
      Math.round(smoothNy * 100) / 100,
    );
  };

  container.addEventListener('pointerdown', (event) => {
    if (event.button !== 0) return;
    event.preventDefault();
    container.classList.add('dragging');
    try { container.setPointerCapture(event.pointerId); } catch { /* 忽略已释放的指针 */ }
    // 重置平滑状态:每次按下从零开始,避免上次残留
    smoothNx = 0; smoothNy = 0;
    const rect = container.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    let dx = clamp(event.clientX - cx);
    let dy = clamp(event.clientY - cy);
    setKnob(dx, dy);
    emit(dx, dy);
    const onMoveEv = (e) => {
      dx = clamp(e.clientX - cx);
      dy = clamp(e.clientY - cy);
      setKnob(dx, dy);
      emit(dx, dy);
    };
    const onUp = (e) => {
      container.classList.remove('dragging');
      try { container.releasePointerCapture(e.pointerId); } catch { /* 忽略已释放的指针 */ }
      container.removeEventListener('pointermove', onMoveEv);
      container.removeEventListener('pointerup', onUp);
      container.removeEventListener('pointercancel', onUp);
      setKnob(0, 0);
      container._joystickHandlers?.onEnd?.();
    };
    container.addEventListener('pointermove', onMoveEv);
    container.addEventListener('pointerup', onUp);
    container.addEventListener('pointercancel', onUp);
  });

  // 拦截右键菜单:拖拽中误触右键不会弹出浏览器上下文菜单
  container.addEventListener('contextmenu', (e) => e.preventDefault());
}
