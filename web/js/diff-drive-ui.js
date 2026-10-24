// The controller, never browser storage, owns permission and actual mode.
export function installDiffDriveUi({ addPosture, addStatus, send, stop }) {
  let posture = null;
  let received = 0;
  let online = false;
  let pending = null;
  const controls = document.createElement('div');
  controls.className = 'behavior-pills';
  controls.innerHTML = `<button type="button" data-diff="enable">启用传统差速</button>
    <button type="button" data-diff="enter">进入四轮差速</button>
    <button type="button" data-diff="exit">退出差速 / 站立</button>
    <button type="button" data-diff="disable">关闭传统差速</button>
    <span role="status" data-diff-status></span>
    <small>仅平整地面固定站姿；无横移、主动腿部适应或越障。任务导航仍使用 RL。</small>`;
  const anchor = document.querySelector('.behavior-pills') || document.querySelector('[data-behavior]')?.parentElement;
  if (!anchor) return;
  anchor.after(controls);
  const buttons = Object.fromEntries([...controls.querySelectorAll('[data-diff]')].map(b => [b.dataset.diff, b]));
  const label = controls.querySelector('[data-diff-status]');
  function render() {
    const fresh = online && performance.now() - received < 1500;
    const capable = fresh && posture?.diff_capable === true;
    const active = posture?.fsm === 'RLFSMStateDiffDrive';
    const enabled = posture?.diff_enabled === true;
    buttons.enable.disabled = !capable || enabled || !!pending || !posture?.diff_mission_idle;
    buttons.disable.disabled = !capable || !enabled || !!pending;
    buttons.enter.disabled = !capable || !enabled || !!pending || !posture?.diff_mission_idle || !posture?.diff_nav ||
      posture?.fsm !== 'RLFSMStateGetUp' || !posture.settled ||
      posture.can_ready !== true || posture.stand_ready !== true;
    buttons.exit.disabled = !fresh || !active || !!pending;
    label.textContent = !fresh ? '状态过期 / 未连接' : !capable ? '小脑未提供差速能力' :
      pending ? '等待小脑确认…' : active ? '当前：四轮差速' : enabled ? '差速已启用，起立完成后可进入' : '差速未启用';
    // The normal RL button must not silently switch policies.
    document.querySelectorAll('[data-behavior="locomotion"]').forEach(b => {
      if (active) { b.disabled = true; b.dataset.diffLocked = '1'; }
      else if (b.dataset.diffLocked) { delete b.dataset.diffLocked; b.disabled = !fresh; }
    });
    if (pending && performance.now() - pending.at > 6000) {
      pending = null;
      label.textContent = '切换未确认；请检查反馈，勿重复操作';
    }
  }
  addPosture(p => {
    posture = p; received = performance.now();
    if (pending && ((pending.cmd === 'diff_enable' && p.diff_enabled) ||
        (pending.cmd === 'diff_disable' && !p.diff_enabled && p.fsm !== 'RLFSMStateDiffDrive') ||
        (pending.cmd === 'diff_drive' && p.fsm === 'RLFSMStateDiffDrive' && p.settled) ||
        (pending.cmd === 'diff_exit' && p.fsm === 'RLFSMStateGetUp' && p.settled))) pending = null;
    render();
  });
  addStatus(s => { online = s.connectionStatus === 'online'; if (!online) { posture = null; received = 0; pending = null; } render(); });
  controls.addEventListener('click', e => {
    const b = e.target.closest('[data-diff]');
    if (!b || b.disabled) return;
    stop();
    const cmd = { enable: 'diff_enable', disable: 'diff_disable', enter: 'diff_drive', exit: 'diff_exit' }[b.dataset.diff];
    if (send(cmd)) pending = { cmd, at: performance.now() };
    render();
  });
  window.addEventListener('blur', stop);
  document.addEventListener('visibilitychange', () => { if (document.hidden) stop(); });
  setInterval(render, 250);
  render();
}
