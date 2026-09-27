/**
 * recorder-replay.js
 * 从 IndexedDB 读出 recording 帧,按录制时相对 tsRelMs 节奏播放,
 * 通过 ros-bridge.js 暴露的 injectMotorFrame / injectImuFrame 注入同一管道,
 * 驱动仿真 + 可视化曲线,与真实 ROS 在线效果一致。
 */

import {
  MOTOR_STATE_TOPIC, IMU_STATE_TOPIC,
  injectMotorFrame, injectImuFrame, setReplayMutingRealCallbacks,
  lastMessageAt, lastImuMessageAt, messageCount, imuMessageCount, lastBlockCount,
} from './ros-bridge.js?v=981';
import { loadRecordingFrames, getRecording } from './recorder-storage.js?v=967';

/** 回放追赶阈值(ms),超过此时长说明浏览器卡顿或后台标签被节流 → 直接跳帧前进而不是一帧帧补 */
const CATCH_UP_THRESHOLD_MS = 60;
/** 帧间隔(ms)。我们用 requestAnimationFrame 调度,所以这里只影响最大跳帧粒度,不强制 timer 频率 */
const RAF_HINT_INTERVAL_MS = 10;

let _state = {
  isReplaying: false,
  recordingId: null,
  frames: [],
  cursor: 0,
  wallStartMs: 0,
  totalDurationMs: 0,
  rafId: 0,
  replayMotorSeq: 0,
  replayImuSeq: 0,
  meta: null,
};

/** 视频监控模式:拖动条 seek-and-inject 用,与 replay 状态解耦 */
let _currentSeekMs = 0;

const _listeners = new Set();
function emit(evt) { _listeners.forEach((fn) => fn(evt)); }

export function addReplayListener(fn) { _listeners.add(fn); return () => _listeners.delete(fn); }
export function isReplaying() { return !!_state.isReplaying; }
export function getReplayState() {
  const hasFrames = _state.frames.length > 0;
  const elapsedMs = _state.isReplaying
    ? Math.min(_state.totalDurationMs, performance.now() - _state.wallStartMs)
    : (hasFrames ? Math.min(_state.totalDurationMs, _currentSeekMs) : 0);
  const reachedEnd = hasFrames && _state.cursor >= Math.max(0, _state.frames.length);
  return {
    isReplaying: _state.isReplaying,
    recordingId: _state.recordingId,
    cursor: _state.cursor,
    totalFrames: _state.frames.length,
    elapsedMs,
    totalDurationMs: _state.totalDurationMs,
    progress: _state.totalDurationMs > 0 ? (elapsedMs / _state.totalDurationMs) : 0,
    reachedEnd,
    meta: _state.meta,
  };
}

/**
 * 从 recording 开始复现。
 * @param {number} recordingId
 * @param {{startMs?:number}} [opts]  startMs:从该毫秒位置开始播放(视频监控模式)
 * @returns {Promise<void>}  当播放完成(播放到最后一帧)时 resolve;中途 stopReplay 会 reject。
 */
export async function startReplay(recordingId, opts = {}) {
  stopReplay(true, true /* silent, keepFrames 以便 startReplay 内部能复用 frames(但此处会重新加载覆盖) */);
  const rid = Number(recordingId);
  const startMs = Math.max(0, Number(opts.startMs) || 0);
  const [meta, frames] = await Promise.all([getRecording(rid), loadRecordingFrames(rid)]);
  if (!meta) throw new Error(`录制不存在:${rid}`);
  if (!frames.length) throw new Error(`录制没有可播放的帧:${rid}`);

  try { setReplayMutingRealCallbacks(true); } catch (_) {}

  const totalDurationMs = frames[frames.length - 1].tsRelMs || 0;
  let cursor = 0;
  if (startMs > 0) {
    let lo = 0, hi = frames.length;
    while (lo < hi) {
      const mid = (lo + hi) >>> 1;
      if (frames[mid].tsRelMs < startMs) lo = mid + 1;
      else hi = mid;
    }
    cursor = lo;
  }

  _state = {
    isReplaying: true,
    recordingId: rid,
    frames,
    cursor,
    wallStartMs: performance.now() - startMs,
    totalDurationMs,
    rafId: 0,
    replayMotorSeq: 0,
    replayImuSeq: 0,
    meta,
  };
  _currentSeekMs = startMs;

  emit({ type: 'start', recordingId: rid, meta });
  // 让 messageCount/imuMessageCount 回调 UI 的人能看到计数。我们覆写的是全局 get 导出,这些是 let 导出,main.js 用的是同名的快照绑定到模块变量,所以直接修改导入值不行 → 改为在 UI 状态栏直接读 getReplayState()。
  // 启动 RAF 驱动
  let nextTickAt = performance.now() + RAF_HINT_INTERVAL_MS;
  const loop = (now) => {
    if (!_state.isReplaying) return;
    try {
      // 每一帧(最多 N 帧追赶)处理:播放到 wallElapsed >= frames[cursor].tsRelMs
      const wallElapsed = now - _state.wallStartMs;
      let processed = 0;
      while (_state.cursor < _state.frames.length) {
        const f = _state.frames[_state.cursor];
        if (f.tsRelMs > wallElapsed + CATCH_UP_THRESHOLD_MS) break; // 还没到这帧
        // 若累计偏差过大(例如用户切后台回来),直接把 wallStartMs 向前推进,让当前帧「正应该现在播放」,避免雪崩式跳帧
        const drift = wallElapsed - f.tsRelMs;
        if (drift > CATCH_UP_THRESHOLD_MS) {
          _state.wallStartMs += (drift - CATCH_UP_THRESHOLD_MS / 2);
        }
        try {
          _dispatchFrame(f);
        } catch (frameErr) {
          // ⚠️ 单帧错误不影响整个 RAF 循环,否则一帧抛错整个复现卡死,
          //   用户会看到「复现中一直到不了 100%」,这里只 console 不中断。
          try { console.warn('[replay] dispatch frame 错误(已跳过):', frameErr, 'seq=', _state.cursor, '/', _state.frames.length); } catch (_) {}
        }
        _state.cursor += 1;
        processed += 1;
        if (processed > 200) break; // 单次 RAF 避免处理过多导致阻塞
      }
      // 进度通知(节流到每 ~100ms 级别,这里每帧都发也可以,外面自己节流)
      const elapsed = Math.min(_state.totalDurationMs, performance.now() - _state.wallStartMs);
      _currentSeekMs = elapsed;
      emit({
        type: 'progress',
        recordingId: rid,
        elapsedMs: elapsed,
        totalMs: _state.totalDurationMs,
        progress: _state.totalDurationMs > 0 ? elapsed / _state.totalDurationMs : 0,
        cursor: _state.cursor,
        totalFrames: _state.frames.length,
      });

      if (_state.cursor >= _state.frames.length) {
        // ══════════════════════════════════════════════════════════
        // 完成前先强制推送一次 100% progress(让顶部状态栏 / 拖动条立刻到位)
        // ⚠️ 注意:此处 NOT 立即设置 _state.isReplaying = false
        //   目的:让 complete 事件触发后,main.js 的 applyRecorderUiState 仍然读到 replaying=true,
        //        从而在 200~250ms 内稳定显示「▶ 复现中 100%」;main.js 监听 complete 的 setTimeout
        //        250ms 后调用 stopReplay(silent=true) 统一把 isReplaying 清回 false 并取消 muting。
        // ══════════════════════════════════════════════════════════
        emit({
          type: 'progress',
          recordingId: rid,
          elapsedMs: _state.totalDurationMs,
          totalMs:   _state.totalDurationMs,
          progress: 1,
          cursor:    _state.frames.length,
          totalFrames: _state.frames.length,
        });
        emit({
          type: 'complete',
          recordingId: rid,
          elapsedMs: _state.totalDurationMs,
          totalMs: _state.totalDurationMs,
        });
        return;
      }
    } catch (loopErr) {
      // RAF 级别兜底:避免整个复现卡死
      try { console.warn('[replay] RAF loop 异常(将停止复现):', loopErr); } catch (_) {}
      _state.isReplaying = false;
      try { setReplayMutingRealCallbacks(false); } catch (_) {}
      emit({ type: 'error', recordingId: rid, error: String(loopErr && loopErr.message || loopErr) });
      return;
    }
    const nxt = performance.now();
    if (nxt >= nextTickAt) { nextTickAt = nxt + RAF_HINT_INTERVAL_MS; _state.rafId = requestAnimationFrame(loop); }
    else _state.rafId = requestAnimationFrame(loop);
  };
  _state.rafId = requestAnimationFrame(loop);
}

/** 停止复现(silent=true 时仅内部复位,不对外发 stop 事件;keepFrames=true 时保留帧供后续 seek-and-inject 使用) */
export function stopReplay(silent = false, keepFrames = false) {
  if (!_state.isReplaying && !silent && _state.frames.length === 0 && !_state.rafId) {
    try { setReplayMutingRealCallbacks(false); } catch (_) {}
    return;
  }
  const rid = _state.recordingId;
  _state.isReplaying = false;
  if (_state.rafId) { cancelAnimationFrame(_state.rafId); _state.rafId = 0; }
  try { setReplayMutingRealCallbacks(false); } catch (_) {}
  if (!silent) emit({ type: 'stop', recordingId: rid });
  if (!keepFrames) {
    _state.frames = [];
    _state.cursor = 0;
    _state.recordingId = null;
    _state.meta = null;
    _state.totalDurationMs = 0;
    _state.wallStartMs = 0;
    _state.replayMotorSeq = 0;
    _state.replayImuSeq = 0;
    _state.rafId = 0;
  }
}

/**
 * 跳到指定毫秒位置(供拖动条使用)。
 * - 若不在复现中,对 startReplay 后或 playing 状态下有效;
 * - 用二分查找找到第一个 frames[i].tsRelMs >= tsMs 的 index,然后 cursor = index,wallStartMs = now - targetTs。
 * - seek 操作会 emit progress 事件一次,便于 UI 同步。
 * @param {number} tsMs  目标相对毫秒位置(0..totalDurationMs)
 * @returns {boolean} 是否成功执行
 */
export function seekTo(tsMs) {
  if (!_state.frames || _state.frames.length === 0) return false;
  const total = _state.totalDurationMs;
  const target = Math.max(0, Math.min(total, Number(tsMs) || 0));
  const frames = _state.frames;

  let lo = 0, hi = frames.length;
  while (lo < hi) {
    const mid = (lo + hi) >>> 1;
    if (frames[mid].tsRelMs < target) lo = mid + 1;
    else hi = mid;
  }
  _state.cursor = lo;
  _state.wallStartMs = performance.now() - target;
  _currentSeekMs = target;

  emit({
    type: 'progress',
    recordingId: _state.recordingId,
    elapsedMs: target,
    totalMs:   total,
    progress:  total > 0 ? target / total : 0,
    cursor:    _state.cursor,
    totalFrames: frames.length,
    seeked:    true,
  });
  return true;
}

/**
 * 视频监控模式:seek-and-inject,在不自动播放的情况下把帧注入到仿真。
 * 即使 isReplaying=false 也能工作,只要 frames 已经加载。
 * @param {number} tsMs 目标相对毫秒位置
 * @returns {boolean} 是否成功执行
 */
export function seekAndInject(tsMs) {
  if (!_state.frames || _state.frames.length === 0) return false;
  const total = _state.totalDurationMs;
  const target = Math.max(0, Math.min(total, Number(tsMs) || 0));
  const frames = _state.frames;

  // 找最后一个 <= target 的 motor 帧 和 IMU 帧
  let lastMotor = null;
  let lastImu = null;
  for (let i = 0; i < frames.length; i++) {
    const f = frames[i];
    if (f.tsRelMs > target) break;
    if (f.topic === MOTOR_STATE_TOPIC) lastMotor = f;
    else if (f.topic === IMU_STATE_TOPIC) lastImu = f;
  }

  try { setReplayMutingRealCallbacks(true); } catch (_) {}
  if (lastMotor) injectMotorFrame(lastMotor.data || []);
  if (lastImu) injectImuFrame(lastImu.data || []);

  // 更新 cursor(供后续 startReplay 从该位置继续)
  let lo = 0, hi = frames.length;
  while (lo < hi) {
    const mid = (lo + hi) >>> 1;
    if (frames[mid].tsRelMs < target) lo = mid + 1;
    else hi = mid;
  }
  _state.cursor = lo;
  _currentSeekMs = target;

  emit({
    type: 'progress',
    recordingId: _state.recordingId,
    elapsedMs: target,
    totalMs:   total,
    progress:  total > 0 ? target / total : 0,
    cursor:    _state.cursor,
    totalFrames: frames.length,
    seeked:    true,
  });
  return true;
}

/** 当前是否有已加载的帧(用于视频监控模式判断是否可 seek) */
export function hasFrames() {
  return _state.frames.length > 0;
}

/** 获取当前 seek 位置(视频监控模式下用) */
export function getCurrentSeekMs() {
  return _currentSeekMs;
}

/** 视频监控模式:清理帧(切换录制记录时使用) */
export function clearFrames() {
  // 切换/清空记录时恢复真实回调(muting 只应在复现或主动 seek 期间生效)
  try { setReplayMutingRealCallbacks(false); } catch (_) {}
  _state.frames = [];
  _state.cursor = 0;
  _state.recordingId = null;
  _state.meta = null;
  _state.totalDurationMs = 0;
  _state.wallStartMs = 0;
  _state.replayMotorSeq = 0;
  _state.replayImuSeq = 0;
  _currentSeekMs = 0;
}

/**
 * 视频监控模式:仅加载帧,不启动 RAF 循环。
 * 用于选中记录时预加载帧,使拖动条可用。
 */
export async function loadFramesForPreview(recordingId) {
  if (_state.isReplaying) stopReplay(true);
  const rid = Number(recordingId);
  const [meta, frames] = await Promise.all([getRecording(rid), loadRecordingFrames(rid)]);
  if (!meta) throw new Error(`录制不存在:${rid}`);
  if (!frames.length) throw new Error(`录制没有可播放的帧:${rid}`);

  _state.frames = frames;
  _state.cursor = 0;
  _state.recordingId = rid;
  _state.totalDurationMs = frames[frames.length - 1].tsRelMs || 0;
  _state.meta = meta;
  _state.wallStartMs = 0;
  _state.replayMotorSeq = 0;
  _state.replayImuSeq = 0;
  _currentSeekMs = 0;
  return { recordingId: rid, totalDurationMs: _state.totalDurationMs, meta };
}

/** 根据帧 topic 分发到 injectMotor/injectImu,并重置相关「全局计数」(可选) */
function _dispatchFrame(f) {
  if (f.topic === MOTOR_STATE_TOPIC) {
    _state.replayMotorSeq += 1;
    injectMotorFrame(f.data || []);
  } else if (f.topic === IMU_STATE_TOPIC) {
    _state.replayImuSeq += 1;
    injectImuFrame(f.data || []);
  }
}

// 导出「播放到第几帧」的计数器,便于列表行高亮当前播放进度
export function getReplayCounters() {
  return { motorSeq: _state.replayMotorSeq, imuSeq: _state.replayImuSeq };
}
