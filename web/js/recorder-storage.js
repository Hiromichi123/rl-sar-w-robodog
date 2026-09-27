/**
 * recorder-storage.js
 * 把在线接收到的 ROS 数据帧持久化到浏览器 IndexedDB,供后续复现。
 *
 * DB: ros_recorder_db  v1
 *   objectStore  recordings   每次录制(会话)一条,包含元信息
 *       keyPath: id(autoIncrement: true)
 *       index:   by_started_at  startedAt desc
 *   objectStore  frames       每一帧一条(motor / imu 混存,用 topic 区分)
 *       keyPath: [recordingId, seq]  (复合主键,同一次 recording 内 seq 单调递增)
 *       index:   by_recordingId   recordingId
 *
 * 所有 API 均返回 Promise。
 */

const DB_NAME = 'ros_recorder_db';
const DB_VERSION = 1;

const STORE_RECORDINGS = 'recordings';
const STORE_FRAMES = 'frames';
const INDEX_REC_BY_START = 'by_started_at';
const INDEX_FRAME_BY_RID = 'by_recordingId';

/** 电机/IMU 帧批量写入阈值(每累计 N 条 flush 一次事务),兼顾吞吐与崩溃损失 */
const BATCH_WRITE_SIZE = 60;
/** flush 最大时间间隔(ms),保证即使帧稀疏也会定期落盘 */
const FLUSH_INTERVAL_MS = 500;

/** @type {IDBDatabase|null} */
let _db = null;

export function isSupported() {
  return typeof indexedDB !== 'undefined';
}

/** 打开(或初始化)数据库。已打开时直接复用同一个 IDBDatabase 实例。 */
export function openRecorderDB() {
  return new Promise((resolve, reject) => {
    if (_db) { resolve(_db); return; }
    if (!isSupported()) { reject(new Error('当前浏览器不支持 IndexedDB')); return; }
    const req = indexedDB.open(DB_NAME, DB_VERSION);
    req.onupgradeneeded = (evt) => {
      const db = /** @type {IDBDatabase} */ (evt.target.result);
      if (!db.objectStoreNames.contains(STORE_RECORDINGS)) {
        const rec = db.createObjectStore(STORE_RECORDINGS, { keyPath: 'id', autoIncrement: true });
        rec.createIndex(INDEX_REC_BY_START, 'startedAt', { unique: false });
      }
      if (!db.objectStoreNames.contains(STORE_FRAMES)) {
        const frames = db.createObjectStore(STORE_FRAMES, { keyPath: ['recordingId', 'seq'] });
        frames.createIndex(INDEX_FRAME_BY_RID, 'recordingId', { unique: false });
      }
    };
    req.onsuccess = (evt) => {
      _db = /** @type {IDBDatabase} */ (evt.target.result);
      _db.onclose = () => { _db = null; };
      resolve(_db);
    };
    req.onerror = (evt) => {
      const err = evt.target.error || req.error || new Error('IndexedDB 打开失败');
      _db = null; reject(err);
    };
    req.onblocked = () => reject(new Error('IndexedDB 被旧版本标签页占用,请关闭其它页面后重试'));
  });
}

/** 关闭 DB (主要供测试/切换版本时用) */
export function closeRecorderDB() {
  if (_db) { try { _db.close(); } catch (_) { /* ignore */ } _db = null; }
}

// ─────────────────────── 运行时录制状态(内存) ───────────────────────
/**
 * 当前是否正在录制。每次 startRecording 产生一个新 entry。
 * 允许多次 startRecording(不同 id 独立),但 UI 层会使用「最新一个」。
 */
const _activeRecorders = new Map();

/**
 * 开始一段新录制
 * @param {{rosUrl:string, modelId:(string|number), name?:string}} meta
 * @returns {Promise<{recordingId:number}>}
 */
export async function startRecording(meta = {}) {
  const db = await openRecorderDB();
  const startedAt = Date.now();
  const name = meta.name || `录制 ${new Date(startedAt).toLocaleString('zh-CN', { hour12: false })}`;
  const rec = {
    name,
    rosUrl: (meta.rosUrl && String(meta.rosUrl)) || '',
    modelId: meta.modelId == null ? '' : String(meta.modelId),
    startedAt,
    endedAt: null,
    motorFrameCount: 0,
    imuFrameCount: 0,
    totalFrames: 0,
    firstTimestampMs: null,     // 录制内第一帧的相对起始锚点(ms performance.now 基)
    lastTimestampMs: null,      // 录制内最后一帧锚点
    durationMs: 0,
  };

  const recordingId = await new Promise((resolve, reject) => {
    const tx = db.transaction(STORE_RECORDINGS, 'readwrite');
    const store = tx.objectStore(STORE_RECORDINGS);
    const putReq = store.add(rec);
    putReq.onsuccess = () => resolve(putReq.result);
    putReq.onerror = () => reject(putReq.error || new Error('recordings add 失败'));
    tx.onerror = () => reject(tx.error || new Error('写入 recordings 失败'));
  });

  _activeRecorders.set(recordingId, {
    rec,
    /** 内存缓冲,每 BATCH_WRITE_SIZE 条批量写入 IndexedDB,减少事务数 */
    buffer: [],
    seq: 0,
    flushTimer: setTimeout(() => _flushBufferFor(recordingId), FLUSH_INTERVAL_MS),
  });
  return { recordingId };
}

/**
 * 录制一帧(可同时用于 motor 与 imu 两种 topic,靠 topic 字段区分)
 * @param {number} recordingId
 * @param {{topic:string, timestampMs:number, data:number[]}} frame  timestampMs 用 performance.now() 作为相对时间基
 */
export function recordFrame(recordingId, frame) {
  if (!_activeRecorders.has(recordingId)) return false;
  const ar = _activeRecorders.get(recordingId);
  if (!frame || typeof frame !== 'object') return false;
  const topic = (frame && frame.topic) || '';
  if (ar.rec.firstTimestampMs == null) ar.rec.firstTimestampMs = frame.timestampMs;
  ar.rec.lastTimestampMs = frame.timestampMs;
  // 累计时长 = 新段相对时长 + 续录前已有时长(跨会话合并时时间基已重置)
  ar.rec.durationMs = Math.max(0, ar.rec.lastTimestampMs - ar.rec.firstTimestampMs) + (ar.rec.tsOffsetMs || 0);
  if (topic === '/rl_real/motor_state') ar.rec.motorFrameCount += 1;
  else if (topic === '/rl_real/imu_state') ar.rec.imuFrameCount += 1;
  ar.rec.totalFrames += 1;
  ar.buffer.push({
    recordingId,
    seq: ar.seq++,
    topic,
    // tsRelMs 相对首帧;续录(resume)后叠加已有时长偏移,保证合并记录时间轴单调连续
    // (performance.now() 每次刷新页面会归零,跨会话续录必须重置时间基)
    tsRelMs: Math.max(0, (frame.timestampMs - (ar.rec.firstTimestampMs || frame.timestampMs))) + (ar.rec.tsOffsetMs || 0),
    tsAbsMs: frame.timestampMs,
    data: Array.isArray(frame.data) ? frame.data : [],
  });
  if (ar.buffer.length >= BATCH_WRITE_SIZE) _flushBufferFor(recordingId);
  return true;
}

/** 判断 recordingId 是否正在录制中(UI 判断用) */
export function isRecording(recordingId) {
  return _activeRecorders.has(Number(recordingId));
}

/**
 * 结束录制:flush 缓冲 + 补 recordings endedAt/计数/durationMs
 * @param {number} recordingId
 */
export async function stopRecording(recordingId) {
  const rid = Number(recordingId);
  const ar = _activeRecorders.get(rid);
  if (!ar) return null;
  if (ar.flushTimer) { clearTimeout(ar.flushTimer); ar.flushTimer = null; }
  await _flushBufferFor(rid, true);
  const db = await openRecorderDB();
  ar.rec.endedAt = Date.now();
  if (ar.rec.firstTimestampMs != null && ar.rec.lastTimestampMs != null) {
    ar.rec.durationMs = Math.max(0, ar.rec.lastTimestampMs - ar.rec.firstTimestampMs);
  }
  return await new Promise((resolve, reject) => {
    const tx = db.transaction(STORE_RECORDINGS, 'readwrite');
    const store = tx.objectStore(STORE_RECORDINGS);
    const req = store.put({ ...ar.rec, id: rid });
    req.onsuccess = () => {
      _activeRecorders.delete(rid);
      resolve(ar.rec);
    };
    req.onerror = () => reject(req.error || new Error('stopRecording put 失败'));
    tx.onerror = () => reject(tx.error || new Error('stopRecording 事务失败'));
  });
}

/** 把内存 buffer 中的帧批量写入 IndexedDB。isLast=true 时清理 flushTimer 引用 */
async function _flushBufferFor(recordingId, isLast = false) {
  const rid = Number(recordingId);
  const ar = _activeRecorders.get(rid);
  if (!ar) return;
  if (ar.flushTimer) { clearTimeout(ar.flushTimer); ar.flushTimer = null; }
  if (ar.buffer.length === 0) {
    if (!isLast) ar.flushTimer = setTimeout(() => _flushBufferFor(rid), FLUSH_INTERVAL_MS);
    return;
  }
  const batch = ar.buffer;
  ar.buffer = [];
  try {
    const db = await openRecorderDB();
    await new Promise((resolve, reject) => {
      const tx = db.transaction(STORE_FRAMES, 'readwrite');
      const store = tx.objectStore(STORE_FRAMES);
      // 使用 put 循环写入(复合主键自动避免冲突)
      let done = 0; const total = batch.length;
      const onNext = () => {
        while (done < total) {
          const r = store.put(batch[done++]);
          // 大多数浏览器允许不绑定每次 onsuccess,直接用 tx.oncomplete 也可,这里保险
          r.onerror = () => reject(r.error || new Error('frames put 失败'));
        }
      };
      tx.oncomplete = () => resolve();
      tx.onerror = () => reject(tx.error || new Error('frames flush 事务失败'));
      tx.onabort = () => reject(tx.error || new Error('frames flush 事务中止'));
      onNext();
    });
  } finally {
    if (!isLast && !ar.flushTimer) {
      ar.flushTimer = setTimeout(() => _flushBufferFor(rid), FLUSH_INTERVAL_MS);
    }
  }
}

/**
 * 把内存中的录制元信息(帧计数/时长)写回数据库,但不结束录制会话。
 * 用于报警记录:每个报警周期结束时刷新总帧数,会话保持打开以便下次合并续录。
 */
export async function syncRecordingMeta(recordingId) {
  const rid = Number(recordingId);
  const ar = _activeRecorders.get(rid);
  if (!ar) return null;
  const db = await openRecorderDB();
  await new Promise((resolve, reject) => {
    const tx = db.transaction(STORE_RECORDINGS, 'readwrite');
    const req = tx.objectStore(STORE_RECORDINGS).put({ ...ar.rec, id: rid });
    req.onsuccess = () => resolve(ar.rec);
    req.onerror = () => reject(req.error || new Error('syncRecordingMeta put 失败'));
  });
  return ar.rec;
}

/**
 * 恢复(续录)一条已存在的录制:重建内存会话,seq 从已有最大值继续。
 * 用于报警记录的跨页面合并(相同报警写同一条 recording)。
 * @returns {Promise<number|null>} rid(记录不存在时返回 null)
 */
export async function resumeRecording(recordingId) {
  const rid = Number(recordingId);
  if (_activeRecorders.has(rid)) return rid;
  const db = await openRecorderDB();
  const lastSeq = await new Promise((resolve, reject) => {
    const tx = db.transaction(STORE_FRAMES, 'readonly');
    const idx = tx.objectStore(STORE_FRAMES).index(INDEX_FRAME_BY_RID);
    const req = idx.openCursor(IDBKeyRange.only(rid), 'prev');
    req.onsuccess = (evt) => { const c = evt.target.result; resolve(c ? c.value.seq + 1 : 0); };
    req.onerror = () => reject(req.error || new Error('resumeRecording cursor 失败'));
  });
  const rec = await new Promise((resolve, reject) => {
    const tx = db.transaction(STORE_RECORDINGS, 'readonly');
    const rq = tx.objectStore(STORE_RECORDINGS).get(rid);
    rq.onsuccess = () => resolve(rq.result);
    rq.onerror = () => reject(rq.error || new Error('resumeRecording get 失败'));
  });
  if (!rec) return null;
  const tsOffsetMs = Number(rec.durationMs) || 0;
  _activeRecorders.set(rid, {
    rec: { ...rec, firstTimestampMs: null, tsOffsetMs },
    buffer: [],
    seq: lastSeq,
    flushTimer: setTimeout(() => _flushBufferFor(rid), FLUSH_INTERVAL_MS),
  });
  return rid;
}

/**
 * 列出所有录制记录(按开始时间倒序,最新在前)
 * @returns {Promise<Array<{id:number,name:string,startedAt:number,endedAt:number|null,durationMs:number,totalFrames:number,motorFrameCount:number,imuFrameCount:number,rosUrl:string,modelId:string}>>}
 */
export async function listRecordings() {
  const db = await openRecorderDB();
  return await new Promise((resolve, reject) => {
    const tx = db.transaction(STORE_RECORDINGS, 'readonly');
    const store = tx.objectStore(STORE_RECORDINGS);
    const recs = [];
    // 如果有按 startedAt 的索引,就倒序;否则退化为全表倒序追加(这里直接用索引)
    const idx = store.index(INDEX_REC_BY_START);
    const req = idx.openCursor(null, 'prev');
    req.onsuccess = (evt) => {
      const cursor = evt.target.result;
      if (cursor) { recs.push(cursor.value); cursor.continue(); }
      else resolve(recs.map(_sanitizeRecording));
    };
    req.onerror = () => reject(req.error || new Error('listRecordings 失败'));
    tx.onerror = () => reject(tx.error || new Error('listRecordings 事务失败'));
  });
}

/** 对 recording 行做一个轻量的 sanitize(保证字段完整,UI 读取方便) */
function _sanitizeRecording(r) {
  return {
    id: Number(r.id),
    name: String(r.name || ''),
    rosUrl: String(r.rosUrl || ''),
    modelId: r.modelId == null ? '' : String(r.modelId),
    startedAt: Number(r.startedAt || 0),
    endedAt: r.endedAt == null ? null : Number(r.endedAt),
    durationMs: Number(r.durationMs || 0),
    motorFrameCount: Number(r.motorFrameCount || 0),
    imuFrameCount: Number(r.imuFrameCount || 0),
    totalFrames: Number(r.totalFrames || 0),
  };
}

/**
 * 按 id 获取单个 recording 元信息
 */
export async function getRecording(id) {
  const rid = Number(id);
  const db = await openRecorderDB();
  return await new Promise((resolve, reject) => {
    const tx = db.transaction(STORE_RECORDINGS, 'readonly');
    const store = tx.objectStore(STORE_RECORDINGS);
    const req = store.get(rid);
    req.onsuccess = () => resolve(req.result ? _sanitizeRecording(req.result) : null);
    req.onerror = () => reject(req.error || new Error('getRecording 失败'));
    tx.onerror = () => reject(tx.error || new Error('getRecording 事务失败'));
  });
}

/**
 * 重命名 recording
 */
export async function renameRecording(id, newName) {
  const rid = Number(id);
  const name = String(newName || '').trim() || '未命名录制';
  const db = await openRecorderDB();
  return await new Promise((resolve, reject) => {
    const tx = db.transaction(STORE_RECORDINGS, 'readwrite');
    const store = tx.objectStore(STORE_RECORDINGS);
    const gr = store.get(rid);
    gr.onsuccess = () => {
      if (!gr.result) return reject(new Error('record not found'));
      gr.result.name = name;
      const req = store.put(gr.result);
      req.onsuccess = () => resolve(_sanitizeRecording(gr.result));
      req.onerror = () => reject(req.error || new Error('rename put 失败'));
    };
    gr.onerror = () => reject(gr.error || new Error('rename get 失败'));
    tx.onerror = () => reject(tx.error || new Error('rename 事务失败'));
  });
}

/**
 * 删除 recording(连带删除关联的所有 frames)
 */
export async function deleteRecording(id) {
  const rid = Number(id);
  const db = await openRecorderDB();
  // 如果是进行中的录制,先强制 stopRecording(丢缓冲)
  if (_activeRecorders.has(rid)) {
    try {
      const ar = _activeRecorders.get(rid);
      if (ar.flushTimer) clearTimeout(ar.flushTimer);
      _activeRecorders.delete(rid);
    } catch (_) { /* ignore */ }
  }
  await new Promise((resolve, reject) => {
    const tx = db.transaction([STORE_RECORDINGS, STORE_FRAMES], 'readwrite');
    // 先删 frames (by_recordingId 索引 键范围只删这个 recordingId)
    const frameStore = tx.objectStore(STORE_FRAMES);
    const frameIdx = frameStore.index(INDEX_FRAME_BY_RID);
    const keyRange = IDBKeyRange.only(rid);
    const cursorReq = frameIdx.openCursor(keyRange);
    cursorReq.onsuccess = (evt) => {
      const cursor = evt.target.result;
      if (cursor) { cursor.delete(); cursor.continue(); }
    };
    cursorReq.onerror = () => reject(cursorReq.error || new Error('delete frames cursor 失败'));
    // 再删 recordings 主记录
    const recDel = tx.objectStore(STORE_RECORDINGS).delete(rid);
    recDel.onerror = () => reject(recDel.error || new Error('delete recording 主记录失败'));
    tx.oncomplete = () => resolve();
    tx.onerror = () => reject(tx.error || new Error('deleteRecording 事务失败'));
    tx.onabort = () => reject(tx.error || new Error('deleteRecording 事务中止'));
  });
  return true;
}

/**
 * 读取某 recording 的所有帧(按 seq 升序),方便复现器按时间排序播放。
 * 如果 recording 帧量很大(10min+ 几十万帧),这里一次读全内存可能爆。
 * 对本项目每 1 分钟 ≈ 6000 帧,基本足够。如需上到小时级,可改为流式逐帧 onFrame。
 * @param {number} id recordingId
 * @returns {Promise<Array<{recordingId:number,seq:number,topic:string,tsRelMs:number,tsAbsMs:number,data:number[]}>>}
 */
export async function loadRecordingFrames(id) {
  const rid = Number(id);
  const db = await openRecorderDB();
  return await new Promise((resolve, reject) => {
    const tx = db.transaction(STORE_FRAMES, 'readonly');
    const store = tx.objectStore(STORE_FRAMES);
    const idx = store.index(INDEX_FRAME_BY_RID);
    const keyRange = IDBKeyRange.only(rid);
    const frames = [];
    const req = idx.openCursor(keyRange, 'next');
    req.onsuccess = (evt) => {
      const cursor = evt.target.result;
      if (cursor) { frames.push(cursor.value); cursor.continue(); }
      else {
        // 兼容修复:修复历史遗留的跨会话合并记录(performance.now 跨刷新归零导致 tsRelMs 倒退/为负)。
        // 逐帧钳制为单调不减,保证回放时间轴有效;新录制已从根源修复,不会触发此分支。
        let prev = -Infinity;
        for (const f of frames) {
          if (!Number.isFinite(f.tsRelMs) || f.tsRelMs < prev) f.tsRelMs = prev === -Infinity ? 0 : prev + 1;
          prev = f.tsRelMs;
        }
        resolve(frames);
      }
    };
    req.onerror = () => reject(req.error || new Error('loadRecordingFrames cursor 失败'));
    tx.onerror = () => reject(tx.error || new Error('loadRecordingFrames 事务失败'));
  });
}
