/**
 * 机器人模型加载与解析。
 *
 * 支持两种模型格式:
 *  - OBJ + 简化 XML (模型 1/2): model/<id>/robot.xml + model/<id>/meshes/*.obj
 *  - STL + MJCF XML (模型 3): model/<id>/<path>/JXG.xml + model/<id>/<path>/meshes/*.stl
 *
 * 通过 MODEL_REGISTRY 中的 xmlPath / meshDir / format 字段区分。
 *
 * 解析流程:
 *  1. 根据 modelId 从 MODEL_REGISTRY 查找配置
 *  2. fetch XML 文件并解析 asset > mesh 列表
 *  3. 递归构建 body 树:解析 joint 与 geom
 *  4. 异步加载所有 mesh 文件(OBJLoader 或 STLLoader)
 *  5. 收集受控关节(actuator 中的 position/motor 元素)
 *  6. 初始化控制面板、笛卡尔控制、关节注释
 *
 * 关节类型:
 *  - slide:沿 axis 平移
 *  - hinge:绕 axis 旋转
 *  - free: 6 自由度浮动基(跳过,不加入控制)
 */
import * as THREE from 'three';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import {
  jointStates, bodyObjects, visibleControls, shared,
  setCurrentModelId, currentModelId, MODEL_REGISTRY,
} from './state.js?v=1';
import { scene, status, jointCountElement } from './scene.js?v=2';
import { setJointPose, renderControlPanel } from './joints.js?v=1';
import { initializeCartesianControl, enforceGroundContact, setGroundSnapMode } from './cartesian.js?v=1018';
import { initializeJointAnnotations, initializeStaticMarkers } from './annotations.js?v=949';

const objLoader = new OBJLoader();
const stlLoader = new STLLoader();

/** 并发限制器:限制同时进行的网格加载请求数,避免浏览器连接池溢出导致 ERR_ABORTED */
const MAX_CONCURRENT = 5;
let activeCount = 0;
const waitQueue = [];
function runWithLimit(task) {
  return new Promise((resolve, reject) => {
    const exec = () => {
      activeCount += 1;
      task().then(resolve, reject).finally(() => {
        activeCount -= 1;
        if (waitQueue.length) waitQueue.shift()();
      });
    };
    if (activeCount < MAX_CONCURRENT) exec();
    else waitQueue.push(exec);
  });
}

/** 加载代际计数器:每次开始新一轮加载时递增,用于作废旧模型的异步回调 */
let loadGeneration = 0;

/** 开启新的加载代际,返回当前代际号 */
export function nextLoadGeneration() {
  loadGeneration += 1;
  return loadGeneration;
}

// ── 模型配置查找 ─────────────────────────────────────────

/**
 * 根据模型 ID 从注册表查找配置,生成资源路径前缀。
 * @param {string|number} modelId
 * @returns {{ xml: string, mesh: (file: string) => string, format: string, config: object }}
 */
function pathsForModel(modelId) {
  const id = String(modelId);
  const config = MODEL_REGISTRY.find((m) => m.id === id);
  if (!config) throw new Error(`未注册的模型: ${id}`);
  return {
    xml: `../${config.xmlPath}`,
    mesh: (file) => `../${config.meshDir}${file}`,
    format: config.format || 'obj',
    config,
  };
}

// ── XML 解析工具函数 ─────────────────────────────────────

/** 解析 XML 属性中的数字列表(空格分隔) */
function numbers(value, fallback = [0, 0, 0]) {
  return value ? value.trim().split(/\s+/).map(Number) : fallback;
}

/** 应用 XML 元素的位移(pos)与旋转(quat)到 Three.js 对象 */
function applyTransform(object, element) {
  const position = numbers(element.getAttribute('pos'));
  object.position.set(position[0], position[1], position[2]);
  const q = numbers(element.getAttribute('quat'), [1, 0, 0, 0]);
  object.quaternion.set(q[1], q[2], q[3], q[0]);
}

/** 统一为网格应用标准材质并启用阴影 */
function applyMeshMaterial(root, rgba) {
  const [r, g, b, alpha = 1] = numbers(rgba, [0.72, 0.74, 0.8, 1]);
  const color = new THREE.Color(r, g, b);
  if (root.isMesh) {
    root.material = new THREE.MeshStandardMaterial({
      color, roughness: 0.42, metalness: 0.2,
      transparent: alpha < 1, opacity: alpha,
    });
    root.castShadow = true;
    root.receiveShadow = true;
  } else {
    root.traverse((node) => {
      if (!node.isMesh) return;
      node.material = new THREE.MeshStandardMaterial({
        color, roughness: 0.42, metalness: 0.2,
        transparent: alpha < 1, opacity: alpha,
      });
      node.castShadow = true;
      node.receiveShadow = true;
    });
  }
}

// ── 网格加载器 ───────────────────────────────────────────

/**
 * 根据格式加载单个网格文件并添加到 holder。
 * @param {string} url - 网格文件 URL
 * @param {string} format - 'obj' | 'stl'
 * @param {THREE.Group} holder - 承载网格的父节点
 * @param {string|null} rgba - 颜色 RGBA 属性
 * @param {{onProgress: (loaded: number, total: number) => void}} opts
 */
function loadMesh(url, format, holder, rgba, opts) {
  return runWithLimit(() => {
    const loader = format === 'stl' ? stlLoader : objLoader;
    return loader.loadAsync(url).then((result) => {
      if (format === 'stl') {
        const geometry = result;
        geometry.computeVertexNormals();
        const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({
          color: 0.9, roughness: 0.42, metalness: 0.2,
        }));
        applyMeshMaterial(mesh, rgba);
        holder.add(mesh);
      } else {
        applyMeshMaterial(result, rgba);
        holder.add(result);
      }
      opts.onProgress();
    }).catch((err) => {
      console.warn(`网格加载失败: ${url}`, err?.message || err);
      opts.onProgress();
    });
  });
}

// ── 模型加载 ─────────────────────────────────────────────

/** 转圈遮罩控制 */
const _loadingOverlay = document.querySelector('#loading-overlay');
const _loadingText = document.querySelector('#loading-text');
function showLoading(text = '加载中...') {
  if (!_loadingOverlay) return;
  _loadingOverlay.classList.remove('is-hidden');
  if (_loadingText) _loadingText.textContent = text;
}
function hideLoading() {
  if (!_loadingOverlay) return;
  _loadingOverlay.classList.add('is-hidden');
}

/**
 * 加载机器人模型。
 *
 * @param {string|number} [modelId] - 模型 ID;不传则使用 currentModelId
 */
// ── 机器狗朝向箭头(模型 2 专用) ──────────────────────────
/** 构建躯干朝向箭头:圆柱柄 + 锥形头,挂在 modelRoot 下方随身体移动/倾斜(+X 为前向) */
function buildHeadingArrow() {
  const mat = new THREE.MeshBasicMaterial({
    color: 0x22d3ee, transparent: true, opacity: 0.85,
    depthWrite: false,
  });
  // 柄:圆柱(默认沿 Y,旋转到 X 轴)
  const shaftGeo = new THREE.CylinderGeometry(0.04, 0.04, 0.55, 16);
  shaftGeo.rotateZ(-Math.PI / 2);
  const shaft = new THREE.Mesh(shaftGeo, mat);
  shaft.position.x = -0.13; // 尾部 -0.405 → 0.145
  shaft.renderOrder = 2;
  // 头:锥形(尖端 +Y → +X)
  const headGeo = new THREE.ConeGeometry(0.13, 0.3, 24);
  headGeo.rotateZ(-Math.PI / 2);
  const head = new THREE.Mesh(headGeo, mat);
  head.position.x = 0.295; // 底 0.145 → 尖 0.445
  head.renderOrder = 2;
  const group = new THREE.Group();
  group.add(shaft, head);
  // 悬浮在躯干上方(base_link 原点之上 0.28m),水平指向 +X
  group.position.set(0, 0, 0.28);
  return group;
}

/** 轮子转动标识:为 foot/wheel 轮体的转动节点(content)加一条与轮直径等长的
 *  橙色标条,贴在轮「外侧」面(左轮贴左、右轮贴右),随轮转动如轮辐般清晰可见。
 *  外侧方向由 updateWheelSpokeSides() 在 animate 中每帧校正,防止关节旋转后标条跑到内侧 */
const _wheelSpokes = [];   // [{ mark, node, axisIdx, barH, basePos }]
function addWheelSpoke(state) {
  const node = state.content || state.pivot;
  if (!node) return;
  // 外侧判断:车头 +X、左 +Y 约定,轮心世界 y≥0 为左侧轮 → 标记贴 +轴侧
  const worldPos = node.getWorldPosition(new THREE.Vector3());
  const side = worldPos.y >= 0 ? 1 : -1;
  // 临时摘下节点再取包围盒:避免节点自身旋转把世界 AABB 转换回局部时尺寸膨胀
  const parent = node.parent;
  if (parent) parent.remove(node);
  node.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(node);
  if (parent) parent.add(node);
  const size = box.getSize(new THREE.Vector3());
  const center = box.getCenter(new THREE.Vector3());
  const dims = [size.x, size.y, size.z];
  const centers = [center.x, center.y, center.z];    // 注意:Vector3 无数字索引,必须显式转数组
  const axisIdx = dims.indexOf(Math.min(...dims));   // 最薄维度 = 轮轴
  const longIdx = dims.indexOf(Math.max(...dims));   // 最长维度 = 直径方向
  const radius = dims[longIdx] / 2;
  if (!(radius > 0.01)) return;
  // 标条:长度 = 轮直径,窄条截面,贴在轮子外侧端面(远离身体一侧)
  const barLen = radius * 2;
  const barH = Math.max(0.02, dims[axisIdx] * 0.25);    // 轴向厚(加粗,更醒目)
  const barW = Math.max(0.03, radius * 0.22);           // 径向宽(加宽,易辨认)
  const len = [barW, barW, barW];
  len[longIdx] = barLen;
  len[axisIdx] = barH;
  const geo = new THREE.BoxGeometry(len[0], len[1], len[2]);
  const mat = new THREE.MeshBasicMaterial({ color: 0xff9f43 });
  const mark = new THREE.Mesh(geo, mat);
  // 径向分量归零(标条绕关节轴转动),仅在轴向(最薄维度)贴到轮体外侧端面;
  // 轴向位置由 axisCenter + 端面偏移决定(见下方),兼容转轴不在轮厚中点的脚轮
  const halfThick = dims[axisIdx] / 2;
  // 轮体沿轴向相对关节原点的偏心:四足脚轮的转轴在轮「内侧」端面附近,
  // 轮体整体悬向外侧(FL/RL center≈+0.053、FR/RR≈-0.052)。不补偿会把标条
  // 埋进轮体内部(只能透过轮毂镂空看到黄块),必须叠加该偏心才贴得到外侧端面。
  const axisCenter = centers[axisIdx];
  mark.position.set(0, 0, 0);
  mark.position.setComponent(axisIdx, axisCenter + side * (halfThick + barH / 2 + 0.005));
  mark.renderOrder = 5;
  node.add(mark);
  _wheelSpokes.push({ mark, node, axisIdx, barH, halfThick, axisCenter });
}

/** 每帧校正轮子示意条的外侧方向:用「身体中心 → 轮心」的水平方向作为外侧方向,
 *  转到节点局部后取轴向分量符号——机器人任意转向/轮子转向后,
 *  标条始终贴在远离身体的外侧端面,不会跑到内侧 */
const _spokeRootPos = new THREE.Vector3();
const _spokeWheelPos = new THREE.Vector3();
const _spokeOutward = new THREE.Vector3();
const _spokeTmpInvQ = new THREE.Quaternion();
export function updateWheelSpokeSides() {
  const root = shared.modelRoot;
  if (!root) return;
  root.getWorldPosition(_spokeRootPos);
  for (const s of _wheelSpokes) {
    const { mark, node, axisIdx, barH, halfThick, axisCenter } = s;
    if (!mark.parent) continue;
    // 身体中心 → 轮心 的水平方向 = 外侧方向(不管机器人怎么转都成立)
    node.getWorldPosition(_spokeWheelPos);
    _spokeOutward.copy(_spokeWheelPos).sub(_spokeRootPos);
    _spokeOutward.z = 0;                       // 只取水平面,忽略高度差
    if (_spokeOutward.lengthSq() < 1e-6) continue;
    _spokeOutward.normalize();
    // 转到节点局部坐标,取轴向分量符号决定标条贴哪一侧端面
    node.getWorldQuaternion(_spokeTmpInvQ).invert();
    _spokeOutward.applyQuaternion(_spokeTmpInvQ);
    const sgn = _spokeOutward.getComponent(axisIdx) >= 0 ? 1 : -1;
    // axisCenter 补偿轮体轴向偏心;barH/2 让标条内表面与轮面仅隔 0.005 间隙
    mark.position.setComponent(axisIdx, axisCenter + sgn * (halfThick + barH / 2 + 0.005));
  }
}

/** 清空轮子示意条引用(切换模型时调用) */
export function clearWheelSpokes() { _wheelSpokes.length = 0; }

export async function loadModel(modelId) {
  const id = String(modelId ?? currentModelId);
  setCurrentModelId(id);
  showLoading('加载模型骨架...');
  const gen = nextLoadGeneration();
  clearWheelSpokes();   // 清空旧模型示意条引用
  const paths = pathsForModel(id);
  const format = paths.format;

  const response = await fetch(paths.xml);
  if (!response.ok) throw new Error(`无法读取模型 ${id} 描述文件`);
  const xml = new DOMParser().parseFromString(await response.text(), 'application/xml');

  // 1. 解析 asset 中的 mesh 定义
  //    键为 mesh 名称,值为文件名(去掉路径前缀,处理大小写)
  const meshFiles = new Map();
  xml.querySelectorAll('asset > mesh').forEach((mesh) => {
    const fileName = mesh.getAttribute('file').replace(/^mujoco_meshes\//i, '');
    meshFiles.set(mesh.getAttribute('name').toLowerCase(), fileName);
  });

  // 2. 创建模型根
  shared.modelRoot = new THREE.Group();
  shared.modelRoot.name = `Robot_${id}`;
  scene.add(shared.modelRoot);

  const meshTasks = [];
  let requestedMeshes = 0;
  let loadedMeshes = 0;
  const updateStatus = () => {
    status.textContent = `模型 ${id} · 加载网格 ${loadedMeshes} / ${requestedMeshes}`;
  };

  // 3. 递归构建刚体树
  function buildBody(bodyElement, parent) {
    const body = new THREE.Group();
    body.name = bodyElement.getAttribute('name') || 'body';
    bodyObjects.set(body.name, body);
    applyTransform(body, bodyElement);
    parent.add(body);

    // 惯量参数(质量加权重心用):<inertial pos mass> 定义在 body 局部系,
    // 必须在 free-joint 提前 return 之前解析(浮动基躯干自身也有质量)
    const inertialEl = [...bodyElement.children].find((el) => el.tagName === 'inertial');
    if (inertialEl) {
      const mass = parseFloat(inertialEl.getAttribute('mass') || '0');
      const ipos = numbers(inertialEl.getAttribute('pos'), [0, 0, 0]);
      if (Number.isFinite(mass) && mass > 0) {
        body.userData.mass = mass;
        body.userData.comLocal = new THREE.Vector3(ipos[0], ipos[1], ipos[2]);
      }
    }

    // 3a. 解析关节(joint)
    const jointElement = [...bodyElement.children].find((el) => el.tagName === 'joint');
    const pivot = new THREE.Group();
    const content = new THREE.Group();
    let contentParent = body;

    if (jointElement) {
      const jointType = jointElement.getAttribute('type') || 'hinge';

      if (jointType === 'free') {
        // 浮动基关节:6 自由度,不加入控制面板,跳过
        // 但仍将 body 直接挂到 parent 下(不通过 pivot/content 结构)
        [...bodyElement.children]
          .filter((el) => el.tagName === 'body')
          .forEach((child) => buildBody(child, body));
        [...bodyElement.children]
          .filter((el) => el.tagName === 'geom')
          .forEach((geom) => attachGeom(geom, body));
        return;
      }

      const jointPosition = new THREE.Vector3(...numbers(jointElement.getAttribute('pos')));
      pivot.position.copy(jointPosition);
      body.add(pivot);
      pivot.add(content);
      content.position.copy(jointPosition).multiplyScalar(-1);
      contentParent = content;

      const range = numbers(jointElement.getAttribute('range'));
      const axis = new THREE.Vector3(...numbers(jointElement.getAttribute('axis'), [0, 0, 1])).normalize();
      const min = range[0] ?? -Math.PI;
      const max = range[1] ?? Math.PI;
      const initialValue = THREE.MathUtils.clamp(0, min, max);
      const state = {
        name: jointElement.getAttribute('name'),
        pivot, content, jointPosition,
        axis,
        type: jointType,
        min, max,
        value: initialValue, target: initialValue, velocity: 0,
        controlMode: 'position',
      };
      jointStates.set(state.name, state);
      setJointPose(state, initialValue);
    }

    // 3b. 挂载几何体(geom)到 contentParent
    [...bodyElement.children]
      .filter((el) => el.tagName === 'geom')
      .forEach((geom) => attachGeom(geom, contentParent));

    // 3c. 递归处理子 body
    [...bodyElement.children]
      .filter((el) => el.tagName === 'body')
      .forEach((child) => buildBody(child, contentParent));
  }

  /** 将 geom 元素挂载到指定容器,mesh 类型触发异步加载 */
  function attachGeom(geom, target) {
    if (geom.getAttribute('type') !== 'mesh') return;
    const meshName = geom.getAttribute('mesh');
    const file = meshFiles.get(meshName?.toLowerCase());
    if (!file) return;

    requestedMeshes += 1;
    const holder = new THREE.Group();
    applyTransform(holder, geom);
    target.add(holder);

    const scaleAttr = xml.querySelector(`asset > mesh[name="${meshName}"]`)?.getAttribute('scale');
    const meshScale = numbers(scaleAttr, [1, 1, 1]);

    meshTasks.push(
      loadMesh(paths.mesh(file), format, holder, geom.getAttribute('rgba'), {
        onProgress: () => {
          loadedMeshes += 1;
          updateStatus();
        },
      }).then(() => {
        if (meshScale[0] !== 1 || meshScale[1] !== 1 || meshScale[2] !== 1) {
          holder.scale.set(meshScale[0], meshScale[1], meshScale[2]);
        }
      }),
    );
  }

  // 4. 从 worldbody 启动构建
  const worldBody = xml.querySelector('worldbody');
  [...worldBody.children]
    .filter((el) => el.tagName === 'body')
    .forEach((body) => buildBody(body, shared.modelRoot));

  // 5. 收集受控关节
  //    position 驱动器(模型 1/2) + motor 驱动器(模型 3)
  const controlled = new Set();
  const actuatorSection = xml.querySelector('actuator');
  if (actuatorSection) {
    const positionActuators = actuatorSection.querySelectorAll('position, motor');
    positionActuators.forEach((actuator) => {
      const jointName = actuator.getAttribute('joint');
      const state = jointStates.get(jointName);
      if (state) {
        visibleControls.push(state);
        controlled.add(state.name);
      }
    });
  }

  // 6. 初始化 UI 与控制器(不依赖 mesh,可立即执行)
  renderControlPanel();
  initializeCartesianControl();
  initializeJointAnnotations();
  initializeStaticMarkers();

  // 6.5 机器狗朝向箭头:挂在躯干上方,随身体移动与倾斜(仅模型 2)
  //     高度先给默认值,mesh 加载完成后按躯干实际包围盒再校准(防穿模)
  const headingArrow = id === '2' ? buildHeadingArrow() : null;
  if (headingArrow) shared.modelRoot.add(headingArrow);

  // 7. 四足机器人站立姿态初始化(模型 2)
  //    不依赖 mesh,提前执行让模型骨架立即可见、可交互
  if (id === '2') {
    const standingAngles = {
      'FL_hip_joint': 0, 'FL_thigh_joint': -0.85, 'FL_calf_joint': 1.71, 'FL_foot_joint': 0,
      'FR_hip_joint': 0, 'FR_thigh_joint': -0.85, 'FR_calf_joint': 1.71, 'FR_foot_joint': 0,
      'RL_hip_joint': 0, 'RL_thigh_joint': 0.85, 'RL_calf_joint': -1.71, 'RL_foot_joint': 0,
      'RR_hip_joint': 0, 'RR_thigh_joint': 0.85, 'RR_calf_joint': -1.71, 'RR_foot_joint': 0,
    };
    for (const [name, angle] of Object.entries(standingAngles)) {
      const state = jointStates.get(name);
      if (state) {
        const clamped = THREE.MathUtils.clamp(angle, state.min, state.max);
        state.value = clamped;
        state.target = clamped;
        setJointPose(state, clamped);
        const input = document.querySelector(`input[data-joint="${name}"]`);
        if (input) {
          input.value = String(clamped);
          const output = input.parentElement.querySelector('output');
          if (output) output.textContent = clamped.toFixed(2);
        }
      }
    }
    setGroundSnapMode(true);
    shared.modelRoot.position.z = 0.1;
    enforceGroundContact();
  }

  // 8. 网格懒加载:不阻塞主线程,后台分批加载 mesh
  //    每个 mesh 加载完成后自动显示(已挂载到 holder),用户可立即看到骨架与已加载部分
  //    骨架(body 树 + 关节)已就绪,UI 可交互;mesh 在后台逐步填充
  status.textContent = `模型 ${id} · 加载网格 0 / ${requestedMeshes}`;
  if (jointCountElement) jointCountElement.textContent = `${visibleControls.length} 关节`;
  showLoading(`加载网格 0 / ${requestedMeshes}`);

  // 轮询更新转圈进度文字(~5fps 足够,避免频繁 DOM 写入)
  const _progressTimer = setInterval(() => {
    if (gen !== loadGeneration) { clearInterval(_progressTimer); return; }
    showLoading(`加载网格 ${loadedMeshes} / ${requestedMeshes}`);
    status.textContent = `模型 ${id} · 加载网格 ${loadedMeshes} / ${requestedMeshes}`;
  }, 200);

  Promise.all(meshTasks)
    .then(() => {
      clearInterval(_progressTimer);
      if (gen !== loadGeneration) return;
      // 箭头高度校准:测量 base_link 躯干在 modelRoot 局部空间的包围盒顶,
      // 把箭头放到躯干顶 + 锥底半径 + 余量,避免穿模
      if (headingArrow && shared.modelRoot) {
        const baseBody = shared.modelRoot.getObjectByName('base_link');
        if (baseBody) {
          baseBody.updateWorldMatrix(true, true);
          const inv = shared.modelRoot.matrixWorld.clone().invert();
          const box = new THREE.Box3().setFromObject(baseBody).applyMatrix4(inv);
          headingArrow.position.z = box.max.z + 0.16; // 锥底半径 0.13 + 余量 0.03
        }
      }
      // 轮子转动标识:所有 foot/wheel 关节加贴面辐条(全部模型通用)
      jointStates.forEach((state, name) => {
        if (/foot|wheel/i.test(name)) addWheelSpoke(state);
      });
      status.textContent = `模型 ${id} 就绪 · ${loadedMeshes} 个网格`;
      hideLoading();
    })
    .catch((err) => {
      clearInterval(_progressTimer);
      if (gen !== loadGeneration) return;
      status.textContent = `模型 ${id} 就绪 · ${loadedMeshes}/${requestedMeshes} 个网格(部分加载失败)`;
      hideLoading();
      console.warn(`[model] 部分 mesh 加载失败:`, err?.message || err);
    });
}
