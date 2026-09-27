/**
 * 重心(CoM)可视化模块。
 *
 * 数据来源:模型加载时 model.js 把每个 body 的 <inertial pos mass>
 * 解析到 body.userData.{mass, comLocal}(comLocal 为 body 局部系质心)。
 * 每帧按质量加权求整机世界系重心:
 *   CoM_world = Σ mass_i · body_i.localToWorld(comLocal_i) / Σ mass_i
 * 刚体均为静态 OBJ 拼装(非 SkinnedMesh),body 世界矩阵随关节/底盘运动更新,
 * 因此该重心是语义稳定的物理重心,不依赖变形后的几何包围盒。
 *
 * 可视化元素:
 *  - 红球:重心点(外层线框球增强辨识)
 *  - 黄色竖线:重心到地面(z=0)的投影线
 *  - 红色圆环:重心在地面的投影点(判断是否落在支撑多边形内)
 *  - 标签:总质量 + 重心离地高度
 */
import * as THREE from 'three';
import { CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';
import { scene } from './scene.js?v=2';
import { shared, bodyObjects } from './state.js?v=1';

const COM_COLOR = 0xff4d6d;
const DROP_COLOR = 0xffd166;
const SPHERE_R = 0.038;

/** 是否可见(由 #com-toggle 开关控制) */
let visible = false;
/** 当前构建依据的 modelRoot(切换模型时重建刚体清单) */
let builtRoot = null;
/** @type {Array<{node: THREE.Group, mass: number}>} */
let bodies = [];
let totalMass = 0;

// ── 场景对象(懒构建一次) ─────────────────────────────────
const group = new THREE.Group();
group.name = 'CoMViz';
group.visible = false;
// 所有材质 depthTest:false,需高 renderOrder 保证在模型之后绘制(透视叠加)
group.renderOrder = 999;
scene.add(group);

// 重心实点球(depthTest:false 透视显示:重心位于躯干内部时仍可见)
const sphere = new THREE.Mesh(
  new THREE.SphereGeometry(SPHERE_R, 20, 16),
  new THREE.MeshBasicMaterial({ color: COM_COLOR, depthTest: false, depthWrite: false }),
);
group.add(sphere);
sphere.renderOrder = 999;
// 外层线框球
const wire = new THREE.Mesh(
  new THREE.SphereGeometry(SPHERE_R * 1.55, 16, 12),
  new THREE.MeshBasicMaterial({ color: COM_COLOR, wireframe: true, transparent: true, opacity: 0.55, depthTest: false }),
);
group.add(wire);
wire.renderOrder = 999;

// 到地面的投影线(2 点,每帧改 attribute)
const dropGeo = new THREE.BufferGeometry();
dropGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
const dropLine = new THREE.Line(
  dropGeo,
  new THREE.LineBasicMaterial({ color: DROP_COLOR, transparent: true, opacity: 0.9, depthTest: false }),
);
group.add(dropLine);
dropLine.renderOrder = 999;

// 地面投影圆环
const ring = new THREE.Mesh(
  new THREE.RingGeometry(0.05, 0.075, 32),
  new THREE.MeshBasicMaterial({ color: COM_COLOR, side: THREE.DoubleSide, transparent: true, opacity: 0.85, depthTest: false }),
);
ring.rotation.x = -Math.PI / 2;
group.add(ring);
ring.renderOrder = 999;

// 文字标签
const labelEl = document.createElement('div');
labelEl.className = 'com-label';
const label = new CSS2DObject(labelEl);
label.position.set(0, 0.09, 0);
sphere.add(label);

// ── 复用临时对象,避免每帧分配 ────────────────────────────
const _tmp = new THREE.Vector3();
const _com = new THREE.Vector3();

/** 按当前 bodyObjects 构建刚体质量清单(模型切换后自动重建) */
function rebuildIfNeeded() {
  const root = shared.modelRoot;
  if (!root) return false;
  if (builtRoot === root && bodies.length > 0) return true;
  builtRoot = root;
  bodies = [];
  totalMass = 0;
  bodyObjects.forEach((node) => {
    const mass = node.userData?.mass;
    if (Number.isFinite(mass) && mass > 0 && node.userData.comLocal) {
      bodies.push({ node, mass });
      totalMass += mass;
    }
  });
  return bodies.length > 0;
}

/** 设置重心可视化显隐 */
export function setComVisible(v) {
  visible = !!v;
  group.visible = visible;
  if (visible) updateComViz();
}

export function isComVisible() { return visible; }

/**
 * 每帧更新:质量加权求重心并移动标记。
 * 应在 animate 循环 renderer.render 之前调用(内部强制刷新世界矩阵)。
 */
export function updateComViz() {
  if (!visible) return;
  if (!rebuildIfNeeded()) {
    group.visible = false;
    document.body.dataset.comDebug = JSON.stringify({
      bodies: bodies.length, mass: +totalMass.toFixed(2),
      root: !!shared.modelRoot, visible: false,
      totalBodiesInMap: bodyObjects.size,
    });
    return;
  }
  group.visible = true;
  const root = shared.modelRoot;
  // 强制刷新整棵模型树的世界矩阵(本帧关节/底盘位姿可能刚更新,渲染尚未发生)
  root.updateMatrixWorld(true);

  _com.set(0, 0, 0);
  for (const { node, mass } of bodies) {
    _tmp.copy(node.userData.comLocal);
    node.localToWorld(_tmp);
    _com.addScaledVector(_tmp, mass);
  }
  _com.multiplyScalar(1 / totalMass);

  // 重心球
  sphere.position.copy(_com);

  // 投影线:重心 → (x, y, 0)
  const pos = dropGeo.attributes.position;
  pos.setXYZ(0, _com.x, _com.y, Math.max(_com.z, 0));
  pos.setXYZ(1, _com.x, _com.y, 0.012);
  pos.needsUpdate = true;

  // 地面圆环
  ring.position.set(_com.x, _com.y, 0.012);

  // 标签:总质量 + 离地高度
  labelEl.textContent = `重心 ${totalMass.toFixed(1)} kg · 高 ${Math.max(_com.z, 0).toFixed(2)} m`;
  // 调试钩子(与 curves.js 的 data-curves-debug 同风格):刚体数/总质量/CoM 坐标
  document.body.dataset.comDebug = JSON.stringify({
    bodies: bodies.length, mass: +totalMass.toFixed(2),
    x: +_com.x.toFixed(3), y: +_com.y.toFixed(3), z: +_com.z.toFixed(3),
    root: !!root, visible: group.visible,
  });
}
