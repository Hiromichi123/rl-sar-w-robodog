/**
 * 关节注释可视化模块。
 *
 * 为每个受控关节在 3D 场景中绘制注释:
 *  - 锚点:跟随关节位置的小球(Mesh)
 *  - 连线:从锚点到标签位置的 Line
 *  - 标签:CSS2DObject 显示关节中文名
 *
 * 布局策略:
 *  - 手臂关节:在锚点的相机右方/上方偏移(交替错开避免重叠)
 *  - 底盘关节:以模型根为基准,按世界高度偏移(前后高低不同)
 *  - 其他关节:固定水平/垂直偏移
 *
 * 每帧 updateJointAnnotations 根据相机朝向重新计算标签位置,
 * 保证标签始终在屏幕上合理位置(不被关节遮挡)。
 */
import * as THREE from 'three';
import { CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';
import { visibleControls, jointAnnotations, annotationVectors, shared, bodyObjects } from './state.js?v=1';
import { camera, labelRenderer } from './scene.js?v=2';
import { labelForJoint } from './joints.js?v=1';

/** 注释是否可见(由 #annotations-toggle 开关控制) */
let annotationsVisible = false;

// ── 注释布局与配色 ───────────────────────────────────────

/** 返回关节注释的屏幕布局偏移(水平/垂直或世界高度) */
function annotationLayoutForJoint(name) {
  // 四足机器人腿关节
  const legMatch = name.match(/^(FL|FR|RL|RR)_(hip|thigh|calf|foot)_joint$/);
  if (legMatch) {
    const side = legMatch[1];
    const part = legMatch[2];
    const sideSign = side === 'FL' || side === 'RL' ? 1 : -1;
    const hOff = sideSign * 0.5;
    let vOff = 0;
    if (part === 'hip') vOff = 0.25;
    else if (part === 'thigh') vOff = 0;
    else if (part === 'calf') vOff = -0.25;
    else if (part === 'foot') vOff = -0.35;
    return { horizontal: hOff, vertical: vOff };
  }
  const arm = name.match(/^robot_(left|right)_joint(\d)$/);
  if (arm) {
    const side = arm[1] === 'left' ? 1 : -1;
    const index = Number(arm[2]);
    return { horizontal: side * (0.53 + (index % 2) * 0.04), vertical: 0.32 - (index - 1) * 0.09 };
  }
  if (name.includes('_finger_')) {
    return { horizontal: name.includes('_left_') ? 0.64 : -0.64, vertical: -0.18 };
  }
  if (name.includes('head_pitch')) return { horizontal: 0.43, vertical: 0.26 };
  if (name.includes('head_yaw')) return { horizontal: -0.43, vertical: 0.36 };
  if (name === 'lift_joint') return { horizontal: -1.02, vertical: 0.3 };
  const module = name.match(/^(fl|fr|bl|br)_/);
  if (module) {
    const left = module[1].endsWith('l') ? 1 : -1;
    const wheel = name.includes('_wheel_');
    const front = module[1].startsWith('f');
    const worldHeight = front ? (wheel ? 0.3 : 0.42) : (wheel ? 0.08 : 0.18);
    return { horizontal: left * (wheel ? 0.92 : 0.8), worldHeight, useRoot: true };
  }
  return { horizontal: 0.52, vertical: 0.12 };
}

/** 返回关节注释的颜色(按部位区分) */
function annotationColorForJoint(name) {
  // 四足机器人腿:前腿青色、后腿紫色
  if (/^(FL|FR)_/.test(name)) return 0x57c7ff;
  if (/^(RL|RR)_/.test(name)) return 0xc59cff;
  if (name.includes('_left_')) return 0x57c7ff;
  if (name.includes('_right_')) return 0xff8b6b;
  if (name.includes('head')) return 0xffd166;
  if (name.includes('steering') || name.includes('_wheel_')) return 0x7ee787;
  return 0xc59cff;
}

/** 更新注释条目的文本标签 */
function updateAnnotationLabel(entry) {
  entry.element.querySelector('span').textContent = labelForJoint(entry.state.name);
  entry.element.title = entry.state.name;
}

// ── 注释初始化与更新 ─────────────────────────────────────

/** 为所有受控关节创建注释标记球、连线与文字标签 */
export function initializeJointAnnotations() {
  const markerGeometry = new THREE.SphereGeometry(0.009, 10, 7);
  visibleControls.forEach((state) => {
    const color = annotationColorForJoint(state.name);
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(6), 3));
    const line = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.88 }));
    line.frustumCulled = false;
    const marker = new THREE.Mesh(
      markerGeometry,
      new THREE.MeshBasicMaterial({ color, depthTest: false, transparent: true, opacity: 0.95 }),
    );
    marker.renderOrder = 20;
    const element = document.createElement('div');
    element.className = 'joint-annotation';
    element.dataset.joint = state.name;
    element.innerHTML = '<span></span>';
    element.style.borderColor = `#${color.toString(16).padStart(6, '0')}`;
    const label = new CSS2DObject(element);
    const entry = {
      state, line, marker, label, element,
      layout: annotationLayoutForJoint(state.name),
    };
    updateAnnotationLabel(entry);
    line.visible = annotationsVisible;
    marker.visible = annotationsVisible;
    label.visible = annotationsVisible;
    shared.modelRoot.add(line, marker, label);
    jointAnnotations.push(entry);
  });
  updateJointAnnotations();
}

/**
 * 每帧更新所有注释的连线端点与标签位置。
 * 标签位置跟随关节锚点,并按相机朝向做屏幕偏移,
 * 保证在不同视角下都清晰可读(不被关节遮挡)。
 */
/** 注释更新节流:~30fps 足够(人眼难以区分更高频的标签位置更新) */
let _annotUpdateLastAt = 0;

export function updateJointAnnotations() {
  if (!shared.modelRoot || !annotationsVisible) return;
  const now = performance.now();
  if (now - _annotUpdateLastAt < 33) return;
  _annotUpdateLastAt = now;
  const v = annotationVectors;
  shared.modelRoot.updateMatrixWorld(true);
  camera.updateMatrixWorld(true);
  // 取相机的右方与上方方向(世界坐标),用于标签偏移
  v.cameraRight.setFromMatrixColumn(camera.matrixWorld, 0).normalize();
  v.cameraUp.setFromMatrixColumn(camera.matrixWorld, 1).normalize();
  jointAnnotations.forEach((entry) => {
    if (!entry.state?.pivot) return;
    // 锚点:关节轴心在世界坐标的位置
    entry.state.pivot.getWorldPosition(v.anchorWorld);
    // 局部锚点:锚点在 modelRoot 局部空间的位置(标记球/连线起点)
    v.anchorLocal.copy(v.anchorWorld);
    shared.modelRoot.worldToLocal(v.anchorLocal);
    // 端点基准:底盘关节以模型根为基准,其余以关节锚点为基准
    if (entry.layout.useRoot) {
      shared.modelRoot.getWorldPosition(v.endpointWorld);
    } else {
      v.endpointWorld.copy(v.anchorWorld);
    }
    // 端点:沿相机右方偏移 horizontal
    v.endpointWorld.addScaledVector(v.cameraRight, entry.layout.horizontal);
    if (entry.layout.worldHeight !== undefined) {
      // 底盘关节:使用固定世界高度(不跟随相机垂直方向)
      v.endpointWorld.z = entry.layout.worldHeight;
    } else {
      // 其他关节:沿相机上方再偏移 vertical
      v.endpointWorld.addScaledVector(v.cameraUp, entry.layout.vertical);
    }
    // 世界坐标 → modelRoot 局部坐标(标签/连线挂在 modelRoot 下)
    v.endpointLocal.copy(v.endpointWorld);
    shared.modelRoot.worldToLocal(v.endpointLocal);
    // 写入连线两端顶点:起点=关节锚点,终点=标签端点
    const positions = entry.line.geometry.attributes.position;
    positions.setXYZ(0, v.anchorLocal.x, v.anchorLocal.y, v.anchorLocal.z);
    positions.setXYZ(1, v.endpointLocal.x, v.endpointLocal.y, v.endpointLocal.z);
    positions.needsUpdate = true;
    // 标记球与标签分别落到锚点/端点
    entry.marker.position.copy(v.anchorLocal);
    entry.label.position.copy(v.endpointLocal);
  });
  // 静态标记(雷达/电池仓):刚体附加在 base_link 上,局部位置固定
  staticAnnotations.forEach((entry) => {
    if (!entry.parent) return;
    entry.label.position.set(entry.position[0], entry.position[1], entry.position[2] + 0.08);
    const positions = entry.line.geometry.attributes.position;
    positions.setXYZ(0, entry.position[0], entry.position[1], entry.position[2]);
    positions.setXYZ(1, entry.label.position.x, entry.label.position.y, entry.label.position.z);
    positions.needsUpdate = true;
  });
}

/** 切换注释可见性并立即刷新布局 */
export function setAnnotationsVisible(visible) {
  annotationsVisible = visible;
  jointAnnotations.forEach((entry) => {
    entry.line.visible = visible;
    entry.marker.visible = visible;
    entry.label.visible = visible;
  });
  // 同步静态标记
  staticAnnotations.forEach((entry) => {
    entry.line.visible = visible;
    entry.marker.visible = visible;
    entry.label.visible = visible;
  });
  updateJointAnnotations();
}

// ── 静态标记(雷达/电池仓等) ────────────────────────────────

/** 静态注释列表(非关节标记) */
const staticAnnotations = [];

/** 静态标记配置:名称、位置、颜色 */
const STATIC_MARKERS = [
  { name: '雷达', position: [0.22, 0, 0.12], color: 0x00ff88 },
  { name: '电池仓', position: [-0.1, 0, -0.08], color: 0xffaa00 },
];

/**
 * 为四足机器人创建静态标记(雷达/电池仓)。
 * 这些标记附加到 base_link body:标记球与标签随机身刚体运动。
 */
export function initializeStaticMarkers() {
  const baseLink = bodyObjects.get('base_link');
  if (!baseLink) return;

  const markerGeometry = new THREE.SphereGeometry(0.015, 12, 8);

  STATIC_MARKERS.forEach((config) => {
    const color = config.color;
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(6), 3));
    const line = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.88 }));
    line.frustumCulled = false;

    const marker = new THREE.Mesh(
      markerGeometry,
      new THREE.MeshBasicMaterial({ color, depthTest: false, transparent: true, opacity: 0.95 }),
    );
    marker.renderOrder = 20;
    marker.position.set(...config.position);

    const element = document.createElement('div');
    element.className = 'joint-annotation';
    element.dataset.static = config.name;
    element.innerHTML = '<span></span>';
    element.style.borderColor = `#${color.toString(16).padStart(6, '0')}`;
    element.querySelector('span').textContent = config.name;
    element.title = config.name;

    const label = new CSS2DObject(element);
    label.position.set(config.position[0], config.position[1], config.position[2] + 0.08);

    const entry = {
      line, marker, label, element,
      position: config.position,
      name: config.name,
      parent: baseLink,
    };

    line.visible = annotationsVisible;
    marker.visible = annotationsVisible;
    label.visible = annotationsVisible;

    baseLink.add(line, marker, label);
    staticAnnotations.push(entry);
  });
}

/** 清理静态标记 */
export function clearStaticAnnotations() {
  for (const entry of staticAnnotations) {
    entry.element?.remove?.();
    entry.line?.geometry?.dispose?.();
    entry.line?.material?.dispose?.();
    entry.marker?.geometry?.dispose?.();
    entry.marker?.material?.dispose?.();
  }
  staticAnnotations.length = 0;
}
