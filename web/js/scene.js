/**
 * Three.js 场景初始化。
 *
 * 集中创建并导出渲染所需的全部对象:
 *  - DOM 引用(canvas、viewport、状态栏、控制面板)
 *  - 场景(scene)、WebGL 渲染器、CSS2D 标签渲染器
 *  - 相机与 OrbitControls(轨道控制器)
 *  - 灯光(半球光 + 主光 + 边缘光)
 *  - 地板与网格
 *
 * 相机 up 轴设为 Z(机器人 Z 轴朝上,符合 MuJoCo/URDF 惯例)。
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { CSS2DRenderer } from 'three/addons/renderers/CSS2DRenderer.js';

// ── DOM 引用 ──────────────────────────────────────────────
export const canvas = document.querySelector('#scene');
export const viewport = document.querySelector('.viewport');
export const status = document.querySelector('#load-status');
export const controlsElement = document.querySelector('#controls');
export const jointCountElement = document.querySelector('#joint-count');

// ── Three.js 场景初始化 ───────────────────────────────────
export const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1e1e1e);
scene.fog = new THREE.Fog(0x1e1e1e, 5, 14);

export const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2)); // 限制最大像素比,避免高 DPI 设备性能下降
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap; // 软阴影
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping; // 电影级色调映射
renderer.toneMappingExposure = 1.12;

// CSS2D 渲染器:用于关节注释标签(HTML 元素叠加在 3D 场景上)
// 挂到 canvas 的父节点而非固定 .viewport:移动端会把 canvas 搬入面板内的
// 「仿真」页签容器,注释层必须随之移动(桌面端父节点仍是 .viewport,无差异)
export const labelRenderer = new CSS2DRenderer();
labelRenderer.domElement.className = 'annotation-layer';
canvas.parentElement.append(labelRenderer.domElement);

export const camera = new THREE.PerspectiveCamera(38, 1, 0.05, 100);
camera.up.set(0, 0, 1); // Z 轴朝上(机器人坐标系惯例)

export const orbit = new OrbitControls(camera, renderer.domElement);
orbit.enableDamping = true;       // 启用阻尼,让相机运动更平滑
orbit.dampingFactor = 0.075;
orbit.minDistance = 1.2;
orbit.maxDistance = 8;
// 移除 maxPolarAngle 限制,允许查看底部视角

// ── 视角控制 ─────────────────────────────────────────────
/** 默认视角:相机位置与注视目标 */
export const defaultView = {
  camera: new THREE.Vector3(3.25, -4.1, 2.55),
  target: new THREE.Vector3(0, 0, 0.95),
};

/**
 * 设置相机位置与注视目标。
 * @param {THREE.Vector3} position - 相机位置
 * @param {THREE.Vector3} target - 注视目标(默认为 defaultView.target)
 */
export function setView(position, target = defaultView.target) {
  camera.position.copy(position);
  orbit.target.copy(target);
  orbit.update();
}

setView(defaultView.camera);

// ── 灯光 ─────────────────────────────────────────────────
// 半球光:天窗色 + 地面色,提供基础环境照明
// (强度适中:过高会把阴影"洗掉",机器人会看起来像悬浮在空中)
scene.add(new THREE.HemisphereLight(0xddeaff, 0x263042, 1.7));

// 主光:模拟太阳光,带阴影
const keyLight = new THREE.DirectionalLight(0xffffff, 3.5);
keyLight.position.set(3, -4, 6);
keyLight.castShadow = true;
keyLight.shadow.mapSize.set(2048, 2048);
keyLight.shadow.camera.left = -3;
keyLight.shadow.camera.right = 3;
keyLight.shadow.camera.top = 3;
keyLight.shadow.camera.bottom = -3;
keyLight.shadow.bias = -0.0004; // 防止阴影痤疮
scene.add(keyLight);

// 边缘光:冷色补光,勾勒机器人轮廓
const rimLight = new THREE.DirectionalLight(0x5fceff, 1.4);
rimLight.position.set(-4, 2, 3);
scene.add(rimLight);

// ── 地板与网格 ───────────────────────────────────────────
// 地板:亮度和 hue 与整体主题一致 — 太暗会让阴影不可见,机器人看起来像悬浮
const floor = new THREE.Mesh(
  new THREE.PlaneGeometry(30, 30),
  new THREE.MeshStandardMaterial({ color: 0x414f6e, roughness: 0.9, metalness: 0.05 }),
);
floor.receiveShadow = true;
scene.add(floor);

// 网格:旋转到 XY 平面(Z 朝上),略微抬升避免与地板重叠(z-fighting)
const grid = new THREE.GridHelper(12, 24, 0x4c607d, 0x334257);
grid.rotation.x = Math.PI / 2;
grid.position.z = 0.002;
grid.material.opacity = 0.42;
grid.material.transparent = true;
scene.add(grid);
