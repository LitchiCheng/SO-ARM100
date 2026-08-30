/* arm3d.js — SO-ARM100 3D 手臂显示（three.js 本地模块，无 CDN）
 *
 * 运动学与网格来源：
 *   - 6 关节链 = MuJoCo 仿真模型 model/trs_so_arm100/so_arm100.xml（TRS SO-ARM100，z-up，单位 m）
 *   - 关节角 = 遥测 pos_deg × π/180（mujoco 零位系，恒等映射；
 *     已验证 so100_real_control.py 直接以 qpos(rad) × 180/π 作为电机度数下发）
 *   - 网格 = 同目录 assets/ 的 STL（拷贝至 static/model/so100/，米制，无需缩放）
 *   - GitHub 官方 URDF（TheRobotStudio/SO-ARM100 so100.urdf）与上述 MJCF 的
 *     关节 2–6 连杆坐标系约定不一致（数值验证无法用逐关节常数偏移调和），故不采用。
 */
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";

const MODEL_DIR = "/model/so100/";
const DEG2RAD = Math.PI / 180;

// 基座 body（无关节）：[STL 文件名, 是否电机壳体(黑)]
const BASE_MESHES = [["Base", false], ["Base_Motor", true]];

// 6 关节链（qpos/motor 顺序：1..6）。
// pos/quat = body 固定变换，运动学律（逐关节从 MuJoCo 真值拟合并 10 姿态验证，最大误差 0.0000 mm/deg）：
//   world = parent · T(pos) · R(quat) · Rot(axis, +θ)，axis 在 body 系，枢轴 = body 原点
// 注意 quat 顺序为 three.js 的 (x,y,z,w)：MJCF/XML 的四元数是 (w,x,y,z)，不可照抄
//（照抄曾导致 J1 绕 (1,1,0)/√2 旋转、J6 绕 Z 而非 Y，表现为 J1 与基座"脱节"）
const CHAIN = [
  { motor: 1, name: "shoulder_pan",  pos: [0, -0.0452, 0.0165],  quat: [0.707108, 0, 0, 0.707105],
    axis: [0, 1, 0], meshes: [["Rotation_Pitch", false], ["Rotation_Pitch_Motor", true]] },
  { motor: 2, name: "shoulder_lift", pos: [0, 0.1025, 0.0306],   quat: [0.707105, 0, 0, 0.707109],
    axis: [1, 0, 0], meshes: [["Upper_Arm", false], ["Upper_Arm_Motor", true]] },
  { motor: 3, name: "elbow_flex",    pos: [0, 0.11257, 0.028],   quat: [-0.707105, 0, 0, 0.707109],
    axis: [1, 0, 0], meshes: [["Lower_Arm", false], ["Lower_Arm_Motor", true]] },
  { motor: 4, name: "wrist_flex",    pos: [0, 0.0052, 0.1349],   quat: [-0.707105, 0, 0, 0.707109],
    axis: [1, 0, 0], meshes: [["Wrist_Pitch_Roll", false], ["Wrist_Pitch_Roll_Motor", true]] },
  // J5 显示偏移：物理零位与模型零位差 90°（用户实测"再多转 90 度就和可视化对应上"）。
  // 仅影响 3D 显示，不影响遥测/下发。若实际看起来转反了方向，把这里改成 +90 即可。
  { motor: 5, name: "wrist_roll",    pos: [0, -0.0601, 0],       quat: [0, 0.707105, 0, 0.707109],
    axis: [0, 1, 0], offset_deg: -90, meshes: [["Fixed_Jaw", false], ["Fixed_Jaw_Motor", true]] },
  { motor: 6, name: "gripper",       pos: [-0.0202, -0.0244, 0], quat: [-0.000004, 1, -0.000004, 0],
    axis: [0, 0, 1], meshes: [["Moving_Jaw", false]] },
];

/* ---------------- API（app.js 调用） ---------------- */

const api = { ready: false, _pending: null };
api.setTelemetry = function (t) { this._pending = t; };
window.Arm3D = api;

/* ---------------- 工具 ---------------- */

function jointColor(j) {
  if (!j) return "#5a5f6e";
  if (j.temp > 50) return "#e5484d";
  if (j.moving) return "#f5a524";
  if (j.torque === 0) return "#8b8d98";
  return "#46d16a";
}

function showMsg(text) {
  const el = document.getElementById("arm3d-msg");
  if (el) { el.textContent = text; el.classList.remove("hidden"); }
}
function hideMsg() {
  const el = document.getElementById("arm3d-msg");
  if (el) el.classList.add("hidden");
}

// 关节角度标签（canvas sprite，始终面向相机）
function makeLabel() {
  const cv = document.createElement("canvas");
  cv.width = 224; cv.height = 56;
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.SRGBColorSpace;
  const spr = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  spr.renderOrder = 100;
  spr.scale.set(0.078, 0.0195, 1);
  spr.userData = { canvas: cv, tex };
  return spr;
}

function drawLabel(spr, text, color) {
  const cv = spr.userData.canvas, ctx = cv.getContext("2d");
  ctx.clearRect(0, 0, cv.width, cv.height);
  ctx.font = "bold 30px monospace";
  const w = ctx.measureText(text).width + 26;
  ctx.fillStyle = "rgba(13,15,20,0.78)";
  ctx.strokeStyle = color;
  ctx.lineWidth = 2.5;
  const x = 8, y = 8, h = 40;
  if (ctx.roundRect) { ctx.beginPath(); ctx.roundRect(x, y, w, h, 8); ctx.fill(); ctx.stroke(); }
  else { ctx.fillRect(x, y, w, h); ctx.strokeRect(x, y, w, h); }
  ctx.fillStyle = "#e8eaf0";
  ctx.textBaseline = "middle";
  ctx.fillText(text, x + 13, y + h / 2 + 1);
  spr.userData.tex.needsUpdate = true;
}

/* ---------------- 场景构建 ---------------- */

async function init() {
  showMsg("加载 3D 模型…");
  const wrap = document.getElementById("arm3d-wrap");
  const canvas = document.getElementById("arm3d-canvas");

  let renderer;
  try {
    renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  } catch (e) {
    showMsg("WebGL 不可用，3D 显示关闭");
    return;
  }
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 10);

  // 灯光
  scene.add(new THREE.HemisphereLight(0xffffff, 0x2c313d, 1.15));
  const key = new THREE.DirectionalLight(0xffffff, 1.6);
  key.position.set(0.5, 0.9, 0.7);
  scene.add(key);
  const fill = new THREE.DirectionalLight(0x88aaff, 0.35);
  fill.position.set(-0.6, 0.3, -0.5);
  scene.add(fill);

  // 地面网格（MJCF z-up → three y-up：模型整体绕 X 转 -90°）
  const grid = new THREE.GridHelper(1.2, 24, 0x39415a, 0x222834);
  grid.position.y = -0.001;
  scene.add(grid);

  // 材质（与 MJCF 一致：橙色连杆 + 黑色电机壳体）
  const matOrange = new THREE.MeshStandardMaterial({ color: 0xff5400, roughness: 0.55, metalness: 0.05 });
  const matBlack = new THREE.MeshStandardMaterial({ color: 0x3a3f4a, roughness: 0.45, metalness: 0.3 });
  const mat = { part: matOrange, motor: matBlack };

  const modelRoot = new THREE.Group();
  modelRoot.rotation.x = -Math.PI / 2;   // z-up → y-up
  scene.add(modelRoot);

  // 加载全部 STL（去重缓存）
  const meshCache = {};
  const loader = new STLLoader();
  const allMeshes = [...BASE_MESHES, ...CHAIN.flatMap(c => c.meshes)].map(m => m[0]);
  for (const name of new Set(allMeshes)) {
    try {
      meshCache[name] = await loader.loadAsync(MODEL_DIR + name + ".stl");
    } catch (e) {
      showMsg(`3D 模型加载失败：${name}.stl（${e.message || e}）`);
      return;
    }
  }

  function addBodyMeshes(group, list) {
    for (const [name, isMotor] of list) {
      const m = new THREE.Mesh(meshCache[name], isMotor ? mat.motor : mat.part);
      group.add(m);
    }
  }

  // 基座
  const baseGroup = new THREE.Group();
  addBodyMeshes(baseGroup, BASE_MESHES);
  modelRoot.add(baseGroup);

  // 关节链：parentBody → jointGroup(T·R) → rotor(Rot(axis,θ)) → childBody(网格)
  const rotors = [], axes = [], dots = [], labels = [];
  let parentGroup = baseGroup;
  for (const c of CHAIN) {
    const jointGroup = new THREE.Group();
    jointGroup.position.fromArray(c.pos);
    jointGroup.quaternion.fromArray(c.quat);
    parentGroup.add(jointGroup);

    const rotor = new THREE.Group();
    jointGroup.add(rotor);
    const bodyGroup = new THREE.Group();
    rotor.add(bodyGroup);
    addBodyMeshes(bodyGroup, c.meshes);

    const dot = new THREE.Mesh(
      new THREE.SphereGeometry(0.0075, 16, 12),
      new THREE.MeshBasicMaterial({ color: 0x5a5f6e })
    );
    jointGroup.add(dot);

    const label = makeLabel();
    label.position.set(0.016, 0.014, 0);
    jointGroup.add(label);

    rotors.push(rotor);
    axes.push(new THREE.Vector3().fromArray(c.axis));
    dots.push(dot);
    labels.push(label);
    parentGroup = bodyGroup;
  }

  // 相机对准零位姿态的包围盒
  const box = new THREE.Box3().setFromObject(modelRoot);
  const center = box.getCenter(new THREE.Vector3());
  camera.position.copy(center).add(new THREE.Vector3(0.30, 0.17, 0.40));
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.copy(center);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.minDistance = 0.12;
  controls.maxDistance = 1.5;
  controls.maxPolarAngle = Math.PI * 0.55;

  function resize() {
    const w = wrap.clientWidth, h = wrap.clientHeight;
    if (!w || !h) return;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  resize();
  new ResizeObserver(resize).observe(wrap);

  // 角度状态：遥测帧更新 target + 状态色，rAF 帧内指数平滑逼近
  const targets = CHAIN.map(() => 0);
  const cur = CHAIN.map(() => 0);
  const offsetsRad = CHAIN.map(c => (c.offset_deg || 0) * DEG2RAD);  // 显示偏移（仅 J5 = -90°）

  function onTelemetry(t) {
    for (const j of t.joints) {
      const k = j.id - 1;
      if (k < 0 || k > 5) continue;
      // pos_deg 是当前显示约定（标零后随零位平移）；+viz_offset 还原 MuJoCo 零位系，
      // 保持 3D 模型与仿真的对齐
      if (j.pos_deg != null) targets[k] = (j.pos_deg + (j.viz_offset_deg || 0)) * DEG2RAD;
      dots[k].material.color.set(jointColor(j));
      drawLabel(labels[k], `J${j.id} ${j.pos_deg != null ? j.pos_deg.toFixed(1) : "—"}°`, jointColor(j));
    }
  }
  if (api._pending) { onTelemetry(api._pending); api._pending = null; }

  let last = performance.now();
  function loop(now) {
    requestAnimationFrame(loop);
    const dt = Math.min(0.1, (now - last) / 1000);
    last = now;
    if (api._pending) { onTelemetry(api._pending); api._pending = null; }
    const k = 1 - Math.exp(-dt * 10);
    for (let i = 0; i < 6; i++) {
      cur[i] += (targets[i] - cur[i]) * k;
      rotors[i].quaternion.setFromAxisAngle(axes[i], cur[i] + offsetsRad[i]);
    }
    controls.update();
    renderer.render(scene, camera);
  }
  requestAnimationFrame(loop);
  hideMsg();
  api.ready = true;
}

init().catch(e => showMsg("3D 初始化失败：" + (e.message || e)));
