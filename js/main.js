import * as THREE from 'three';
import { CSS2DRenderer, CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';
import { forwardKinematics, dhMatrix, deg2rad } from './dh.js';

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

const COLORS = { x: 0xd32f2f, y: 0x2e7d32, z: 0x1565c0 };

// Colors of the on-canvas DH dimension annotations.
const DIM_COLORS = { theta: '#d97706', d: '#7c3aed', a: '#0891b2', alpha: '#db2777' };

// Angles are in degrees in the UI/state; converted to radians for the math.
// `scale` is the drawing scale the preset is designed for.
const PRESETS = {
  articulated: {
    label: 'Articulated RRR',
    scale: 10,
    joints: [
      { type: 'R', theta: 50, d: 18, a: 0, alpha: 90 },
      { type: 'R', theta: 60, d: 0, a: 20, alpha: 0 },
      { type: 'R', theta: -85, d: 0, a: 14, alpha: 0 },
    ],
  },
  planar: {
    label: 'Planar RRR',
    scale: 10,
    joints: [
      { type: 'R', theta: 30, d: 0, a: 20, alpha: 0 },
      { type: 'R', theta: 30, d: 0, a: 15, alpha: 0 },
      { type: 'R', theta: -40, d: 0, a: 10, alpha: 0 },
    ],
  },
  scara: {
    label: 'SCARA RRP',
    scale: 10,
    joints: [
      { type: 'R', theta: 0, d: 15, a: 20, alpha: 0 },
      { type: 'R', theta: 45, d: 0, a: 15, alpha: 180 },
      { type: 'P', theta: 0, d: 8, a: 0, alpha: 0 },
    ],
  },
  cylindrical: {
    label: 'Cylindrical RPP',
    scale: 10,
    joints: [
      { type: 'R', theta: 30, d: 10, a: 0, alpha: 0 },
      { type: 'P', theta: 0, d: 12, a: 0, alpha: -90 },
      { type: 'P', theta: 0, d: 15, a: 0, alpha: 0 },
    ],
  },
};
const DEFAULT_PRESET = 'articulated';
const MAX_JOINTS = 8;

// Length ranges are for drawing scale 1 and grow with the scale. Lengths typed into
// the number box are not capped at the slider max (the slider just saturates).
const PARAMS = [
  { key: 'theta', sym: 'θ', unit: '°', min: -180, max: 180, step: 1 },
  { key: 'd', sym: 'd', unit: '', min: -5, max: 5, step: 0.05, length: true },
  { key: 'a', sym: 'a', unit: '', min: 0, max: 5, step: 0.05, length: true },
  { key: 'alpha', sym: 'α', unit: '°', min: -180, max: 180, step: 1 },
];

// Prismatic joints extend only forward along z, so d is limited to >= 0.
function paramRange(p, type) {
  const k = p.length ? state.scale : 1;
  const min = p.key === 'd' && type === 'P' ? 0 : p.min * k;
  return { min, max: p.max * k, step: p.step * k };
}

const isVariable = (key, type) => (type === 'R' ? key === 'theta' : key === 'd');

// Smallest "nice" number (1, 2, 2.5, 5 × 10^k) that is >= v.
function niceCeil(v) {
  if (!(v > 0)) return 1;
  const p = 10 ** Math.floor(Math.log10(v));
  return [1, 2, 2.5, 5, 10].map((m) => m * p).find((n) => n >= v - 1e-12);
}

const round6 = (v) => +v.toFixed(6); // strip float noise from slider values
const clone = (o) => JSON.parse(JSON.stringify(o));
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const fmt = (v, n = 3) => {
  const s = v.toFixed(n);
  return Number(s) === 0 ? (0).toFixed(n) : s; // avoid "-0.000"
};
const short = (v) => String(round6(+v.toFixed(2))); // 18, 2.5, -85 — no trailing zeros

const state = {
  joints: clone(PRESETS[DEFAULT_PRESET].joints),
  grip: 0.3,
  scale: PRESETS[DEFAULT_PRESET].scale, // drawing scale: 1 drawn unit = `scale` real length units
  showAxes: true,
  showDims: true,
  showEE: true, // end-effector coordinate tag on the canvas
  showLabels: true,
  showGrid: true,
};

// ---------------------------------------------------------------------------
// Scene setup
// ---------------------------------------------------------------------------

const viewport = document.getElementById('viewport');

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
viewport.prepend(renderer.domElement);

const labelRenderer = new CSS2DRenderer();
labelRenderer.domElement.className = 'label-layer';
viewport.insertBefore(labelRenderer.domElement, renderer.domElement.nextSibling);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0xf4f6f9);

// Two cameras share one navigation controller; `camera` is whichever is active.
// Orthographic uses a symmetric near/far so nothing is clipped at any distance.
const perspCam = new THREE.PerspectiveCamera(45, 1, 0.05, 500);
const orthoCam = new THREE.OrthographicCamera(-4, 4, 4, -4, -500, 500);
let aspect = 1;
for (const c of [perspCam, orthoCam]) c.up.set(0, 0, 1); // robotics convention: Z is up

// Isometric direction: equal angles to the X, Y and Z axes (front-right-top).
const ISO_DIR = new THREE.Vector3(1, -1, 1).normalize();
let camera = orthoCam; // default: orthographic, isometric
camera.position.copy(ISO_DIR).multiplyScalar(10);

// Orbit/pan center. The camera always looks at it; its orientation lives in
// camera.quaternion so axis-locked rotations may roll the view (like Blender).
const target = new THREE.Vector3();
camera.lookAt(target);

// Unit vector from the target toward the camera (the camera's local +Z).
const backDir = () => new THREE.Vector3(0, 0, 1).applyQuaternion(camera.quaternion);

function setOrthoHalfHeight(halfH) {
  orthoCam.top = halfH;
  orthoCam.bottom = -halfH;
  orthoCam.left = -halfH * aspect;
  orthoCam.right = halfH * aspect;
  orthoCam.updateProjectionMatrix();
}

scene.add(new THREE.HemisphereLight(0xffffff, 0x8a93a3, 1.4));
const sun = new THREE.DirectionalLight(0xffffff, 1.5);
sun.position.set(5, -4, 9);
scene.add(sun);

const grid = new THREE.GridHelper(12, 24, 0xa9b2bf, 0xd9dee5);
grid.rotation.x = Math.PI / 2; // GridHelper lies in XZ; rotate into XY
grid.position.z = -0.3;
scene.add(grid);

const MAT = {
  link: new THREE.MeshStandardMaterial({ color: 0x8b95a5, metalness: 0.35, roughness: 0.4 }),
  revolute: new THREE.MeshStandardMaterial({ color: 0xf59e0b, metalness: 0.2, roughness: 0.45 }),
  sleeve: new THREE.MeshStandardMaterial({
    color: 0x7c3aed, metalness: 0.1, roughness: 0.5, transparent: true, opacity: 0.3, depthWrite: false,
  }),
  rod: new THREE.MeshStandardMaterial({ color: 0xa78bfa, metalness: 0.3, roughness: 0.35 }),
  base: new THREE.MeshStandardMaterial({ color: 0x6b7482, metalness: 0.3, roughness: 0.6 }),
  gripper: new THREE.MeshStandardMaterial({ color: 0x0d9488, metalness: 0.25, roughness: 0.4 }),
  origin: new THREE.MeshBasicMaterial({ color: 0x1d2430 }),
};
const AXIS_MAT = Object.fromEntries(
  Object.entries(COLORS).map(([k, c]) => [k, new THREE.MeshStandardMaterial({ color: c, roughness: 0.4 })]),
);

// Base pedestal under frame {0} (static).
{
  const base = new THREE.Mesh(new THREE.CylinderGeometry(0.55, 0.65, 0.08, 40), MAT.base);
  base.rotation.x = Math.PI / 2;
  base.position.z = -0.26;
  scene.add(base);
}

const toMatrix4 = (T) => new THREE.Matrix4().set(...T.flat());

function makeLabel(content, className, html = false) {
  const el = document.createElement('div');
  el.className = className;
  if (html) el.innerHTML = content;
  else el.textContent = content;
  return new CSS2DObject(el);
}

// Dispose geometries and drop CSS2D label elements (the renderer does not
// remove the DOM element of an object that left the scene).
function clearGroup(group) {
  group.traverse((o) => {
    if (o.geometry) o.geometry.dispose();
    if (o.isCSS2DObject) o.element.remove();
  });
  group.clear();
}

// ---------------------------------------------------------------------------
// Coordinate frames {0} … {n}
// ---------------------------------------------------------------------------

// Arrow built along +Y, then rotated onto the requested axis.
function makeArrow(mat, length, radius) {
  const headLen = radius * 4.5;
  const g = new THREE.Group();
  const shaft = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, length - headLen, 12), mat);
  shaft.position.y = (length - headLen) / 2;
  const head = new THREE.Mesh(new THREE.ConeGeometry(radius * 2.4, headLen, 16), mat);
  head.position.y = length - headLen / 2;
  g.add(shaft, head);
  return g;
}

function makeFrame(index, length) {
  const group = new THREE.Group();
  group.matrixAutoUpdate = false;

  const axes = new THREE.Group();
  const x = makeArrow(AXIS_MAT.x, length, 0.022);
  x.rotation.z = -Math.PI / 2; // +Y -> +X
  const y = makeArrow(AXIS_MAT.y, length, 0.022);
  const z = makeArrow(AXIS_MAT.z, length, 0.022);
  z.rotation.x = Math.PI / 2; // +Y -> +Z
  axes.add(x, y, z, new THREE.Mesh(new THREE.SphereGeometry(0.04, 16, 12), MAT.origin));

  const tip = length + 0.12;
  const lx = makeLabel(`x${index}`, 'axis-label x'); lx.position.set(tip, 0, 0);
  const ly = makeLabel(`y${index}`, 'axis-label y'); ly.position.set(0, tip, 0);
  const lz = makeLabel(`z${index}`, 'axis-label z'); lz.position.set(0, 0, tip);
  axes.add(lx, ly, lz);

  const name = makeLabel(`{${index}}`, 'frame-label');
  name.position.set(-0.18, -0.18, -0.12);

  group.add(axes, name);
  scene.add(group);
  return { group, axes, axisLabels: [lx, ly, lz] };
}

let frames = [];

function rebuildFrames() {
  frames.forEach((f) => { clearGroup(f.group); scene.remove(f.group); });
  const n = state.joints.length;
  frames = Array.from({ length: n + 1 }, (_, i) => makeFrame(i, i === n ? 0.9 : 0.7));
  applyAxesVisibility();
}

// CSS2DRenderer ignores parent visibility, so hide the axis letters explicitly.
function applyAxesVisibility() {
  frames.forEach((f) => {
    f.axes.visible = state.showAxes;
    f.axisLabels.forEach((l) => { l.visible = state.showAxes; });
  });
}

const eeLabel = makeLabel('', 'ee-label');
eeLabel.center.set(0, 1); // anchor bottom-left of the tag at the end-effector origin
scene.add(eeLabel);

// ---------------------------------------------------------------------------
// Robot geometry (rebuilt on every change — only a handful of meshes)
// ---------------------------------------------------------------------------

const robot = new THREE.Group();
scene.add(robot);

// Group whose local coordinates are those of DH frame T.
function frameGroup(T) {
  const g = new THREE.Group();
  g.matrixAutoUpdate = false;
  g.matrix.copy(toMatrix4(T));
  g.matrixWorldNeedsUpdate = true;
  robot.add(g);
  return g;
}

// Cylinder along a local axis ('x' or 'z') between coordinates `from` and `to`.
function cylinderAlong(axis, from, to, radius, mat) {
  const len = Math.abs(to - from);
  if (len < 1e-4) return null;
  const m = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, len, 24), mat);
  const mid = (from + to) / 2;
  if (axis === 'z') { m.rotation.x = Math.PI / 2; m.position.z = mid; }
  else { m.rotation.z = Math.PI / 2; m.position.x = mid; }
  return m;
}

function box(sx, sy, sz, mat, x = 0, y = 0, z = 0) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), mat);
  m.position.set(x, y, z);
  return m;
}

// Ts are the *drawing* transforms (lengths already divided by the scale).
function buildJoint(i, Ts) {
  const j = state.joints[i];
  const d = j.d / state.scale;
  const a = j.a / state.scale;
  const prev = frameGroup(Ts[i]); // joint i+1 acts about z_i (frame {i})
  const cur = frameGroup(Ts[i + 1]);

  if (j.type === 'R') {
    // Revolute: rotating drum on z_{i}, d-offset is a fixed link along z_{i}.
    prev.add(cylinderAlong('z', -0.22, 0.22, 0.17, MAT.revolute));
    const dLink = cylinderAlong('z', 0, d, 0.07, MAT.link);
    if (dLink) prev.add(dLink);
  } else {
    // Prismatic: fixed translucent sleeve + rod sliding out of it by d.
    prev.add(box(0.36, 0.36, 0.7, MAT.sleeve, 0, 0, 0.13));
    const rodStart = -0.22;
    const rodLen = d - rodStart;
    if (rodLen > 1e-4) prev.add(box(0.18, 0.18, rodLen, MAT.rod, 0, 0, rodStart + rodLen / 2));
  }

  // a-offset: link along x_{i+1}, ending at the origin of frame {i+1}.
  const aLink = cylinderAlong('x', -a, 0, 0.07, MAT.link);
  if (aLink) cur.add(aLink);

  const sphere = (r, x) => {
    const m = new THREE.Mesh(new THREE.SphereGeometry(r, 20, 14), MAT.link);
    m.position.x = x;
    return m;
  };
  if (a > 1e-4 && Math.abs(d) > 1e-4) cur.add(sphere(0.085, -a)); // elbow between d and a
  cur.add(sphere(0.09, 0));
}

function buildGripper(T) {
  const g = frameGroup(T);
  const w = state.grip;
  const fingerLen = 0.4;
  g.add(box(0.14, w + 0.26, 0.18, MAT.gripper, 0.07, 0, 0)); // palm
  const fx = 0.14 + fingerLen / 2;
  g.add(box(fingerLen, 0.06, 0.12, MAT.gripper, fx, w / 2 + 0.03, 0));
  g.add(box(fingerLen, 0.06, 0.12, MAT.gripper, fx, -(w / 2 + 0.03), 0));
}

// ---------------------------------------------------------------------------
// DH dimension annotations (drawing-style: dimension lines, angle arcs, labels)
// ---------------------------------------------------------------------------

const dims = new THREE.Group();
scene.add(dims);

const DIM_OFFSET = 0.38; // distance of a linear dimension line from the feature
const lineMats = new Map();
function lineMat(color, dashed) {
  const key = color + dashed;
  if (!lineMats.has(key)) {
    lineMats.set(key, dashed
      ? new THREE.LineDashedMaterial({ color, dashSize: 0.06, gapSize: 0.045 })
      : new THREE.LineBasicMaterial({ color }));
  }
  return lineMats.get(key);
}
const headMats = new Map();
function headMat(color) {
  if (!headMats.has(color)) headMats.set(color, new THREE.MeshBasicMaterial({ color }));
  return headMats.get(color);
}

function addLine(pts, color, dashed = false) {
  const l = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), lineMat(color, dashed));
  if (dashed) l.computeLineDistances();
  dims.add(l);
}

// Small cone whose tip sits at `tip`, pointing along unit vector `dir`.
function addArrowHead(tip, dir, color) {
  const len = 0.11;
  const m = new THREE.Mesh(new THREE.ConeGeometry(0.035, len, 12), headMat(color));
  m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
  m.position.copy(tip).addScaledVector(dir, -len / 2);
  dims.add(m);
}

function addDimLabel(pos, html, color, variable) {
  const l = makeLabel(html, `dim-label${variable ? ' var' : ''}`, true);
  l.element.style.color = color;
  l.position.copy(pos);
  dims.add(l);
}

// Linear dimension between p0 and p1, drawn parallel to the feature and shifted by
// `offDir` (unit), with dashed extension lines and arrowheads at both ends.
function dimLinear(p0, p1, offDir, color, html, variable) {
  const len = p0.distanceTo(p1);
  if (len < 1e-4) return;
  const q0 = p0.clone().addScaledVector(offDir, DIM_OFFSET);
  const q1 = p1.clone().addScaledVector(offDir, DIM_OFFSET);
  addLine([p0, q0.clone().addScaledVector(offDir, 0.08)], color, true);
  addLine([p1, q1.clone().addScaledVector(offDir, 0.08)], color, true);
  addLine([q0, q1], color);
  if (len > 0.25) {
    const dir = q1.clone().sub(q0).normalize();
    addArrowHead(q1, dir, color);
    addArrowHead(q0, dir.clone().negate(), color);
  }
  addDimLabel(q0.clone().lerp(q1, 0.5).addScaledVector(offDir, 0.14), html, color, variable);
}

// Angle dimension: arc around unit `axis` through `center`, starting at unit
// direction `from` and sweeping `angle` (radians, right-hand rule). Dashed rays
// mark the two sides of the angle.
function dimAngle(center, axis, from, angle, r, color, html, variable) {
  if (Math.abs(angle) < deg2rad(0.5)) return;
  const n = Math.max(2, Math.ceil(Math.abs(angle) / deg2rad(4)));
  const pts = [];
  for (let k = 0; k <= n; k++) {
    pts.push(from.clone().applyAxisAngle(axis, (angle * k) / n).multiplyScalar(r).add(center));
  }
  addLine(pts, color);
  const end = from.clone().applyAxisAngle(axis, angle);
  addLine([center, center.clone().addScaledVector(from, r * 1.3)], color, true);
  addLine([center, center.clone().addScaledVector(end, r * 1.3)], color, true);
  const tangent = axis.clone().cross(end).multiplyScalar(Math.sign(angle)).normalize();
  addArrowHead(pts[n], tangent, color);
  const mid = from.clone().applyAxisAngle(axis, angle / 2);
  addDimLabel(center.clone().addScaledVector(mid, r + 0.2), html, color, variable);
}

const col = (T, c) => new THREE.Vector3(T[0][c], T[1][c], T[2][c]);

function buildDims(Ts) {
  clearGroup(dims);
  if (!state.showDims) return;
  state.joints.forEach((j, i) => {
    const k = i + 1;
    const O0 = col(Ts[i], 3), x0 = col(Ts[i], 0), z0 = col(Ts[i], 2); // frame {i}
    const O1 = col(Ts[k], 3), x1 = col(Ts[k], 0);                     // frame {i+1}
    const P = O0.clone().addScaledVector(z0, j.d / state.scale);       // end of the d-offset
    const tag = (sym, v, unit = '') => `${sym}<sub>${k}</sub> = ${short(v)}${unit}`;

    // θ: about z_{i}, from x_{i} to x_{i+1}, drawn where x_{i+1} starts.
    dimAngle(P, z0, x0, deg2rad(j.theta), 0.6, DIM_COLORS.theta, tag('θ', j.theta, '°'), isVariable('theta', j.type));
    // d: along z_{i}, from O_{i} to P.
    dimLinear(O0, P, x1.clone().negate(), DIM_COLORS.d, tag('d', j.d), isVariable('d', j.type));
    // a: along x_{i+1}, from P to O_{i+1}.
    dimLinear(P, O1, z0, DIM_COLORS.a, tag('a', j.a), false);
    // α: about x_{i+1}, from z_{i} to z_{i+1}, drawn at O_{i+1}.
    dimAngle(O1, x1, z0, deg2rad(j.alpha), 0.45, DIM_COLORS.alpha, tag('α', j.alpha, '°'), false);
  });
}

// ---------------------------------------------------------------------------
// Update cycle
// ---------------------------------------------------------------------------

// Real kinematics use the true lengths; the drawing uses lengths divided by the
// drawing scale so that large values (e.g. d = 50) still fit on screen.
function kinematics(lengthScale) {
  return forwardKinematics(state.joints.map((j) => ({
    theta: deg2rad(j.theta),
    d: j.d / lengthScale,
    a: j.a / lengthScale,
    alpha: deg2rad(j.alpha),
  })));
}

function update() {
  const n = state.joints.length;
  const real = kinematics(1);
  const drawn = kinematics(state.scale);

  frames.forEach((f, i) => {
    f.group.matrix.copy(toMatrix4(drawn[i]));
    f.group.matrixWorldNeedsUpdate = true;
  });

  clearGroup(robot);
  for (let i = 0; i < n; i++) buildJoint(i, drawn);
  buildGripper(drawn[n]);
  buildDims(drawn);

  const ee = [0, 1, 2].map((r) => real[n][r][3]);
  eeLabel.visible = state.showEE;
  eeLabel.position.copy(col(drawn[n], 3));
  eeLabel.element.textContent = `EE (${ee.map((v) => fmt(v, 2)).join(', ')})`;

  updateReadouts(real);
}

// Frame the whole robot, keeping the current viewing direction.
function fitView() {
  const box3 = new THREE.Box3().setFromObject(robot);
  box3.expandByPoint(new THREE.Vector3(0, 0, 0));
  const sphere = box3.getBoundingSphere(new THREE.Sphere());
  const r = Math.max(sphere.radius, 1.5) + 0.8; // margin for axes and labels
  const dir = backDir();
  let dist;
  if (camera.isOrthographicCamera) {
    setOrthoHalfHeight(aspect >= 1 ? r : r / aspect);
    orthoCam.zoom = 1;
    orthoCam.updateProjectionMatrix();
    dist = r * 3;
  } else {
    const halfV = deg2rad(perspCam.fov) / 2;
    const halfH = Math.atan(Math.tan(halfV) * aspect);
    dist = r / Math.sin(Math.min(halfV, halfH));
  }
  viewAnim = null;
  target.copy(sphere.center);
  camera.position.copy(sphere.center).addScaledVector(dir, dist);
}

// ---------------------------------------------------------------------------
// Projection toggle and view cube
// ---------------------------------------------------------------------------

// Switch projection while keeping the same view direction and on-screen size.
function setProjection(kind) {
  const next = kind === 'ortho' ? orthoCam : perspCam;
  if (next !== camera) {
    const offset = camera.position.clone().sub(target);
    const dir = backDir();
    const tanHalf = Math.tan(deg2rad(perspCam.fov) / 2);
    if (next === orthoCam) {
      // visible half-height at the target distance becomes the ortho half-height
      setOrthoHalfHeight(offset.length() * tanHalf);
      orthoCam.zoom = 1;
      orthoCam.updateProjectionMatrix();
      orthoCam.position.copy(target).addScaledVector(dir, offset.length());
    } else {
      const halfH = orthoCam.top / orthoCam.zoom;
      perspCam.position.copy(target).addScaledVector(dir, halfH / tanHalf);
    }
    next.quaternion.copy(camera.quaternion);
    camera = next;
  }
  document.querySelectorAll('#projection button').forEach((b) => {
    b.classList.toggle('active', b.dataset.proj === kind);
  });
}

// Faces in world coordinates (Z up). n = outward normal (also the direction the
// camera sits in for that view); u, v = the face's right and up when viewed head-on.
const V = (x, y, z) => new THREE.Vector3(x, y, z);
const CUBE_FACES = [
  { name: 'Top', axis: '+Z', n: V(0, 0, 1), u: V(1, 0, 0), v: V(0, 1, 0), view: V(0, -1e-3, 1) },
  { name: 'Bottom', axis: '−Z', n: V(0, 0, -1), u: V(1, 0, 0), v: V(0, -1, 0), view: V(0, 1e-3, -1) },
  { name: 'Front', axis: '−Y', n: V(0, -1, 0), u: V(1, 0, 0), v: V(0, 0, 1) },
  { name: 'Back', axis: '+Y', n: V(0, 1, 0), u: V(-1, 0, 0), v: V(0, 0, 1) },
  { name: 'Right', axis: '+X', n: V(1, 0, 0), u: V(0, 1, 0), v: V(0, 0, 1) },
  { name: 'Left', axis: '−X', n: V(-1, 0, 0), u: V(0, -1, 0), v: V(0, 0, 1) },
];
const CUBE_HALF = 30; // px

let viewAnim = null; // { q0, q1, dist, t0 }

// Level (Z-up, no roll) camera orientation looking at the target from `dir`.
function lookQuat(dir) {
  const m = new THREE.Matrix4().lookAt(dir, new THREE.Vector3(), new THREE.Vector3(0, 0, 1));
  return new THREE.Quaternion().setFromRotationMatrix(m);
}

// Smoothly orbit the camera to look at the target from direction `dir`.
function animateViewTo(dir) {
  viewAnim = {
    q0: camera.quaternion.clone(),
    q1: lookQuat(dir.clone().normalize()),
    dist: camera.position.distanceTo(target),
    t0: performance.now(),
  };
}

function stepViewAnim(now) {
  if (!viewAnim) return;
  const t = Math.min(1, (now - viewAnim.t0) / 350);
  const e = t < 0.5 ? 2 * t * t : 1 - (-2 * t + 2) ** 2 / 2; // ease in-out
  camera.quaternion.copy(viewAnim.q0).slerp(viewAnim.q1, e);
  camera.position.copy(target).addScaledVector(backDir(), viewAnim.dist);
  if (t >= 1) viewAnim = null;
}

// Each face is split into a 3×3 grid of click zones, as in Blender: the center
// is the face view, the strips along the sides are edge views (between two
// faces) and the small squares are corner views (between three faces).
// Zone (i, j) looks from direction n + i·u + j·v.
const AXIS_NAMES = [['Left', 'Right'], ['Front', 'Back'], ['Bottom', 'Top']]; // −/+ X, Y, Z
function viewName(dir) {
  const parts = [2, 1, 0] // Top/Bottom first, then Front/Back, then Left/Right
    .filter((k) => Math.abs(dir.getComponent(k)) > 0.5)
    .map((k) => AXIS_NAMES[k][dir.getComponent(k) > 0 ? 1 : 0]);
  return parts.join('-');
}

const cubeEl = document.getElementById('viewcube');
const zonesByKey = new Map(); // same edge/corner appears on 2–3 faces; highlight together
const faceEls = CUBE_FACES.map((f) => {
  const elFace = document.createElement('div');
  elFace.className = 'face';
  elFace.dataset.axis = f.axis.slice(1).toLowerCase();
  for (const j of [1, 0, -1]) {   // rows top → bottom (v up)
    for (const i of [-1, 0, 1]) { // columns left → right (u right)
      const dir = f.n.clone().addScaledVector(f.u, i).addScaledVector(f.v, j);
      const key = dir.toArray().join(',');
      const zone = document.createElement('button');
      zone.className = `zone ${i && j ? 'corner' : i || j ? 'edge' : 'center'}`;
      zone.title = `${viewName(dir)} view`;
      zone.setAttribute('aria-label', zone.title);
      const target = i || j ? dir : (f.view || f.n);
      zone.addEventListener('click', () => animateViewTo(target));
      zone.addEventListener('pointerenter', () => zonesByKey.get(key).forEach((z) => z.classList.add('hot')));
      zone.addEventListener('pointerleave', () => zonesByKey.get(key).forEach((z) => z.classList.remove('hot')));
      if (!zonesByKey.has(key)) zonesByKey.set(key, []);
      zonesByKey.get(key).push(zone);
      elFace.append(zone);
    }
  }
  const label = document.createElement('div');
  label.className = 'face-label';
  label.innerHTML = `<span>${f.name}</span><small>${f.axis}</small>`;
  elFace.append(label);
  cubeEl.append(elFace);
  return elFace;
});

// Orient the cube to match the camera: world directions -> view space -> CSS
// space (CSS y points down). Each face gets its own matrix3d.
function updateViewCube() {
  camera.updateMatrixWorld(); // refresh matrixWorldInverse for this frame's pose
  const toCss = (w) => {
    const c = w.clone().transformDirection(camera.matrixWorldInverse);
    return [c.x, -c.y, c.z];
  };
  CUBE_FACES.forEach((f, i) => {
    const n = toCss(f.n), u = toCss(f.u), v = toCss(f.v);
    const elFace = faceEls[i];
    const facing = n[2];
    elFace.style.visibility = facing > 0.02 ? 'visible' : 'hidden';
    elFace.style.transform = `matrix3d(${[
      u[0], u[1], u[2], 0,
      -v[0], -v[1], -v[2], 0,
      n[0], n[1], n[2], 0,
      n[0] * CUBE_HALF, n[1] * CUBE_HALF, n[2] * CUBE_HALF, 1,
    ].map((x) => x.toFixed(5)).join(',')})`;
    elFace.style.filter = `brightness(${(0.8 + 0.2 * facing).toFixed(3)})`;
    elFace.style.zIndex = Math.round(facing * 100);
  });
}

document.getElementById('view-iso').addEventListener('click', () => animateViewTo(ISO_DIR));
document.querySelectorAll('#projection button').forEach((b) => {
  b.addEventListener('click', () => setProjection(b.dataset.proj));
});

// Blender-style numpad shortcuts: 1 front, 3 right, 7 top (Ctrl = opposite),
// 5 toggles projection, 0 isometric.
window.addEventListener('keydown', (e) => {
  if (e.target.closest?.('input, textarea, select')) return;
  const face = (name) => CUBE_FACES.find((f) => f.name === name);
  const views = { 1: e.ctrlKey ? 'Back' : 'Front', 3: e.ctrlKey ? 'Left' : 'Right', 7: e.ctrlKey ? 'Bottom' : 'Top' };
  if (e.code === 'Numpad5') setProjection(camera === orthoCam ? 'persp' : 'ortho');
  else if (e.code === 'Numpad0') animateViewTo(ISO_DIR);
  else if (e.code.startsWith('Numpad') && views[e.code.slice(6)]) {
    const f = face(views[e.code.slice(6)]);
    animateViewTo(f.view || f.n);
  } else return;
  e.preventDefault();
});

// ---------------------------------------------------------------------------
// Mouse navigation: turntable rotate, pan, zoom — each optionally locked to a
// world axis (toolbar, or hold X / Y / Z while dragging)
// ---------------------------------------------------------------------------

const WORLD_AXES = { x: V(1, 0, 0), y: V(0, 1, 0), z: V(0, 0, 1) };
const nav = { rotate: 'free', pan: 'free' }; // toolbar locks: 'free' | 'x' | 'y' | 'z'
const heldAxes = new Set();                  // axis keys currently held down
let drag = null;                             // { mode: 'rotate' | 'pan', x, y }

const canvas = renderer.domElement;
const rotSpeed = () => (2 * Math.PI) / (canvas.clientHeight || 1); // full turn per viewport height

// Canvas pixel position (y down) of a world point.
function toPx(p) {
  camera.updateMatrixWorld(); // pose may have changed since the last render
  const v = p.clone().project(camera);
  return new THREE.Vector2(((v.x + 1) / 2) * canvas.clientWidth, ((1 - v.y) / 2) * canvas.clientHeight);
}
// On-screen pixels per world unit of direction `w`, measured at the target.
function screenVec(w) {
  const eps = 0.01;
  return toPx(target.clone().addScaledVector(w, eps)).sub(toPx(target)).divideScalar(eps);
}

// Orbit the camera about a world axis through the target (scene appears to turn by -angle).
function orbit(axis, angle) {
  const q = new THREE.Quaternion().setFromAxisAngle(axis, angle);
  camera.position.sub(target).applyQuaternion(q).add(target);
  camera.quaternion.premultiply(q);
}

function translateView(delta) {
  camera.position.add(delta);
  target.add(delta);
}

function worldPerPixel() {
  const h = canvas.clientHeight || 1;
  if (camera.isOrthographicCamera) return (orthoCam.top - orthoCam.bottom) / orthoCam.zoom / h;
  return (2 * camera.position.distanceTo(target) * Math.tan(deg2rad(perspCam.fov) / 2)) / h;
}

// A held axis key overrides the toolbar setting.
function activeLock(mode) {
  for (const k of ['x', 'y', 'z']) if (heldAxes.has(k)) return k;
  return nav[mode];
}

function rotateDrag(dx, dy, px, py) {
  const lock = activeLock('rotate');
  if (lock === 'free') {
    // Turntable: yaw about world Z, pitch about the view's right axis.
    orbit(WORLD_AXES.z, -dx * rotSpeed());
    orbit(new THREE.Vector3(1, 0, 0).applyQuaternion(camera.quaternion), -dy * rotSpeed());
    return;
  }
  const A = WORLD_AXES[lock];
  const toCam = backDir();
  const side = A.clone().cross(toCam); // on-screen direction the near side moves when turning about A
  if (side.length() > 0.2) {
    // Drag across the axis; the near side of the scene follows the mouse.
    const g = screenVec(side.normalize());
    if (g.lengthSq() < 1e-6) return;
    g.normalize();
    orbit(A, -(dx * g.x + dy * g.y) * rotSpeed());
  } else {
    // Axis points (almost) at the viewer: turning about it is a roll, so
    // follow the mouse's angle around the target on screen.
    const c = toPx(target);
    let d = Math.atan2(py - c.y, px - c.x) - Math.atan2(py - dy - c.y, px - dx - c.x);
    d = Math.atan2(Math.sin(d), Math.cos(d)); // wrap to (-π, π]
    orbit(A, d * Math.sign(A.dot(toCam)));
  }
}

function panDrag(dx, dy) {
  const lock = activeLock('pan');
  if (lock === 'free') {
    const k = worldPerPixel();
    const right = new THREE.Vector3(1, 0, 0).applyQuaternion(camera.quaternion);
    const up = new THREE.Vector3(0, 1, 0).applyQuaternion(camera.quaternion);
    translateView(right.multiplyScalar(-dx * k).addScaledVector(up, dy * k));
    return;
  }
  // Slide along the axis so the scene tracks the mouse's motion along the
  // axis' on-screen direction (the part of the drag across it is ignored).
  const A = WORLD_AXES[lock];
  const s = screenVec(A);
  const len2 = s.lengthSq();
  if (len2 < 4) return; // axis points at the viewer: no on-screen direction to slide along
  translateView(A.clone().multiplyScalar(-(dx * s.x + dy * s.y) / len2));
}

canvas.addEventListener('pointerdown', (e) => {
  const pan = e.button === 1 || e.button === 2 || e.shiftKey || e.ctrlKey || e.metaKey;
  if (e.button !== 0 && !pan) return;
  drag = { mode: pan ? 'pan' : 'rotate', x: e.clientX, y: e.clientY };
  viewAnim = null; // user input wins over a running view transition
  try { canvas.setPointerCapture(e.pointerId); } catch { /* pointer already gone */ }
});
canvas.addEventListener('pointermove', (e) => {
  if (!drag) return;
  const dx = e.clientX - drag.x;
  const dy = e.clientY - drag.y;
  drag.x = e.clientX;
  drag.y = e.clientY;
  const rect = canvas.getBoundingClientRect();
  if (drag.mode === 'rotate') rotateDrag(dx, dy, e.clientX - rect.left, e.clientY - rect.top);
  else panDrag(dx, dy);
});
const endDrag = () => { drag = null; };
canvas.addEventListener('pointerup', endDrag);
canvas.addEventListener('pointercancel', endDrag);
canvas.addEventListener('contextmenu', (e) => e.preventDefault());

canvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  const delta = e.deltaY * (e.deltaMode === 1 ? 33 : 1); // lines → pixels
  const s = Math.pow(1.0015, delta); // > 1 zooms out
  viewAnim = null;
  if (camera.isOrthographicCamera) {
    orthoCam.zoom = clamp(orthoCam.zoom / s, 0.02, 200);
    orthoCam.updateProjectionMatrix();
  } else {
    const dist = clamp(camera.position.distanceTo(target) * s, 0.2, 400);
    camera.position.copy(target).addScaledVector(backDir(), dist);
  }
}, { passive: false });

// Hold X / Y / Z to lock the current drag to that axis.
const typing = (e) => e.target.closest?.('input, textarea, select');
window.addEventListener('keydown', (e) => {
  const k = e.key.toLowerCase();
  if (typing(e) || e.ctrlKey || e.metaKey || e.altKey || !WORLD_AXES[k]) return;
  heldAxes.add(k);
  syncLockUI();
});
window.addEventListener('keyup', (e) => {
  if (heldAxes.delete(e.key.toLowerCase())) syncLockUI();
});
window.addEventListener('blur', () => { heldAxes.clear(); syncLockUI(); });

// Toolbar locks
document.querySelectorAll('.seg.lock').forEach((seg) => {
  seg.querySelectorAll('button').forEach((b) => {
    b.addEventListener('click', () => {
      nav[seg.dataset.mode] = b.dataset.axis;
      syncLockUI();
    });
  });
});
function syncLockUI() {
  document.querySelectorAll('.seg.lock').forEach((seg) => {
    const lock = activeLock(seg.dataset.mode);
    seg.querySelectorAll('button').forEach((b) => {
      b.classList.toggle('active', nav[seg.dataset.mode] === b.dataset.axis);
      b.classList.toggle('held', lock !== nav[seg.dataset.mode] && lock === b.dataset.axis);
    });
  });
}

// Guide lines through the target along each locked axis (always drawn on top).
const guides = Object.fromEntries(Object.entries(WORLD_AXES).map(([k, a]) => {
  const line = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([a.clone().multiplyScalar(-200), a.clone().multiplyScalar(200)]),
    new THREE.LineBasicMaterial({ color: COLORS[k], depthTest: false, transparent: true, opacity: 0.85 }),
  );
  line.renderOrder = 10;
  line.visible = false;
  scene.add(line);
  return [k, line];
}));
const pivot = new THREE.Mesh(
  new THREE.SphereGeometry(0.05, 16, 12),
  new THREE.MeshBasicMaterial({ color: 0x1d2430, depthTest: false }),
);
pivot.renderOrder = 11;
scene.add(pivot);

// While dragging show the drag's lock; otherwise show every lock that is set.
function updateGuides() {
  const shown = new Set();
  if (drag) {
    const l = activeLock(drag.mode);
    if (l !== 'free') shown.add(l);
  } else {
    heldAxes.forEach((k) => shown.add(k));
    for (const m of ['rotate', 'pan']) if (nav[m] !== 'free') shown.add(nav[m]);
  }
  for (const [k, line] of Object.entries(guides)) {
    line.visible = shown.has(k);
    line.position.copy(target);
  }
  pivot.visible = shown.size > 0 || !!drag;
  pivot.position.copy(target);
}

// ---------------------------------------------------------------------------
// UI
// ---------------------------------------------------------------------------

const ui = []; // ui[i] = { card, buttons, del, rows: { key: { row, range, num } } }

function el(tag, className, html) {
  const e = document.createElement(tag);
  if (className) e.className = className;
  if (html != null) e.innerHTML = html;
  return e;
}

function buildPresetButtons() {
  const host = document.getElementById('presets');
  for (const [key, p] of Object.entries(PRESETS)) {
    const b = el('button', '', p.label);
    b.addEventListener('click', () => applyPreset(key));
    host.append(b);
  }
  const reset = el('button', '', 'Reset');
  reset.addEventListener('click', () => applyPreset(DEFAULT_PRESET));
  host.append(reset);
}

function buildJointCards() {
  const host = document.getElementById('joints');
  host.innerHTML = '';
  ui.length = 0;
  state.joints.forEach((j, i) => {
    const card = el('section', 'card joint');
    const head = el('div', 'card-head', `<h2>Joint ${i + 1}</h2>`);
    const seg = el('div', 'seg');
    const buttons = {};
    for (const [t, name] of [['R', 'Revolute'], ['P', 'Prismatic']]) {
      const b = el('button', '', name);
      b.dataset.type = t;
      b.addEventListener('click', () => setJointType(i, t));
      seg.append(b);
      buttons[t] = b;
    }
    const del = el('button', 'icon-btn', '✕');
    del.title = `Delete joint ${i + 1}`;
    del.setAttribute('aria-label', del.title);
    del.addEventListener('click', () => deleteJoint(i));
    const tools = el('div', 'card-tools');
    tools.append(seg, del);
    head.append(tools);
    card.append(head);

    const rows = {};
    for (const p of PARAMS) {
      const row = el('div', 'row');
      const id = `j${i}-${p.key}`;
      const label = el('label', '', `${p.sym}<sub>${i + 1}</sub>`);
      label.htmlFor = id;
      const range = el('input');
      range.type = 'range';
      range.id = id;
      const num = el('input');
      num.type = 'number';
      num.step = 'any';
      row.append(label, range, num, el('span', 'unit', p.unit));

      range.addEventListener('input', () => {
        const v = round6(parseFloat(range.value));
        state.joints[i][p.key] = v;
        num.value = v;
        update();
      });
      const onNum = (commit) => {
        const v = parseFloat(num.value);
        if (!Number.isFinite(v)) return;
        const { min, max } = paramRange(p, state.joints[i].type);
        const c = p.length ? Math.max(v, min) : clamp(v, min, max);
        state.joints[i][p.key] = c;
        range.value = c;
        if (commit) num.value = c;
        update();
      };
      num.addEventListener('input', () => onNum(false));
      num.addEventListener('change', () => onNum(true));

      card.append(row);
      rows[p.key] = { row, range, num };
    }
    host.append(card);
    ui[i] = { card, buttons, del, rows };
  });
}

function syncJointUI(i) {
  const j = state.joints[i];
  const u = ui[i];
  u.card.dataset.type = j.type;
  u.buttons.R.classList.toggle('active', j.type === 'R');
  u.buttons.P.classList.toggle('active', j.type === 'P');
  u.del.disabled = state.joints.length <= 1;
  for (const p of PARAMS) {
    const { row, range, num } = u.rows[p.key];
    const { min, max, step } = paramRange(p, j.type);
    range.min = num.min = min;
    range.max = max;
    range.step = step;
    if (p.length) num.removeAttribute('max');
    else num.max = max;
    range.value = j[p.key];
    num.value = j[p.key];
    const v = isVariable(p.key, j.type);
    row.classList.toggle('var', v);
    row.title = v ? 'Joint variable' : 'Fixed link parameter';
  }
}

function syncAllUI() {
  const n = state.joints.length;
  state.joints.forEach((_, i) => syncJointUI(i));
  document.getElementById('config').textContent = state.joints.map((j) => j.type).join('');
  document.getElementById('ee-frame').textContent = `{${n}}`;
  addJointBtn.disabled = n >= MAX_JOINTS;
  addJointBtn.title = n >= MAX_JOINTS ? `At most ${MAX_JOINTS} joints` : 'Add a joint after the last one';
  gripRange.value = gripNum.value = state.grip;
  scaleNum.value = state.scale;
  document.getElementById('grid-info').textContent = `grid cell = ${short(0.5 * state.scale)}`;
}

// Joint count or preset changed: rebuild cards and frames, then reframe the view.
function structureChanged() {
  buildJointCards();
  rebuildFrames();
  syncAllUI();
  update();
  fitView();
}

function setJointType(i, type) {
  const j = state.joints[i];
  j.type = type;
  if (type === 'P' && j.d < 0) j.d = 0;
  syncAllUI();
  update();
}

function applyPreset(key) {
  state.joints = clone(PRESETS[key].joints);
  state.scale = PRESETS[key].scale;
  structureChanged();
}

const addJointBtn = document.getElementById('add-joint');
addJointBtn.addEventListener('click', () => {
  if (state.joints.length >= MAX_JOINTS) return;
  // New revolute joint with a one-drawn-unit link so it is visible right away.
  state.joints.push({ type: 'R', theta: 0, d: 0, a: state.scale, alpha: 0 });
  structureChanged();
});

function deleteJoint(i) {
  if (state.joints.length <= 1) return;
  state.joints.splice(i, 1);
  structureChanged();
}

const gripRange = document.getElementById('grip-range');
const gripNum = document.getElementById('grip-num');
gripRange.addEventListener('input', () => {
  state.grip = parseFloat(gripRange.value);
  gripNum.value = gripRange.value;
  update();
});
gripNum.addEventListener('change', () => {
  const v = parseFloat(gripNum.value);
  if (!Number.isFinite(v)) return;
  state.grip = clamp(v, 0.05, 0.6);
  gripRange.value = gripNum.value = state.grip;
  update();
});

const scaleNum = document.getElementById('scale-num');
function setScale(v) {
  if (!(v > 0)) return;
  state.scale = v;
  syncAllUI();
  update();
  fitView();
}
scaleNum.addEventListener('change', () => {
  const v = parseFloat(scaleNum.value);
  if (v > 0) setScale(v);
  else scaleNum.value = state.scale; // reject 0 / negative / empty
});
// Auto: pick a nice scale so the robot's total reach draws at about 4 units.
document.getElementById('scale-auto').addEventListener('click', () => {
  const reach = state.joints.reduce((s, j) => s + Math.abs(j.d) + j.a, 0);
  setScale(niceCeil(reach / 4));
});

// Collapsible sidebar (choice remembered per browser when storage is available).
const panelToggle = document.getElementById('toggle-panel');
function setPanelCollapsed(collapsed) {
  document.body.classList.toggle('panel-collapsed', collapsed);
  panelToggle.textContent = collapsed ? '▶' : '◀';
  panelToggle.title = collapsed ? 'Show sidebar' : 'Hide sidebar';
  panelToggle.setAttribute('aria-label', panelToggle.title);
  try { localStorage.setItem('dh-panel-collapsed', collapsed ? '1' : '0'); } catch { /* ignore */ }
}
panelToggle.addEventListener('click', () => {
  setPanelCollapsed(!document.body.classList.contains('panel-collapsed'));
});
try { setPanelCollapsed(localStorage.getItem('dh-panel-collapsed') === '1'); } catch { setPanelCollapsed(false); }

document.getElementById('fit-view').addEventListener('click', fitView);

// View toggles: `state` is the source of truth. Browsers (notably Firefox) may
// restore a checkbox's previous state on reload regardless of the HTML
// `checked` attribute, so each box is explicitly set from state at startup.
const VIEW_TOGGLES = {
  'show-ee': ['showEE', () => { eeLabel.visible = state.showEE; }],
  'show-dims': ['showDims', () => update()],
  'show-axes': ['showAxes', () => applyAxesVisibility()],
  'show-labels': ['showLabels', () => labelRenderer.domElement.classList.toggle('hide-labels', !state.showLabels)],
  'show-grid': ['showGrid', () => { grid.visible = state.showGrid; }],
};
for (const [id, [key, apply]] of Object.entries(VIEW_TOGGLES)) {
  const box = document.getElementById(id);
  box.checked = state[key];
  box.addEventListener('change', () => {
    state[key] = box.checked;
    apply();
  });
}

function xyzHTML(p) {
  return ['x', 'y', 'z'].map((a, k) => `<span><b style="color:var(--${a})">${a}</b>${fmt(p[k])}</span>`).join('');
}

function matrixHTML(T) {
  const cells = T.flatMap((row, r) => row.map((v, c) => `<div class="${c === 3 && r < 3 ? 'pos' : ''}">${fmt(v)}</div>`));
  return `<div class="matrix">${cells.join('')}</div>`;
}

// T with both indices on the right: reference frame above, target frame below.
const tLabel = (from, to) => `T<span class="ss"><sup>${from}</sup><sub>${to}</sub></span>`;

// Ts = [T⁰₀, T⁰₁, …, T⁰ₙ] with real lengths.
function updateReadouts(Ts) {
  const n = state.joints.length;
  const Tn = Ts[n];
  document.getElementById('ee-pos').innerHTML = xyzHTML([Tn[0][3], Tn[1][3], Tn[2][3]]);

  const parts = state.joints.map((j, i) => {
    const A = dhMatrix(deg2rad(j.theta), j.d, j.a, deg2rad(j.alpha));
    return `<div class="tm"><div class="tm-title">${tLabel(i, i + 1)} <span class="muted">= A<sub>${i + 1}</sub></span></div>${matrixHTML(A)}</div>`;
  });
  const product = state.joints.map((_, i) => `A<sub>${i + 1}</sub>`).join('');
  document.getElementById('matrices').innerHTML = parts.join('') +
    `<div class="tm final"><div class="tm-title">${tLabel(0, n)} <span class="muted">= ${product}</span></div>${matrixHTML(Tn)}</div>`;

  const cell = (j, key, text) => `<td class="${isVariable(key, j.type) ? 'var' : ''}">${text}${isVariable(key, j.type) ? '*' : ''}</td>`;
  document.getElementById('dh-table').innerHTML =
    '<tr><th>i</th><th>type</th><th>θ (°)</th><th>d</th><th>a</th><th>α (°)</th></tr>' +
    state.joints
      .map((j, i) => `<tr data-type="${j.type}"><td>${i + 1}</td><td>${j.type === 'R' ? 'Rev' : 'Pri'}</td>` +
        cell(j, 'theta', fmt(j.theta, 1)) + cell(j, 'd', fmt(j.d, 2)) +
        cell(j, 'a', fmt(j.a, 2)) + cell(j, 'alpha', fmt(j.alpha, 1)) + '</tr>')
      .join('');
}

// ---------------------------------------------------------------------------
// Render loop
// ---------------------------------------------------------------------------

function resize() {
  const w = viewport.clientWidth;
  const h = viewport.clientHeight;
  if (!w || !h) return;
  renderer.setSize(w, h);
  labelRenderer.setSize(w, h);
  aspect = w / h;
  perspCam.aspect = aspect;
  perspCam.updateProjectionMatrix();
  setOrthoHalfHeight(orthoCam.top); // keep height, refit width
  // setSize clears the canvas; redraw now so it never shows a blank frame.
  renderer.render(scene, camera);
  labelRenderer.render(scene, camera);
}
new ResizeObserver(resize).observe(viewport);

function animate(now) {
  requestAnimationFrame(animate);
  stepViewAnim(now);
  updateGuides();
  updateViewCube();
  renderer.render(scene, camera);
  labelRenderer.render(scene, camera);
}

buildPresetButtons();
// Optional preset via URL hash, e.g. index.html#scara
const hashPreset = location.hash.slice(1);
if (PRESETS[hashPreset]) {
  state.joints = clone(PRESETS[hashPreset].joints);
  state.scale = PRESETS[hashPreset].scale;
}
resize();
setProjection('ortho');
syncLockUI();
structureChanged();
requestAnimationFrame(animate);
