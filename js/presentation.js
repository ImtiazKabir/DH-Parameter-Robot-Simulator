// Presentation mode: a guided, animated walkthrough of how to read the DH
// parameters off a (revolute-only) robot. It uses the loaded robot's own frames
// as the answer, so the table it builds matches the sidebar's DH table.
//
// Every sub-step has `apply()` (its lasting result: an axis drawn, a table cell
// filled) and `animate()` (the show that ends in that same state). Going to
// sub-step k = clear, apply 0..k-1 instantly, animate k. That makes Back,
// Rewatch and jumping exact.

import * as THREE from 'three';

const V3 = THREE.Vector3;
const CANCEL = Symbol('cancel');
const easeInOut = (t) => (t < 0.5 ? 2 * t * t : 1 - (-2 * t + 2) ** 2 / 2);
const sub = (k) => `<sub>${k}</sub>`;
const sym = (s, k) => `${s}${sub(k)}`;

// Cancellable animation run: every await checks for cancellation.
class Run {
  constructor(speed) {
    this.cancelled = false;
    this.speed = speed;
  }

  async tween(ms, fn, ease = easeInOut) {
    if (this.cancelled) throw CANCEL;
    if (!Number.isFinite(this.speed)) { fn(1); return; } // instant mode
    const dur = ms / this.speed;
    const t0 = performance.now();
    for (;;) {
      const t = Math.min(1, (performance.now() - t0) / dur);
      fn(ease(t));
      if (t >= 1) return;
      await new Promise((r) => requestAnimationFrame(r));
      if (this.cancelled) throw CANCEL;
    }
  }

  wait(ms) { return this.tween(ms, () => {}, (t) => t); }
}

const STEPS = [
  { short: 'Intro', name: 'Overview' },
  { short: '1 · dᵢ', name: 'Joint offsets dᵢ' },
  { short: '2 · aᵢ', name: 'Link lengths aᵢ' },
  { short: '3 · z', name: 'Define the z axes' },
  { short: '4 · x', name: 'Define the x axes' },
  { short: '5 · αᵢ', name: 'Link twists αᵢ' },
  { short: '6 · θᵢ', name: 'Joint angles θᵢ' },
  { short: 'Done', name: 'Summary' },
];

export function createPresentation(ctx) {
  const { state, scene, COLORS, DIM_COLORS } = ctx;
  const el = (id) => document.getElementById(id);
  const ui = {
    root: el('pres'),
    steps: el('pres-steps'),
    dots: el('pres-dots'),
    kicker: el('pres-kicker'),
    title: el('pres-title'),
    body: el('pres-body'),
    back: el('pres-back'),
    rewatch: el('pres-rewatch'),
    next: el('pres-next'),
    exit: el('pres-exit'),
    tableBox: el('pres-table'),
    table: el('pres-dh'),
    tableToggle: el('pres-table-toggle'),
  };

  // Scene overlay: `persist` holds the lasting results, `tmp` the current show.
  const root = new THREE.Group();
  const persist = new THREE.Group();
  const tmp = new THREE.Group();
  root.add(persist, tmp);
  root.visible = false;
  scene.add(root);

  let active = false;
  let script = [];
  let idx = 0;
  let run = null;
  let speed = 1;
  let axes = new Map();   // 'z0' → { group, mat, label, origin, dir }
  let filled = new Set(); // 'd0', 'a2', 'alpha1', 'theta0'
  let flash = null;       // key of the cell filled by the current sub-step
  let saved = null;

  // ---------------------------------------------------------------------------
  // Geometry helpers (drawn space; values in real units)
  // ---------------------------------------------------------------------------
  const col = (T, c) => new V3(T[0][c], T[1][c], T[2][c]);
  const geo = () => {
    const Ts = ctx.drawnTransforms();
    const n = state.joints.length;
    const O = (k) => col(Ts[k], 3);
    const X = (k) => col(Ts[k], 0);
    const Y = (k) => col(Ts[k], 1);
    const Z = (k) => col(Ts[k], 2);
    const P = (j) => O(j).addScaledVector(Z(j), state.joints[j].d / state.scale); // end of d_j
    return { Ts, n, O, X, Y, Z, P };
  };
  const EPS = 1e-6;
  const val = (v) => ctx.short(v);

  // Describe a direction in words, e.g. "up (+Z)" or "toward +X −Y".
  function dirWords(v) {
    if (v.z > 0.9) return 'up (+Z)';
    if (v.z < -0.9) return 'down (−Z)';
    const parts = [];
    ['X', 'Y', 'Z'].forEach((a, i) => {
      const c = v.getComponent(i);
      if (Math.abs(c) > 0.3) parts.push(`${c > 0 ? '+' : '−'}${a}`);
    });
    return `toward ${parts.join(' ')}`;
  }

  // ---------------------------------------------------------------------------
  // Overlay builders
  // ---------------------------------------------------------------------------
  const UP = new V3(0, 1, 0);

  function addLabel(group, html, pos, cls, color) {
    const l = ctx.makeLabel(html, cls, true);
    if (color) l.element.style.color = color;
    l.position.copy(pos);
    group.add(l);
    return l;
  }

  // Axis arrows are drawn on top of the robot (no depth test): a z axis runs
  // right through the middle of its joint and offset link, and must stay visible.
  const AXIS_LEN = 0.95;
  function axisArrow(kind, origin, dir, opacity = 1) {
    const mat = new THREE.MeshStandardMaterial({
      color: COLORS[kind], roughness: 0.4, transparent: true, opacity, depthTest: false,
    });
    const arrow = ctx.makeArrow(mat, AXIS_LEN, 0.04);
    arrow.traverse((m) => { m.renderOrder = 5; });
    arrow.quaternion.setFromUnitVectors(UP, dir);
    arrow.position.copy(origin);
    return { arrow, mat };
  }

  // Coordinate axis arrow for frame k (kind 'x' | 'z').
  function addAxis(kind, k, g) {
    const origin = g.O(k);
    const dir = kind === 'x' ? g.X(k) : g.Z(k);
    const base = 1;
    const len = AXIS_LEN;
    const { arrow, mat } = axisArrow(kind, origin, dir);
    persist.add(arrow);
    const label = addLabel(persist, sym(kind, k), origin.clone().addScaledVector(dir, len + 0.16), `pres-axis pres-${kind}`);
    label.element.style.opacity = base;
    const obj = { arrow, mat, label, base, origin, dir, len };
    axes.set(`${kind}${k}`, obj);
    return obj;
  }

  function addOrigin(k, g) {
    const m = new THREE.Mesh(new THREE.SphereGeometry(0.06, 16, 12), new THREE.MeshBasicMaterial({ color: 0x1d2430, transparent: true, depthTest: false }));
    m.renderOrder = 6;
    m.position.copy(g.O(k));
    persist.add(m);
    axes.set(`o${k}`, { arrow: m, mat: m.material, label: null, base: 1 });
  }

  // Dim every persistent item except `keys` (null = restore all).
  function setFocus(keys) {
    for (const [key, o] of axes) {
      const op = !keys || keys.includes(key) ? o.base : 0.1;
      o.mat.opacity = op;
      if (o.label) o.label.element.style.opacity = op;
    }
  }

  function tube(points, radius, mat) {
    const curve = points.length === 2
      ? new THREE.LineCurve3(points[0], points[1])
      : new THREE.CatmullRomCurve3(points);
    return new THREE.Mesh(new THREE.TubeGeometry(curve, Math.max(2, points.length * 2), radius, 8, false), mat);
  }

  function dashedLine(a, b, color, opacity = 0.9) {
    const l = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([a, b]),
      new THREE.LineDashedMaterial({ color, dashSize: 0.08, gapSize: 0.06, transparent: true, opacity }),
    );
    l.computeLineDistances();
    tmp.add(l);
    return l;
  }

  // Dashed rotation-axis line through `p` along `dir`, growing from the middle.
  async function showAxisLine(r, p, dir, color = '#475569') {
    const l = dashedLine(p.clone().addScaledVector(dir, -1.5), p.clone().addScaledVector(dir, 1.5), color, 0.8);
    l.position.copy(p);
    l.geometry.translate(-p.x, -p.y, -p.z);
    await r.tween(400, (e) => l.scale.setScalar(Math.max(e, 1e-3)));
    return l;
  }

  function cone(color) {
    return new THREE.Mesh(new THREE.ConeGeometry(0.05, 0.14, 14), new THREE.MeshBasicMaterial({ color }));
  }

  // Dimension line from p0 to p1, offset by `off`, growing; then its label.
  async function growDim(r, p0, p1, off, color, html) {
    const q0 = p0.clone().add(off), q1 = p1.clone().add(off);
    dashedLine(p0, q0.clone().addScaledVector(off.clone().normalize(), 0.06), color);
    dashedLine(p1, q1.clone().addScaledVector(off.clone().normalize(), 0.06), color);
    const mat = new THREE.MeshBasicMaterial({ color });
    let line = null;
    const head = cone(color);
    const dir = q1.clone().sub(q0).normalize();
    head.quaternion.setFromUnitVectors(UP, dir);
    tmp.add(head);
    await r.tween(700, (e) => {
      if (line) { tmp.remove(line); line.geometry.dispose(); }
      const end = q0.clone().lerp(q1, Math.max(e, 0.01));
      line = tube([q0, end], 0.016, mat);
      tmp.add(line);
      head.position.copy(end).addScaledVector(dir, -0.07);
    });
    const tail = cone(color);
    tail.quaternion.setFromUnitVectors(UP, dir.clone().negate());
    tail.position.copy(q0).addScaledVector(dir, 0.07);
    tmp.add(tail);
    addLabel(tmp, html, q0.clone().lerp(q1, 0.5).addScaledVector(off.clone().normalize(), 0.22), 'dim-label var pres-pop', color);
  }

  // Growing angle arc about `axis` from direction `from`, ending at `angle` rad.
  async function growArc(r, center, axis, from, angle, radius, color, name) {
    const mat = new THREE.MeshBasicMaterial({ color });
    const head = cone(color);
    tmp.add(head);
    const label = addLabel(tmp, '', center, 'dim-label var', color);
    let arc = null;
    const deg = THREE.MathUtils.radToDeg(angle);
    await r.tween(1300, (e) => {
      const a = angle * e;
      const N = Math.max(2, Math.ceil(Math.abs(a) / 0.06));
      const pts = [];
      for (let k = 0; k <= N; k++) {
        pts.push(from.clone().applyAxisAngle(axis, (a * k) / N).multiplyScalar(radius).add(center));
      }
      if (arc) { tmp.remove(arc); arc.geometry.dispose(); }
      if (Math.abs(a) > 0.02) {
        arc = tube(pts, 0.016, mat);
        tmp.add(arc);
      } else arc = null;
      const end = from.clone().applyAxisAngle(axis, a);
      const tangent = axis.clone().cross(end).multiplyScalar(Math.sign(angle) || 1).normalize();
      head.visible = Math.abs(a) > 0.08;
      head.quaternion.setFromUnitVectors(UP, tangent);
      head.position.copy(end).multiplyScalar(radius).add(center);
      const mid = from.clone().applyAxisAngle(axis, a / 2);
      label.position.copy(center).addScaledVector(mid, radius + 0.32);
      label.element.innerHTML = `${name} = ${Math.round(deg * e)}°`;
    });
  }

  // Labelled copy of a persistent axis arrow (for sliding it next to another axis).
  function axisCopy(key) {
    const o = axes.get(key);
    const mat = o.mat.clone();
    mat.opacity = 1;
    const g = o.arrow.clone();
    g.traverse((m) => { if (m.isMesh) m.material = mat; });
    const [kind, k] = [key[0], key.slice(1)];
    // Child of the arrow (local +Y = axis direction), so it travels with the copy.
    addLabel(g, sym(kind, k), new V3(0, o.len + 0.16, 0), `pres-axis pres-${kind}`);
    tmp.add(g);
    return g;
  }

  async function slide(r, obj, from, to, ms = 700) {
    if (from.distanceTo(to) < 1e-4) { obj.position.copy(to); return; }
    await r.tween(ms, (e) => obj.position.copy(from).lerp(to, e));
  }

  // Swing joint j back and forth (presentation offset only; the student's θ is untouched).
  async function wiggle(r, j, { cycles = 2, amp = 35, ms = 2600, onFrame } = {}) {
    state.presSpin = j; // checker on the joint drum while it turns
    await r.tween(ms, (e) => {
      state.presOffset[j] = amp * Math.sin(2 * Math.PI * cycles * e);
      ctx.update();
      if (onFrame) onFrame();
    }, (t) => t);
    state.presOffset[j] = 0;
    state.presSpin = null;
    ctx.update();
  }

  // ---------------------------------------------------------------------------
  // Showing several valid choices, then picking one
  // ---------------------------------------------------------------------------

  // Translucent candidate arrow with its own label (child, travels with it).
  function ghost(kind, origin, dir, html) {
    const o = axisArrow(kind, origin, dir, 0.4);
    o.label = addLabel(o.arrow, html, new V3(0, AXIS_LEN + 0.2, 0), `pres-axis pres-${kind} pres-option`);
    tmp.add(o.arrow);
    return o;
  }

  async function growAll(r, objs, ms = 600) {
    await r.tween(ms, (e) => objs.forEach((o) => o.arrow.scale.set(1, Math.max(e, 1e-3), 1)));
  }

  // Fade out every candidate but `keep`, make it solid, then hand over to the
  // lasting arrow (same place and direction).
  async function choose(r, cands, keep, kind, k, g) {
    addLabel(cands[keep].arrow, '✓ chosen', new V3(0, AXIS_LEN * 0.55, 0), 'pres-choice pres-pop');
    await r.tween(700, (e) => cands.forEach((c, i) => {
      if (i === keep) c.mat.opacity = 0.4 + 0.6 * e;
      else {
        c.mat.opacity = 0.4 * (1 - e);
        c.label.element.style.opacity = String(1 - e);
      }
    }));
    await r.wait(350);
    cands.forEach((c) => { ctx.clearGroup(c.arrow); tmp.remove(c.arrow); });
    return addAxis(kind, k, g);
  }

  // Small right-angle mark at `o` between unit directions a and b.
  function rightAngle(o, a, b, color = 0x1d2430) {
    const s2 = 0.2;
    const mat = new THREE.MeshBasicMaterial({ color, depthTest: false, transparent: true });
    const p1 = o.clone().addScaledVector(a, s2);
    const p2 = p1.clone().addScaledVector(b, s2);
    const p3 = o.clone().addScaledVector(b, s2);
    const grp = new THREE.Group();
    grp.add(tube([p1, p2], 0.008, mat), tube([p2, p3], 0.008, mat));
    grp.traverse((m) => { m.renderOrder = 7; });
    tmp.add(grp);
    return grp;
  }

  // Infinitely many choices (any direction ⟂ axis): a dashed ring and a ghost
  // arrow sweeping once around it, coming to rest on `chosen`.
  async function sweepChoice(r, kind, k, g, origin, axis, chosen, html) {
    const pts = [];
    for (let t = 0; t <= 64; t++) pts.push(chosen.clone().applyAxisAngle(axis, (t / 64) * Math.PI * 2).multiplyScalar(AXIS_LEN).add(origin));
    const ring = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.LineDashedMaterial({ color: COLORS[kind], dashSize: 0.07, gapSize: 0.05, depthTest: false }),
    );
    ring.computeLineDistances();
    ring.renderOrder = 5;
    tmp.add(ring);
    const c = ghost(kind, origin, chosen, html);
    await growAll(r, [c], 400);
    await r.tween(2200, (e) => {
      const d = chosen.clone().applyAxisAngle(axis, Math.PI * 2 * (1 - e));
      c.arrow.quaternion.setFromUnitVectors(UP, d);
    });
    return choose(r, [c], 0, kind, k, g);
  }

  // ---------------------------------------------------------------------------
  // Camera
  // ---------------------------------------------------------------------------
  const cam = () => ctx.getCamera();
  function quatLook(dir, up) {
    const m = new THREE.Matrix4().lookAt(dir.clone().normalize(), new V3(), up.clone().normalize());
    return new THREE.Quaternion().setFromRotationMatrix(m);
  }
  function getPose() {
    const o = ctx.orthoCam;
    return { q: cam().quaternion.clone(), target: ctx.target.clone(), halfH: o.top / o.zoom };
  }
  function setPose(p) {
    const c = cam();
    ctx.setOrthoHalfHeight(p.halfH);
    ctx.orthoCam.zoom = 1;
    ctx.orthoCam.updateProjectionMatrix();
    c.quaternion.copy(p.q);
    ctx.target.copy(p.target);
    c.position.copy(p.target).addScaledVector(new V3(0, 0, 1).applyQuaternion(p.q), 40);
  }
  async function camTo(r, p, ms = 900) {
    const a = getPose();
    await r.tween(ms, (e) => setPose({
      q: a.q.clone().slerp(p.q, e),
      target: a.target.clone().lerp(p.target, e),
      halfH: a.halfH + (p.halfH - a.halfH) * e,
    }));
  }
  const isoQ = () => quatLook(ctx.ISO_DIR, new V3(0, 0, 1));
  function overviewPose() {
    const sphere = ctx.robotBox().getBoundingSphere(new THREE.Sphere());
    const r = Math.max(sphere.radius, 1.5) + 1.1;
    const asp = ctx.getAspect();
    return { q: isoQ(), target: sphere.center.clone(), halfH: asp >= 1 ? r : r / asp };
  }
  function focusPose(point) {
    const o = overviewPose();
    return { q: o.q, target: o.target.clone().lerp(point, 0.55), halfH: o.halfH * 0.8 };
  }
  const axisPose = (eyeDir, up, center) => ({ q: quatLook(eyeDir, up), target: center.clone(), halfH: 1.9 });

  // ---------------------------------------------------------------------------
  // Script
  // ---------------------------------------------------------------------------
  function rules(list, on) {
    return `<ol class="pres-rules">${list.map((t, i) => `<li class="${i === on ? 'on' : ''}">${t}</li>`).join('')}</ol>`;
  }
  const here = (t) => `<p class="pres-here"><b>Here:</b> ${t}</p>`;

  function buildScript() {
    const n = state.joints.length;
    const s = [{ step: 0, kind: 'intro' }];
    for (let j = 0; j < n; j++) s.push({ step: 1, kind: 'd', j });
    for (let j = 0; j < n; j++) s.push({ step: 2, kind: 'a', j });
    for (let k = 0; k <= n; k++) s.push({ step: 3, kind: 'z', k });
    for (let k = 0; k <= n; k++) s.push({ step: 4, kind: 'x', k });
    for (let j = 0; j < n; j++) s.push({ step: 5, kind: 'alpha', j });
    for (let j = 0; j < n; j++) s.push({ step: 6, kind: 'theta', j });
    s.push({ step: 7, kind: 'summary' });
    return s;
  }

  // Lasting result of a sub-step.
  function apply(s, g) {
    switch (s.kind) {
      case 'd': case 'a': case 'alpha': case 'theta': filled.add(`${s.kind}${s.j}`); break;
      case 'z': addAxis('z', s.k, g); break;
      case 'x': addAxis('x', s.k, g); addOrigin(s.k, g); break;
      default: break;
    }
  }

  function text(s, g) {
    const n = g.n;
    const J = (j) => state.joints[j];
    switch (s.kind) {
      case 'intro': return {
        title: 'How to read DH parameters off a robot',
        body: `<p>We'll find all four DH parameters of this ${n}-joint robot in six steps: first the lengths
          <b>dᵢ</b> and <b>aᵢ</b> by turning each joint, then the frames (<b>z</b>, then <b>x</b>), and finally
          the angles <b>αᵢ</b> and <b>θᵢ</b>. The table fills in as we go.</p>
          <p class="pres-keys">Next ▶ <kbd>→</kbd> · Back <kbd>←</kbd> · Rewatch <kbd>R</kbd> · Exit <kbd>Esc</kbd>.
          Click a step above to jump to it. You can still drag to look around.</p>`,
      };
      case 'd': {
        const I = s.j + 1, d = J(s.j).d, last = s.j === n - 1;
        return {
          title: `Joint ${I}`,
          body: `<p><b>d${sub(I)}</b> is the <b>joint offset</b>: the distance along joint ${I}'s rotation axis
            between the two links it connects. To spot it, turn joint ${I} and watch the part between this joint and the
            ${last ? 'end-effector frame' : `next joint's axis`}: anything that only <i>twists in place</i> about joint ${I}'s
            own axis lies along that axis.</p>
            ${rules(['A part twists about the axis → d<sub>i</sub> is its length.', 'Nothing twists → d<sub>i</sub> = 0.'], Math.abs(d) > EPS ? 0 : 1)}
            ${here(Math.abs(d) > EPS ? `the checkered part twists about joint ${I}'s axis, so <b>d${sub(I)} = ${val(d)}</b>.`
              : `nothing between joint ${I} and the ${last ? 'end-effector frame' : 'next joint'} twists in place, so <b>d${sub(I)} = 0</b>.`)}`,
        };
      }
      case 'a': {
        const I = s.j + 1, a = J(s.j).a, last = s.j === n - 1;
        return {
          title: `Joint ${I}`,
          body: `<p><b>a${sub(I)}</b> is the <b>link length</b>: the distance between joint ${I}'s axis and the
            ${last ? 'end-effector frame’s z axis' : `next joint's axis`}, measured along the line perpendicular to both.
            Turn joint ${I}: the part that <i>swings around</i> the axis — sweeping through space instead of twisting in place — has this length.</p>
            ${rules(['A part swings out from the axis → a<sub>i</sub> is its length.', 'Nothing swings (the next axis meets this one) → a<sub>i</sub> = 0.'], a > EPS ? 0 : 1)}
            ${here(a > EPS ? `the highlighted link swings around joint ${I}'s axis (see its path), so <b>a${sub(I)} = ${val(a)}</b>.`
              : `nothing swings out from joint ${I}'s axis — the ${last ? 'end-effector frame' : 'next joint'} sits on it — so <b>a${sub(I)} = 0</b>.`)}`,
        };
      }
      case 'z': {
        if (s.k === n) {
          const par = Math.abs(g.Z(n).dot(g.Z(n - 1))) > 1 - 1e-6;
          return {
            title: `End-effector frame — z${sub(n)}`,
            body: `<p>Frame {${n}} is the <b>end-effector (tool) frame</b>. It has no joint, so z${sub(n)} is a
              free choice; a common choice is to keep it parallel to z${sub(n - 1)}.</p>
              ${here(par ? `z${sub(n)} can point either way along that line (both shown); we choose ${dirWords(g.Z(n))}.`
                : `this model tilts z${sub(n)} away from z${sub(n - 1)} (by α${sub(n)}); we keep it as in the model.`)}`,
          };
        }
        const I = s.k + 1, d = J(s.k).d;
        let h;
        if (Math.abs(d) > EPS) {
          h = d > 0 ? `joint ${I} has an offset (d${sub(I)} = ${val(d)}), so z${sub(s.k)} points along it, ${dirWords(g.Z(s.k))}.`
            : `z${sub(s.k)} lies along the offset, but this model points it the other way, so d${sub(I)} = ${val(d)} is negative.`;
        } else h = `joint ${I} has no offset, so both directions along its axis are valid (both shown); we choose ${dirWords(g.Z(s.k))}.`;
        return {
          title: `Joint ${I} — z${sub(s.k)}`,
          body: `<p>A joint's <b>z axis is its axis of rotation</b>. Joint ${I} turns about z${sub(s.k)}
            (in DH, joint i turns about z<sub>i−1</sub>).</p>
            ${rules([`If the joint has an offset d, z points along the offset (from this joint toward the next link).`,
              `If there is no offset, z may point either way along the rotation axis — pick one.`], Math.abs(d) > EPS ? 0 : 1)}
            ${here(h)}`,
        };
      }
      case 'x': {
        if (s.k === 0) return {
          title: 'Base frame — x₀',
          body: `<p>The base frame {0} has no previous link, so <b>x₀</b> can be <b>any direction perpendicular to z₀</b>.
            The choice sets the zero for θ₁.</p>
            ${here(`every direction around the ring is valid; we choose x₀ ${dirWords(g.X(0))}.`)}`,
        };
        const I = s.k, a = J(I - 1).a;
        const cross = g.Z(I - 1).clone().cross(g.Z(I)).length() > 1e-6;
        let h;
        if (a > EPS) h = `link ${I} has length a${sub(I)} = ${val(a)}, so x${sub(I)} points along it, away from joint ${I}.`;
        else if (cross) h = `a${sub(I)} = 0, so z${sub(I - 1)} and z${sub(I)} meet at O${sub(I)}. x${sub(I)} must be perpendicular to both (see the right-angle marks), which leaves exactly two options, ±(z${sub(I - 1)} × z${sub(I)}); we choose ${dirWords(g.X(I))}.`;
        else h = `a${sub(I)} = 0 and z${sub(I - 1)}, z${sub(I)} lie on one line, so every direction around the ring is perpendicular to both; we choose ${dirWords(g.X(I))}.`;
        return {
          title: `Frame {${I}} — x${sub(I)}`,
          body: `<p><b>x${sub(I)}</b> runs along the <b>common normal</b> from z${sub(I - 1)} to z${sub(I)} — the shortest line
            joining the two axes. The frame's origin O${sub(I)} is where x${sub(I)} meets z${sub(I)}.</p>
            ${rules(['If there is a link (a<sub>i</sub> > 0), x<sub>i</sub> points along the link, away from joint i.',
              'If there is no link (a<sub>i</sub> = 0), x<sub>i</sub> is perpendicular to both z<sub>i−1</sub> and z<sub>i</sub> (±z<sub>i−1</sub> × z<sub>i</sub>) — pick one.'], a > EPS ? 0 : 1)}
            ${here(h)}`,
        };
      }
      case 'alpha': {
        const I = s.j + 1, al = J(s.j).alpha;
        return {
          title: `Link ${I} — α${sub(I)}`,
          body: `<p><b>α${sub(I)}</b> is the <b>link twist</b>: the angle from z${sub(I - 1)} to z${sub(I)}, measured
            <b>about x${sub(I)}</b> (right-hand rule: thumb along x${sub(I)}, counter-clockwise is positive when x${sub(I)} points at you).
            We slide z${sub(I - 1)} along x${sub(I)} so both axes start at O${sub(I)}, then look straight down x${sub(I)}.</p>
            ${here(Math.abs(al) > 0.05 ? `turning z${sub(I - 1)} onto z${sub(I)} takes <b>α${sub(I)} = ${val(al)}°</b>.`
              : `z${sub(I - 1)} and z${sub(I)} are parallel, so <b>α${sub(I)} = 0°</b>.`)}`,
        };
      }
      case 'theta': {
        const I = s.j + 1, th = J(s.j).theta;
        return {
          title: `Joint ${I} — θ${sub(I)}`,
          body: `<p><b>θ${sub(I)}</b> is the <b>joint angle</b>: the angle from x${sub(I - 1)} to x${sub(I)}, measured
            <b>about z${sub(I - 1)}</b>. For a revolute joint it is the <b>joint variable</b> — it changes as the joint turns.
            We slide x${sub(I - 1)} along z${sub(I - 1)} (by d${sub(I)}) and x${sub(I)} back along itself (by a${sub(I)}) so both
            start at the same point, then look straight down z${sub(I - 1)}.</p>
            ${here(`in this pose <b>θ${sub(I)} = ${val(th)}°</b> (the joint variable).`)}`,
        };
      }
      default: return {
        title: 'The complete DH table',
        body: `<p>Every frame is placed and every parameter found. The table matches the DH table in the sidebar.</p>
          <p>Each row gives one link transform, A<sub>i</sub> = Rot<sub>z</sub>(θ<sub>i</sub>)·Trans<sub>z</sub>(d<sub>i</sub>)·Trans<sub>x</sub>(a<sub>i</sub>)·Rot<sub>x</sub>(α<sub>i</sub>),
          and T<sup>0</sup><sub>${n}</sub> = A<sub>1</sub>⋯A<sub>${n}</sub> places the end effector.</p>`,
      };
    }
  }

  // The show for each sub-step; it must end in the state `apply` produces.
  async function animate(s, r, g) {
    const J = (j) => state.joints[j];
    switch (s.kind) {
      case 'intro':
      case 'summary':
        await camTo(r, overviewPose());
        return;

      case 'd': {
        const { j } = s;
        const mid = g.O(j).lerp(g.O(j + 1), 0.5);
        await camTo(r, focusPose(mid));
        await showAxisLine(r, g.O(j), g.Z(j));
        state.presHighlight = { joint: j, part: 'twist' };
        await wiggle(r, j);
        await showAxisLine(r, g.O(j + 1), g.Z(j + 1), '#94a3b8');
        const d = J(j).d;
        if (Math.abs(d) > EPS) {
          await growDim(r, g.O(j), g.P(j), g.X(j + 1).negate().multiplyScalar(0.4), DIM_COLORS.d, `d${sub(j + 1)} = ${val(d)}`);
        } else {
          addLabel(tmp, `d${sub(j + 1)} = 0`, g.O(j).addScaledVector(g.Z(j), 0.35).addScaledVector(g.X(j + 1), -0.5), 'dim-label var pres-pop', DIM_COLORS.d);
        }
        break;
      }

      case 'a': {
        const { j } = s;
        const mid = g.O(j).lerp(g.O(j + 1), 0.5);
        await camTo(r, focusPose(mid));
        await showAxisLine(r, g.O(j), g.Z(j));
        state.presHighlight = { joint: j, part: 'swing' };
        const a = J(j).a;
        // Trail of the link's tip while it swings.
        const trailMat = new THREE.MeshBasicMaterial({ color: 0xf59e0b, transparent: true, opacity: 0.75 });
        const pts = [];
        let trail = null;
        const track = () => {
          if (a <= EPS) return;
          const tip = col(ctx.drawnTransforms()[j + 1], 3);
          if (!pts.length || pts[pts.length - 1].distanceTo(tip) > 0.03) pts.push(tip);
          if (pts.length < 3) return;
          if (trail) { tmp.remove(trail); trail.geometry.dispose(); }
          trail = tube(pts, 0.012, trailMat);
          tmp.add(trail);
        };
        await wiggle(r, j, { onFrame: track });
        await showAxisLine(r, g.O(j + 1), g.Z(j + 1), '#94a3b8');
        if (a > EPS) {
          await growDim(r, g.P(j), g.O(j + 1), g.Z(j).multiplyScalar(0.4), DIM_COLORS.a, `a${sub(j + 1)} = ${val(a)}`);
        } else {
          addLabel(tmp, `a${sub(j + 1)} = 0`, g.O(j + 1).addScaledVector(g.Z(j), 0.4), 'dim-label var pres-pop', DIM_COLORS.a);
        }
        break;
      }

      case 'z': {
        const { k } = s;
        await camTo(r, focusPose(g.O(k)));
        await showAxisLine(r, g.O(k), g.Z(k));
        if (k < g.n) await wiggle(r, k, { cycles: 1, amp: 25, ms: 1400 });
        // Two valid directions: no offset on this joint, or the end-effector z
        // kept parallel to the previous axis. Show both, then choose.
        const twoWays = k < g.n
          ? Math.abs(J(k).d) <= EPS
          : Math.abs(g.Z(g.n).dot(g.Z(g.n - 1))) > 1 - 1e-6;
        if (twoWays) {
          const z = g.Z(k);
          const cands = [ghost('z', g.O(k), z, `${sym('z', k)} option 1`), ghost('z', g.O(k), z.clone().negate(), `${sym('z', k)} option 2`)];
          await growAll(r, cands);
          await r.wait(1200);
          await choose(r, cands, 0, 'z', k, g);
          return;
        }
        const o = addAxis('z', k, g);
        await r.tween(600, (e) => o.arrow.scale.set(1, Math.max(e, 1e-3), 1));
        return; // the arrow added here is the lasting result
      }

      case 'x': {
        const { k } = s;
        await camTo(r, focusPose(g.O(k)));
        if (k > 0 && J(k - 1).a > EPS) {
          state.presHighlight = { joint: k - 1, part: 'swing' };
          ctx.update();
          await r.wait(500);
        }
        const a = k > 0 ? J(k - 1).a : 0;
        if (k === 0) {
          // Base frame: any direction perpendicular to z₀.
          await sweepChoice(r, 'x', 0, g, g.O(0), g.Z(0), g.X(0), `${sym('x', 0)} ?`);
        } else if (a > EPS) {
          // Rule 1: along the link.
          const x = addAxis('x', k, g);
          await r.tween(600, (e) => x.arrow.scale.set(1, Math.max(e, 1e-3), 1));
        } else if (g.Z(k - 1).clone().cross(g.Z(k)).length() > 1e-6) {
          // Rule 2, axes intersect at O_k: bring z_{k-1} there, show that x must be
          // perpendicular to both z's, which leaves ±(z_{k-1} × z_k); then choose.
          const zc = axisCopy(`z${k - 1}`);
          await slide(r, zc, g.O(k - 1), g.O(k), 700);
          const x = g.X(k);
          const cands = [ghost('x', g.O(k), x, `${sym('x', k)} option 1`), ghost('x', g.O(k), x.clone().negate(), `${sym('x', k)} option 2`)];
          await growAll(r, cands);
          for (const c of [x, x.clone().negate()]) {
            rightAngle(g.O(k), c, g.Z(k - 1));
            rightAngle(g.O(k), c, g.Z(k));
            await r.wait(400);
          }
          await r.wait(1000);
          await choose(r, cands, 0, 'x', k, g);
        } else {
          // Rule 2, collinear axes: any direction perpendicular to the shared axis.
          await sweepChoice(r, 'x', k, g, g.O(k), g.Z(k), g.X(k), `${sym('x', k)} ?`);
        }
        addOrigin(k, g);
        return;
      }

      case 'alpha': {
        const { j } = s; // α_{j+1}: z_j → z_{j+1} about x_{j+1}, at O_{j+1}
        const keys = [`z${j}`, `z${j + 1}`, `x${j + 1}`];
        setFocus(keys);
        ctx.setDefocus(true);
        await camTo(r, focusPose(g.O(j + 1)), 700);
        const copy = axisCopy(`z${j}`);
        await slide(r, copy, g.O(j), g.P(j), 500);
        await slide(r, copy, g.P(j), g.O(j + 1), 800);
        await camTo(r, axisPose(g.X(j + 1), g.Z(j), g.O(j + 1)), 1000);
        await growArc(r, g.O(j + 1), g.X(j + 1), g.Z(j), THREE.MathUtils.degToRad(J(j).alpha), 0.6, DIM_COLORS.alpha, `α${sub(j + 1)}`);
        break;
      }

      case 'theta': {
        const { j } = s; // θ_{j+1}: x_j → x_{j+1} about z_j, at P_j
        const keys = [`x${j}`, `x${j + 1}`, `z${j}`];
        setFocus(keys);
        ctx.setDefocus(true);
        await camTo(r, focusPose(g.P(j)), 700);
        const cj = axisCopy(`x${j}`);
        const cn = axisCopy(`x${j + 1}`);
        await slide(r, cj, g.O(j), g.P(j), 700);
        await slide(r, cn, g.O(j + 1), g.P(j), 800);
        await camTo(r, axisPose(g.Z(j), g.Y(j), g.P(j)), 1000);
        await growArc(r, g.P(j), g.Z(j), g.X(j), THREE.MathUtils.degToRad(J(j).theta), 0.6, DIM_COLORS.theta, `θ${sub(j + 1)}`);
        break;
      }
      default: return;
    }
    // Sub-steps whose result is a table cell fill it at the end of the show.
    if (['d', 'a', 'alpha', 'theta'].includes(s.kind)) {
      filled.add(`${s.kind}${s.j}`);
      flash = `${s.kind}${s.j}`;
      renderTable();
    }
  }

  // ---------------------------------------------------------------------------
  // UI
  // ---------------------------------------------------------------------------
  function renderTable() {
    const cell = (kind, j, text) => {
      const key = `${kind}${j}`;
      const on = filled.has(key);
      return `<td class="${on ? 'on' : 'q'}${flash === key ? ' flash' : ''}">${on ? text : '?'}</td>`;
    };
    ui.table.innerHTML = '<tr><th>i</th><th>θ<sub>i</sub></th><th>d<sub>i</sub></th><th>a<sub>i</sub></th><th>α<sub>i</sub></th></tr>'
      + state.joints.map((jt, j) => `<tr><td>${j + 1}</td>${cell('theta', j, `${val(jt.theta)}°<span class="var">*</span>`)}${
        cell('d', j, val(jt.d))}${cell('a', j, val(jt.a))}${cell('alpha', j, `${val(jt.alpha)}°`)}</tr>`).join('');
  }

  function renderUI() {
    const s = script[idx];
    const g = geo();
    ui.steps.innerHTML = '';
    STEPS.forEach((st, k) => {
      const b = document.createElement('button');
      b.className = `pres-chip${k === s.step ? ' active' : ''}${k < s.step ? ' done' : ''}`;
      b.innerHTML = st.short;
      b.title = st.name;
      b.addEventListener('click', () => goTo(script.findIndex((x) => x.step === k)));
      ui.steps.append(b);
    });
    ui.dots.innerHTML = '';
    const same = script.map((x, k) => [x, k]).filter(([x]) => x.step === s.step);
    if (same.length > 1) {
      same.forEach(([, k], n) => {
        const b = document.createElement('button');
        b.className = `pres-dot${k === idx ? ' active' : ''}${k < idx ? ' done' : ''}`;
        b.title = `Part ${n + 1} of ${same.length}`;
        b.setAttribute('aria-label', b.title);
        b.addEventListener('click', () => goTo(k));
        ui.dots.append(b);
      });
    }
    const t = text(s, g);
    const stepNo = s.step >= 1 && s.step <= 6 ? `Step ${s.step} / 6 · ` : '';
    const part = same.length > 1 ? ` · part ${same.findIndex(([, k]) => k === idx) + 1} of ${same.length}` : '';
    ui.kicker.innerHTML = `${stepNo}${STEPS[s.step].name}${part}`;
    ui.title.innerHTML = t.title;
    ui.body.innerHTML = t.body;
    ui.back.disabled = idx === 0;
    ui.next.textContent = idx === script.length - 1 ? 'Finish ✓' : 'Next ▶';
  }

  async function goTo(k) {
    if (!active || k < 0 || k >= script.length) return;
    if (run) run.cancelled = true;
    idx = k;
    // Reset to the model pose, then rebuild everything before sub-step k.
    state.presOffset = new Array(state.joints.length).fill(0);
    state.presHighlight = null;
    state.presSpin = null;
    ctx.setDefocus(false);
    ctx.update();
    ctx.clearGroup(persist);
    ctx.clearGroup(tmp);
    axes = new Map();
    filled = new Set();
    flash = null;
    const g = geo();
    for (let i = 0; i < k; i++) apply(script[i], g);
    setFocus(null);
    renderTable();
    renderUI();
    const r = (run = new Run(speed));
    try {
      await animate(script[k], r, g);
    } catch (e) {
      if (e !== CANCEL) throw e;
    }
  }

  // ---------------------------------------------------------------------------
  // Enter / exit
  // ---------------------------------------------------------------------------
  function enter() {
    if (active || !ctx.isRevoluteOnly()) return;
    active = true;
    saved = {
      viewMode: state.viewMode,
      proj: ctx.getCamera().isOrthographicCamera ? 'ortho' : 'persp',
      collapsed: document.body.classList.contains('panel-collapsed'),
      toggles: Object.fromEntries(Object.entries(ctx.VIEW_TOGGLES).map(([id, [key]]) => [id, state[key]])),
      rotating: state.rotating,
    };
    ctx.setViewMode('single');
    ctx.setProjection('ortho');
    saved.pose = { ...getPose(), pos: cam().position.clone() };
    for (const id of ['show-ee', 'show-dims', 'show-axes', 'show-labels']) ctx.setToggle(id, false);
    ctx.setPanelCollapsed(true);
    state.rotating = null;
    document.body.classList.add('presenting');
    ui.root.hidden = false;
    ui.tableBox.hidden = false;
    root.visible = true;
    script = buildScript();
    window.addEventListener('keydown', onKey);
    goTo(0);
  }

  function exit() {
    if (!active) return;
    if (run) run.cancelled = true;
    active = false;
    window.removeEventListener('keydown', onKey);
    ctx.clearGroup(persist);
    ctx.clearGroup(tmp);
    root.visible = false;
    ui.root.hidden = true;
    ui.tableBox.hidden = true;
    document.body.classList.remove('presenting');
    state.presOffset = null;
    state.presHighlight = null;
    state.presSpin = null;
    state.rotating = saved.rotating;
    ctx.setDefocus(false);
    for (const [id, on] of Object.entries(saved.toggles)) ctx.setToggle(id, on);
    ctx.setPanelCollapsed(saved.collapsed);
    setPose(saved.pose);
    ctx.setProjection(saved.proj);
    ctx.setViewMode(saved.viewMode);
    ctx.update();
  }

  function onKey(e) {
    if (e.target.closest?.('input, textarea, select')) return;
    if (e.key === 'ArrowRight' || e.key === 'PageDown') goTo(idx + 1);
    else if (e.key === 'ArrowLeft' || e.key === 'PageUp') goTo(idx - 1);
    else if (e.key === 'r' || e.key === 'R') goTo(idx);
    else if (e.key === 'Escape') exit();
    else return;
    e.preventDefault();
  }

  ui.back.addEventListener('click', () => goTo(idx - 1));
  ui.rewatch.addEventListener('click', () => goTo(idx));
  ui.next.addEventListener('click', () => (idx === script.length - 1 ? exit() : goTo(idx + 1)));
  ui.exit.addEventListener('click', exit);
  ui.tableToggle.addEventListener('click', () => {
    const hide = !ui.tableBox.classList.contains('collapsed');
    ui.tableBox.classList.toggle('collapsed', hide);
    ui.tableToggle.textContent = hide ? 'Show DH table' : 'Hide';
  });

  return {
    enter,
    exit,
    goTo,
    isActive: () => active,
    // for tests / power users
    setSpeed: (s) => { speed = s; },
    get index() { return idx; },
    get script() { return script; },
    get filled() { return filled; },
    get axes() { return axes; },
  };
}
