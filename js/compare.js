// Compare mode: is a second DH table B equivalent to the current robot A?
//
// Two DH tables are equivalent when they describe the same physical robot,
// only with different (equally valid) frame choices. Then, in every pose:
//   • each joint turns (or slides) about the same line in space — the joint
//     axes coincide as lines, though a z axis may point the other way;
//   • the end-effector point is the same.
// We test that in the current pose and in a few other poses, where every
// joint of B is moved by the same amount as A's (with the sign flipped where
// B's z axis points the other way). B is drawn in blue on top of A.

import * as THREE from 'three';
import { forwardKinematics, deg2rad } from './dh.js';

const V3 = THREE.Vector3;
const UP = new V3(0, 1, 0);
const col = (T, c) => new V3(T[0][c], T[1][c], T[2][c]);
const sub = (k) => `<sub>${k}</sub>`;

// Joint-variable offsets used for the "other poses" test (deg for R, × size for P).
const TEST_POSES = [
  [37, -52, 23, 61, -18, 44, -29, 15],
  [-64, 28, -41, -12, 57, -33, 49, -21],
  [12, 71, -66, 38, -47, 26, -15, 58],
];

export function createCompare(ctx) {
  const { state, scene, COLORS, makeArrow, makeLabel, clearGroup, short } = ctx;
  const $ = (id) => document.getElementById(id);
  const ui = {
    panel: $('cmp'), tableA: $('cmp-a'), tableB: $('cmp-b'), verdict: $('cmp-verdict'),
    frames: $('cmp-frames'),
  };

  const group = new THREE.Group();
  group.visible = false;
  scene.add(group);
  let active = false;

  const MAT_B = new THREE.MeshStandardMaterial({
    color: 0x2563eb, roughness: 0.5, transparent: true, opacity: 0.4, depthWrite: false,
  });
  const MAT_EE = new THREE.MeshBasicMaterial({ color: 0x2563eb });

  // ---------------------------------------------------------------------------
  // Kinematics and the equivalence test (real units)
  // ---------------------------------------------------------------------------
  function fk(joints, scale = 1, delta = null) {
    return forwardKinematics(joints.map((j, i) => ({
      theta: deg2rad(j.theta + (j.type === 'R' && delta ? delta[i] : 0)),
      d: (j.d + (j.type === 'P' && delta ? delta[i] : 0)) / scale,
      a: j.a / scale,
      alpha: deg2rad(j.alpha),
    })));
  }

  const ANGLE_TOL = deg2rad(0.1); // typed values are often rounded
  const sizeOf = (J) => 1 + J.reduce((s, j) => s + Math.abs(j.d) + Math.abs(j.a), 0);

  // Do A and B agree in one pose? Per joint: same axis line (±direction);
  // plus the end-effector point.
  function poseMatch(A, B, dA, dB) {
    const TA = fk(A, 1, dA), TB = fk(B, 1, dB);
    const n = A.length;
    const tol = 2e-3 * sizeOf(A);
    const axes = [];
    for (let i = 0; i < n; i++) {
      const pA = col(TA[i], 3), uA = col(TA[i], 2);
      const pB = col(TB[i], 3), uB = col(TB[i], 2);
      const c = uA.dot(uB);
      const parallel = Math.acos(Math.min(1, Math.abs(c))) < ANGLE_TOL;
      const dist = pB.clone().sub(pA).cross(uA).length();
      axes.push({ ok: parallel && dist < tol, sign: c < 0 ? -1 : 1 });
    }
    const eeOk = col(TA[n], 3).distanceTo(col(TB[n], 3)) < tol;
    return { axes, eeOk, ok: axes.every((a) => a.ok) && eeOk };
  }

  function evaluate() {
    const A = state.joints, B = state.cmpJoints;
    if (A.length !== B.length) {
      return { level: 'no', msg: `A has ${A.length} joint${A.length > 1 ? 's' : ''} and B has ${B.length}: they cannot describe the same robot.` };
    }
    const typeDiff = A.map((j, i) => (j.type !== B[i].type ? i + 1 : 0)).filter(Boolean);
    if (typeDiff.length) {
      return { level: 'no', msg: `Joint ${typeDiff.join(', ')} is revolute in one table and prismatic in the other.` };
    }
    const zero = A.map(() => 0);
    const now = poseMatch(A, B, zero, zero);
    if (!now.ok) {
      const bad = now.axes.map((a, i) => (a.ok ? 0 : i + 1)).filter(Boolean);
      const parts = [];
      if (bad.length) parts.push(`the axis of joint ${bad.join(', ')} is in a different place`);
      if (!now.eeOk) parts.push('the end-effector point is in a different place');
      return { level: 'no', now, msg: `Not the same robot in this pose: ${parts.join(' and ')}.` };
    }
    // Same now; does it stay the same when the joints move?
    const signs = now.axes.map((a) => a.sign);
    const size = sizeOf(A);
    for (const pose of TEST_POSES) {
      const dA = A.map((j, i) => (j.type === 'R' ? pose[i % pose.length] : (pose[i % pose.length] / 100) * size));
      const dB = dA.map((v, i) => v * signs[i]);
      if (!poseMatch(A, B, dA, dB).ok) {
        return { level: 'pose', now, msg: 'The two drawings coincide in this pose, but not once the joints move: the frames are chained differently, so this is a coincidence of the pose, not an equivalent table.' };
      }
    }
    // Joint-variable relations: q_B = s·q_A + c.
    const rel = A.map((j, i) => {
      const k = i + 1, s = signs[i];
      const q = j.type === 'R' ? 'θ' : 'd';
      const c = j.type === 'R' ? wrap(B[i].theta - s * j.theta) : B[i].d - s * j.d;
      const unit = j.type === 'R' ? '°' : '';
      const cTxt = Math.abs(c) < 1e-6 ? '' : ` ${c < 0 ? '−' : '+'} ${short(Math.abs(c))}${unit}`;
      return `${q}${sub(k)}<sup>B</sup> = ${s < 0 ? '−' : ''}${q}${sub(k)}<sup>A</sup>${cTxt}`;
    });
    const flipped = signs.map((s, i) => (s < 0 ? i + 1 : 0)).filter(Boolean);
    return {
      level: 'yes',
      now,
      msg: `Same robot: the joint axes and the end-effector point coincide in this pose and in ${TEST_POSES.length} other test poses.`
        + (flipped.length ? ` B's z axis points the other way at joint ${flipped.join(', ')}, so that joint variable changes sign.` : ''),
      rel,
    };
  }
  const wrap = (deg) => ((((deg + 180) % 360) + 360) % 360) - 180;

  // ---------------------------------------------------------------------------
  // Drawing (drawn units, current pose)
  // ---------------------------------------------------------------------------
  function rod(p0, p1, r, mat) {
    const len = p0.distanceTo(p1);
    if (len < 1e-4) return null;
    const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, 20), mat);
    m.quaternion.setFromUnitVectors(UP, p1.clone().sub(p0).normalize());
    m.position.copy(p0).lerp(p1, 0.5);
    group.add(m);
    return m;
  }

  function axisLine(p, u, color, opacity) {
    const l = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([p.clone().addScaledVector(u, -1.4), p.clone().addScaledVector(u, 1.4)]),
      new THREE.LineDashedMaterial({ color, dashSize: 0.09, gapSize: 0.06, transparent: true, opacity }),
    );
    l.computeLineDistances();
    group.add(l);
  }

  function label(html, pos, cls, color) {
    const l = makeLabel(html, cls, true);
    if (color) l.element.style.color = color;
    l.position.copy(pos);
    group.add(l);
  }

  function draw(result) {
    clearGroup(group);
    const A = state.joints, B = state.cmpJoints;
    const TA = fk(A, state.scale), TB = fk(B, state.scale);
    // A's joint axes (grey) — B's (green where they match A's, red where not).
    A.forEach((_, i) => axisLine(col(TA[i], 3), col(TA[i], 2), 0x64748b, 0.7));
    B.forEach((j, i) => {
      const O = col(TB[i], 3), z = col(TB[i], 2);
      const ok = result.now?.axes[i]?.ok;
      axisLine(O, z, ok ? 0x16a34a : 0xdc2626, 0.95);
      // B's robot: drum on the joint axis, then the d and a parts of the link.
      if (j.type === 'R') rod(O.clone().addScaledVector(z, -0.25), O.clone().addScaledVector(z, 0.25), 0.2, MAT_B);
      else rod(O.clone().addScaledVector(z, -0.3), O.clone().addScaledVector(z, 0.45), 0.2, MAT_B);
      const P = O.clone().addScaledVector(z, j.d / state.scale);
      rod(O, P, 0.09, MAT_B);
      rod(P, col(TB[i + 1], 3), 0.09, MAT_B);
    });
    const n = B.length;
    const eeB = col(TB[n], 3);
    const dot = new THREE.Mesh(new THREE.SphereGeometry(0.1, 18, 12), MAT_EE);
    dot.position.copy(eeB);
    group.add(dot);
    label('B', eeB.clone().add(new V3(0, 0, 0.32)), 'cmp-tag');
    // B's frames, primed to tell them apart from A's.
    if (ui.frames.checked) {
      for (let k = 0; k <= n; k++) {
        const O = col(TB[k], 3);
        for (const [kind, c] of [['x', 0], ['z', 2]]) {
          const dir = col(TB[k], c);
          const mat = new THREE.MeshStandardMaterial({ color: COLORS[kind], roughness: 0.4, transparent: true, opacity: 0.75, depthTest: false });
          const arrow = makeArrow(mat, 0.75, 0.026);
          arrow.traverse((m) => { m.renderOrder = 6 + (m.userData.arrowHead ? 0.5 : 0); }); // tip after shaft
          arrow.quaternion.setFromUnitVectors(UP, dir);
          arrow.position.copy(O);
          group.add(arrow);
          label(`${kind}′${sub(k)}`, O.clone().addScaledVector(dir, 0.92), `axis-label ${kind} cmp-axis`);
        }
      }
    }
  }

  // ---------------------------------------------------------------------------
  // Panel
  // ---------------------------------------------------------------------------
  const HEAD = '<tr><th>i</th><th>type</th><th>θ (°)</th><th>d</th><th>a</th><th>α (°)</th>';

  function renderA() {
    ui.tableA.innerHTML = `${HEAD}</tr>` + state.joints.map((j, i) =>
      `<tr><td>${i + 1}</td><td>${j.type}</td><td>${short(j.theta)}</td><td>${short(j.d)}</td><td>${short(j.a)}</td><td>${short(j.alpha)}</td></tr>`).join('');
  }

  // B is editable; rebuilt only when rows change so typing keeps focus.
  function renderB() {
    ui.tableB.innerHTML = `${HEAD}<th></th></tr>`;
    state.cmpJoints.forEach((j, i) => {
      const tr = document.createElement('tr');
      tr.innerHTML = `<td>${i + 1}</td>`;
      const tdType = document.createElement('td');
      const sel = document.createElement('select');
      sel.innerHTML = '<option value="R">R</option><option value="P">P</option>';
      sel.value = j.type;
      sel.addEventListener('change', () => { j.type = sel.value; refresh(); });
      tdType.append(sel);
      tr.append(tdType);
      for (const key of ['theta', 'd', 'a', 'alpha']) {
        const td = document.createElement('td');
        const inp = document.createElement('input');
        inp.type = 'number';
        inp.step = 'any';
        inp.value = j[key];
        inp.setAttribute('aria-label', `${key} ${i + 1}`);
        inp.addEventListener('input', () => {
          const v = parseFloat(inp.value);
          if (Number.isFinite(v)) { j[key] = v; refresh(); }
        });
        td.append(inp);
        tr.append(td);
      }
      const tdDel = document.createElement('td');
      const del = document.createElement('button');
      del.className = 'ghost';
      del.textContent = '✕';
      del.title = `Remove row ${i + 1}`;
      del.disabled = state.cmpJoints.length <= 1;
      del.addEventListener('click', () => { state.cmpJoints.splice(i, 1); renderB(); refresh(); });
      tdDel.append(del);
      tr.append(tdDel);
      ui.tableB.append(tr);
    });
  }

  function renderVerdict(r) {
    const icon = { yes: '✓', pose: '≈', no: '✗' }[r.level];
    const title = { yes: 'Equivalent DH tables', pose: 'Match in this pose only', no: 'Not equivalent' }[r.level];
    ui.verdict.className = `cmp-verdict ${r.level}`;
    ui.verdict.innerHTML = `<div class="cmp-verdict-title">${icon} ${title}</div><p>${r.msg}</p>`
      + (r.rel ? `<p class="cmp-rel">${r.rel.join('<br>')}</p>` : '');
  }

  function refresh() {
    if (!active) return;
    renderA();
    const r = evaluate();
    draw(r);
    renderVerdict(r);
  }

  const copyA = () => { state.cmpJoints = JSON.parse(JSON.stringify(state.joints)); };

  function open() {
    if (active) return;
    active = true;
    if (!state.cmpJoints) copyA();
    ui.panel.hidden = false;
    group.visible = true;
    document.getElementById('compare').classList.add('active');
    renderB();
    refresh();
  }

  function close() {
    if (!active) return;
    active = false;
    ui.panel.hidden = true;
    group.visible = false;
    clearGroup(group);
    document.getElementById('compare').classList.remove('active');
  }

  $('compare').addEventListener('click', () => (active ? close() : open()));
  $('cmp-close').addEventListener('click', close);
  $('cmp-min').addEventListener('click', () => {
    const min = ui.panel.classList.toggle('min');
    $('cmp-min').textContent = min ? '+' : '–';
    $('cmp-min').title = min ? 'Show the tables' : 'Shrink the panel to its verdict';
  });
  $('cmp-copy').addEventListener('click', () => { copyA(); renderB(); refresh(); });
  $('cmp-add').addEventListener('click', () => {
    state.cmpJoints.push({ type: 'R', theta: 0, d: 0, a: 0, alpha: 0 });
    renderB();
    refresh();
  });
  ui.frames.addEventListener('change', refresh);

  return { open, close, refresh, isActive: () => active, evaluate };
}
