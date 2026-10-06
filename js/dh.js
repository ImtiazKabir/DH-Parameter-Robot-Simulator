// Pure Denavit–Hartenberg math (standard / distal convention). No Three.js here.
// Matrices are 4x4 row-major nested arrays: T[row][col].

export const deg2rad = (deg) => (deg * Math.PI) / 180;

export function identity() {
  return [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ];
}

export function matMul(A, B) {
  const C = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
  for (let r = 0; r < 4; r++) {
    for (let c = 0; c < 4; c++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += A[r][k] * B[k][c];
      C[r][c] = s;
    }
  }
  return C;
}

// A_i = Rz(theta) · Tz(d) · Tx(a) · Rx(alpha)   (angles in radians)
export function dhMatrix(theta, d, a, alpha) {
  const ct = Math.cos(theta), st = Math.sin(theta);
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  return [
    [ct, -st * ca,  st * sa, a * ct],
    [st,  ct * ca, -ct * sa, a * st],
    [0,   sa,       ca,      d],
    [0,   0,        0,       1],
  ];
}

// joints: [{theta, d, a, alpha}] in radians / length units.
// Returns [T_0^0, T_0^1, T_0^2, ..., T_0^n].
export function forwardKinematics(joints) {
  const Ts = [identity()];
  let T = identity();
  for (const j of joints) {
    T = matMul(T, dhMatrix(j.theta, j.d, j.a, j.alpha));
    Ts.push(T);
  }
  return Ts;
}

// Apply homogeneous transform T to a 3D point p = [x, y, z].
export function transformPoint(T, p) {
  return [0, 1, 2].map((r) => T[r][0] * p[0] + T[r][1] * p[1] + T[r][2] * p[2] + T[r][3]);
}
