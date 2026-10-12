/** Port of ctw_esp32s3_0909 IK and damped Gauss–Newton FK.
 * Public units: mm, radians. Firmware adapter: mm * 100, roll/pitch * 18000/PI.
 * Axis semantics: x=Fwd, y=Thrust, z=Side. Twist is an independent seventh axis.
 */
export type Pose = { x: number; y: number; z: number; roll: number; pitch: number; twist: number };
export const ZERO: Pose = { x: 0, y: 0, z: 0, roll: 0, pitch: 0, twist: 0 };
export const C = { arm: 105, rod: 262, pitchRod: 275, baseX: 212.8, baseY: 47.5, pitchY: 95, half: 100, comp: 84.75, offset: 80, bias: Math.PI / 12 };
export const NAMES = ["lowerLeft", "upperLeft", "pitchLeft", "pitchRight", "upperRight", "lowerRight"];
const Y = [1, -1, -1, -1, -1, 1], R = [1, -1, -1, 1, 1, -1], Z = [0, 0, 1, -1, 0, 0], ZR = [0, 0, -1, 1, 0, 0];
const keys = ["x", "y", "z", "roll", "pitch"] as const;
export function legArguments(p: Pose, i: number) {
  const pitch = i === 2 || i === 3;
  const a = C.bias - p.pitch;
  return {
    u: C.baseX - p.x + (pitch ? C.comp * Math.sin(a) : 0),
    v: C.baseY + (pitch ? C.pitchY : 0) + Y[i] * p.y + R[i] * C.half * Math.sin(p.roll) - (pitch ? C.comp * Math.cos(a) : 0),
    w: pitch ? C.offset + Z[i] * p.z + ZR[i] * C.comp * Math.sin(p.roll) : 0,
    length: pitch ? C.pitchRod : C.rod,
  };
}
export function inverse(p: Pose): { q: number[]; valid: boolean; reason: string } {
  if (!Object.values(p).every(Number.isFinite)) return { q: [], valid: false, reason: "输入包含非有限数值" };
  let valid = true;
  const q = NAMES.map((_, i) => {
    const { u, v, w, length } = legArguments(p, i);
    const distance = Math.hypot(u, v), radius2 = length ** 2 - w ** 2;
    const arg = (u * u + v * v + C.arm ** 2 - radius2) / (2 * C.arm * distance);
    // Preserve firmware's branch, but report an unreachable target before clamping.
    if (radius2 <= 0 || distance < 1e-9 || !Number.isFinite(arg) || Math.abs(arg) > 1 + 1e-10) valid = false;
    return Math.atan2(u, v) + Math.acos(Math.max(-1, Math.min(1, arg))) - Math.PI;
  });
  return { q, valid, reason: valid ? "" : "目标超出连杆几何可达范围" };
}
export function evaluate(p: Pose, q: number[]) {
  let valid = q.length === 6 && q.every(Number.isFinite) && Object.values(p).every(Number.isFinite);
  const residuals: number[] = [], jacobian: number[][] = [];
  for (let i = 0; i < 6; i++) {
    const a = legArguments(p, i), pitch = i === 2 || i === 3;
    const radius = Math.sqrt(a.length ** 2 - a.w ** 2);
    const dx = a.u + C.arm * Math.sin(q[i]), dy = a.v + C.arm * Math.cos(q[i]);
    const distance = Math.hypot(dx, dy);
    if (!(radius > 0) || !(distance > 1e-6)) valid = false;
    residuals.push(distance - radius);
    const du = [-1, 0, 0, 0, pitch ? -C.comp * Math.cos(C.bias - p.pitch) : 0];
    const dv = [0, Y[i], 0, R[i] * C.half * Math.cos(p.roll), pitch ? -C.comp * Math.sin(C.bias - p.pitch) : 0];
    const dr = [0, 0, pitch ? -a.w * Z[i] / radius : 0, pitch ? -a.w * ZR[i] * C.comp * Math.cos(p.roll) / radius : 0, 0];
    // Deliberately mirrors firmware's Jacobian expression, including radius/distance.
    jacobian.push(du.map((d, j) => (dx * d + dy * dv[j] - radius * dr[j]) / distance));
  }
  const max = Math.max(...residuals.map(Math.abs));
  const rms = Math.sqrt(residuals.reduce((s, r) => s + r * r, 0) / 6);
  return { valid: valid && Number.isFinite(max), residuals, jacobian, max, rms };
}
export function solveLinear(matrix: number[][], rhs: number[]) {
  const a = matrix.map((row, i) => [...row, rhs[i]]);
  for (let c = 0; c < 5; c++) {
    let pivot = c;
    for (let r = c + 1; r < 5; r++) if (Math.abs(a[r][c]) > Math.abs(a[pivot][c])) pivot = r;
    if (!Number.isFinite(a[pivot][c]) || Math.abs(a[pivot][c]) < 1e-12) return null;
    [a[c], a[pivot]] = [a[pivot], a[c]];
    const diagonal = a[c][c];
    for (let j = c; j < 6; j++) a[c][j] /= diagonal;
    for (let r = 0; r < 5; r++) if (r !== c) {
      const factor = a[r][c];
      for (let j = c; j < 6; j++) a[r][j] -= factor * a[c][j];
    }
  }
  const result = a.map((row) => row[5]);
  return result.every(Number.isFinite) ? result : null;
}
export function forward(q: number[], twist: number, seed: Pose = ZERO, tolerance = 1e-3, maxIterations = 32) {
  let pose = { ...seed, twist }, damping = 1e-4, iterations = 0;
  for (; iterations < maxIterations; iterations++) {
    const e = evaluate(pose, q);
    if (!e.valid || e.max <= tolerance) break;
    const norms = keys.map((_, j) => Math.hypot(...e.jacobian.map((row) => row[j])) || 1);
    const n = e.jacobian.map((row) => row.map((v, j) => v / norms[j]));
    const matrix = keys.map((_, i) => keys.map((__, j) => n.reduce((s, row) => s + row[i] * row[j], i === j ? damping : 0)));
    const rhs = keys.map((_, j) => -n.reduce((s, row, i) => s + row[j] * e.residuals[i], 0));
    const alpha = solveLinear(matrix, rhs);
    if (!alpha) { damping *= 10; if (damping > 1e6) break; continue; }
    const candidate = { ...pose };
    keys.forEach((key, j) => candidate[key] += alpha[j] / norms[j]);
    const next = evaluate(candidate, q);
    if (next.valid && next.rms < e.rms) { pose = candidate; damping = Math.max(1e-9, damping * 0.1); }
    else { damping *= 10; if (damping > 1e6) break; }
  }
  const e = evaluate(pose, q);
  return { pose, iterations, converged: e.valid && e.max <= tolerance, max: e.max, rms: e.rms };
}
