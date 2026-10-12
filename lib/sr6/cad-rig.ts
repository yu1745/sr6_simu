import * as T from 'three';
import { ZERO, type Pose } from './kinematics.ts';
import { cadPlatform } from './cad-kinematics.ts';

export type CadPart = { id: number; name: string; positions: number[]; indices: number[]; center: number[] };
export type CadData = { source: string; sha256: string; parts: CadPart[]; partFiles?: string[]; spec: {
  source_sha256: string; sha256: string;
  links: {motor: number; pivot: number[]; lower: number[]; upper: number[]}[];
} };
export type CadStatus = { valid: boolean; residual: number; angles: number[]; reason: string };

// Explicit mapping for this STEP revision; IDs are import order, not persistent CAD names.
const SOURCE_HASH = '313963a09a1765475e251350939f98f91b5c5edd8b0160401d6a5b7f204cfe94';
export const CAD_LINKS = [
  { motor: 0, crank: 15, baseX: 50, lower: 21, upper: 18, rod: [19, 20, 22] },
  { motor: 1, crank: 16, baseX: 145, lower: 26, upper: 23, rod: [24, 25, 27] },
  { motor: 2, crank: 58, baseX: 240, lower: 62, upper: 60, rod: [59, 61, 63] },
  { motor: 3, crank: 17, baseX: 240, lower: 52, upper: 50, rod: [49, 51, 53] },
  { motor: 4, crank: 14, baseX: 145, lower: 31, upper: 28, rod: [29, 30, 32] },
  { motor: 5, crank: 0, baseX: 50, lower: 36, upper: 33, rod: [34, 35, 37] },
] as const;
const PLATFORM = [38, 39, 40, 46, 54, 55, 56, 57, 64, 65, 66, 67, 68, 69, 70, 71];
const TWIST = [41, 42, 43, 44, 45, 47, 48];
const X = new T.Vector3(1, 0, 0), Z = new T.Vector3(0, 0, 1);
// Circular hole centers measured from the STEP B-rep, in its original mm axes.
const twistPivot = new T.Vector3(95.141985, 270.186359, -62.585786);
const wrap = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));

/** Rigid transform taking sourceAnchor to targetAnchor. No mesh scaling. */
function rigid(rotation: T.Quaternion, sourceAnchor: T.Vector3, targetAnchor: T.Vector3) {
  return new T.Matrix4().compose(
    targetAnchor.clone().sub(sourceAnchor.clone().applyQuaternion(rotation)),
    rotation, new T.Vector3(1, 1, 1),
  );
}

export function createCadRig(data: CadData, meshes: Map<number, T.Mesh>) {
  if (data.source !== 'sr6_mini_complete.stp' || data.spec?.source_sha256 !== SOURCE_HASH
    || data.sha256 !== data.spec.sha256 || data.parts.length !== 72 || data.spec.links.length !== 6)
    throw new Error('STEP 版本与关节标定不匹配，请重新标定');
  const centers = new Map(data.parts.map(p => [p.id, new T.Vector3(...p.center)]));
  const links = CAD_LINKS.map(link => {
    const measured = data.spec.links.find(l => l.motor === link.motor);
    const lower = measured ? new T.Vector3(...measured.lower) : centers.get(link.lower)!;
    const upper = measured ? new T.Vector3(...measured.upper) : centers.get(link.upper)!;
    const pivot = measured ? new T.Vector3(...measured.pivot) : new T.Vector3(link.baseX, 58.999984, lower.z);
    const arm = lower.clone().sub(pivot);
    const direction = upper.clone().sub(lower);
    return { ...link, lowerPart: link.lower, upperPart: link.upper, lower, upper, pivot, radius: Math.hypot(arm.x, arm.y),
      initialAngle: Math.atan2(arm.y, arm.x), length: direction.length(), direction: direction.normalize() };
  });
  for (const mesh of meshes.values()) mesh.matrixAutoUpdate = false;
  let previous = links.map(l => l.initialAngle);

  function update(pose: Pose, commandedAngles?: number[]): CadStatus {
    const platform = cadPlatform(pose);
    const matrices = new Map<number, T.Matrix4>();
    PLATFORM.forEach(id => matrices.set(id, platform));
    const twist = platform.clone().multiply(rigid(
      new T.Quaternion().setFromAxisAngle(X, pose.twist), twistPivot, twistPivot));
    TWIST.forEach(id => matrices.set(id, twist));
    const angles: number[] = [];
    let residual = 0;
    for (const [i, link] of links.entries()) {
      const end = link.upper.clone().applyMatrix4(platform);
      const delta = end.clone().sub(link.pivot);
      const distance = Math.hypot(delta.x, delta.y);
      const cosine = (delta.lengthSq() + link.radius ** 2 - link.length ** 2) / (2 * link.radius * distance);
      if (!Number.isFinite(cosine) || Math.abs(cosine) > 1 + 1e-9) {
        return { valid: false, residual: Infinity, angles: [],
          reason: `CAD M${link.motor + 1} 超出实测连杆可达范围，保留上一 CAD 位姿` };
      }
      const offset = Math.acos(T.MathUtils.clamp(cosine, -1, 1));
      const heading = Math.atan2(delta.y, delta.x);
      const candidates = [heading + offset, heading - offset];
      const branchSeed = commandedAngles ? link.initialAngle + commandedAngles[i] : previous[i];
      const angle = candidates.reduce((best, a) =>
        Math.abs(wrap(a - branchSeed)) < Math.abs(wrap(best - branchSeed)) ? a : best);
      angles.push(angle);
      const crankRotation = new T.Quaternion().setFromAxisAngle(Z, wrap(angle - link.initialAngle));
      const crank = rigid(crankRotation, link.pivot, link.pivot);
      const tip = link.lower.clone().applyMatrix4(crank);
      const direction = end.clone().sub(tip);
      residual = Math.max(residual, Math.abs(direction.length() - link.length));
      const rodRotation = new T.Quaternion().setFromUnitVectors(link.direction, direction.normalize());
      const rod = rigid(rodRotation, link.lower, tip);
      matrices.set(link.crank, crank);
      matrices.set(link.lowerPart, crank);
      matrices.set(link.upperPart, platform);
      link.rod.forEach(id => matrices.set(id, rod));
    }
    // Commit together only after every modeled linkage closes successfully.
    for (const [id, matrix] of matrices) {
      const mesh = meshes.get(id)!;
      mesh.matrix.copy(matrix);
      mesh.matrixWorldNeedsUpdate = true;
    }
    previous = angles;
    return { valid: true, residual, angles: angles.map((a, i) => wrap(a - links[i].initialAngle)), reason: '' };
  }
  return { update, reset: () => { previous = links.map(l => l.initialAngle); return update(ZERO); } };
}
