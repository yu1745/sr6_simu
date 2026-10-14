import * as T from 'three';
import { ZERO, solveLinear, type Pose } from './kinematics.ts';
import type { CadData } from './cad-rig.ts';
import type { InverseJacobian } from './statics.ts';

const keys = ['x', 'y', 'z', 'roll', 'pitch'] as const;
export const CAD_PLATFORM_PIVOT = [100.062295, 270.186359, -62.585786] as const;
const pivot = new T.Vector3(...CAD_PLATFORM_PIVOT);
const wrap = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));

// Upright display: STEP X = up, STEP -Y = forward, STEP -Z = side.
// R0 is the independent output shaft, rotating about STEP X.
export function cadPlatform(p: Pose) {
  const rotation = new T.Quaternion().setFromAxisAngle(new T.Vector3(0,1,0), p.roll)
    .multiply(new T.Quaternion().setFromAxisAngle(new T.Vector3(0,0,1), p.pitch));
  return new T.Matrix4().compose(
    pivot.clone().add(new T.Vector3(p.y,-p.x,-p.z)).sub(pivot.clone().applyQuaternion(rotation)),
    rotation, new T.Vector3(1,1,1));
}

export function createCadKinematics(data: CadData) {
  const links = [...data.spec.links].sort((a,b)=>a.motor-b.motor).map(l=>{
    const pivot = new T.Vector3(...l.pivot), lower = new T.Vector3(...l.lower);
    const upper = new T.Vector3(...l.upper), arm = lower.clone().sub(pivot);
    return {pivot, lower, upper, radius:Math.hypot(arm.x,arm.y),
      initial:Math.atan2(arm.y,arm.x), length:lower.distanceTo(upper)};
  });
  function inverse(p: Pose, seed = Array(6).fill(0) as number[]) {
    if (!Object.values(p).every(Number.isFinite)) return {q:[],valid:false,reason:'输入包含非有限数值'};
    const matrix = cadPlatform(p), q: number[] = [];
    for (const [i,l] of links.entries()) {
      const d = l.upper.clone().applyMatrix4(matrix).sub(l.pivot);
      const c = (d.lengthSq()+l.radius**2-l.length**2)/(2*l.radius*Math.hypot(d.x,d.y));
      if (!Number.isFinite(c) || Math.abs(c)>1+1e-9)
        return {q:[],valid:false,reason:`CAD M${i+1} 超出实测连杆可达范围`};
      const heading = Math.atan2(d.y,d.x), offset = Math.acos(T.MathUtils.clamp(c,-1,1));
      const candidates = [wrap(heading+offset-l.initial),wrap(heading-offset-l.initial)];
      q.push(candidates.reduce((a,b)=>Math.abs(wrap(a-seed[i]))<Math.abs(wrap(b-seed[i]))?a:b));
    }
    return {q,valid:true,reason:''};
  }
  function evaluate(p: Pose, q: number[]) {
    if (q.length!==6 || !q.every(Number.isFinite) || !Object.values(p).every(Number.isFinite))
      return {valid:false,residuals:Array(6).fill(Infinity) as number[],max:Infinity,rms:Infinity};
    const matrix = cadPlatform(p);
    const residuals = links.map((l,i)=>{
      const a = l.initial+q[i];
      const lower = l.pivot.clone().add(new T.Vector3(l.radius*Math.cos(a),l.radius*Math.sin(a),l.lower.z-l.pivot.z));
      return lower.distanceTo(l.upper.clone().applyMatrix4(matrix))-l.length;
    });
    return {valid:residuals.every(Number.isFinite),residuals,max:Math.max(...residuals.map(Math.abs)),
      rms:Math.hypot(...residuals)/Math.sqrt(6)};
  }
  // Differentiate each rigid rod constraint: r·(dUpper - dLower)=0.
  // Thus dq_i/dp_j = r·(dUpper/dp_j) / r·(dLower/dq_i).
  // This is the inverse-kinematics Jacobian, not the closure residual Jacobian.
  function inverseJacobian(p:Pose,q:number[]):InverseJacobian{
    const closure=evaluate(p,q);
    if(!closure.valid||closure.max>1e-3)return {valid:false,values:[],reason:'静力分析需要已闭合的接收器位姿'};
    const matrix=cadPlatform(p),rotation=new T.Quaternion().setFromRotationMatrix(matrix);
    const rollAxis=new T.Vector3(0,1,0);
    const pitchAxis=new T.Vector3(0,0,1).applyAxisAngle(rollAxis,p.roll);
    const values:number[][]=[];
    for(const [i,link] of links.entries()){
      const a=link.initial+q[i];
      const lower=link.pivot.clone().add(new T.Vector3(link.radius*Math.cos(a),link.radius*Math.sin(a),link.lower.z-link.pivot.z));
      const r=link.upper.clone().applyMatrix4(matrix).sub(lower);
      const denominator=r.dot(new T.Vector3(-link.radius*Math.sin(a),link.radius*Math.cos(a),0));
      if(!Number.isFinite(denominator)||Math.abs(denominator)<1e-8)
        return {valid:false,values:[],reason:`M${i+1} 接近运动学奇异位形`};
      const offset=link.upper.clone().sub(pivot).applyQuaternion(rotation);
      const derivatives=[new T.Vector3(0,-1,0),new T.Vector3(1,0,0),new T.Vector3(0,0,-1),
        rollAxis.clone().cross(offset),pitchAxis.clone().cross(offset)];
      values.push(derivatives.map(d=>r.dot(d)/denominator));
    }
    return {valid:values.every(row=>row.every(Number.isFinite)),values,reason:''};
  }
  function forward(q: number[], twist: number, seed: Pose = ZERO) {
    let pose = {...seed,twist}, damping = 1e-4, iterations = 0;
    for (;iterations<60;iterations++) {
      const e = evaluate(pose,q);
      if (!e.valid || e.max<=1e-3) break;
      const columns = keys.map((key,j)=>{
        const h = j<3 ? 1e-3 : 1e-6;
        const a = evaluate({...pose,[key]:pose[key]+h},q);
        const b = evaluate({...pose,[key]:pose[key]-h},q);
        return a.residuals.map((v,i)=>(v-b.residuals[i])/(2*h));
      });
      const norms = columns.map(c=>Math.hypot(...c)||1);
      const n = e.residuals.map((_,i)=>columns.map((c,j)=>c[i]/norms[j]));
      const matrix = keys.map((_,i)=>keys.map((__,j)=>n.reduce((s,r)=>s+r[i]*r[j],i===j?damping:0)));
      const rhs = keys.map((_,j)=>-n.reduce((s,r,i)=>s+r[j]*e.residuals[i],0));
      const alpha = solveLinear(matrix,rhs);
      if (!alpha) {damping*=10;if(damping>1e6)break;continue;}
      const candidate = {...pose};
      keys.forEach((key,j)=>candidate[key]+=alpha[j]/norms[j]);
      const next = evaluate(candidate,q);
      if(next.valid && next.rms<e.rms){pose=candidate;damping=Math.max(1e-9,damping*.1);}
      else {damping*=10;if(damping>1e6)break;}
    }
    const e = evaluate(pose,q);
    return {pose,iterations,converged:e.valid&&e.max<=1e-3,max:e.max,rms:e.rms};
  }
  return {inverse,forward,evaluate,inverseJacobian};
}
export type CadKinematics = ReturnType<typeof createCadKinematics>;
