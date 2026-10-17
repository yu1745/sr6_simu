import { solveLinear, type Pose } from './kinematics.ts';

export const GRAVITY_MPS2 = 9.80665;
export type Acceleration = readonly [number,number,number]; // Fwd / Thrust / Side, m/s²
export const NO_ACCELERATION:Acceleration = [0,0,0];
export type InverseJacobian = {valid:boolean;values:number[][];reason:string};
// Force of the moving assembly on the fixed base, expressed in STEP axes.
// Moments are about the fixed base sheet (#1) center, not an individual mount.
export type BaseReaction = {
  originMm:readonly [number,number,number];
  gravity:readonly number[];inertia:readonly number[];total:readonly number[];
};
export type TorqueAnalysis = {
  valid:boolean;reason:string;gravity:number[];inertia:number[];total:number[];
  massKg:number;acceleration:Acceleration;relativeResidual:number;
  baseReaction?:BaseReaction;
};

// Port of ctw_esp32s3_0909/src/ik_static_analysis.cpp::staticForceAnalysis.
// J = dq/d(receiver pose), columns x/y/z in mm and roll/pitch in radians.
// Virtual work: J^T tau = wrench. Six motors actuate five platform DOFs;
// R0 is independent. Column scaling and damping match firmware (lambda=1e-3).
export function staticForceAnalysis(jacobian:number[][],wrench:readonly number[]){
  const invalid={valid:false,torques:Array(6).fill(0) as number[],relativeResidual:Infinity};
  if(jacobian.length!==6||jacobian.some(row=>row.length!==5||!row.every(Number.isFinite))||
    wrench.length!==5||!wrench.every(Number.isFinite))return invalid;
  const norms=wrench.map((_,j)=>Math.hypot(...jacobian.map(row=>row[j])));
  if(norms.some(v=>v<1e-8))return invalid;
  const scaled=jacobian.map(row=>row.map((v,j)=>v/norms[j]));
  const normal=wrench.map((_,i)=>wrench.map((__,j)=>scaled.reduce((sum,row)=>sum+row[i]*row[j],i===j?1e-6:0)));
  const load=wrench.map((v,j)=>v/norms[j]);
  const alpha=solveLinear(normal,load);
  if(!alpha)return invalid;
  const torques=scaled.map(row=>row.reduce((sum,v,j)=>sum+v*alpha[j],0));
  if(!torques.every(Number.isFinite))return invalid;
  const residual=load.map((v,j)=>scaled.reduce((sum,row,i)=>sum+row[j]*torques[i],0)-v);
  return {valid:true,torques,relativeResidual:Math.hypot(...residual)/Math.max(1e-12,Math.hypot(...load))};
}

// Firmware view uses the original closed-form IK in JS double precision.
// CAD has its own analytical inverse Jacobian, built from measured anchors.
export function numericalInverseJacobian(pose:Pose,solve:(p:Pose)=>{q:number[];valid:boolean}):InverseJacobian{
  const keys=['x','y','z','roll','pitch'] as const;
  const values=Array.from({length:6},()=>Array(5).fill(0) as number[]);
  for(const [j,key] of keys.entries()){
    const h=j<3?1e-3:1e-6;
    const plus=solve({...pose,[key]:pose[key]+h}),minus=solve({...pose,[key]:pose[key]-h});
    if(!plus.valid||!minus.valid||plus.q.length!==6||minus.q.length!==6)
      return {valid:false,values:[],reason:'静力雅可比邻域不可达'};
    for(let i=0;i<6;i++){
      const delta=plus.q[i]-minus.q[i];
      values[i][j]=Math.atan2(Math.sin(delta),Math.cos(delta))/(2*h);
    }
  }
  return {valid:values.every(row=>row.every(Number.isFinite)),values,reason:''};
}

/** Centered receiver model: weight in N, m=weight/g; F_inertia=m*a.
 * No COM offset, link/rotor mass, or assumed rotational inertia tensor. */
export function receiverTorques(jacobian:InverseJacobian,weightN:number,acceleration:Acceleration=NO_ACCELERATION):TorqueAnalysis{
  const massKg=weightN/GRAVITY_MPS2;
  const invalid=(reason:string):TorqueAnalysis=>({valid:false,reason,gravity:[],inertia:[],total:[],massKg,acceleration,relativeResidual:Infinity});
  if(!Number.isFinite(weightN)||weightN<0||acceleration.length!==3||!acceleration.every(Number.isFinite))
    return invalid('重力或加速度无效');
  if(!jacobian.valid)return invalid(jacobian.reason||'静力雅可比无效');
  // One translation unit here is 1 mm, so generalized force is N * 1e-3 m/mm.
  // Unlike the firmware's scaled pose unit (1e-5 m), angles already use rad.
  const gravity=staticForceAnalysis(jacobian.values,[0,weightN*1e-3,0,0,0]);
  const inertia=staticForceAnalysis(jacobian.values,[...acceleration.map(a=>massKg*a*1e-3),0,0]);
  if(!gravity.valid||!inertia.valid)return invalid('静力分配失败或机构接近奇异位形');
  return {valid:true,reason:'',gravity:gravity.torques,inertia:inertia.torques,
    total:gravity.torques.map((v,i)=>v+inertia.torques[i]),massKg,acceleration,
    relativeResidual:Math.max(gravity.relativeResidual,inertia.relativeResidual)};
}

export function torqueNumber(value:number){
  if(!Number.isFinite(value))return '   —    ';
  const magnitude=Math.abs(value)<1e-12?0:Math.abs(value),sign=value<0&&magnitude>0?'-':'+';
  const fixed=magnitude.toFixed(3);
  const scientific=magnitude>0&&magnitude<.001;
  const digits=!scientific&&fixed.length<=7?fixed.padStart(7,'0'):magnitude.toExponential(magnitude<1e100?1:0).replace(/e([+-])(\d)$/,'e$10$2').padStart(7,' ');
  return sign+digits;
}
export function torqueExpression(result:TorqueAnalysis,index:number){
  if(!result.valid)return '   —    (   —       —    )';
  const inertia=torqueNumber(result.inertia[index]);
  return `${torqueNumber(result.total[index])}(${torqueNumber(result.gravity[index])}${inertia})`;
}
