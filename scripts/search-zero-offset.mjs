import fs from 'node:fs';
import * as T from 'three';
import {cadPlatform,createCadKinematics} from '../lib/sr6/cad-kinematics.ts';
import {receiverFromTip} from '../lib/sr6/virtual-stick.ts';

const data=JSON.parse(fs.readFileSync('public/sr6-cad.json','utf8'));
const normals=JSON.parse(fs.readFileSync('cad/joint-axis-normals.json','utf8'));
const baseline=JSON.parse(fs.readFileSync('cad/workspace-220mm-12_5deg-side-off.json','utf8'));
if(data.sha256!==baseline.cadSha256)throw Error('Stale baseline');
const solver=createCadKinematics(data),rad=Math.PI/180;
const targets=baseline.cases.map(c=>c.target);
const samples=targets.map(target=>{
  const pose=receiverFromTip({...target,roll:target.roll*rad,pitch:target.pitch*rad},220);
  const matrix=cadPlatform(pose),pin=new T.Vector3(0,0,1).transformDirection(matrix);
  return {target,pose,matrix,pin};
});
const links=[...data.spec.links].sort((a,b)=>a.motor-b.motor).map(l=>{
  const pivot=new T.Vector3(...l.pivot),lower=new T.Vector3(...l.lower),upper=new T.Vector3(...l.upper);
  return {...l,pivot,lower,upper,arm:lower.clone().sub(pivot),direction:upper.clone().sub(lower).normalize(),normal:new T.Vector3(...normals[l.motor].upper_eye)};
});
const z=new T.Vector3(0,0,1),qRod=new T.Quaternion();
const cos=Math.cos(19*rad);
function angles(sample,offset){
  const pose={...sample.pose,x:sample.pose.x+offset.x,y:sample.pose.y+offset.y,z:sample.pose.z+offset.z};
  const ik=solver.inverse(pose);if(!ik.valid)return null;
  const shift=new T.Vector3(offset.y,-offset.x,-offset.z);
  return links.map((l,i)=>{
    const tip=l.arm.clone().applyAxisAngle(z,ik.q[i]).add(l.pivot);
    const end=l.upper.clone().applyMatrix4(sample.matrix).add(shift);
    qRod.setFromUnitVectors(l.direction,end.sub(tip).normalize());
    return Math.abs(l.normal.clone().applyQuaternion(qRod).dot(sample.pin));
  });
}
// Validate the reduced geometry against every previously recorded rig pose.
samples.forEach((s,i)=>{
  const dots=angles(s,{x:0,y:0,z:0}),old=baseline.cases[i];
  if(Boolean(dots)!==old.valid)throw Error('Reachability differs from baseline');
  if(dots)dots.forEach((d,j)=>{
    const a=Math.acos(T.MathUtils.clamp(d,-1,1))/rad;
    if(Math.abs(a-old.jointAnglesDeg[j])>1e-6)throw Error('Joint-angle oracle differs from rig');
  });
});
const results=new Map();
function check(x,y,z){
  const key=[x,y,z].join(',');if(results.has(key))return results.get(key);
  const offset={x,y,z};let reachable=0,predictedCollisions=0,minDot=1;
  const sideCounts=[0,0],sideDots=[1,1];
  for(const s of samples){
    const dots=angles(s,offset);if(!dots)continue;
    reachable++;if(dots.some(d=>d<cos))predictedCollisions++;
    minDot=Math.min(minDot,...dots);
    for(let side=0;side<2;side++){const part=dots.slice(side*3,side*3+3);if(part.some(d=>d<cos))sideCounts[side]++;sideDots[side]=Math.min(sideDots[side],...part);}
  }
  const row={offset,sideCounts,sideMaxAngles:sideDots.map(d=>Math.acos(T.MathUtils.clamp(d,-1,1))/rad),reachable,unreachable:samples.length-reachable,predictedCollisions,maxJointAngleDeg:Math.acos(T.MathUtils.clamp(minDot,-1,1))/rad,offsetNormMm:Math.hypot(x,y,z)};
  results.set(key,row);return row;
}
const compare=(a,b)=>b.reachable-a.reachable||a.predictedCollisions-b.predictedCollisions||a.maxJointAngleDeg-b.maxJointAngleDeg||a.offsetNormMm-b.offsetNormMm;
// Symmetry constraint: no lateral zero shift. Exhaustive 5 mm translation grid.
for(let x=-80;x<=80;x+=5)for(let y=-80;y<=80;y+=5)check(x,y,0);
const byCount=(a,b)=>a.predictedCollisions-b.predictedCollisions||a.maxJointAngleDeg-b.maxJointAngleDeg;
const bySafe=(a,b)=>(b.reachable-b.predictedCollisions)-(a.reachable-a.predictedCollisions)||a.maxJointAngleDeg-b.maxJointAngleDeg;
const home=check(0,0,0);
const preservesBoth=r=>r.reachable>=home.reachable&&r.sideCounts.every((n,i)=>n<=home.sideCounts[i])&&r.sideMaxAngles.every((n,i)=>n<=home.sideMaxAngles[i]+1e-8);
const ranked=[...results.values()].sort(compare);
const bestReach=ranked[0].reachable;
const output={cadSha256:data.sha256,compensationMm:220,sideInputDisabled:true,baselineInputAxes:baseline.axes,
  definition:'receiver = full220mmCompensation(target) + fixed translation offset (Fwd,Thrust,Side); rotation unchanged. Original mounting axes, ball locations on receiver and original hardware. Target Side remains zero, fixed Side zero bias is prohibited; lateral symmetry preserved.',
  screening:'Original six upper-eye angles >19deg predict interference; rank by more reachable samples first, then fewer predicted collision samples, then lower worst angle. Actual solids must validate shortlist.',
  bounds:{x:[-80,80],y:[-80,80],z:[0,0]},offsetCandidates:ranked.length,posesPerCandidate:samples.length,
  baseline:home,bestBothSides:[...ranked].filter(preservesBoth).sort(byCount)[0],best:ranked[0],bestMaxAngle:ranked.filter(r=>r.reachable===bestReach).sort((a,b)=>a.maxJointAngleDeg-b.maxJointAngleDeg)[0],
  bestWithoutReachabilityLoss:[...ranked].filter(r=>r.reachable>=baseline.reachable).sort(byCount)[0],
  bestUsableSamples:[...ranked].sort(bySafe)[0],
  bestSmallOffset:ranked.filter(r=>Math.abs(r.offset.x)<=20&&Math.abs(r.offset.y)<=20&&Math.abs(r.offset.z)<=10)[0],
  singleAxis:Object.fromEntries(['x','y','z'].map(axis=>[axis,ranked.filter(r=>['x','y','z'].every(k=>k===axis||r.offset[k]===0))[0]])),
  ranked};
fs.writeFileSync('cad/zero-offset-search-220-symmetric.json',JSON.stringify(output,null,2));
console.log(JSON.stringify({...output,ranked:undefined},null,2));
