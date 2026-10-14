import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as T from 'three';
import { ZERO } from '../lib/sr6/kinematics.ts';
import { createCadRig } from '../lib/sr6/cad-rig.ts';
import { createCadKinematics } from '../lib/sr6/cad-kinematics.ts';
import { createRobotDynamics, bodyWrench } from '../lib/sr6/rigid-dynamics.ts';
import { aggregateRigidBodies, validateMassProperties } from '../lib/sr6/mass-properties.ts';
import { GRAVITY_MPS2 } from '../lib/sr6/statics.ts';
import { solveStick } from '../lib/sr6/virtual-stick.ts';

const data=JSON.parse(fs.readFileSync(new URL('../public/sr6-cad.json',import.meta.url),'utf8'));
const masses=JSON.parse(fs.readFileSync(new URL('../public/sr6-mass-properties.json',import.meta.url),'utf8'));
const close=(a,b,tol=1e-8)=>assert.ok(Math.abs(a-b)<=tol,`${a} != ${b}`);
validateMassProperties(masses,data.sha256);
assert.throws(()=>validateMassProperties(masses,'different-revision'));
close(masses.parts.find(p=>p.id===39).massKg,.388);
assert.equal(masses.includedCount,64);
assert.ok(!masses.parts.some(p=>p.material==='al5052'),'All aluminum is modeled and colored as 6061');
assert.equal(masses.parts.find(p=>p.id===49).material,'al6061');
assert.ok(masses.parts.filter(p=>p.included).every(p=>p.motion!=='fixed'));
assert.equal(masses.virtualStickMassKg,0);
const solver=createCadKinematics(data),model=createRobotDynamics(data,masses,solver);
const bodies=aggregateRigidBodies(masses);
assert.equal(bodies.length,14);
close(bodies.reduce((s,b)=>s+b.massKg,0),masses.includedMassKg);

// Exact two-point-mass parallel-axis reference (separation 2 m).
const pair={parts:[-1,1].map((x,id)=>({id,motion:'test',included:true,massKg:2,centerMm:[x*1000,0,0],inertiaCentroidKgM2:[[0,0,0],[0,0,0],[0,0,0]]}))};
const merged=aggregateRigidBodies(pair)[0];
close(merged.massKg,4);close(merged.centerMm.length(),0);
assert.deepEqual(merged.inertia.elements,[0,0,0,0,4,0,0,0,4]);

const unit=[new T.Vector3(1,0,0),new T.Vector3(0,1,0),new T.Vector3(0,0,1)];
const zero=new T.Vector3();
const isolated=bodyWrench(4,new T.Matrix3().set(1,0,0,0,2,0,0,0,3),
  {acceleration:[2,-1,3],omega:[1,2,3],alpha:[.1,.2,.3]},
  {linear:[...unit,zero,zero],angular:[zero,zero,zero,unit[0],unit[1]]});
isolated.gravity.forEach((v,i)=>close(v,i===0?4*GRAVITY_MPS2:0));
[8,-4,12,6.1,-5.6].forEach((v,i)=>close(isolated.inertia[i],v));

const meshes=new Map(data.parts.map(p=>[p.id,new T.Mesh()]));
const rig=createCadRig(data,meshes);
const poses=Array.from({length:12},(_,i)=>({...ZERO,x:4*Math.sin(i),y:5*Math.cos(i),z:3*Math.sin(i*.7),roll:.08*Math.sin(i*.4),pitch:.07*Math.cos(i*.5),twist:.2*Math.sin(i)}));
for(const pose of poses){
  const ik=solver.inverse(pose);assert.ok(ik.valid);assert.ok(rig.update(pose,ik.q).valid);
  const matrices=model.transforms(pose,ik.q);
  for(const p of masses.parts.filter(p=>p.included)){
    const actual=meshes.get(p.id).matrix.elements,expected=matrices.get(p.motion).elements;
    actual.forEach((v,i)=>close(v,expected[i],1e-8));
  }
  // Gravity is the independent derivative of potential energy built using the
  // rendering rig and each original part, not the dynamics' body Jacobians.
  function potential(p){
    const q=solver.inverse(p,ik.q);assert.ok(q.valid);assert.ok(rig.update(p,q.q).valid);
    return masses.parts.filter(part=>part.included).reduce((sum,part)=>
      sum+part.massKg*GRAVITY_MPS2*new T.Vector3(...part.centerMm).applyMatrix4(meshes.get(part.id).matrix).x*.001,0);
  }
  const loads=model.generalizedLoads(pose,ik.q,220,0,null);
  for(const [column,key] of ['x','y','z','roll','pitch'].entries()){
    const h=column<3?.01:1e-5;
    close(loads.gravity[column],(potential({...pose,[key]:pose[key]+h})-potential({...pose,[key]:pose[key]-h}))/(2*h),2e-7);
    close(loads.inertia[column],0);
  }
  const result=model.torques(pose,ik.q,220,100,null);assert.ok(result.valid,result.reason);
  const j=solver.inverseJacobian(pose,ik.q).values;
  const all=model.generalizedLoads(pose,ik.q,220,100,null);
  for(let c=0;c<5;c++)close(j.reduce((s,row,i)=>s+row[c]*result.gravity[i],0),all.gravity[c],1e-4);
}

// Static gravity survives zero payload; moving structural masses produce roll
// inertia even while the compensated massless rod tip remains stationary.
const initial=solver.inverse(ZERO);
const noPayload=model.torques(ZERO,initial.q,220,0,null);
assert.ok(noPayload.gravity.some(v=>Math.abs(v)>.01));
model.reset();let motion,solution,maxStructure=0,maxPayload=0;
for(let i=0;i<400;i++){
  const t=i/60;
  solution=solveStick({...ZERO,roll:.15*Math.sin(t*2)},{enabled:true,length:220},p=>solver.inverse(p));
  assert.ok(solution.valid);
  motion=model.sample(solution.receiver,solution.q,220,1000+t*1000);
  maxPayload=Math.max(maxPayload,Math.hypot(...(motion.payload?.acceleration??[0,0,0])));
  const result=model.torques(solution.receiver,solution.q,220,100,motion);assert.ok(result.valid,result.reason);
  maxStructure=Math.max(maxStructure,...result.inertia.map(Math.abs));
}
assert.ok(maxPayload<1e-8,`Compensated tip should stay stationary: ${maxPayload}`);
assert.ok(maxStructure>.001,`Moving mechanism needs inertia compensation: ${maxStructure}`);
const frozen=structuredClone(motion),before=model.torques(solution.receiver,solution.q,220,100,motion);
model.reset(); // Pause resets history, not the immutable rendered snapshot.
assert.deepEqual(motion,frozen);
assert.deepEqual(model.torques(solution.receiver,solution.q,220,100,motion),before);
assert.deepEqual(model.sample(solution.receiver,solution.q,220,99999),{});

// With compensation off, the tip moves during roll and its inertia is included.
model.reset();let movingTip=0;
for(let i=0;i<200;i++){
  const pose={...ZERO,roll:.15*Math.sin(i/30)},q=solver.inverse(pose).q;
  const motion=model.sample(pose,q,220,1000+i*1000/60);
  movingTip=Math.max(movingTip,Math.hypot(...(motion.payload?.acceleration??[0,0,0])));
}
assert.ok(movingTip>.05);
console.log(`Rigid dynamics: 64 parts / 14 bodies, STEP transforms, parallel-axis theorem, gravity-energy gradient, gyroscopic torque, tip payload and frozen pause passed. Peak structural roll inertia ${maxStructure.toFixed(4)} N m.`);
