import assert from 'node:assert/strict';
import {readFileSync,writeFileSync} from 'node:fs';
import * as T from 'three';
import {ZERO} from '../lib/sr6/kinematics.ts';
import {createCadKinematics} from '../lib/sr6/cad-kinematics.ts';
import {createRobotDynamics} from '../lib/sr6/rigid-dynamics.ts';
const root=new URL('../',import.meta.url);
const read=p=>JSON.parse(readFileSync(new URL(p,root),'utf8'));
const data=read('public/sr6-6010-cad.json'),masses=read('public/sr6-6010-mass-properties.json');
const solver=createCadKinematics(data),model=createRobotDynamics(data,masses,solver);
function at(y,h){
  const pose={...ZERO,y},q=solver.inverse(pose);assert.ok(q.valid,`L0 at ${y}: ${q.reason}`);
  const states=[y-h,y,y+h].map(y=>{
    const pose={...ZERO,y},q=solver.inverse(pose);assert.ok(q.valid);
    return model.bodyPoses(pose,q.q,0);
  });
  const result={receiver:0,otherMoving:0};
  for(const body of model.bodies){
    const [minus,current,plus]=states.map(s=>s.get(body.id));
    const v=plus.center.clone().sub(minus.center).multiplyScalar(1/(2*h*.001));
    const delta=plus.rotation.clone().multiply(minus.rotation.clone().invert());
    if(delta.w<0)delta.set(-delta.x,-delta.y,-delta.z,-delta.w);
    const n=Math.hypot(delta.x,delta.y,delta.z);
    const w=new T.Vector3(delta.x,delta.y,delta.z).multiplyScalar((n<1e-12?2:2*Math.atan2(n,delta.w)/n)/(2*h*.001));
    const r=new T.Matrix3().setFromMatrix4(new T.Matrix4().makeRotationFromQuaternion(current.rotation));
    const inertia=r.clone().multiply(body.inertia).multiply(r.clone().transpose());
    const group=['platform','twist'].includes(body.id)?'receiver':'otherMoving';
    result[group]+=body.massKg*v.lengthSq()+w.dot(w.clone().applyMatrix3(inertia));
  }
  return result;
}
const rows=[];
for(let i=0;i<=10;i++){
  const l0=i/10,y=-60+120*l0,a=at(y,.001),b=at(y,.0005);
  for(const k of Object.keys(a))assert.ok(Math.abs(a[k]-b[k])<1e-7);
  rows.push({l0,yMm:y,...a,total:a.receiver+a.otherMoving,otherPercent:100*a.otherMoving/(a.receiver+a.otherMoving)});
}
writeFileSync(new URL('cad/inertia-comparison-6010-l0.json',root),JSON.stringify({sha256:data.sha256,rangeMm:120,otherAxes:ZERO,payloadKg:0,unit:'kg',rows},null,2));
console.log('| L0 | 升降位置 mm | 接收器 kg | 非接收器活动部分 kg | 合计 kg | 非接收器占比 |');
console.log('|---:|---:|---:|---:|---:|---:|');
for(const r of rows)console.log(`| ${r.l0.toFixed(1)} | ${r.yMm.toFixed(0)} | ${r.receiver.toFixed(4)} | ${r.otherMoving.toFixed(4)} | ${r.total.toFixed(4)} | ${r.otherPercent.toFixed(1)}% |`);
