import assert from 'node:assert/strict';
import fs from 'node:fs';
import { execFileSync } from 'node:child_process';
import { ZERO, inverse } from '../lib/sr6/kinematics.ts';
import { createCadKinematics } from '../lib/sr6/cad-kinematics.ts';
import { GRAVITY_MPS2, NO_ACCELERATION, numericalInverseJacobian, receiverTorques, staticForceAnalysis, torqueExpression, torqueNumber } from '../lib/sr6/statics.ts';
import { createPayloadMotionEstimator, MAX_LINEAR_ACCELERATION } from '../lib/sr6/payload-motion.ts';
import { solveStick } from '../lib/sr6/virtual-stick.ts';

const close=(a,b,tolerance=1e-7)=>assert.ok(Math.abs(a-b)<=tolerance,`${a} != ${b} (tol ${tolerance})`);
const data=JSON.parse(fs.readFileSync(new URL('../public/sr6-cad.json',import.meta.url),'utf8'));
const cad=createCadKinematics(data);
const poses=[{...ZERO}];
for(let i=0;i<36;i++){
  const t=i*.27;
  poses.push({x:8*Math.sin(t),y:10*Math.cos(t),z:5*Math.sin(t*.7),roll:.1*Math.sin(t),pitch:.1*Math.cos(t),twist:.2*Math.sin(t)});
}
let samples=0;
for(const pose of poses){
  const ik=cad.inverse(pose);assert.ok(ik.valid);
  const j=cad.inverseJacobian(pose,ik.q);assert.ok(j.valid,j.reason);
  const numerical=numericalInverseJacobian(pose,p=>cad.inverse(p,ik.q));assert.ok(numerical.valid);
  j.values.forEach((row,i)=>row.forEach((v,k)=>close(v,numerical.values[i][k],1e-7)));
  const weight=GRAVITY_MPS2*2;
  const gravity=receiverTorques(j,weight),twice=receiverTorques(j,weight*2);
  assert.ok(gravity.valid);close(gravity.massKg,2);
  const up=receiverTorques(j,weight,[0,GRAVITY_MPS2,0]);
  const fall=receiverTorques(j,weight,[0,-GRAVITY_MPS2,0]);
  for(let motor=0;motor<6;motor++){
    close(gravity.inertia[motor],0);
    close(twice.gravity[motor],2*gravity.gravity[motor]);
    close(up.inertia[motor],gravity.gravity[motor]);
    close(up.total[motor],2*gravity.gravity[motor]);
    close(fall.total[motor],0);
  }
  // Verify the resulting shaft work against independent arbitrary virtual motion.
  const acceleration=[1.3,-.8,2.1];
  const load=[2*acceleration[0]*.001,(weight+2*acceleration[1])*.001,2*acceleration[2]*.001,0,0];
  const both=receiverTorques(j,weight,acceleration);assert.ok(both.valid);
  const combined=staticForceAnalysis(j.values,load);assert.ok(combined.valid);
  both.total.forEach((v,i)=>close(v,combined.torques[i]));
  for(let column=0;column<5;column++){
    const achieved=j.values.reduce((sum,row,i)=>sum+row[column]*both.total[i],0);
    close(achieved,load[column],2e-5);
  }
  const zero=receiverTorques(j,0,acceleration);assert.ok(zero.total.every(v=>v===0));
  samples++;
}
assert.equal(staticForceAnalysis(Array.from({length:6},()=>[0,0,0,0,0]),[0,1,0,0,0]).valid,false);
assert.equal(receiverTorques({valid:true,values:[],reason:''},-1).valid,false);
assert.equal(receiverTorques({valid:false,values:[],reason:'unreachable'},10).valid,false);
assert.equal(cad.inverseJacobian({...ZERO,x:5000},Array(6).fill(0)).valid,false);
assert.equal(torqueExpression({valid:true,total:[.1],gravity:[.2],inertia:[-.1]},0),'+000.100(+000.200-000.100)');
for(const value of [0,-0,.00001,-.00001,.1,-.1,9.999,10,999.999,1000,1e100,Number.MAX_VALUE,NaN])assert.equal(torqueNumber(value).length,8);
assert.equal(torqueNumber(.00001),'+1.0e-05');
assert.equal(torqueExpression({valid:false},0).length,26);

// Compare the filtered acceleration against the firmware algorithm's behavior.
const estimator=createPayloadMotionEstimator();
for(let i=0;i<150;i++)assert.deepEqual(estimator.update(ZERO,1000+i*10),NO_ACCELERATION);
estimator.reset();
let last;
for(let i=0;i<=200;i++){
  const t=i*.01;
  last=estimator.update({...ZERO,y:.5*2*t*t*1000},1000+i*10);
}
close(last[1],2,1e-5);close(last[0],0);close(last[2],0);
// Same script at 2x speed, sampled in real seconds: acceleration is 4x.
estimator.reset();
for(let i=0;i<=200;i++){
  const scriptTime=i*.01*2;
  last=estimator.update({...ZERO,y:.5*2*scriptTime*scriptTime*1000},1000+i*10);
}
close(last[1],8,1e-5);
estimator.reset();
for(let i=0;i<=200;i++)last=estimator.update({...ZERO,x:i*.5},1000+i*10);
close(last[0],0,1e-8); // constant velocity settles to zero inertial compensation
estimator.reset();
assert.deepEqual(estimator.update({...ZERO,y:500},5000),NO_ACCELERATION);
assert.deepEqual(estimator.update({...ZERO,y:-500},6000),NO_ACCELERATION); // gap resets instead of spiking
last=estimator.update({...ZERO,y:10000},6010);
assert.ok(last.every(v=>Math.abs(v)<=MAX_LINEAR_ACCELERATION));
estimator.reset();assert.deepEqual(estimator.update(ZERO,100),NO_ACCELERATION);

// Pure angular tip commands still accelerate the actual receiver under stick compensation.
estimator.reset();let maximum=0;
for(let i=0;i<150;i++){
  const target={...ZERO,roll:.08*Math.sin(i*.03)};
  const result=solveStick(target,{enabled:true,length:220},p=>cad.inverse(p));
  assert.ok(result.valid);
  const a=estimator.update(result.receiver,1000+i*10);
  maximum=Math.max(maximum,Math.hypot(...a));
}
assert.ok(maximum>.01,'Use receiver acceleration, not a stationary commanded tip');

// Browser CAD frames often exceed the firmware's 50 ms watchdog interval.
for(const intervalMs of [60,100,200,500]){
  estimator.reset();
  for(let i=0;i<=100;i++){
    const t=i*intervalMs*.001;
    last=estimator.update({...ZERO,y:1000*t*t},1000+i*intervalMs);
  }
  close(last[1],2,1e-5);
  assert.equal(estimator.getStatus().state,'tracking');
  estimator.reset();let peak=0;
  for(let i=0;i<100;i++){
    const target={...ZERO,roll:.15*Math.sin(i*intervalMs*.001*Math.PI)};
    const result=solveStick(target,{enabled:true,length:220},p=>cad.inverse(p));
    assert.ok(result.valid);
    const a=estimator.update(result.receiver,1000+i*intervalMs);
    const torques=receiverTorques(cad.inverseJacobian(result.receiver,result.q),100,a);
    assert.ok(torques.valid);
    peak=Math.max(peak,...torques.inertia.map(Math.abs));
  }
  assert.ok(peak>.01,`Compensated roll inertia must remain visible at ${intervalMs} ms/frame`);
}
estimator.update(ZERO,100000);
assert.equal(estimator.getStatus().state,'gap');
estimator.reset();assert.equal(estimator.getStatus().state,'idle');

// Optional C++ parity check after building scripts/statics-reference.cpp.
const executable=process.argv[2];
if(executable){
  const input=poses.map(p=>[p.x,p.y,p.z,p.roll,p.pitch,p.twist,19.6133,1.3,-.8,2.1].join(' ')).join('\n')+'\n';
  const reference=execFileSync(executable,[],{input,encoding:'utf8'}).trim().split(/\r?\n/);
  assert.equal(reference.length,poses.length);
  for(const [index,line] of reference.entries()){
    const [g,a,...values]=line.split(/\s+/).map(Number);assert.equal(g,1);assert.equal(a,1);
    const j=numericalInverseJacobian(poses[index],inverse);
    const result=receiverTorques(j,19.6133,[1.3,-.8,2.1]);assert.ok(result.valid);
    [...result.gravity,...result.inertia].forEach((v,i)=>close(v,values[i],5e-4+Math.abs(values[i])*2e-4));
  }
  console.log(`${poses.length} cases matched the unmodified firmware C++ static/gravity solver.`);
}
console.log(`${samples} CAD poses: analytical Jacobian, virtual work, gravity/inertia superposition, free fall, mass scaling, zero load, invalid geometry and filtered receiver acceleration passed.`);
