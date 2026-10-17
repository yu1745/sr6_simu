import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {turningPoints,septic,sampleBuffered,bufferedPoints} from '../lib/sr6/buffered-trajectory.ts';
import {createReactionHistory} from '../lib/sr6/reaction-history.ts';
import {createCadKinematics} from '../lib/sr6/cad-kinematics.ts';
import {createRobotDynamics} from '../lib/sr6/rigid-dynamics.ts';
import {ZERO} from '../lib/sr6/kinematics.ts';
const close=(a,b,tol=1e-8)=>assert.ok(Math.abs(a-b)<tol,`${a} != ${b}`);
// Fixture from MFP ProtocolAndPlanTests.InitialHoldAndTurningPointsMatchCtwPlanner.
const points=[[0,0],[20588,0],[21000,50],[22000,100],[23000,50],[24000,0]].map(([at,pos])=>({at,pos}));
assert.deepEqual(turningPoints(points).map(p=>p.at),[0,20588,22000,24000]);
for(const u of [0,1]){
  const s=septic(u);assert.equal(s.position,u);
  for(const key of ['velocity','acceleration','jerk'])assert.equal(s[key],0);
}
close(septic(.5).position,.5);close(septic(.5).velocity,2.1875);
for(let i=0;i<100;i++)assert.ok(septic((i+1)/100).position>=septic(i/100).position);
assert.equal(sampleBuffered(points,18000).position,0);
for(const at of [20588,22000,24000])for(const key of ['velocity','acceleration','jerk'])close(sampleBuffered(points,at)[key],0);
const normal=sampleBuffered(points,21200),fast=sampleBuffered(points,21200,2);
close(fast.position,normal.position);close(fast.velocity,normal.velocity*2);
close(fast.acceleration,normal.acceleration*4);close(fast.jerk,normal.jerk*8);
const long=bufferedPoints([{at:0,pos:0},{at:220000,pos:100}],.5);
for(let i=1;i<long.length;i++)assert.ok((long[i].at-long[i-1].at)/.5<=60000);
const history=createReactionHistory();let state={epoch:1,timeMs:0,playing:true};history.sync(state);
assert.ok(history.append({epoch:1,timeMs:10},[1,2,3,4,5,6],state));
state={...state,playing:false};
assert.equal(history.append({epoch:1,timeMs:20},[2,3,4,5,6,7],state),false);
assert.equal(history.points.length,1);assert.equal(history.points[0].timeMs,10);
state={...state,playing:true};assert.ok(history.append({epoch:1,timeMs:20},[2,3,4,5,6,7],state));
state={...state,epoch:2};assert.ok(history.sync(state));assert.equal(history.points.length,0);
assert.equal(history.append({epoch:1,timeMs:30},[1,2,3,4,5,6],state),false);
const read=p=>JSON.parse(readFileSync(new URL(p,import.meta.url),'utf8'));
const data=read('../public/sr6-6010-cad.json'),masses=read('../public/sr6-6010-mass-properties.json');
const solver=createCadKinematics(data),model=createRobotDynamics(data,masses,solver);
const strokes=[{at:0,pos:0},{at:1000,pos:100},{at:2000,pos:0}];
function motion(t,rate=1,h=.001){
  const sample=offset=>{
    const pose={...ZERO,y:(sampleBuffered(strokes,t+offset*1000*rate,rate).position/100-.5)*40};
    const q=solver.inverse(pose);assert.ok(q.valid);return {pose,q:q.q};
  };
  const center=sample(0);return model.referenceMotion(center.pose,center.q,110,sample,h);
}
// Receiver translates along STEP X. Analytical septic acceleration is independent
// of render FPS; full mechanism must settle at the reversal too.
close(motion(250).platform.acceleration[0],sampleBuffered(strokes,250).acceleration*.0004,1e-6);
close(motion(250,2).platform.acceleration[0],motion(250).platform.acceleration[0]*4,1e-5);
for(const body of Object.values(motion(1000))){
  assert.ok(Math.hypot(...body.acceleration)<1e-4);
  assert.ok(Math.hypot(...body.alpha)<1e-3);
  assert.ok(Math.hypot(...body.omega)<1e-6);
}
for(const [id,b] of Object.entries(motion(350)))for(let i=0;i<3;i++){
  close(b.acceleration[i],motion(350,1,.0005)[id].acceleration[i],1e-5);
  close(b.alpha[i],motion(350,1,.0005)[id].alpha[i],1e-4);
}
console.log('MFP turning-point fixture, septic endpoints/derivatives, speed scaling, long segments, pause/seek history, and reference mechanism dynamics passed.');
