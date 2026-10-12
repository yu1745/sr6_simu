import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { inverse,forward,evaluate,ZERO } from '../lib/sr6/kinematics.ts';
import { geometry,segmentDistance } from '../lib/sr6/geometry.ts';
const cases=[{...ZERO}];
for(let i=0;i<120;i++){
  const t=i/12;
  cases.push({x:14*Math.sin(t),y:29*Math.cos(t),z:12*Math.sin(t*.7),roll:.18*Math.sin(t*.8),pitch:.18*Math.cos(t*.6),twist:.3*Math.sin(t)});
}
let worst=0,angleDelta=0,referenceDelta=0;
const start=performance.now();
for(const p of cases){
  const ik=inverse(p);assert(ik.valid);
  assert(evaluate(p,ik.q).max<1e-9);
  const fk=forward(ik.q,p.twist);assert(fk.converged);
  worst=Math.max(worst,fk.max);
  for(const k of ['x','y','z'])assert(Math.abs(fk.pose[k]-p[k])<.01);
  for(const k of ['roll','pitch'])assert(Math.abs(fk.pose[k]-p[k])<.0001);
  assert.equal(fk.pose.twist,p.twist);
  const legs=geometry(p,ik.q);
  for(const l of legs)assert(Math.abs(Math.hypot(...l.end.map((x,j)=>x-l.tip[j]))-l.nominal)<1e-8);
  // STEP correspondence: right pitch runs from positive X toward positive X,
  // with mirrored left attachment. Regression for the wrong lateral sign.
  assert(legs[2].end[0]<legs[2].tip[0]);
  assert(legs[3].end[0]>legs[3].tip[0]);
  assert(segmentDistance(legs[2].tip,legs[2].end,legs[3].tip,legs[3].end)>8);
}
assert(!inverse({...ZERO,x:2000}).valid);
assert(!forward([NaN,0,0,0,0,0],0).converged);
assert(!forward([0,0,0,0,0,0],NaN).converged);
assert.equal(segmentDistance([0,0,0],[1,0,0],[0,2,0],[1,2,0]),2);
assert.equal(segmentDistance([0,0,0],[1,0,0],[.5,-1,0],[.5,1,0]),0);
const homeLegs=geometry(ZERO,inverse(ZERO).q);
assert.equal(homeLegs[2].tip[0],-13);
assert.equal(homeLegs[2].end[0],-93);
assert.equal(homeLegs[3].tip[0],13);
assert.equal(homeLegs[3].end[0],93);
const elapsed=performance.now()-start;
const ref=spawnSync(fileURLToPath(new URL('../.reference.exe',import.meta.url)),[],{input:cases.map(p=>Object.values(p).join(' ')).join('\n'),encoding:'utf8'});
if(ref.error)throw ref.error;
assert.equal(ref.status,0,ref.stderr);
const lines=ref.stdout.trim().split(/\r?\n/);assert.equal(lines.length,cases.length);
lines.forEach((line,i)=>{
  const values=line.split(/\s+/).map(Number), p=cases[i], ik=inverse(p), fk=forward(ik.q,p.twist);
  for(let j=0;j<6;j++)angleDelta=Math.max(angleDelta,Math.abs(ik.q[j]-values[j]));
  assert.equal(values[11],1);
  ['x','y','z'].forEach((k,j)=>referenceDelta=Math.max(referenceDelta,Math.abs(fk.pose[k]-values[6+j])));
});
assert(angleDelta<1e-6);assert(referenceDelta<.01);
console.log(JSON.stringify({poses:cases.length,worstClosureMm:worst,maxCppIkDeltaRad:angleDelta,maxCppFkDeltaMm:referenceDelta,nodeRoundTripMsPerPose:elapsed/cases.length},null,2));
