import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {createCadKinematics} from '../lib/sr6/cad-kinematics.ts';
import {createRobotDynamics} from '../lib/sr6/rigid-dynamics.ts';
import {ZERO} from '../lib/sr6/kinematics.ts';

const read=name=>JSON.parse(readFileSync(new URL(`../public/${name}.json`,import.meta.url),'utf8'));
const baseline=read('sr6-cad'),variant=read('sr6-6010-cad');
const massA=read('sr6-mass-properties'),massB=read('sr6-6010-mass-properties');
assert.deepEqual(variant.spec.links,baseline.spec.links);
assert.equal(variant.spec.variant_sources.baseline_sha256,baseline.sha256);
const replaced=new Set(Object.keys(variant.spec.variant_sources.replacement_map).map(Number));
assert.equal(replaced.size,13);
let preserved=0;
for(const p of baseline.parts){
  if(replaced.has(p.id))assert.notDeepEqual(variant.parts[p.id],p);
  else {assert.deepEqual(variant.parts[p.id],p);preserved++;}
}
assert.equal(preserved,65);
for(const data of [baseline,variant]){
  const actual=createHash('sha256').update(readFileSync(new URL(`../cad/${data.source}`,import.meta.url))).digest('hex');
  assert.equal(data.sha256,actual);
}
const solverA=createCadKinematics(baseline),solverB=createCadKinematics(variant);
const dynamicsA=createRobotDynamics(baseline,massA,solverA),dynamicsB=createRobotDynamics(variant,massB,solverB);
assert.ok(Math.abs(massA.includedMassKg-massB.includedMassKg)<1e-7);
for(const pose of [ZERO,...['x','y','z','roll','pitch','twist'].flatMap(key=>[-1,1].map(sign=>({...ZERO,[key]:sign*(key.length===1?10:.1)})))]){
  const a=solverA.inverse(pose),b=solverB.inverse(pose);
  assert.ok(a.valid&&b.valid);assert.deepEqual(a.q,b.q);
  assert.ok(solverB.evaluate(pose,b.q).max<1e-7);
  const ta=dynamicsA.torques(pose,a.q,110,10,null),tb=dynamicsB.torques(pose,b.q,110,10,null);
  assert.ok(ta.valid&&tb.valid);
  for(let i=0;i<6;i++)assert.ok(Math.abs(ta.total[i]-tb.total[i])<1e-6);
}
console.log('Both variants: STEP hashes, 13 replacements, 65 exact shared meshes, 13 pose cases and consistent moving-mass torques passed');
