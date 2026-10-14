import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as T from 'three';
import { ZERO } from '../lib/sr6/kinematics.ts';
import { cadPlatform, createCadKinematics } from '../lib/sr6/cad-kinematics.ts';
import { createCadRig } from '../lib/sr6/cad-rig.ts';

const data = JSON.parse(fs.readFileSync(new URL('../public/sr6-cad.json', import.meta.url), 'utf8'));
const links = [...data.spec.links].sort((a,b) => a.motor-b.motor);
const solver = createCadKinematics(data);
const meshes = new Map(data.parts.map(p => [p.id, new T.Mesh()]));
const rig = createCadRig(data, meshes);
const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
const closeAngles = (a,b) => a.forEach((v,i) => assert.ok(Math.abs(wrap(v-b[i])) < 1e-8));
const cross = (a,b) => a.x*b.y-a.y*b.x;
// Independent geometric oracle: reflect each crank tip in the pivot-to-upper
// line to construct the other exact rod-length solution (including at home).
function reflected(pose, q) {
  return links.map((l,i) => {
    const pivot = new T.Vector3(...l.pivot);
    const arm = new T.Vector3(...l.lower).sub(pivot);
    const target = new T.Vector3(...l.upper).applyMatrix4(cadPlatform(pose)).sub(pivot);
    const initial = Math.atan2(arm.y,arm.x);
    return wrap(2*Math.atan2(target.y,target.x)-2*initial-q[i]);
  });
}
function check(pose, q) {
  assert.ok(solver.evaluate(pose,q).max < 1e-7, 'rod lengths must close');
  links.forEach((l,i) => {
    const pivot = new T.Vector3(...l.pivot);
    const arm = new T.Vector3(...l.lower).sub(pivot);
    const homeTarget = new T.Vector3(...l.upper).sub(pivot);
    const target = new T.Vector3(...l.upper).applyMatrix4(cadPlatform(pose)).sub(pivot);
    const tip = arm.clone().applyAxisAngle(new T.Vector3(0,0,1),q[i]);
    assert.ok(cross(homeTarget,arm)*cross(target,tip) >= -1e-6, `M${i+1} changed assembly side`);
  });
}
const wrongHome = reflected(ZERO,Array(6).fill(0));
assert.ok(solver.evaluate(ZERO,wrongHome).max < 1e-7, 'wrong branch also closes');
const recovered = solver.inverse(ZERO,wrongHome);
assert.ok(recovered.valid);
closeAngles(recovered.q,Array(6).fill(0));
check(ZERO,recovered.q);
assert.equal(solver.forward(wrongHome,0,ZERO).converged,false, 'FK must not accept the forbidden assembly');

// Direct seeks and large changes are not guaranteed to have a nearby seed.
let checked = 0;
for (const x of [-100,-30,0,30,100]) for (const y of [-180,-60,0,60,180]) {
  for (const pitch of [-.4,0,.4]) {
    const pose = {...ZERO,x,y,pitch,z:12,roll:.13};
    const solution = solver.inverse(pose);
    if (!solution.valid) continue;
    check(pose,solution.q);
    closeAngles(solver.inverse(pose,reflected(pose,solution.q)).q,solution.q);
    closeAngles(solver.inverse(pose,wrongHome.map(a=>a+4*Math.PI)).q,solution.q);
    const status = rig.update(pose);
    assert.ok(status.valid,status.reason);
    closeAngles(status.angles,solution.q);
    assert.ok(status.residual < 1e-7);
    assert.ok(rig.update(pose,solution.q.map(a=>a+2*Math.PI)).valid);
    const before = [...meshes.values()].map(m=>[...m.matrix.elements]);
    assert.equal(rig.update(pose,reflected(pose,solution.q)).valid,false);
    [...meshes.values()].forEach((m,i)=>assert.deepEqual(m.matrix.elements,before[i]));
    checked++;
  }
}
assert.ok(checked > 20);
let previous = Array(6).fill(0);
for (let i=0;i<=240;i++) {
  const t=i*Math.PI/120;
  const pose={...ZERO,x:25*Math.sin(t),y:55*Math.sin(t),z:20*Math.sin(t),roll:.3*Math.sin(t),pitch:.3*Math.sin(t)};
  const solution=solver.inverse(pose,previous);
  assert.ok(solution.valid);
  check(pose,solution.q);
  solution.q.forEach((a,j)=>assert.ok(Math.abs(wrap(a-previous[j]))<.1,'continuous trajectory must not jump'));
  const fk=solver.forward(solution.q,0,pose);
  assert.ok(fk.converged);
  previous=solution.q;
}
assert.ok(rig.reset().valid);
for (const mesh of meshes.values()) mesh.matrix.elements.forEach((v,i)=>assert.ok(Math.abs(v-new T.Matrix4().elements[i])<1e-7));
assert.equal(solver.inverse({...ZERO,x:5000}).valid,false);
assert.equal(solver.inverse({...ZERO,pitch:NaN}).valid,false);
assert.equal(rig.update({...ZERO,pitch:NaN}).valid,false);
assert.equal(rig.update(ZERO,[NaN]).valid,false);

// At full extension the two roots merge. Moving back must recover the same
// side even if the caller supplies the reflected branch as its next seed.
// Offset the crank plane in Z to exercise the projected rod-length equation.
const tangentSolver=createCadKinematics({spec:{links:Array.from({length:6},(_,motor)=>({
  motor,pivot:[0,0,0],lower:[0,1,3],upper:[2,0,3],
}))}});
const extension=1+Math.sqrt(5);
const tangentPose={...ZERO,y:extension-2};
const tangent=tangentSolver.inverse(tangentPose);
assert.ok(tangent.valid);
assert.ok(tangentSolver.evaluate(tangentPose,tangent.q).max<1e-7);
for(const gap of [1e-6,1e-4,.01]) {
  const pose={...tangentPose,y:tangentPose.y-gap};
  const result=tangentSolver.inverse(pose,tangent.q.map(a=>-Math.PI-a));
  assert.ok(result.valid);
  assert.ok(result.q.every(a=>Math.sin(a+Math.PI/2)>0));
  assert.ok(tangentSolver.evaluate(pose,result.q).max<1e-7);
}
assert.equal(tangentSolver.inverse({...tangentPose,y:tangentPose.y+.001}).valid,false);
console.log(`${checked} direct seeks and 241 trajectory frames: assembly side, wrong-seed recovery, FK rejection, rig agreement, closure and rollback passed`);
