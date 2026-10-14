import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as T from 'three';
import { createCadRig, CAD_LINKS } from '../lib/sr6/cad-rig.ts';
import { ZERO } from '../lib/sr6/kinematics.ts';

const data = JSON.parse(fs.readFileSync(new URL('../public/sr6-cad.json', import.meta.url), 'utf8'));
const meshes = new Map(data.parts.map(p => [p.id, new T.Mesh()]));
const rig = createCadRig(data, meshes);
assert.equal(CAD_LINKS.length, 6);
assert.deepEqual(data.spec.links.map(l => l.motor), [0,1,2,3,4,5]);
assert.ok(Math.abs(data.spec.links[2].length-data.spec.links[3].length)<1e-8);
assert.equal(data.spec.receiver_fasteners.length, 4);
assert.ok(rig.reset().valid);
for (const mesh of meshes.values()) {
  const identity = new T.Matrix4();
  assert.ok(mesh.matrix.elements.every((v, i) => Math.abs(v - identity.elements[i]) < 1e-7));
}
const center = id => new T.Vector3(...data.parts.find(p => p.id === id).center);
const anchors = CAD_LINKS.map(link => {
  const spec = data.spec.links.find(l => l.motor === link.motor);
  const lower = spec ? new T.Vector3(...spec.lower) : center(link.lower);
  const upper = spec ? new T.Vector3(...spec.upper) : center(link.upper);
  return {link, lower, upper, length: lower.distanceTo(upper)};
});
let count = 0;
for (const key of ['x','y','z','roll','pitch','twist']) {
  for (const sign of [-1,1]) {
    const pose = {...ZERO, [key]: sign * (key.length === 1 ? 10 : 0.1)};
    const status = rig.update(pose);
    assert.ok(status.valid, status.reason);
    assert.ok(status.residual < 1e-7);
    assert.ok(meshes.get(key === 'twist' ? 47 : 38).matrix.elements.some((v,i) => Math.abs(v-new T.Matrix4().elements[i])>1e-5));
    for (const {link,lower,upper,length} of anchors) {
      const movedLower = lower.clone().applyMatrix4(meshes.get(link.lower).matrix);
      const movedUpper = upper.clone().applyMatrix4(meshes.get(link.upper).matrix);
      const rodMatrix = meshes.get(link.rod[0]).matrix;
      assert.ok(movedLower.distanceTo(lower.clone().applyMatrix4(rodMatrix)) < 1e-7);
      assert.ok(movedUpper.distanceTo(upper.clone().applyMatrix4(rodMatrix)) < 1e-7);
      assert.ok(Math.abs(movedLower.distanceTo(movedUpper)-length) < 1e-7);
      assert.ok(Math.abs(rodMatrix.determinant()-1) < 1e-10);
    }
    // Fixed base and six motor bodies stay still.
    for (const id of [1,2,4,6,8,10,12]) assert.deepEqual(meshes.get(id).matrix.elements,new T.Matrix4().elements);
    for (const id of [54,55,56,57,64,65,66,67,68,69,70,71,72,73,74,75,76,77]) assert.deepEqual(meshes.get(id).matrix.elements,meshes.get(38).matrix.elements);
    count++;
  }
}
const before = [...meshes].map(([id,m]) => [id, [...m.matrix.elements]]);
assert.equal(rig.update({...ZERO,x:5000}).valid, false);
for (const [id,matrix] of before) assert.deepEqual(meshes.get(id).matrix.elements,matrix);
assert.ok(rig.reset().valid);
console.log(`${count} motion cases: rigid length, both rod endpoints, fixed motors, R0, unreachable rollback and zero reset passed`);
