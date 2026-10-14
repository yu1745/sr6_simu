// Independent sign check: measure a point on the CAD receiver after its actual
// rigid transform, rather than testing two mutually inverse stick formulas.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as T from 'three';
import { ZERO } from '../lib/sr6/kinematics.ts';
import { createCadKinematics } from '../lib/sr6/cad-kinematics.ts';
import { createCadRig } from '../lib/sr6/cad-rig.ts';
import { solveStick, tipFromReceiver } from '../lib/sr6/virtual-stick.ts';
import { createStickVisual } from '../lib/sr6/stick-visual.ts';

const data = JSON.parse(fs.readFileSync(new URL('../public/sr6-cad.json', import.meta.url), 'utf8'));
const meshes = new Map(data.parts.map(p => [p.id, new T.Mesh()]));
const rig = createCadRig(data, meshes), solver = createCadKinematics(data);
const length = 220, rad = Math.PI / 180;
// User-confirmed mounting: STEP -X is the neutral rod direction.
// Keep this independent of the rendering/compensation formulas under test.
const pivot = new T.Vector3(100.062295, 270.186359, -62.585786);
const neutralTip = pivot.clone().add(new T.Vector3(-length, 0, 0));
const poseDelta = v => ({ Fwd: -v.y, Thrust: v.x, Side: -v.z });
const rows = [], frames = [];
const visual = createStickVisual();
const frame=visual.group.getObjectByName('stick-tip-frame');
const directions=['X','Y','Z'].map(name=>new T.Vector3(0,1,0).applyQuaternion(frame.getObjectByName(name).quaternion));
assert.ok(directions[0].clone().cross(directions[1]).distanceTo(directions[2])<1e-10,'Tip axes must form a right-handed frame');
for (const axis of ['roll', 'pitch']) {
  for (const degrees of [10, -10]) {
    const target = { ...ZERO, [axis]: degrees * rad };
    for (const enabled of [false, true]) {
      const result = solveStick(target, { enabled, length }, p => solver.inverse(p));
      assert.ok(result.valid, result.reason);
      assert.equal(result.scale, 1, 'Use fully reachable cases to isolate the sign');
      const status = rig.update(result.receiver, result.q);
      assert.ok(status.valid, status.reason);
      assert.ok(status.residual < 1e-7);
      const matrix = meshes.get(38).matrix;
      const movedTip = neutralTip.clone().applyMatrix4(matrix);
      visual.update(matrix,length,meshes.get(47).matrix);
      visual.group.updateMatrixWorld(true);
      const marker=visual.group.getObjectByName('stick-tip-frame');
      const rod=visual.group.getObjectByName('virtual-stick');
      assert.ok(marker.position.distanceTo(movedTip)<1e-9);
      const expectedRotation=new T.Quaternion().setFromRotationMatrix(meshes.get(47).matrix);
      assert.ok(marker.quaternion.angleTo(expectedRotation)<1e-7);
      assert.ok(marker.quaternion.angleTo(new T.Quaternion())>.1,'Roll/pitch must rotate the tip frame');
      const rodStart=new T.Vector3(0,-.5,0).applyMatrix4(rod.matrixWorld);
      const rodEnd=new T.Vector3(0,.5,0).applyMatrix4(rod.matrixWorld);
      assert.ok(rodStart.distanceTo(pivot.clone().applyMatrix4(matrix))<1e-9);
      assert.ok(rodEnd.distanceTo(movedTip)<1e-9);
      assert.ok(Math.abs(rodStart.distanceTo(rodEnd)-length)<1e-9);
      const error = movedTip.distanceTo(neutralTip);
      const delta = poseDelta(movedTip.clone().sub(neutralTip));
      if (enabled) {
        assert.ok(error < 1e-7, `${axis} ${degrees}: compensated tip drift ${error}`);
        const recovered = tipFromReceiver(result.receiver, length);
        for (const key of Object.keys(ZERO)) assert.ok(Math.abs(recovered[key] - target[key]) < 1e-10);
      } else {
        const lateral = axis === 'roll' ? delta.Side : delta.Fwd;
        assert.equal(Math.sign(lateral), Math.sign(degrees) * (axis === 'roll' ? -1 : 1));
        assert.ok(error > 38);
      }
      rows.push({ axis, degrees, compensation: enabled,
        receiverFwd: +result.receiver.x.toFixed(5),
        receiverThrust: +result.receiver.y.toFixed(5),
        receiverSide: +result.receiver.z.toFixed(5),
        tipFwd: +delta.Fwd.toFixed(5), tipThrust: +delta.Thrust.toFixed(5),
        tipSide: +delta.Side.toFixed(5), tipError: +error.toFixed(9) });
      const overlay=[];
      if(process.argv.includes('--render-data'))visual.group.traverse(object=>{
        if(object instanceof T.Mesh)overlay.push({positions:Array.from(object.geometry.attributes.position.array),
          indices:Array.from(object.geometry.index.array),elements:object.matrixWorld.toArray(),color:object.material.color.toArray()});
      });
      if (degrees === 10) frames.push({ axis, enabled, overlay,
        matrices: [...meshes].map(([id, mesh]) => ({ id, elements: mesh.matrix.toArray() })),
        rod: [pivot.clone().applyMatrix4(matrix).toArray(), movedTip.toArray()] });
    }
  }
}
for(const size of [1,300]){
  const matrix=meshes.get(38).matrix;
  visual.update(matrix,size,meshes.get(47).matrix);visual.group.updateMatrixWorld(true);
  const endpoint=pivot.clone().add(new T.Vector3(-size,0,0)).applyMatrix4(matrix);
  assert.ok(visual.group.getObjectByName('stick-tip-frame').position.distanceTo(endpoint)<1e-9);
}
// Check actual rendered axis directions, including R0 and a combined rotation.
for(const pose of [{...ZERO,twist:.4},{...ZERO,twist:-.4},{...ZERO,roll:.1,pitch:-.1,twist:.4},ZERO]){
  assert.ok(rig.update(pose).valid);
  const platform=meshes.get(38).matrix,output=meshes.get(47).matrix;
  visual.update(platform,length,output);visual.group.updateMatrixWorld(true);
  assert.ok(frame.position.distanceTo(neutralTip.clone().applyMatrix4(platform))<1e-9);
  for(const [i,name] of ['X','Y','Z'].entries()){
    const actual=new T.Vector3(0,1,0).applyQuaternion(frame.getObjectByName(name).getWorldQuaternion(new T.Quaternion()));
    assert.ok(actual.distanceTo(directions[i].clone().transformDirection(output))<1e-9);
  }
  if(pose.twist)assert.ok(frame.quaternion.angleTo(new T.Quaternion().setFromRotationMatrix(platform))>.3);
}
visual.dispose();
console.table(rows);
console.log('Passed: 8 roll/pitch compensation cases; rod lengths 1/220/300 mm; tip axes follow roll/pitch/R0, combined rotation and zero reset without displacing the tip.');
if (process.argv.includes('--render-data')) {
  const out = new URL('../work/stick-direction/', import.meta.url);
  fs.mkdirSync(out, { recursive: true });
  fs.writeFileSync(new URL('cases.json', out), JSON.stringify({ rows, frames, neutralRod: [pivot.toArray(), neutralTip.toArray()] }));
}
