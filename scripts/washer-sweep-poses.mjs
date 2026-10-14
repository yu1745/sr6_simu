import fs from 'node:fs';
import * as T from 'three';
import {createCadRig} from '../lib/sr6/cad-rig.ts';
import {ZERO} from '../lib/sr6/kinematics.ts';
import {receiverFromTip} from '../lib/sr6/virtual-stick.ts';
const data=JSON.parse(fs.readFileSync('public/sr6-cad.json','utf8'));
const meshes=new Map(data.parts.map(p=>[p.id,new T.Mesh()]));
const rig=createCadRig(data,meshes), cases=[];
for(const length of [0,220])for(const roll of [-25,-12.5,0,12.5,25])for(const pitch of [-25,-12.5,0,12.5,25]){
 rig.reset();const pose=receiverFromTip({...ZERO,roll:roll*Math.PI/180,pitch:pitch*Math.PI/180},length);
 const status=rig.update(pose);cases.push({roll,pitch,stickLength:length,valid:status.valid,reason:status.reason,matrices:status.valid?Object.fromEntries([...meshes].map(([id,m])=>[id,m.matrix.toArray()])):null});
}
fs.writeFileSync('cad/washer-sweep-poses.json',JSON.stringify(cases));
console.log({poses:cases.length,reachable:cases.filter(c=>c.valid).length});
