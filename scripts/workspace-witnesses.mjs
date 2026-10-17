import fs from 'node:fs';
import * as T from 'three';
import {createCadRig} from '../lib/sr6/cad-rig.ts';

const report=JSON.parse(fs.readFileSync('cad/workspace-220mm-12_5deg.json','utf8'));
const data=JSON.parse(fs.readFileSync('public/sr6-cad.json','utf8'));
if(report.cadSha256!==data.sha256)throw Error('Scan uses a different CAD revision');
const meshes=new Map(data.parts.map(p=>[p.id,new T.Mesh()])),rig=createCadRig(data,meshes);
const valid=report.cases.filter(c=>c.valid);
const worst=valid.reduce((a,b)=>Math.max(...a.jointAnglesDeg)>Math.max(...b.jointAnglesDeg)?a:b);
const chosen=[worst,...valid.filter(c=>c.target.x===0&&c.target.y===0&&c.target.z===0&&Math.abs(c.target.roll)===12.5&&Math.abs(c.target.pitch)===12.5),
  valid.find(c=>Object.values(c.target).every(v=>v===0))];
const rows=chosen.map(c=>{
  const status=rig.update(c.receiver);if(!status.valid)throw Error(status.reason);
  return {target:c.target,pose:c.receiver,angles:c.jointAnglesDeg,max:Math.max(...c.jointAnglesDeg),
    matrices:Object.fromEntries([...meshes].map(([id,m])=>[id,m.matrix.toArray()]))};
});
fs.mkdirSync('work/workspace-scan',{recursive:true});
fs.writeFileSync('work/workspace-scan/witnesses.json',JSON.stringify(rows));
console.log(`Prepared ${rows.length} STEP verification witnesses`);
