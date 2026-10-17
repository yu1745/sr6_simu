import fs from 'node:fs';
import * as T from 'three';
import {createCadRig} from '../lib/sr6/cad-rig.ts';
const data=JSON.parse(fs.readFileSync('public/sr6-cad.json','utf8'));
const meshes=new Map(data.parts.map(p=>[p.id,new T.Mesh()])),rig=createCadRig(data,meshes);
const output=[];
const designs=process.argv.length>2?[[Number(process.argv[2]),process.argv[3]??process.argv[2]]]:[[160,'160'],[110,'count-110']];
for(const [length,key] of designs){
  const filename=`cad/workspace-${length}mm-mount-${key}.json`;
  const r=JSON.parse(fs.readFileSync(filename,'utf8'));
  const baseline=JSON.parse(fs.readFileSync(`cad/workspace-${length}mm-12_5deg-side-off.json`,'utf8'));
  const valid=r.cases.filter(c=>c.valid),peak=c=>Math.max(0,...c.hits.map(h=>h.volumeMm3));
  const largest=valid.reduce((a,b)=>peak(a)>peak(b)?a:b);
  const cleared=valid.filter(c=>peak(c)<=.001).map(c=>({c,old:baseline.cases.find(b=>JSON.stringify(b.target)===JSON.stringify(c.target))})).sort((a,b)=>peak(b.old)-peak(a.old))[0];
  const selected=[largest,cleared.c,valid.find(c=>Object.values(c.target).every(v=>v===0)),valid.find(c=>c.target.x===0&&c.target.y===0&&c.target.roll===12.5&&c.target.pitch===-12.5)];
  for(const c of selected){
    if(!rig.update(c.receiver).valid)throw Error('Invalid witness');
    for(const [motor,ids,axis] of [[2,[60,70,71,76,77],r.mountConfig.M3.axis],[3,[50,68,69,74,75],r.mountConfig.M4.axis]]){
      const center=new T.Vector3(...data.spec.links.find(l=>l.motor===motor).upper);
      const rotation=new T.Quaternion().setFromUnitVectors(new T.Vector3(0,0,1),new T.Vector3(...axis));
      const local=new T.Matrix4().compose(center.clone().sub(center.clone().applyQuaternion(rotation)),rotation,new T.Vector3(1,1,1));
      for(const id of ids)meshes.get(id).matrix.multiply(local);
    }
    output.push({source:filename,target:c.target,pose:c.receiver,angles:c.jointAnglesDeg,meshHits:c.hits,
      matrices:Object.fromEntries([...meshes].map(([id,m])=>[id,m.matrix.toArray()]))});
  }
}
fs.writeFileSync(process.argv[4]??'work/workspace-scan/mount-witnesses.json',JSON.stringify(output));
console.log(`Prepared ${output.length} modified-mount STEP witnesses`);
