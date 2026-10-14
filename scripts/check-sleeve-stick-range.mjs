import fs from 'node:fs';
import * as T from 'three';
import {createCadRig} from '../lib/sr6/cad-rig.ts';
import {ZERO} from '../lib/sr6/kinematics.ts';
import {receiverFromTip} from '../lib/sr6/virtual-stick.ts';
const data=JSON.parse(fs.readFileSync('public/sr6-cad.json','utf8')), normals=JSON.parse(fs.readFileSync('cad/joint-axis-normals.json','utf8'));
const meshes=new Map(data.parts.map(p=>[p.id,new T.Mesh()])),rig=createCadRig(data,meshes),limit=18.8608;
function evaluate(length,roll,pitch){
 rig.reset();const status=rig.update(receiverFromTip({...ZERO,roll:roll*Math.PI/180,pitch:pitch*Math.PI/180},length));
 if(!status.valid)return {valid:false,reason:status.reason,max:Infinity};
 const pin=new T.Vector3(0,0,1).transformDirection(meshes.get(38).matrix);
 const angles=data.spec.links.map(l=>new T.Vector3(...normals[l.motor].upper_eye).transformDirection(meshes.get(l.upper_eye).matrix)).map(n=>Math.acos(Math.min(1,Math.abs(n.dot(pin))))*180/Math.PI);
 const max=Math.max(...angles);return {valid:true,max,motor:angles.indexOf(max)+1,angles};
}
const rows=[];
for(let length=0;length<=220;length++){
 const cases=[[-25,0],[25,0],[0,-25],[0,25],[-25,-25],[-25,25],[25,-25],[25,25]].map(([r,p])=>({roll:r,pitch:p,...evaluate(length,r,p)}));
 rows.push({length,valid:cases.every(c=>c.valid),max:Math.max(...cases.map(c=>c.max)),cases});
}
const limits=[];
for(const length of [220,180,150,100,50,25,0]){
 const row={length};
 for(const mode of ['roll','pitch','combined']){
  function passed(a){const vals=mode==='roll'?[[-a,0],[a,0]]:mode==='pitch'?[[0,-a],[0,a]]:[[-a,-a],[-a,0],[-a,a],[0,-a],[0,a],[a,-a],[a,0],[a,a]];return vals.every(([r,p])=>evaluate(length,r,p).max<=limit);}
  let lo=0,hi=30;for(let i=0;i<22;i++){const mid=(lo+hi)/2;if(passed(mid))lo=mid;else hi=mid;}row[mode]=lo;
 }
 limits.push(row);
}
const best=rows.filter(r=>r.valid).sort((a,b)=>a.max-b.max)[0];
const result={criterion:'Upper SA6TK joint angles only, steel sleeve nominal contact limit; full compensation, no clamp; not complete collision certification',limit,best25:best,lengthLimits:limits,lengthSweep:rows};
fs.writeFileSync('cad/steel-sleeve-stick-range.json',JSON.stringify(result,null,2));
console.log(JSON.stringify({best25:best,lengthLimits:limits,zero:rows.find(r=>r.length===0),validLengths25:rows.filter(r=>r.valid&&r.max<=limit).length},null,2));
