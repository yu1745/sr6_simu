import fs from 'node:fs';
import {execFileSync} from 'node:child_process';
import * as T from 'three';
import ManifoldModule from 'manifold-3d';
import { createCadRig, CAD_LINKS } from '../lib/sr6/cad-rig.ts';
import { collisionParts } from '../lib/sr6/collision-scope.ts';
import { receiverFromTip } from '../lib/sr6/virtual-stick.ts';
import { ZERO } from '../lib/sr6/kinematics.ts';

const steps=Number(process.argv[2]??5);
if(!Number.isInteger(steps)||steps<3||steps%2!==1)throw Error('Use an odd grid size >=3');
const out=process.argv[3]??'cad/workspace-220mm-12_5deg.json';
const length=Number(process.argv[4]??220);
const sideOff=process.argv[5]==='side-off';
const zeroOffset={x:Number(process.argv[7]??0),y:Number(process.argv[8]??0),z:Number(process.argv[9]??0)};
if(!Object.values(zeroOffset).every(Number.isFinite))throw Error('Invalid zero offset');
if(!Number.isFinite(length)||length<0)throw Error('Invalid compensation length');
const data=JSON.parse(fs.readFileSync('public/sr6-cad.json','utf8'));
const normals=JSON.parse(fs.readFileSync('cad/joint-axis-normals.json','utf8'));
const meshes=new Map(data.parts.map(p=>[p.id,new T.Mesh()]));
const rig=createCadRig(data,meshes);
const mountKey=process.argv[6]==='original'?undefined:process.argv[6];
const mountSearch=mountKey?JSON.parse(fs.readFileSync('cad/mount-axis-search.json','utf8')):null;
if(mountSearch&&mountSearch.cadSha256!==data.sha256)throw Error('Stale mount search');
const mountConfig=mountSearch?.results[mountKey];
if(mountKey&&!mountConfig)throw Error('Unknown mount configuration');
const mounts=mountConfig?[
  {motor:2,ids:[60,70,71,76,77],axis:mountConfig.M3.axis},
  {motor:3,ids:[50,68,69,74,75],axis:mountConfig.M4.axis},
].map(m=>{
  const center=new T.Vector3(...data.spec.links.find(l=>l.motor===m.motor).upper);
  const q=new T.Quaternion().setFromUnitVectors(new T.Vector3(0,0,1),new T.Vector3(...m.axis));
  const matrix=new T.Matrix4().compose(center.clone().sub(center.clone().applyQuaternion(q)),q,new T.Vector3(1,1,1));
  if(center.clone().applyMatrix4(matrix).distanceTo(center)>1e-9)throw Error('Mount moved ball center');
  return {...m,matrix};
}):[];
const wasm=await ManifoldModule();wasm.setup();
const groups=new Map();
[38,39,40,46,54,55,56,57,64,65,66,67,68,69,70,71,72,73,74,75,76,77].forEach(id=>groups.set(id,'platform'));
const excluded=new Set(),key=(a,b)=>[a,b].sort((x,y)=>x-y).join(':');
for(const l of CAD_LINKS){
  groups.set(l.upper,'platform');l.rod.forEach(id=>groups.set(id,`rod${l.motor}`));
  const spec=data.spec.links.find(s=>s.motor===l.motor);
  excluded.add(key(l.upper,spec.upper_eye));
}
const parts=collisionParts(data).map(p=>{
  const mesh=new wasm.Mesh({numProp:3,vertProperties:new Float32Array(p.positions),triVerts:new Uint32Array(p.indices)});
  mesh.merge();const solid=new wasm.Manifold(mesh);
  if(solid.status()!=='NoError')throw Error(`Non-manifold part ${p.id}; scan cannot omit it`);
  const bounds=new T.Box3().setFromArray(p.positions);
  return {...p,solid,bounds,group:groups.get(p.id)??'fixed'};
});
const pairs=[];
for(let i=0;i<parts.length;i++)for(let j=i+1;j<parts.length;j++){
  const a=parts[i],b=parts[j];
  if(a.group!==b.group&&!excluded.has(key(a.id,b.id)))pairs.push([a,b]);
}
const grid=limit=>Array.from({length:steps},(_,i)=>-limit+2*limit*i/(steps-1));
const axes={x:grid(30),y:grid(60),z:sideOff?[0]:grid(30),roll:grid(12.5),pitch:grid(12.5)};
const total=Object.values(axes).reduce((n,a)=>n*a.length,1);
const result={cadSha256:data.sha256,sourceCommit:execFileSync('git',['rev-parse','HEAD'],{encoding:'utf8'}).trim(),stickLengthMm:length,fullCompensation:true,
  mountConfig:mountConfig??null,
  zeroOffset,
  axes,angleUnit:'degrees',scope:'Same 33 parts and intentional joint/rigid-group exclusions as live collision display; twist does not move these parts.',
  meshVolumeThresholdMm3:1e-7,significantVolumeThresholdMm3:1e-3,
  total:0,reachable:0,unreachable:0,colliding:0,significantColliding:0,
  upperJointLimitDeg:18.8608,upperJointOverLimit:0,maxJoint:{angleDeg:0},
  parts:parts.map(p=>({id:p.id,name:p.name})),pairSummary:{},cases:[]};
const start=performance.now();
function check(target){
  const pose=receiverFromTip({...target,roll:target.roll*Math.PI/180,pitch:target.pitch*Math.PI/180},length);
  pose.x+=zeroOffset.x;pose.y+=zeroOffset.y;pose.z+=zeroOffset.z;
  const status=rig.update(pose);
  const row={target,receiver:pose,valid:status.valid};
  result.total++;
  if(!status.valid){result.unreachable++;row.reason=status.reason;result.cases.push(row);return;}
  result.reachable++;
  for(const m of mounts)for(const id of m.ids)meshes.get(id).matrix.multiply(m.matrix);
  const boxes=new Map(parts.map(p=>[p.id,p.bounds.clone().applyMatrix4(meshes.get(p.id).matrix)]));
  row.jointAnglesDeg=data.spec.links.map(l=>{
    const pin=new T.Vector3(0,0,1).transformDirection(meshes.get(l.upper_ball).matrix);
    const normal=new T.Vector3(...normals[l.motor].upper_eye).transformDirection(meshes.get(l.upper_eye).matrix);
    return Math.acos(T.MathUtils.clamp(Math.abs(normal.dot(pin)),-1,1))*180/Math.PI;
  });
  const angleDeg=Math.max(...row.jointAnglesDeg);
  if(angleDeg>result.upperJointLimitDeg)result.upperJointOverLimit++;
  if(angleDeg>result.maxJoint.angleDeg)result.maxJoint={angleDeg,motor:row.jointAnglesDeg.indexOf(angleDeg)+1,target};
  row.hits=[];
  for(const [a,b] of pairs){
    if(!boxes.get(a.id).intersectsBox(boxes.get(b.id)))continue;
    const toA=meshes.get(a.id).matrix.clone().invert().multiply(meshes.get(b.id).matrix);
    let moved,common;
    try{
      moved=b.solid.transform(toA.elements);common=a.solid.intersect(moved);
      if(common.status()!=='NoError')throw Error(`Boolean failed ${a.id}:${b.id}`);
      const volume=common.isEmpty()?0:common.volume();
      if(volume>result.meshVolumeThresholdMm3){
        row.hits.push({parts:[a.id,b.id],volumeMm3:volume});
        const id=key(a.id,b.id),summary=result.pairSummary[id]??={parts:[a.id,b.id],names:[a.name,b.name],count:0,significantCount:0,maxVolumeMm3:0};
        summary.count++;if(volume>result.significantVolumeThresholdMm3)summary.significantCount++;
        if(volume>summary.maxVolumeMm3){summary.maxVolumeMm3=volume;summary.target=target;}
      }
    }finally{common?.delete();moved?.delete();}
  }
  if(row.hits.length)result.colliding++;
  if(row.hits.some(h=>h.volumeMm3>result.significantVolumeThresholdMm3))result.significantColliding++;
  result.cases.push(row);
}
// Neutral is already contained in the odd-sized grid.
for(const x of axes.x)for(const y of axes.y)for(const z of axes.z)for(const roll of axes.roll)for(const pitch of axes.pitch){
  check({...ZERO,x,y,z,roll,pitch});
  if(result.total%25===0)console.log(JSON.stringify({checked:result.total,of:total,reachable:result.reachable,colliding:result.colliding,elapsedSeconds:Math.round((performance.now()-start)/1000)}));
}
result.elapsedSeconds=(performance.now()-start)/1000;
fs.writeFileSync(out,JSON.stringify(result,null,2));
parts.forEach(p=>p.solid.delete());
console.log(JSON.stringify({...result,cases:undefined,parts:undefined},null,2));
