import fs from 'node:fs';
import * as T from 'three';
import {createCadRig} from '../lib/sr6/cad-rig.ts';

const data=JSON.parse(fs.readFileSync('public/sr6-cad.json','utf8'));
const normals=JSON.parse(fs.readFileSync('cad/joint-axis-normals.json','utf8'));
const meshes=new Map(data.parts.map(p=>[p.id,new T.Mesh()])),rig=createCadRig(data,meshes);
const rad=Math.PI/180;
const sets={};
for(const length of [220,160,110]){
  const report=JSON.parse(fs.readFileSync(`cad/workspace-${length}mm-12_5deg-side-off.json`,'utf8'));
  if(report.cadSha256!==data.sha256)throw Error('Stale scan');
  sets[length]=report.cases.filter(c=>c.valid).map(c=>{
    if(!rig.update(c.receiver).valid)throw Error('Pose became unreachable');
    const inverse=meshes.get(38).matrix.clone().invert();
    const movable=new Set([60,70,71,76,77,50,68,69,74,75]);
    return {target:c.target,fixedCollision:c.hits.some(h=>h.volumeMm3>.001&&!h.parts.some(id=>movable.has(id))),normals:[2,3].map(motor=>{
      const link=data.spec.links.find(l=>l.motor===motor);
      const n=new T.Vector3(...normals[motor].upper_eye).transformDirection(meshes.get(link.upper_eye).matrix).transformDirection(inverse);
      if(n.z<0)n.negate();return n;
    })};
  });
}
const axis=(a,b)=>new T.Vector3(Math.tan(a*rad),Math.tan(b*rad),1).normalize();
function score(samples,j,a,b){
  const n=axis(a,b);let dot=1;
  for(const s of samples)dot=Math.min(dot,Math.abs(n.dot(s.normals[j])));
  return Math.acos(T.MathUtils.clamp(dot,-1,1))/rad;
}
function search(samples,j){
  let best={a:0,b:0,maxAngleDeg:score(samples,j,0,0)};
  for(let a=-30;a<=30;a++)for(let b=-30;b<=30;b++){
    const value=score(samples,j,a,b);if(value<best.maxAngleDeg)best={a,b,maxAngleDeg:value};
  }
  for(const step of [.1,.01]){
    const center={...best};
    for(let i=-10;i<=10;i++)for(let k=-10;k<=10;k++){
      const a=center.a+i*step,b=center.b+k*step;
      const value=score(samples,j,a,b);if(value<best.maxAngleDeg)best={a,b,maxAngleDeg:value};
    }
  }
  const n=axis(best.a,best.b),worst=samples.reduce((a,b)=>Math.abs(n.dot(a.normals[j]))<Math.abs(n.dot(b.normals[j]))?a:b);
  return {...best,axis:n.toArray(),totalTiltDeg:Math.acos(n.z)/rad,baselineMaxDeg:score(samples,j,0,0),worstTarget:worst.target};
}
const output={cadSha256:data.sha256,criterion:'Minimize maximum upper-eye angle across reachable grid poses. Ball centers fixed. Only the independent M3/M4 pins can tilt; shared M1/M2 and M5/M6 pins are fixed by their two centers.',
  axisConvention:'Unit pin direction in receiver-local STEP coordinates; a=atan2(nx,nz), b=atan2(ny,nz), degrees. Original axis +Z.',
  search:'a,b each -30..30 deg at 1 deg, refined locally to 0.1 and 0.01 deg; finite search, not a global continuous guarantee.',results:{}};
for(const [name,samples] of Object.entries({...sets,combined:[...sets[160],...sets[110]]})){
  output.results[name]={sampleCount:samples.length,M3:search(samples,0),M4:search(samples,1)};
}
// Screening heuristic only: a nominal angular limit predicts sleeve contact.
// Exact 33-part boolean scans must validate shortlisted configurations.
function countSearch(samples){
  const words=Math.ceil(samples.length/32),cos=Math.cos(19*rad);
  const candidates=[0,1].map(j=>{
    const list=[];
    for(let a=-15;a<=15;a++)for(let b=-15;b<=15;b++){
      const n=axis(a,b),mask=new Uint32Array(words);let minDot=1;
      samples.forEach((s,i)=>{
        const d=Math.abs(n.dot(s.normals[j]));minDot=Math.min(minDot,d);
        if(s.fixedCollision||d<cos)mask[i>>>5]|=1<<(i&31);
      });
      list.push({a,b,axis:n.toArray(),totalTiltDeg:Math.acos(n.z)/rad,maxAngleDeg:Math.acos(minDot)/rad,mask});
    }
    return list;
  });
  const pop=n=>{n-=(n>>>1)&0x55555555;n=(n&0x33333333)+((n>>>2)&0x33333333);return (((n+(n>>>4))&0x0f0f0f0f)*0x01010101)>>>24;};
  let bestCount=Infinity,bestMax=Infinity,best;
  for(const a of candidates[0])for(const b of candidates[1]){
    let count=0;for(let w=0;w<words;w++)count+=pop(a.mask[w]|b.mask[w]);
    const max=Math.max(a.maxAngleDeg,b.maxAngleDeg);
    if(count<bestCount||(count===bestCount&&max<bestMax)){
      bestCount=count;bestMax=max;best={M3:{...a,mask:undefined},M4:{...b,mask:undefined}};
    }
  }
  return {...best,predictedCollisions:bestCount,sampleCount:samples.length,criterion:'Minimize union of fixed-part actual mesh collisions and M3/M4 angles >19deg; +/-15deg two-axis grid at 1deg. Heuristic shortlist, requires actual mesh validation.'};
}
for(const length of [220,160,110])output.results[`count-${length}`]=countSearch(sets[length]);
fs.writeFileSync('cad/mount-axis-search.json',JSON.stringify(output,null,2));
console.log(JSON.stringify(output,null,2));
