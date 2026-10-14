import * as T from 'three';

export type MaterialInfo={name:string;densityKgM3:number|null;color:string;source?:string};
export type PartMass={
  id:number;name:string;label:string;material:string;included:boolean;motion:string;reason:string;
  volumeMm3:number;centerMm:number[];massKg:number|null;inertiaCentroidKgM2:number[][]|null;
  principalKgM2:number[]|null;method:string;
};
export type MassProperties={version:number;source:string;sha256:string;materials:Record<string,MaterialInfo>;
  parts:PartMass[];includedMassKg:number;includedCount:number;virtualStickMassKg:number;payloadIsAdditional:boolean};
export type RigidBodyMass={id:string;partIds:number[];massKg:number;centerMm:T.Vector3;inertia:T.Matrix3};

export function validateMassProperties(data:MassProperties,sha256:string){
  if(data.version!==1||data.sha256!==sha256||data.parts.length!==78)throw Error('材质/惯量数据与当前 STEP 版本不匹配');
  const ids=new Set<number>();let mass=0;
  for(const p of data.parts){
    if(ids.has(p.id)||!data.materials[p.material])throw Error('材质零件清单无效');
    ids.add(p.id);
    if(!p.included)continue;
    if(!p.massKg||!Number.isFinite(p.massKg)||p.massKg<=0||p.centerMm.length!==3||!p.centerMm.every(Number.isFinite)||
      !p.inertiaCentroidKgM2||p.inertiaCentroidKgM2.length!==3||p.inertiaCentroidKgM2.some(row=>row.length!==3||!row.every(Number.isFinite)))
      throw Error(`零件 #${p.id} 质量或惯量无效`);
    mass+=p.massKg;
  }
  if(Math.abs(mass-data.includedMassKg)>1e-8)throw Error('运动质量汇总不一致');
}

/** Parallel-axis aggregation is exact for parts sharing the same rigid transform. */
export function aggregateRigidBodies(data:MassProperties):RigidBodyMass[]{
  const groups=new Map<string,PartMass[]>();
  for(const p of data.parts.filter(p=>p.included)){
    const group=groups.get(p.motion)??[];group.push(p);groups.set(p.motion,group);
  }
  return [...groups].map(([id,parts])=>{
    const massKg=parts.reduce((sum,p)=>sum+p.massKg!,0),centerMm=new T.Vector3();
    for(const p of parts)centerMm.addScaledVector(new T.Vector3(...p.centerMm),p.massKg!/massKg);
    const values=Array(9).fill(0) as number[];
    for(const p of parts){
      const r=new T.Vector3(...p.centerMm).sub(centerMm).multiplyScalar(.001).toArray(),r2=r.reduce((s,x)=>s+x*x,0);
      for(let i=0;i<3;i++)for(let j=0;j<3;j++)values[i*3+j]+=p.inertiaCentroidKgM2![i][j]+p.massKg!*((i===j?r2:0)-r[i]*r[j]);
    }
    return {id,partIds:parts.map(p=>p.id),massKg,centerMm,inertia:new T.Matrix3().set(...values as [number,number,number,number,number,number,number,number,number])};
  });
}
