// Zero-pose comparison: geometric inertia and mechanism-reflected inertia.
import assert from 'node:assert/strict';
import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import * as T from 'three';
import {ZERO} from '../lib/sr6/kinematics.ts';
import {CAD_PLATFORM_PIVOT,createCadKinematics} from '../lib/sr6/cad-kinematics.ts';
import {aggregateRigidBodies} from '../lib/sr6/mass-properties.ts';
import {createRobotDynamics} from '../lib/sr6/rigid-dynamics.ts';

const root=new URL('../',import.meta.url);
const read=name=>JSON.parse(readFileSync(new URL(name,root),'utf8'));
const data=read('public/sr6-6010-cad.json'),masses=read('public/sr6-6010-mass-properties.json');
assert.equal(createHash('sha256').update(readFileSync(new URL(`cad/${data.source}`,root))).digest('hex'),data.sha256);
const solver=createCadKinematics(data),model=createRobotDynamics(data,masses,solver);
const receiver=id=>id==='platform'||id==='twist';
const group=id=>receiver(id)?'receiver':'otherMoving';
const names={receiver:'接收器部分',otherMoving:'非接收器活动部分'};
const keys=['x','y','z','roll','pitch','twist'];
const labels=['Fwd 前后','Thrust 升降','Side 侧移','Roll','Pitch','R0'];
const zeroQ=solver.inverse(ZERO);assert.ok(zeroQ.valid);
const positions=model.bodyPoses(ZERO,zeroQ.q,0);
const matrixRows=m=>[0,1,2].map(i=>[0,1,2].map(j=>m.elements[j*3+i]));
const reference=new T.Vector3(...CAD_PLATFORM_PIVOT).multiplyScalar(.001);
const grouped=aggregateRigidBodies({...masses,parts:masses.parts.map(p=>({...p,motion:group(p.motion)}))});
const groups=Object.fromEntries(grouped.map(b=>{
  const r=b.centerMm.clone().multiplyScalar(.001).sub(reference).toArray(),r2=r.reduce((s,v)=>s+v*v,0);
  const atCenter=matrixRows(b.inertia);
  const atReference=atCenter.map((row,i)=>row.map((v,j)=>v+b.massKg*((i===j?r2:0)-r[i]*r[j])));
  return [b.id,{label:names[b.id],partIds:b.partIds,count:b.partIds.length,massKg:b.massKg,
    centerMm:b.centerMm.toArray(),inertiaAtGroupCenterKgM2:atCenter,inertiaAtReceiverOriginKgM2:atReference}];
}));
function rotvec(q){
  if(q.w<0)q.set(-q.x,-q.y,-q.z,-q.w);
  const n=Math.hypot(q.x,q.y,q.z);
  return new T.Vector3(q.x,q.y,q.z).multiplyScalar(n<1e-12?2:2*Math.atan2(n,q.w)/n);
}
function effective(stepScale){
  const matrices={receiver:Array.from({length:6},()=>Array(6).fill(0)),otherMoving:Array.from({length:6},()=>Array(6).fill(0))};
  const jac=new Map(model.bodies.map(b=>[b.id,{v:[],w:[]}]));
  for(const [i,key] of keys.entries()){
    const h=(i<3?.001:1e-6)*stepScale;
    const plusPose={...ZERO,[key]:h},minusPose={...ZERO,[key]:-h};
    const plusQ=solver.inverse(plusPose),minusQ=solver.inverse(minusPose);
    assert.ok(plusQ.valid&&minusQ.valid);
    const plus=model.bodyPoses(plusPose,plusQ.q,0),minus=model.bodyPoses(minusPose,minusQ.q,0);
    const denominator=2*h*(i<3?.001:1); // translational coordinates expressed in metres
    for(const body of model.bodies){
      const p=plus.get(body.id),m=minus.get(body.id),j=jac.get(body.id);
      j.v.push(p.center.clone().sub(m.center).multiplyScalar(1/denominator));
      j.w.push(rotvec(p.rotation.clone().multiply(m.rotation.clone().invert())).multiplyScalar(1/denominator));
    }
  }
  for(const body of model.bodies){
    const j=jac.get(body.id),a=matrices[group(body.id)];
    const rotation=new T.Matrix3().setFromMatrix4(new T.Matrix4().makeRotationFromQuaternion(positions.get(body.id).rotation));
    const inertia=rotation.clone().multiply(body.inertia).multiply(rotation.clone().transpose());
    for(let i=0;i<6;i++)for(let k=0;k<6;k++)a[i][k]+=body.massKg*j.v[i].dot(j.v[k])+j.w[i].dot(j.w[k].clone().applyMatrix3(inertia));
  }
  return matrices;
}
const massMatrices=effective(1),halfStep=effective(.5);
let convergence=0;
for(const id of Object.keys(groups)){
  const a=massMatrices[id],b=halfStep[id];
  for(let i=0;i<6;i++)for(let j=0;j<6;j++){
    const scale=Math.max(1e-10,Math.sqrt(a[i][i]*a[j][j]));
    convergence=Math.max(convergence,Math.abs(a[i][j]-b[i][j])/scale);
    assert.ok(Math.abs(a[i][j]-a[j][i])<1e-10);
  }
  groups[id].effectiveMassMatrixSI=a;
}
assert.ok(convergence<1e-5,`Derivative convergence ${convergence}`);
assert.ok(Math.abs(groups.receiver.massKg+groups.otherMoving.massKg-masses.includedMassKg)<1e-10);
// Independent rigid-platform reference for translation and rotation about its origin.
for(let i=0;i<3;i++)assert.ok(Math.abs(massMatrices.receiver[i][i]-groups.receiver.massKg)<1e-7);
assert.ok(Math.abs(massMatrices.receiver[3][3]-groups.receiver.inertiaAtReceiverOriginKgM2[1][1])<1e-9);
assert.ok(Math.abs(massMatrices.receiver[4][4]-groups.receiver.inertiaAtReceiverOriginKgM2[2][2])<1e-9);
assert.ok(Math.abs(massMatrices.otherMoving[5][5])<1e-16);
const axes=keys.map((key,i)=>{
  const r=massMatrices.receiver[i][i],o=massMatrices.otherMoving[i][i];
  return {key,label:labels[i],unit:i<3?'kg':'kg*m^2',receiver:r,otherMoving:o,
    otherToReceiver:o/r,receiverPercent:100*r/(r+o),otherMovingPercent:100*o/(r+o)};
});
const result={source:data.source,sha256:data.sha256,pose:ZERO,payloadKg:0,
  referencePointMm:[...CAD_PLATFORM_PIVOT],coordinates:['Fwd (m)','Thrust (m)','Side (m)','Roll (rad)','Pitch (rad)','R0 (rad)'],
  virtualStickCompensation:false,groups,axes,checks:{derivativeStepHalvingNormalizedError:convergence},
  method:'M_group = sum(m Jv^T Jv + Jw^T I_world Jw); kinetic energy = 0.5 velocity^T M velocity',
  limitations:['Zero pose only; effective inertia varies with pose.','No motor internal rotor inertia, gearing, friction or external payload.',
    'R0 motor uses existing 388 g measured mass and cylinder approximation.','Other moving parts are articulated; common-origin geometric inertia is not their reflected inertia.']};
writeFileSync(new URL('cad/inertia-comparison-6010.json',root),JSON.stringify(result,null,2)+'\n');
const f=n=>n.toFixed(6),pct=n=>n.toFixed(1);
const rows=[
  '# 6010 接收器与非接收器活动部分惯量对比','',
  '基准：6010 装配零位，所有位姿增量为 0；虚拟杆补偿关闭、额外负载为 0。数据来自当前 STEP 与对应质量文件。','',
  '接收器：接收器两件钣金、R0 电机及输出件、随接收器运动的紧固件/钢套、六个上端鱼眼内球。鱼眼外壳随连杆摆动，归入非接收器活动部分。R0 附属件质量已合并在电机 388 g 中，不重复统计。',
  '非接收器活动部分：六个一级摇臂、四根主光轴、两根俯仰二级臂、随各连杆摆动的鱼眼外壳、六个下端鱼眼内球。固定底座和六个底部电机不在两组内。','',
  '| 项目 | 接收器部分 | 非接收器活动部分 |','|---|---:|---:|',
  `| 计入质量的实体数 | ${groups.receiver.count} | ${groups.otherMoving.count} |`,
  `| 质量 kg | ${f(groups.receiver.massKg)} | ${f(groups.otherMoving.massKg)} |`,
  `| 运动质量占比 | ${pct(100*groups.receiver.massKg/masses.includedMassKg)}% | ${pct(100*groups.otherMoving.massKg/masses.includedMassKg)}% |`,'',
  '## 实际机构的等效惯量','',
  '用各刚体质心速度雅可比和角速度雅可比计算 M = Σ(m JvᵀJv + JωᵀI Jω)。下表为其对角项：其他坐标保持不变时，某轴单独加速需要承担的惯性系数。平移列为等效质量 kg，转动列为等效转动惯量 kg·m²。转动围绕现有接收器运动学原点，不启用杆尖补偿。','',
  '| 轴 | 单位 | 接收器部分 | 非接收器活动部分 | 非接收器/接收器 | 非接收器占总量 |','|---|---|---:|---:|---:|---:|',
  ...axes.map(a=>`| ${a.label} | ${a.unit} | ${f(a.receiver)} | ${f(a.otherMoving)} | ${a.otherToReceiver.toFixed(3)} | ${pct(a.otherMovingPercent)}% |`),'',
  '该表不是各电机轴上的惯量或力矩，也不包含重力项。多轴同时运动时需使用完整矩阵的耦合项；零位单轴结果不能替代全工作空间或脚本峰值。R0 行只对应独立输出旋转，其电机定子和接收器框架不会随 R0 一起转。','',
  '## 同一参考点的几何转动惯量','',
  `参考点 STEP XYZ = (${CAD_PLATFORM_PIVOT.join(', ')}) mm，两个分组使用相同参考点和相同轴线。按平行轴定理汇总，单位 kg·m²。非接收器组按零位形状作几何统计，不假定它在真实机构中整体刚性旋转。`,'',
  '| 轴线 | 接收器部分 | 非接收器活动部分 |','|---|---:|---:|',
  ...['STEP X（升降方向）','STEP Y（前后方向 / Roll 轴线）','STEP Z（侧向 / Pitch 轴线）'].map((s,i)=>`| ${s} | ${f(groups.receiver.inertiaAtReceiverOriginKgM2[i][i])} | ${f(groups.otherMoving.inertiaAtReceiverOriginKgM2[i][i])} |`),'',
  '完整矩阵、各组质心、质心惯量和零件 ID 见 [JSON](../cad/inertia-comparison-6010.json)。','',
  '## 校验及适用范围','',
  `两组质量之和等于全机构运动质量 ${f(masses.includedMassKg)} kg。数值微分步长减半的最大归一化差值为 ${convergence.toExponential(3)}；接收器平移等效质量与实体总质量一致，Roll/Pitch 等效惯量与平行轴定理结果一致。`,
  '沿用当前材质密度、R0 电机 388 g 实测质量和圆柱惯量近似。未纳入底部电机内部转子的独立自转、传动反射惯量、摩擦或额外负载。开启虚拟杆尖补偿后，Roll/Pitch 会伴随接收器平移，需要重新按该约束折算。','',
  '复现：`node scripts/compare-6010-inertia.mjs`。','',
];
writeFileSync(new URL('docs/INERTIA-COMPARISON-6010.md',root),rows.join('\n'));
console.log(JSON.stringify({mass:{receiver:groups.receiver.massKg,otherMoving:groups.otherMoving.massKg},axes,convergence},null,2));
