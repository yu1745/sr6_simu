import * as T from 'three';
import { CAD_LINKS, type CadData } from './cad-rig.ts';
import { CAD_PLATFORM_PIVOT, cadPlatform, type CadKinematics } from './cad-kinematics.ts';
import { aggregateRigidBodies, validateMassProperties, type MassProperties } from './mass-properties.ts';
import { GRAVITY_MPS2, NO_ACCELERATION, staticForceAnalysis, type BaseReaction, type TorqueAnalysis } from './statics.ts';
import { MAX_LINEAR_ACCELERATION, MAX_SAMPLE_GAP_MS, MOTION_FILTER_HZ } from './payload-motion.ts';
import type { Pose } from './kinematics.ts';

type Vec3=[number,number,number];
export type BodyRates={acceleration:Vec3;omega:Vec3;alpha:Vec3};
export type RobotMotion=Record<string,BodyRates>;
type BodyPose={center:T.Vector3;rotation:T.Quaternion};
type Derivatives={linear:T.Vector3[];angular:T.Vector3[]};
const KEYS=['x','y','z','roll','pitch'] as const;
const X=new T.Vector3(1,0,0),Z=new T.Vector3(0,0,1);
const TWIST_PIVOT=new T.Vector3(95.141985,270.186359,-62.585786);

function rigid(rotation:T.Quaternion,pivot:T.Vector3,target=pivot){
  return new T.Matrix4().compose(target.clone().sub(pivot.clone().applyQuaternion(rotation)),rotation,new T.Vector3(1,1,1));
}
/** World-axis logarithm of a relative rotation, including a stable small-angle limit. */
function rotationVector(q:T.Quaternion){
  if(q.w<0)q.set(-q.x,-q.y,-q.z,-q.w);
  const n=Math.hypot(q.x,q.y,q.z);
  return new T.Vector3(q.x,q.y,q.z).multiplyScalar(n<1e-12?2:2*Math.atan2(n,q.w)/n);
}
const vector=(v:readonly number[])=>new T.Vector3(v[0],v[1],v[2]);

/** Generalized inverse-dynamics demand by virtual work, in source STEP axes.
 * Translations are differentiated per mm; angular coordinates per radian.
 * Gyroscopic demand is I*alpha + omega x (I*omega), about the body COM.
 */
export function bodyWrench(massKg:number,inertiaWorld:T.Matrix3,rates:BodyRates,derivatives:Derivatives){
  const force=vector(rates.acceleration).multiplyScalar(massKg);
  const omega=vector(rates.omega);
  const moment=vector(rates.alpha).applyMatrix3(inertiaWorld).add(omega.clone().cross(omega.clone().applyMatrix3(inertiaWorld)));
  return {
    gravity:derivatives.linear.map(v=>v.x*massKg*GRAVITY_MPS2),
    inertia:derivatives.linear.map((v,i)=>v.dot(force)+derivatives.angular[i].dot(moment)),
  };
}

/** Wrench exerted by one moving body on the fixed base. STEP +X is upward.
 * Gravity and inertia are kept separate so a static load remains visible.
 */
export function bodyBaseReaction(massKg:number,inertiaWorld:T.Matrix3,rates:BodyRates,
                                 centerM:T.Vector3,originM:T.Vector3){
  const arm=centerM.clone().sub(originM);
  const gravityForce=new T.Vector3(massKg*GRAVITY_MPS2,0,0);
  const inertiaForce=vector(rates.acceleration).multiplyScalar(massKg);
  const omega=vector(rates.omega);
  const spinMoment=vector(rates.alpha).applyMatrix3(inertiaWorld)
    .add(omega.clone().cross(omega.clone().applyMatrix3(inertiaWorld)));
  const onBase=(force:T.Vector3,moment:T.Vector3)=>{
    const torque=arm.clone().cross(force).add(moment);
    return [-force.x,-force.y,-force.z,-torque.x,-torque.y,-torque.z];
  };
  return {gravity:onBase(gravityForce,new T.Vector3()),inertia:onBase(inertiaForce,spinMoment)};
}

export function createRobotDynamics(data:CadData,masses:MassProperties,solver:CadKinematics){
  validateMassProperties(masses,data.sha256);
  const bodies=aggregateRigidBodies(masses);
  const baseSheet=masses.parts.find(part=>part.id===1&&part.motion==='fixed');
  if(!baseSheet||baseSheet.centerMm.length!==3||!baseSheet.centerMm.every(Number.isFinite))
    throw Error('固定底座参考点缺失');
  const baseOriginMm=baseSheet.centerMm as Vec3;
  const baseOriginM=vector(baseOriginMm).multiplyScalar(.001);
  const links=CAD_LINKS.map(link=>{
    const measured=data.spec.links.find(l=>l.motor===link.motor)!;
    const lower=vector(measured.lower),upper=vector(measured.upper);
    return {motor:link.motor,pivot:vector(measured.pivot),lower,upper,direction:upper.clone().sub(lower).normalize()};
  });
  function transforms(pose:Pose,q:readonly number[]){
    const platform=cadPlatform(pose);
    const result=new Map<string,T.Matrix4>([['platform',platform],['twist',platform.clone().multiply(rigid(new T.Quaternion().setFromAxisAngle(X,pose.twist),TWIST_PIVOT))]]);
    for(const link of links){
      const crank=rigid(new T.Quaternion().setFromAxisAngle(Z,q[link.motor]),link.pivot);
      const lower=link.lower.clone().applyMatrix4(crank),upper=link.upper.clone().applyMatrix4(platform);
      const rotation=new T.Quaternion().setFromUnitVectors(link.direction,upper.sub(lower).normalize());
      result.set(`crank${link.motor+1}`,crank);
      result.set(`rod${link.motor+1}`,rigid(rotation,link.lower,lower));
    }
    return result;
  }
  function bodyPoses(pose:Pose,q:readonly number[],stickLength:number){
    const matrices=transforms(pose,q),result=new Map<string,BodyPose>();
    for(const body of bodies){
      const matrix=matrices.get(body.id);
      if(!matrix)throw Error(`未知运动组件 ${body.id}`);
      result.set(body.id,{center:body.centerMm.clone().applyMatrix4(matrix).multiplyScalar(.001),rotation:new T.Quaternion().setFromRotationMatrix(matrix)});
    }
    // The virtual rod has zero mass; only the additional payload acts at its tip.
    const tip=new T.Vector3(...CAD_PLATFORM_PIVOT).addScaledVector(X,-stickLength);
    result.set('payload',{center:tip.applyMatrix4(matrices.get('platform')!).multiplyScalar(.001),rotation:new T.Quaternion()});
    return result;
  }
  let previous:Map<string,BodyPose>|null=null,previousTime=0;
  /** Ideal script dynamics from the planned curve, independent of render cadence.
   * Five-point differentiation in real seconds includes playback speed and IK.
   */
  function referenceMotion(pose:Pose,q:readonly number[],stickLength:number,
                           sample:(offsetSeconds:number)=>{pose:Pose;q:readonly number[]},h=.001):RobotMotion{
    const center=bodyPoses(pose,q,stickLength);
    const frames=[-2,-1,1,2].map(n=>{const p=sample(n*h);return bodyPoses(p.pose,p.q,stickLength);});
    const result:RobotMotion={};
    for(const [id,p] of center){
      const points=frames.map(f=>f.get(id)!);
      const acceleration=points[0].center.clone().negate().addScaledVector(points[1].center,16)
        .addScaledVector(p.center,-30).addScaledVector(points[2].center,16).addScaledVector(points[3].center,-1).multiplyScalar(1/(12*h*h));
      const rotations=points.map(v=>rotationVector(v.rotation.clone().multiply(p.rotation.clone().invert())));
      const omega=rotations[0].clone().addScaledVector(rotations[1],-8).addScaledVector(rotations[2],8).addScaledVector(rotations[3],-1).multiplyScalar(1/(12*h));
      const alpha=rotations[0].clone().negate().addScaledVector(rotations[1],16).addScaledVector(rotations[2],16).addScaledVector(rotations[3],-1).multiplyScalar(1/(12*h*h));
      result[id]={acceleration:acceleration.toArray(),omega:omega.toArray(),alpha:alpha.toArray()};
    }
    return result;
  }
  let velocities=new Map<string,{linear:T.Vector3;angular:T.Vector3}>();
  let rates:RobotMotion={};
  function reset(){previous=null;previousTime=0;velocities=new Map();rates={};}
  function sample(pose:Pose,q:readonly number[],stickLength:number,timeMs:number):RobotMotion{
    const current=bodyPoses(pose,q,stickLength),dt=(timeMs-previousTime)*.001;
    if(!Number.isFinite(timeMs)||!previous||dt<=0||dt>MAX_SAMPLE_GAP_MS*.001){
      reset();previous=current;previousTime=timeMs;return {};
    }
    if(dt<.001)return rates;
    const blend=1-Math.exp(-2*Math.PI*MOTION_FILTER_HZ*dt),next:RobotMotion={};
    for(const [id,p] of current){
      const old=previous.get(id)!,v=velocities.get(id)??{linear:new T.Vector3(),angular:new T.Vector3()};
      const linear=p.center.clone().sub(old.center).multiplyScalar(1/dt).lerp(v.linear,1-blend);
      const angular=rotationVector(p.rotation.clone().multiply(old.rotation.clone().invert())).multiplyScalar(1/dt).lerp(v.angular,1-blend);
      const a=linear.clone().sub(v.linear).multiplyScalar(1/dt).lerp(vector(rates[id]?.acceleration??NO_ACCELERATION),1-blend);
      const alpha=angular.clone().sub(v.angular).multiplyScalar(1/dt).lerp(vector(rates[id]?.alpha??NO_ACCELERATION),1-blend);
      a.clampScalar(-MAX_LINEAR_ACCELERATION,MAX_LINEAR_ACCELERATION);
      next[id]={acceleration:a.toArray(),omega:angular.toArray(),alpha:alpha.toArray()};
      velocities.set(id,{linear,angular});
    }
    previous=current;previousTime=timeMs;rates=next;return next;
  }
  let cacheKey='',cachedDerivatives=new Map<string,Derivatives>();
  function derivatives(pose:Pose,q:readonly number[],stickLength:number){
    const key=JSON.stringify([pose,q,stickLength]);
    if(key===cacheKey)return cachedDerivatives;
    const result=new Map<string,Derivatives>();
    for(const [column,coordinate] of KEYS.entries()){
      const h=column<3?1e-3:1e-6;
      const plusPose={...pose,[coordinate]:pose[coordinate]+h},minusPose={...pose,[coordinate]:pose[coordinate]-h};
      const plusQ=solver.inverse(plusPose,[...q]),minusQ=solver.inverse(minusPose,[...q]);
      if(!plusQ.valid||!minusQ.valid)throw Error('动力学雅可比邻域不可达');
      const plus=bodyPoses(plusPose,plusQ.q,stickLength),minus=bodyPoses(minusPose,minusQ.q,stickLength);
      for(const [id,p] of plus){
        const m=minus.get(id)!,j=result.get(id)??{linear:[],angular:[]};
        j.linear.push(p.center.clone().sub(m.center).multiplyScalar(.5/h));
        j.angular.push(rotationVector(p.rotation.clone().multiply(m.rotation.clone().invert())).multiplyScalar(.5/h));
        result.set(id,j);
      }
    }
    cacheKey=key;cachedDerivatives=result;return result;
  }
  function generalizedLoads(pose:Pose,q:readonly number[],stickLength:number,weightN:number,motion:RobotMotion|null){
    const positions=bodyPoses(pose,q,stickLength),jac=derivatives(pose,q,stickLength);
    const gravity=Array(5).fill(0) as number[],inertia=Array(5).fill(0) as number[];
    const reactionGravity=Array(6).fill(0) as number[],reactionInertia=Array(6).fill(0) as number[];
    const payload={id:'payload',massKg:weightN/GRAVITY_MPS2,inertia:new T.Matrix3().set(0,0,0,0,0,0,0,0,0)};
    for(const body of [...bodies,payload]){
      const rotation=new T.Matrix3().setFromMatrix4(new T.Matrix4().makeRotationFromQuaternion(positions.get(body.id)!.rotation));
      const worldInertia=rotation.clone().multiply(body.inertia).multiply(rotation.clone().transpose());
      const state=motion?.[body.id]??{acceleration:[0,0,0],omega:[0,0,0],alpha:[0,0,0]};
      const load=bodyWrench(body.massKg,worldInertia,state,jac.get(body.id)!);
      for(let i=0;i<5;i++){gravity[i]+=load.gravity[i];inertia[i]+=load.inertia[i];}
      const reaction=bodyBaseReaction(body.massKg,worldInertia,state,positions.get(body.id)!.center,baseOriginM);
      for(let i=0;i<6;i++){reactionGravity[i]+=reaction.gravity[i];reactionInertia[i]+=reaction.inertia[i];}
    }
    const baseReaction:BaseReaction={originMm:baseOriginMm,gravity:reactionGravity,inertia:reactionInertia,
      total:reactionGravity.map((value,i)=>value+reactionInertia[i])};
    return {gravity,inertia,baseReaction};
  }
  function torques(pose:Pose,q:readonly number[],stickLength:number,weightN:number,motion:RobotMotion|null):TorqueAnalysis{
    const payload=motion?.payload?.acceleration??[0,0,0];
    // Convert source STEP acceleration back to Fwd / Thrust / Side for the UI.
    const acceleration:Vec3=[-payload[1],payload[0],-payload[2]];
    const invalid=(reason:string):TorqueAnalysis=>({valid:false,reason,gravity:[],inertia:[],total:[],massKg:weightN/GRAVITY_MPS2,acceleration,relativeResidual:Infinity});
    if(!Number.isFinite(weightN)||weightN<0)return invalid('负载重力无效');
    try{
      const j=solver.inverseJacobian(pose,[...q]);if(!j.valid)return invalid(j.reason);
      const loads=generalizedLoads(pose,q,stickLength,weightN,motion);
      const gravity=staticForceAnalysis(j.values,loads.gravity),inertia=staticForceAnalysis(j.values,loads.inertia);
      if(!gravity.valid||!inertia.valid)return invalid('机构动力学分配失败');
      return {valid:true,reason:'',gravity:gravity.torques,inertia:inertia.torques,
        total:gravity.torques.map((v,i)=>v+inertia.torques[i]),massKg:weightN/GRAVITY_MPS2,acceleration,
        relativeResidual:Math.max(gravity.relativeResidual,inertia.relativeResidual),baseReaction:loads.baseReaction};
    }catch(error){return invalid(error instanceof Error?error.message:String(error));}
  }
  return {bodies,bodyPoses,transforms,sample,referenceMotion,reset,generalizedLoads,torques};
}
export type RobotDynamics=ReturnType<typeof createRobotDynamics>;
