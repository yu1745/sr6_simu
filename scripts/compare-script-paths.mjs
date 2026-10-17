// Offline comparison. No filtering, serial access, or changes to source scripts.
import {readFileSync,writeFileSync,readdirSync,mkdirSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {spawnSync} from 'node:child_process';
import assert from 'node:assert/strict';
import * as T from 'three';
import {bufferedPoints,septic} from '../lib/sr6/buffered-trajectory.ts';
import {sampleTrack} from '../lib/sr6/funscript.ts';
import {ZERO} from '../lib/sr6/kinematics.ts';
import {createCadKinematics} from '../lib/sr6/cad-kinematics.ts';
import {createRobotDynamics,bodyBaseReaction} from '../lib/sr6/rigid-dynamics.ts';

const folder=process.env.SCRIPT_DIR??'C:/Users/wangyu/Downloads/WooHanKyung Booty Bounce';
const out=process.env.COMPARISON_OUT??'work/script-comparison';mkdirSync(out,{recursive:true});
const interpolation=process.env.SCRIPT_INTERPOLATION??'linear';
const mode=process.argv[2]??'prepare';
const axes=[['L0','',0],['R1','.roll',4],['R2','.pitch',5]];
const quant=p=>Math.round(p*99.99)/10000;
const save=(name,x)=>writeFileSync(`${out}/${name}.json`,JSON.stringify(x,null,2));
// Port of MFP KeyframeCollection.Interpolate and MathUtils.Pchip.
function pchip(points,t){
  if(t<=points[0].at)return points[0].pos;
  if(t>=points.at(-1).at)return points.at(-1).pos;
  let lo=0,hi=points.length-1;
  while(hi-lo>1){const mid=(lo+hi)>>1;if(points[mid].at<=t)lo=mid;else hi=mid;}
  const b=points[lo],c=points[hi];
  const a=points[lo-1]??{at:3*b.at-2*c.at,pos:b.pos};
  const d=points[hi+1]??{at:3*c.at-2*b.at,pos:c.pos};
  function slope(a,b,c){
    const h0=b.at-a.at,h1=c.at-b.at,d0=(b.pos-a.pos)/h0,d1=(c.pos-b.pos)/h1;
    const w0=2*h1+h0,w1=h1+2*h0,s=(w0+w1)/(w0/d0+w1/d1);
    return !Number.isFinite(s)||d0*d1<0?0:s;
  }
  const dt=c.at-b.at,dx=t-b.at,u=dx/dt,r=1-u;
  return r*r*(b.pos*(1+2*u)+slope(a,b,c)*dx)+u*u*(c.pos*(3-2*u)-dt*slope(b,c,d)*r);
}
function sample(points,t){
  if(t<=points[0].at)return [points[0].pos,0,0,0];
  if(t>=points.at(-1).at)return [points.at(-1).pos,0,0,0];
  let lo=0,hi=points.length-1;
  while(hi-lo>1){const m=(hi+lo)>>1;if(points[m].at<=t)lo=m;else hi=m;}
  const a=points[lo],b=points[hi],dt=(b.at-a.at)/1000,d=b.pos-a.pos,s=septic((t-a.at)/(b.at-a.at));
  return [a.pos+d*s.position,d*s.velocity/dt,d*s.acceleration/dt**2,d*s.jerk/dt**3];
}
if(mode==='prepare'){
  const groups=[];
  for(const [gi,file] of readdirSync(folder).filter(f=>f.endsWith('.funscript')&&!/\.(roll|pitch)\.funscript$/.test(f)).sort().entries()){
    const stem=file.replace(/\.funscript$/,'');
    const tracks=axes.map(([axis,suffix,index])=>{
      const filename=`${stem}${suffix}.funscript`,bytes=readFileSync(`${folder}/${filename}`),raw=JSON.parse(bytes.toString()).actions;
      assert(raw.every((p,i)=>Number.isFinite(p.pos)&&p.pos>=0&&p.pos<=100&&p.at>=0&&(!i||p.at>raw[i-1].at)),'Invalid/duplicate/out-of-order nodes: abort instead of cleaning');
      const points=bufferedPoints(raw).map(p=>({at:p.at,pos:quant(p.pos)}));
      const peaks=[0,0,0],witnesses=[],safe=[0,0,0];
      for(let i=1;i<points.length;i++){
        const a=points[i-1],b=points[i],dt=(b.at-a.at)/1000,d=Math.abs(b.pos-a.pos);
        assert(dt>=.001&&dt<=60);
        [2.1875,7.513188404399293,52.5].forEach((c,k)=>{
          const value=c*d/dt**(k+1);
          if(value>peaks[k]){peaks[k]=value;witnesses[k]={fromMs:a.at,toMs:b.at,delta:d,durationSeconds:dt};}
          // Also cover MFP's two-count amplitude margin and wire rounding.
          safe[k]=Math.max(safe[k],c*(d+.0003)/dt**(k+1)*1.01);
        });
      }
      return {axis,index,filename,sha256:createHash('sha256').update(bytes).digest('hex'),raw,points,peaks,safe,witnesses};
    });
    const duration=Math.max(...tracks.map(t=>t.raw.at(-1).at));
    const limits=[tracks[0].safe,[0,1,2].map(k=>Math.max(...tracks.slice(1).map(t=>t.safe[k])))];
    const n=Math.ceil(duration)+1,input=Buffer.alloc(n*24);
    // MFP default fixed update: 100 Hz, I10, 4 digits, dirty values only.
    const starts=tracks.map(t=>quant(t.raw[0].pos)),goals=[...starts],sent=tracks.map(t=>t.raw[0].pos/100),received=[0,0,0];
    for(let ms=0;ms<n;ms++){
      const values=Array(6).fill(.5);
      tracks.forEach((track,k)=>{
        let p=starts[k]+(goals[k]-starts[k])*Math.min(1,(ms-received[k])/10);
        if(ms%10===0){
          const value=(interpolation==='pchip'?pchip:sampleTrack)(track.raw,ms)/100;
          if(Math.abs(value-sent[k])*10000>=1){starts[k]=p;goals[k]=quant(value*100);received[k]=ms;sent[k]=value;}
        }
        values[track.index]=p;
      });
      values.forEach((v,i)=>input.writeFloatLE(v,ms*24+i*4));
    }
    const id=`group${gi+1}`;writeFileSync(`${out}/${id}-targets.bin`,input);
    groups.push({id,stem,duration,samples:n,limits,tracks});
    console.log(JSON.stringify({id,stem,duration,limits,tracks:tracks.map(({axis,raw,points,peaks,witnesses})=>({axis,rawNodes:raw.length,v4Nodes:points.length,peaks,witnesses}))}));
  }
  save('inputs',{settings:{model:'6010',l0SpanMm:40,rollSpanDeg:20,pitchSpanDeg:20,rate:1,payloadKg:0,stickCompensation:false,filter:false,tickMs:1,sendIntervalMs:10,interpolation},groups});
}else if(mode==='replay'){
  const {groups}=JSON.parse(readFileSync(`${out}/inputs.json`));
  for(const g of groups){
    const result=spawnSync('work/script-comparison/replay.exe',[`${out}/${g.id}-targets.bin`,`${out}/${g.id}-pva.bin`,'1000',...g.limits.flat().map(String)],{encoding:'utf8'});
    console.log(g.id,result.stdout,result.stderr);assert.equal(result.status,0);
    save(`${g.id}-solver`,JSON.parse(result.stdout));
  }
}else if(mode==='analyze'){
  const input=JSON.parse(readFileSync(`${out}/inputs.json`));
  const data=JSON.parse(readFileSync('public/sr6-6010-cad.json')),mass=JSON.parse(readFileSync('public/sr6-6010-mass-properties.json'));
  const solver=createCadKinematics(data),model=createRobotDynamics(data,mass,solver);
  const origin=new T.Vector3(...mass.parts.find(p=>p.id===1&&p.motion==='fixed').centerMm).multiplyScalar(.001);
  const span=[input.settings.l0SpanMm,input.settings.rollSpanDeg*Math.PI/180,input.settings.pitchSpanDeg*Math.PI/180];
  const keys=['y','roll','pitch'];
  const h=Number(process.env.DERIVATIVE_H??'.0001'),step=Number(process.env.DYNAMICS_STEP_MS??'2');
  const results=[];
  function reaction(state){
    const at=offset=>{
      const pose={...ZERO};keys.forEach((key,k)=>pose[key]=(state[k][0]-.5+state[k][1]*offset+.5*state[k][2]*offset**2)*span[k]);
      const ik=solver.inverse(pose);assert(ik.valid,ik.reason);return {pose,q:ik.q};
    };
    const p=at(0),motion=model.referenceMotion(p.pose,p.q,0,at,h),positions=model.bodyPoses(p.pose,p.q,0),result=Array(6).fill(0);
    for(const body of model.bodies){
      const b=positions.get(body.id),rotation=new T.Matrix3().setFromMatrix4(new T.Matrix4().makeRotationFromQuaternion(b.rotation));
      const inertia=rotation.clone().multiply(body.inertia).multiply(rotation.clone().transpose());
      const r=bodyBaseReaction(body.massKg,inertia,motion[body.id],b.center,origin).inertia;
      r.forEach((v,i)=>result[i]+=v);
    }
    return result;
  }
  for(const g of input.groups){
    const bytes=readFileSync(`${out}/${g.id}-pva.bin`);assert.equal(bytes.length,g.samples*72);
    const paths={};
    for(const path of ['v4','ruckig']){
      const stateAt=ms=>g.tracks.map(t=>path==='v4'?sample(t.points,ms):[0,1,2].map(k=>bytes.readFloatLE(Math.min(g.samples-1,Math.max(0,Math.round(ms)))*72+(6*k+t.index)*4)));
      const peak=Array(8).fill(0),sum=Array(8).fill(0),peakAt=Array(8).fill(0),rateSum=Array(8).fill(0),ratePeak=Array(8).fill(0);
      const ranges=axes.map(()=>[Infinity,-Infinity]),error=Array(3).fill(0),axisPeaks=axes.map(()=>[0,0,0]);
      let previous=null,previousState=null,count=0;
      const trace=['time_s,Fx_N,Fy_N,Fz_N,Mx_Nm,My_Nm,Mz_Nm,L0,R1,R2'];
      for(let ms=0;ms<=g.duration;ms+=step){
        const state=stateAt(ms),r=reaction(state),metrics=[...r,Math.hypot(...r.slice(0,3)),Math.hypot(...r.slice(3))];
        metrics.forEach((v,i)=>{sum[i]+=v*v;if(Math.abs(v)>peak[i]){peak[i]=Math.abs(v);peakAt[i]=ms;}});
        if(previous){
          const dr=r.map((v,i)=>(v-previous[i])/(step*.001));
          [...dr,Math.hypot(...dr.slice(0,3)),Math.hypot(...dr.slice(3))].forEach((v,i)=>{rateSum[i]+=v*v;ratePeak[i]=Math.max(ratePeak[i],Math.abs(v));});
        }
        state.forEach((s,k)=>{
          ranges[k][0]=Math.min(ranges[k][0],s[0]);ranges[k][1]=Math.max(ranges[k][1],s[0]);
          error[k]+=(s[0]-quant(sampleTrack(g.tracks[k].raw,ms)))**2;
          const j=path==='v4'?s[3]:previousState?(s[2]-previousState[k][2])/(step*.001):0;
          [s[1],s[2],j].forEach((v,i)=>axisPeaks[k][i]=Math.max(axisPeaks[k][i],Math.abs(v)));
        });
        trace.push([ms/1000,...r,...state.map(s=>s[0])].join(','));
        previous=r;previousState=state;count++;
      }
      const turns=g.tracks.map((t,k)=>{
        const e=t.points.slice(1,-1).map(p=>Math.abs(stateAt(p.at)[k][0]-p.pos));
        return {axis:t.axis,meanErrorPercent:100*e.reduce((a,b)=>a+b,0)/e.length,maxErrorPercent:100*Math.max(...e)};
      });
      paths[path]={count,peak,peakAtMs:peakAt,rms:sum.map(v=>Math.sqrt(v/count)),rateRms:rateSum.map(v=>Math.sqrt(v/(count-1))),ratePeak,ranges,linearScriptRmsErrorPercent:error.map(v=>100*Math.sqrt(v/count)),turns,axisPeaks};
      writeFileSync(`${out}/${g.id}-${path}-trace.csv`,trace.join('\n'));
      console.log(g.id,path,JSON.stringify(paths[path]));
    }
    results.push({id:g.id,stem:g.stem,duration:g.duration,paths,reductionPercent:Object.fromEntries(['peak','rms','rateRms'].map(m=>[m,paths.ruckig[m].map((v,i)=>100*(1-paths.v4[m][i]/v))]))});
  }
  save(`results-${step}ms-h${h}`,{settings:input.settings,dynamicsStepMs:step,derivativeH:h,baseOriginMm:origin.clone().multiplyScalar(1000).toArray(),coordinates:'source STEP; X up; dynamic inertial reaction only; gravity excluded',metrics:['Fx','Fy','Fz','Mx','My','Mz','forceNorm','momentNorm'],cadSha256:data.sha256,results});
}
