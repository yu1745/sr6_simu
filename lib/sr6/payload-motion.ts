import type { Pose } from './kinematics.ts';
import { NO_ACCELERATION, type Acceleration } from './statics.ts';

// Translational part of ctw_esp32s3_0909/include/executor/payload_motion_estimator.hpp.
// Timestamp is real playback time (ms), not script time: changing rate changes a.
// Defaults match firmware: two 10 Hz first-order filters, |a_i| <= 20 m/s².
export const MOTION_FILTER_HZ=10;
export const MAX_LINEAR_ACCELERATION=20;
// CAD rendering can take >50 ms; firmware's watchdog threshold is unsuitable here.
export const MAX_SAMPLE_GAP_MS=500;
export type MotionSampleStatus={state:'idle'|'paused'|'warming'|'tracking'|'gap';intervalMs:number};
export function createPayloadMotionEstimator(){
  let status:MotionSampleStatus={state:'idle',intervalMs:0};
  let previous:number[]|null=null,previousTime=0;
  let velocity=[0,0,0],acceleration=[0,0,0];
  function reset(){previous=null;previousTime=0;velocity=[0,0,0];acceleration=[0,0,0];status={state:'idle',intervalMs:0};}
  return {
    reset,
    getStatus:():MotionSampleStatus=>({...status}),
    update(receiver:Pose,timeMs:number):Acceleration{
      const position=[receiver.x*.001,receiver.y*.001,receiver.z*.001];
      if(!position.every(Number.isFinite)||!Number.isFinite(timeMs)||timeMs<=0){reset();return NO_ACCELERATION;}
      if(!previous){previous=position;previousTime=timeMs;status={state:'warming',intervalMs:0};return NO_ACCELERATION;}
      const dt=(timeMs-previousTime)*.001;
      if(dt<=0||dt>MAX_SAMPLE_GAP_MS*.001){reset();previous=position;previousTime=timeMs;status={state:'gap',intervalMs:dt*1000};return NO_ACCELERATION;}
      if(dt<.001)return acceleration.map(a=>Math.max(-MAX_LINEAR_ACCELERATION,Math.min(MAX_LINEAR_ACCELERATION,a))) as [number,number,number];
      status={state:'tracking',intervalMs:dt*1000};
      const alpha=1-Math.exp(-2*Math.PI*MOTION_FILTER_HZ*dt);
      const nextVelocity=position.map((p,i)=>velocity[i]+alpha*((p-previous![i])/dt-velocity[i]));
      acceleration=acceleration.map((a,i)=>a+alpha*((nextVelocity[i]-velocity[i])/dt-a));
      velocity=nextVelocity;previous=position;previousTime=timeMs;
      return acceleration.map(a=>Math.max(-MAX_LINEAR_ACCELERATION,Math.min(MAX_LINEAR_ACCELERATION,a))) as [number,number,number];
    },
  };
}
