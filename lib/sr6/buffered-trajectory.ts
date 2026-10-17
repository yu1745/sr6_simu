// Port of local MultiFunPlayer TCodeBufferedPlan.TurningPoints / Smooth.
// Positions here are 0..100 and timestamps are milliseconds (MFP uses 0..1 / s).
export type TrajectoryPoint={at:number;pos:number};
export type ScriptSample={timeMs:number;epoch:number};
export type PlaybackState=ScriptSample&{playing:boolean};
export type ReferenceSample=ScriptSample&{poseAtOffset:(seconds:number)=>import('./kinematics.ts').Pose};
export function turningPoints(points:TrajectoryPoint[]):TrajectoryPoint[]{
  if(!points.length)return [];
  const turns=[points[0]];let direction=0,extreme=points[0],departure=points[0];
  for(const point of points.slice(1)){
    if(direction===0){
      if(point.pos===turns[0].pos)departure=point;
      if(Math.abs(point.pos-turns[0].pos)>=1-1e-10){
        if(departure.at>turns[0].at)turns.push(departure);
        direction=Math.sign(point.pos-turns[0].pos);extreme=point;
      }
    }else if(direction*(point.pos-extreme.pos)>=0)extreme=point;
    else if(direction*(extreme.pos-point.pos)>=1-1e-10){turns.push(extreme);direction=-direction;extreme=point;}
  }
  if(direction!==0&&Math.abs(extreme.pos-turns[turns.length-1].pos)>=1-1e-10)turns.push(extreme);
  if(turns[turns.length-1].at!==points[points.length-1].at)turns.push(points[points.length-1]);
  return turns;
}
export function septic(u:number){
  if(u<=0)return {position:0,velocity:0,acceleration:0,jerk:0};
  if(u>=1)return {position:1,velocity:0,acceleration:0,jerk:0};
  const v=1-u;
  return {position:u**4*(35+u*(-84+u*(70-20*u))),velocity:140*u**3*v**3,
    acceleration:420*u*u*v*v*(1-2*u),jerk:840*u*v*(1-5*u+5*u*u)};
}
const cache=new WeakMap<TrajectoryPoint[],Map<number,TrajectoryPoint[]>>();
export function bufferedPoints(points:TrajectoryPoint[],rate=1){
  let rates=cache.get(points);if(!rates){rates=new Map();cache.set(points,rates);}
  const saved=rates.get(rate);if(saved)return saved;
  const turns=turningPoints(points),result=turns.length?[turns[0]]:[];
  for(let i=1;i<turns.length;i++){
    const a=turns[i-1],b=turns[i],count=Math.ceil((b.at-a.at)/rate/60000);
    for(let n=1;n<=count;n++)result.push({at:a.at+(b.at-a.at)*n/count,pos:a.pos+(b.pos-a.pos)*n/count});
  }
  rates.set(rate,result);return result;
}
/** Derivatives are position units per real second, second², second³. */
export function sampleBuffered(points:TrajectoryPoint[],time:number,rate=1){
  const p=bufferedPoints(points,rate);
  const hold=(position:number)=>({position,velocity:0,acceleration:0,jerk:0});
  if(!p.length)return hold(50);
  if(time<=p[0].at)return hold(p[0].pos);
  if(time>=p[p.length-1].at)return hold(p[p.length-1].pos);
  let lo=0,hi=p.length-1;
  while(hi-lo>1){const mid=(lo+hi)>>1;if(p[mid].at<=time)lo=mid;else hi=mid;}
  const a=p[lo],b=p[hi],s=septic((time-a.at)/(b.at-a.at)),duration=(b.at-a.at)/1000/rate,d=b.pos-a.pos;
  return {position:a.pos+d*s.position,velocity:d*s.velocity/duration,
    acceleration:d*s.acceleration/duration**2,jerk:d*s.jerk/duration**3};
}
