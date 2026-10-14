import type { Pose } from './kinematics';

export type StickSettings = {enabled:boolean;length:number};
export const DEFAULT_STICK:StickSettings={enabled:false,length:220};
// The mounted rod extends along receiver -y (STEP -X at neutral).
// For receiver up axis u: receiver = tip + L*u - L*y_hat.
// Tip translations remain relative to the neutral tip; R0 does not tilt the rod.
export function receiverFromTip(tip:Pose,length:number):Pose {
  return {...tip,x:tip.x-length*Math.sin(tip.pitch),
    y:tip.y-length*(1-Math.cos(tip.pitch)*Math.cos(tip.roll)),
    z:tip.z+length*Math.cos(tip.pitch)*Math.sin(tip.roll)};
}
export function tipFromReceiver(receiver:Pose,length:number):Pose {
  return receiverFromTip(receiver,-length);
}
type Solution={q:number[];valid:boolean;reason:string};
export function solveStick(tip:Pose,settings:StickSettings,solve:(p:Pose)=>Solution){
  const length=settings.enabled?settings.length:0;
  const full=receiverFromTip(tip,length),result=solve(full);
  if(result.valid||length===0)return {...result,receiver:full,scale:1};
  const raw=solve(tip);
  if(!raw.valid)return {...raw,receiver:tip,scale:0};
  let low=0,high=1,best=raw,receiver=tip;
  for(let i=0;i<12;i++){
    const mid=(low+high)/2,next=receiverFromTip(tip,mid*length),candidate=solve(next);
    if(candidate.valid){low=mid;best=candidate;receiver=next;}else high=mid;
  }
  return {...best,receiver,scale:low};
}
