import { C, legArguments, type Pose } from './kinematics.ts';
export type V3 = [number, number, number];
export type Leg = { base: V3; tip: V3; end: V3; radius: number; nominal: number };
/** Coordinates are CAD-oriented: X=lateral, Y=along motor rows, Z=out from base.
 * The lateral offsets below are visualization calibration parameters, not solver inputs.
 */
export function geometry(p: Pose, q: number[]): Leg[] {
  return q.map((angle, i) => {
    const pitch = i === 2 || i === 3;
    const side = i < 3 ? -1 : 1;
    const ySign = i === 0 || i === 5 ? 1 : -1;
    const base: V3 = [side * (pitch ? 13 : 98), pitch ? 237.5 : (ySign === 1 ? 47.5 : 142.5), 47.5];
    const a = legArguments(p, i);
    const tip: V3 = [base[0], base[1] - ySign * C.arm * Math.cos(angle), base[2] - C.arm * Math.sin(angle)];
    // STEP right pitch link: ball centers X=13 -> X=89.4346 (outward).
    // The left receiver also has its attachment at negative X. Each leg stays
    // on its own side; subtracting side*w incorrectly crossed the two links.
    // The solver uses its nominal 80 mm lateral offset (CAD measures 76.4346).
    const end: V3 = [base[0] + side * a.w, base[1] + ySign * a.v, base[2] + a.u];
    return { base, tip, end, radius: pitch ? 4 : 6, nominal: a.length };
  });
}
const sub = (a: V3, b: V3): V3 => [a[0]-b[0], a[1]-b[1], a[2]-b[2]];
const dot = (a: V3, b: V3) => a.reduce((s,v,i) => s + v*b[i], 0);
const clamp = (v: number) => Math.max(0, Math.min(1, v));
export function segmentDistance(a0: V3, a1: V3, b0: V3, b1: V3) {
  const u=sub(a1,a0), v=sub(b1,b0), w=sub(a0,b0);
  const a=dot(u,u), b=dot(u,v), c=dot(v,v), d=dot(u,w), e=dot(v,w);
  let s=0, t=0;
  if(a < 1e-12 && c < 1e-12) return Math.hypot(...w);
  if(a < 1e-12) t=clamp(e/c);
  else if(c < 1e-12) s=clamp(-d/a);
  else {
    const denom=a*c-b*b;
    s=denom > 1e-12 ? clamp((b*e-c*d)/denom) : 0;
    t=(b*s+e)/c;
    if(t<0) { t=0; s=clamp(-d/a); }
    else if(t>1) { t=1; s=clamp((b-d)/a); }
  }
  return Math.hypot(...w.map((x,i)=>x+s*u[i]-t*v[i]));
}
/** Only checks the four 220 mm shaft bodies, excluding eye-joint ends.
 * NOT a whole-machine clearance result; pitch frame and joint seats remain uncalibrated.
 */
export function shaftClearance(legs: Leg[]) {
  const shafts = [0,1,4,5].map(i => {
    const l=legs[i], d=sub(l.end,l.tip), length=Math.hypot(...d), inset=(length-220)/2;
    const a=l.tip.map((x,j)=>x+d[j]*inset/length) as V3;
    const b=l.end.map((x,j)=>x-d[j]*inset/length) as V3;
    return {i,a,b};
  });
  let min=Infinity; let pair:number[]=[];
  for(let i=0;i<shafts.length;i++) for(let j=i+1;j<shafts.length;j++) {
    const x=shafts[i], y=shafts[j];
    const clearance=segmentDistance(x.a,x.b,y.a,y.b)-12;
    if(clearance<min) { min=clearance; pair=[x.i,y.i]; }
  }
  return { min, pair };
}
