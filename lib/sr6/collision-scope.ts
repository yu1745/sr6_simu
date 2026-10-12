import { CAD_LINKS, type CadData } from './cad-rig.ts';

/** User-defined scope for this calibrated assembly; exclusions are intentional. */
export function collisionParts(data:CadData){
  const excluded=new Set([
    1, // base
    2,3,4,5,6,7,8,9,10,11,12,13, // motor bodies and caps
    20,25,30,35, // four guide shafts
    39,40,41,42,43,44,45,46,47,48, // receiver accessories; keep body 38
    54,55,56,57,64,65,66,67,68,69,70,71, // receiver fasteners/spacers
  ]);
  for(const link of CAD_LINKS){
    excluded.add(link.crank);
    excluded.add(link.lower);
    const spec=data.spec.links.find(s=>s.motor===link.motor) as typeof data.spec.links[number]&{lower_eye:number};
    if(!spec||!Number.isInteger(spec.lower_eye))throw new Error('Missing lower-eye collision calibration');
    excluded.add(spec.lower_eye);
  }
  return data.parts.filter(p=>!excluded.has(p.id));
}
