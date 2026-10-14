"""Exact B-rep checks of supplier housing and newly mounted spacers."""
import json,math
from pathlib import Path
import cadquery as cq
import numpy as np
from OCP.gp import gp_Trsf
ROOT=Path(__file__).resolve().parents[1]
parts=cq.importers.importStep(str(ROOT/'cad/sr6_mini_complete.stp')).val().Solids()
spec=json.loads((ROOT/'cad/assembly-spec.json').read_text())
def moved(s,values):
 m=np.array(values).reshape(4,4).T;t=gp_Trsf();t.SetValues(*m[:3,:].flatten().tolist());return s.transformShape(cq.Matrix(t))
def overlap(a,b):
 x,y=a.BoundingBox(),b.BoundingBox()
 if any(getattr(x,k+'max')<getattr(y,k+'min')+1e-7 or getattr(y,k+'max')<getattr(x,k+'min')+1e-7 for k in 'xyz'):return 0.
 return abs(a.intersect(b).Volume())
housing,ball=cq.importers.importStep(str(ROOT/'cad/inputs/SA6TK-jlcfa.step')).val().Solids()
washer=cq.importers.importStep(str(ROOT/'cad/SA6TK_M6_profile_washer_H5.step')).val()
washer=washer.translate((0,0,4.5));pin=cq.Solid.makeCylinder(3,40,cq.Vector(0,0,-20))
local=[]
for angle in range(0,36):
 # Two orthogonal tilt directions, including the shank plane.
 for axis in [(1,0,0),(0,1,0)]:
  h=housing.rotate((0,0,0),axis,angle)
  local.append({'angle':angle,'axis':axis,'washer_gap_mm':h.distance(washer),'washer_overlap_mm3':overlap(h,washer),'pin_overlap_mm3':overlap(h,pin)})
print('Local first contact',next((r for r in local if r['washer_overlap_mm3']>1e-4 or r['pin_overlap_mm3']>1e-4),None),flush=True)
poses=json.loads((ROOT/'cad/washer-sweep-poses.json').read_text(encoding='utf-8-sig'))
rows=[]
hardware=[38,54,55,56,57,*range(64,78)]
for i,case in enumerate(poses):
 row={k:v for k,v in case.items() if k!='matrices'};row['contacts']=[];row['upper_joint_angles_deg']=[]
 if case['valid']:
  m=case['matrices'];p={j:moved(parts[j],m[str(j)]) for j in set(hardware+[l['upper_eye'] for l in spec['links']])}
  for l in spec['links']:
   e=l['upper_eye']
   f=max((f for f in parts[e].Faces() if f.geomType()=='PLANE'),key=lambda f:f.Area())
   normal=np.array(f.normalAt().toTuple());mr=np.array(m[str(e)]).reshape(4,4).T[:3,:3];mp=np.array(m['38']).reshape(4,4).T[:3,:3]
   row['upper_joint_angles_deg'].append(round(math.degrees(math.acos(np.clip(abs((mr@normal)@(mp@np.array([0,0,1]))),0,1))),4))
   for j in hardware:
    v=overlap(p[e],p[j])
    if v>1e-3:row['contacts'].append({'parts':[e,j],'volume_mm3':round(v,6)})
  eyes=[l['upper_eye'] for l in spec['links']]
  for a,e in enumerate(eyes):
   for j in eyes[a+1:]:
    v=overlap(p[e],p[j])
    if v>1e-3:row['contacts'].append({'parts':[e,j],'volume_mm3':round(v,6)})
 rows.append(row)
 print(i,case['stickLength'],case['roll'],case['pitch'],'valid',case['valid'],'contacts',len(row['contacts']),flush=True)
(ROOT/'cad/washer-clearance-report.json').write_text(json.dumps({'local_joint_sweep':local,'assembly_samples':rows,'note':'Sampled nominal rigid geometry; not bearing load rating or continuous workspace certification.'},indent=2))
