"""Build the 6010 variant using only the source base and six motor assemblies.

All other B-reps and preview meshes come directly from the current 8108 model.
Run with: conda run -n sr6_cad python scripts/build-6010.py
"""
import copy
import hashlib
import json
from pathlib import Path

import cadquery as cq
import numpy as np
from OCP.gp import gp_Trsf
from cad_replacements import circle_centers

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'cad/inputs/sr6_minia.stp'
BASELINE = ROOT/'cad/sr6_mini_complete.stp'
TARGET = ROOT/'cad/sr6_6010_complete.stp'
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
print('Import source and baseline', flush=True)
source = cq.importers.importStep(str(SOURCE)).val().Solids()
original = cq.importers.importStep(str(BASELINE)).val().Solids()
data = json.loads((ROOT/'public/sr6-cad.json').read_text(encoding='utf-8'))
assert len(source) == 42 and len(original) == len(data['parts']) == 78
assert data['sha256'] == digest(BASELINE)
# Cyclic proper rotation: source (X,Y,Z) -> baseline (Y,Z,X).
# Bottom plane Z=-4 -> Y=0; first row Y=47.5 -> X=50;
# lateral centre X=0 -> Z=-54. No scale or geometry alteration.
rotation = np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
translation = np.array([2.5,4.,-54.])
tr = gp_Trsf()
tr.SetValues(*map(float, np.column_stack([rotation,translation]).ravel()))
placement = cq.Matrix(tr)
# Stable simulator IDs, source import-order IDs. Every motor has body + rear cover.
mapping = {1:0, 2:1,3:2, 4:3,5:4, 8:5,9:6, 10:7,11:8, 6:9,7:10, 12:11,13:12}
parts = list(original)
for target_id, source_id in mapping.items():
    parts[target_id] = source[source_id].transformShape(placement)

# Verify all twelve mounting-opening edge centres against the existing base.
old_centers = circle_centers(original[1],33.5)
new_centers = circle_centers(parts[1],33.5)
assert len(old_centers) == len(new_centers) == 12
mount_error = max(min(float(np.linalg.norm(a-b)) for b in old_centers) for a in new_centers)
assert mount_error < .001, mount_error
# Motor output seating planes at radius 16.5 match the corresponding source
# crank attachment plane; preserve the existing crank location and zero angle.
seat_checks = []
for motor_id, crank_id, sign in [(2,0,1),(4,14,1),(8,15,-1),(10,16,-1),(6,17,-1),(12,58,1)]:
    seats = circle_centers(parts[motor_id],16.5)
    assert seats
    bb = original[crank_id].BoundingBox()
    seat = min(seats, key=lambda p: abs(p[2]-(bb.zmin if sign>0 else bb.zmax)))
    plane = bb.zmin if sign>0 else bb.zmax
    error = abs(float(seat[2])-plane)
    assert error < .001, (motor_id,error)
    seat_checks.append({'motor_part':motor_id,'crank_part':crank_id,'seat_error_mm':error})

names = {int(k):v for k,v in data['spec']['part_names'].items()}
for i in mapping:
    names[i] = 'U_body_6010' if i == 1 else f'6010_motor_{i}_' + ('cover' if i%2 else 'body')
assembly = cq.Assembly(name='sr6_6010_complete')
for i, part in enumerate(parts):
    assembly.add(part,name=names[i],color=cq.Color(.25,.32,.38) if i in mapping and i!=1 else cq.Color(.5,.65,.72))
print('Export STEP', flush=True)
assembly.export(str(TARGET),exportType='STEP')
spec = copy.deepcopy(data['spec'])
spec.update(output=TARGET.name,sha256=digest(TARGET),part_names=names,variant='6010')
spec['variant_sources'] = {'reference':SOURCE.name,'reference_sha256':digest(SOURCE),
    'baseline':BASELINE.name,'baseline_sha256':digest(BASELINE),
    'rotation':rotation.tolist(),'translation':translation.tolist(),'scale':1,
    'replacement_map':mapping,'preserved_ids':[i for i in range(78) if i not in mapping]}
spec['replacements']['motor_base'] = {'input':SOURCE.name,'part_id':1,'source_id':0,
    'rotation':rotation.tolist(),'translation':translation.tolist(),'part_scale':1,
    'mount_center_max_error_mm':mount_error}
spec['inherited_invalid_source_solids'] = [i for i,p in enumerate(parts) if not p.isValid()]
result = copy.deepcopy(data)
result.update(source=TARGET.name,sha256=spec['sha256'],spec=spec)
print('Tessellate 13 replacement entities; retain 65 baseline meshes exactly',flush=True)
for i in mapping:
    p=parts[i]
    vertices,triangles=p.tessellate(.15,.2)
    b=p.BoundingBox()
    result['parts'][i]={'id':i,'name':names[i],'center':p.Center().toTuple(),
        'bounds':{'min':[b.xmin,b.ymin,b.zmin],'max':[b.xmax,b.ymax,b.zmax]},
        'positions':[round(n,5) for v in vertices for n in v.toTuple()],
        'indices':[n for t in triangles for n in t]}
for i in spec['variant_sources']['preserved_ids']:
    assert result['parts'][i] == data['parts'][i]
    assert parts[i].isSame(original[i])
assert spec['links'] == data['spec']['links']
(ROOT/'public/sr6-6010-cad.json').write_text(json.dumps(result,separators=(',',':')),encoding='utf-8')
(ROOT/'cad/assembly-spec-6010.json').write_text(json.dumps(spec,indent=2),encoding='utf-8')
report={'source_sha256':digest(SOURCE),'baseline_sha256':digest(BASELINE),'output_sha256':digest(TARGET),
    'replaced_count':len(mapping),'preserved_count':78-len(mapping),'mount_center_max_error_mm':mount_error,
    'motor_seats':seat_checks,'links_unchanged':True,'preserved_breps_and_meshes_identical':True,
    'invalid_source_solids':spec['inherited_invalid_source_solids']}
(ROOT/'cad/validation-6010.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2),flush=True)
