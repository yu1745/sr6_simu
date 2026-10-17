"""Verify selected workspace witnesses against STEP B-rep, independently of mesh CSG."""
import json
import sys
from pathlib import Path
import cadquery as cq
from OCP.gp import gp_Trsf

ROOT = Path(__file__).resolve().parents[1]
cases = json.loads((ROOT / (sys.argv[1] if len(sys.argv)>1 else 'work/workspace-scan/witnesses.json')).read_text(encoding='utf-8'))
spec = json.loads((ROOT / 'cad/assembly-spec.json').read_text())
solids = cq.importers.importStep(str(ROOT / 'cad/sr6_mini_complete.stp')).val().Solids()
platform = {38, 54, 55, 56, 57, *range(64, 78)}
groups = {i: 'platform' for i in platform}
exclude = set()
for link in spec['links']:
    groups[link['upper_ball']] = 'platform'
    groups[link['upper_eye']] = f"rod{link['motor']}"
    exclude.add(frozenset([link['upper_ball'], link['upper_eye']]))
groups[49] = 'rod3'
groups[59] = 'rod2'

def moved(shape, matrix):
    tr = gp_Trsf()
    tr.SetValues(*[matrix[col * 4 + row] for row in range(3) for col in range(4)])
    return shape.transformShape(cq.Matrix(tr))

output = []
for case in cases:
    parts = {i: moved(solids[i], case['matrices'][str(i)]) for i in groups}
    boxes = {i: p.BoundingBox() for i, p in parts.items()}
    hits = []
    ids = sorted(parts)
    for ai, a in enumerate(ids):
        for b in ids[ai+1:]:
            if groups[a] == groups[b] or frozenset([a, b]) in exclude:
                continue
            x, y = boxes[a], boxes[b]
            if any(getattr(x,k+'max') < getattr(y,k+'min')+1e-7 or getattr(y,k+'max') < getattr(x,k+'min')+1e-7 for k in 'xyz'):
                continue
            volume = abs(parts[a].intersect(parts[b]).Volume())
            if volume > 1e-5:
                hits.append({'parts': [a,b], 'volumeMm3': volume})
    row = {k: v for k,v in case.items() if k != 'matrices'}
    row['brepHits'] = hits
    output.append(row)
    print(json.dumps(row), flush=True)
(ROOT / (sys.argv[2] if len(sys.argv)>2 else 'cad/workspace-220mm-12_5deg-brep.json')).write_text(json.dumps(output,indent=2),encoding='utf-8')
