"""Measure source and baseline solids before constructing the 6010 variant."""
import json
from pathlib import Path
import cadquery as cq

ROOT = Path(__file__).resolve().parents[1]
result = {}
for label, path in [('6010', ROOT.parent/'sr6_minia.stp'), ('8108', ROOT/'cad/sr6_mini_complete.stp')]:
    print(f'Import {label}', flush=True)
    solids = cq.importers.importStep(str(path)).val().Solids()
    rows = []
    for i, s in enumerate(solids):
        b = s.BoundingBox()
        circles = {}
        for e in s.Edges():
            if e.geomType() == 'CIRCLE':
                r = round(e.radius(), 4)
                if r >= 10:
                    c = [round(v, 5) for v in e.arcCenter().toTuple()]
                    circles[(r, tuple(c))] = {'radius': r, 'center': c}
        rows.append(dict(id=i, volume=s.Volume(), center=s.Center().toTuple(),
                         bounds=[b.xmin,b.ymin,b.zmin,b.xmax,b.ymax,b.zmax], circles=list(circles.values())))
        print(i, round(s.Volume(), 2), [round(v, 3) for v in (b.xmin,b.ymin,b.zmin,b.xmax,b.ymax,b.zmax)], flush=True)
    result[label] = rows
(ROOT/'work/6010-inspection.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
