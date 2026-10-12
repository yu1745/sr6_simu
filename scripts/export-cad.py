"""Local-only STEP tessellation. CAD reference, not yet a calibrated moving rig."""
import json
import hashlib
from pathlib import Path
import cadquery as cq

root = Path(__file__).resolve().parents[1]
source = root.parent / 'sr6_mini.stp'
shape = cq.importers.importStep(str(source)).val()
def bounds(s):
    b = s.BoundingBox()
    return {'min': [b.xmin, b.ymin, b.zmin], 'max': [b.xmax, b.ymax, b.zmax]}

parts = []
for i, solid in enumerate(shape.Solids()):
    vertices, triangles = solid.tessellate(0.15, 0.2)
    parts.append({'id': i, 'bounds': bounds(solid), 'center': solid.Center().toTuple(),
                  'positions': [round(n, 5) for v in vertices for n in v.toTuple()],
                  'indices': [n for t in triangles for n in t]})
target = root / 'public' / 'sr6-cad.json'
target.write_text(json.dumps({'units': 'mm', 'source': source.name,
    'sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
    'bounds': bounds(shape), 'parts': parts}, separators=(',', ':')), encoding='utf-8')
print(f'{len(parts)} solids; {target.stat().st_size} bytes; {target}')
