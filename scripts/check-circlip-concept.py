"""Thin retaining-ring screening, NOT an exact DIN 471 product contour."""
import json
from pathlib import Path
import cadquery as cq

ROOT = Path(__file__).resolve().parents[1]
housing, ball = cq.importers.importStep(str(ROOT/'cad/inputs/SA6TK-jlcfa.step')).val().Solids()
rows = []
for radius in [4.0, 4.15, 4.2, 5.55]:
    # Ball width 9 mm. A single ring contacts its +Z end face.
    ring = cq.Solid.makeCylinder(radius, .7, cq.Vector(0,0,4.5)).cut(
        cq.Solid.makeCylinder(2.85, .7, cq.Vector(0,0,4.5)))
    lo, hi = 0., 35.
    for _ in range(15):
        angle = (lo+hi)/2
        volume = abs(housing.rotate((0,0,0),(1,0,0),angle).intersect(ring).Volume())
        if volume > 1e-7: hi = angle
        else: lo = angle
    rows.append({'outerRadiusMm': radius, 'firstOverlapAngleDeg': [lo,hi],
                 'overlapAt28_7725DegMm3': abs(housing.rotate((0,0,0),(1,0,0),28.7725).intersect(ring).Volume())})
report = {
    'source': 'https://www.beneri.com/en/prodotto/din-471',
    'nominalDIN471size6': {'shaftDiameterMm':6, 'thicknessMm':.7, 'freeInnerDiameterMm':5.6,
                         'grooveDiameterMm':5.7, 'grooveWidthMm':.8, 'bApproxMm':1.3},
    'model': 'Full annular surrogate, no opening or ears. Radius 4.15 = groove radius 2.85 + approximate band width 1.3. Not manufacturer CAD; not an exact circlip tilt limit.',
    'scope': 'Single housing versus one ring, ideal clearance from support. No strength, axial play or full assembly validation.',
    'results': rows}
(ROOT/'cad/circlip-concept-screen.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
