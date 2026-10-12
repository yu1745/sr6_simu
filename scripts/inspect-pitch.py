import cadquery as cq
from pathlib import Path
shape=cq.importers.importStep(str(Path(__file__).resolve().parents[2]/'sr6_mini111.stp')).val()
solids=shape.Solids()
for i in [29,30,31,32,33,34,35,36,37,38,39,40,41]:
    s=solids[i]; b=s.BoundingBox()
    print('SOLID',i,'center',s.Center().toTuple(),'bounds',(b.xmin,b.xmax,b.ymin,b.ymax,b.zmin,b.zmax))
    circles=set()
    for e in s.Edges():
        if e.geomType()=='CIRCLE':
            c=e.arcCenter();r=e.radius()
            if r>=2:
                circles.add(tuple(round(v,3) for v in (*c.toTuple(),r)))
    print('CIRCLES',sorted(circles))
