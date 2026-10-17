"""Build a separate fixed-center 110 mm mount concept and render its real STEP geometry."""
import json, math, argparse
from pathlib import Path
import cadquery as cq
import numpy as np
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray
from OCP.gp import gp_Trsf

ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('--length',type=int,default=110,choices=[110,220])
args=parser.parse_args()
length=args.length
config_key='220' if length==220 else 'count-110'
OUT=ROOT/f'cad/mount-preview-{length}';OUT.mkdir(exist_ok=True)
data=json.loads((ROOT/'public/sr6-cad.json').read_text(encoding='utf-8'))
config=json.loads((ROOT/'cad/mount-axis-search.json').read_text(encoding='utf-8'))['results'][config_key]
print('Importing original STEP',flush=True)
original=cq.importers.importStep(str(ROOT/'cad/sr6_mini_complete.stp')).val().Solids()
parts=list(original); receiver=parts[38];cut_receiver=receiver
mounts=[]
def vector(a):return cq.Vector(*map(float,a))
def cylinder(r,h,p,d=(0,0,1)):return cq.Solid.makeCylinder(r,h,vector(p),vector(d))
def rotation(n):
    z=np.array([0.,0.,1.]);v=np.cross(z,n);c=float(z@n)
    k=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    return np.eye(3)+k+k@k/(1+c)
def moved(shape,r,c):
    tr=gp_Trsf();m=np.column_stack([r,c-r@c]);tr.SetValues(*map(float,m.ravel()))
    return shape.transformShape(cq.Matrix(tr))
for motor,ids,side in [(2,[60,70,71,76,77],-1),(3,[50,68,69,74,75],1)]:
    link=next(l for l in data['spec']['links'] if l['motor']==motor)
    c=np.array(link['upper']);setting=config[f'M{motor+1}'];n=np.array(setting['axis']);r=rotation(n)
    for i in ids:parts[i]=moved(original[i],r,c)
    # Preserve original clamp positions: sleeve seat 9.5 mm inward from the
    # ball, nut seat 13.5 mm inward. Both planes rotate with the pin.
    inward=-side*n
    pad=cylinder(11,4,c+9.5*inward,inward).cut(cylinder(3.25,6,c+8.5*inward,inward)).clean()
    # Clear the old local mounting area; the new pad overlaps its perimeter
    # and is fused into the receiver. This is a local boss concept, not a
    # claim that the existing straight hole accepts a tilted bolt.
    pocket=cylinder(8,60,c-np.array([0,0,30]))
    cut_receiver=cut_receiver.cut(pocket).clean()
    mounts.append({'motor':motor,'center':c,'axis':n,'rotation':r,'pad':pad,'ids':ids,'side':side,'setting':setting})
receiver=cut_receiver
for m in mounts:receiver=receiver.fuse(m['pad']).clean()
assert receiver.isValid(), 'Invalid modified receiver'
assert len(receiver.Solids())==len(original[38].Solids()),'Disconnected mounting pad'
parts[38]=receiver
checks=[]
for m in mounts:
    c,n=m['center'],m['axis'];link=next(l for l in data['spec']['links'] if l['motor']==m['motor'])
    assert np.linalg.norm(m['rotation']@c+(c-m['rotation']@c)-c)<1e-9
    volumes={str(i):abs(receiver.intersect(parts[i]).Volume()) for i in [*m['ids'],link['upper_eye']]}
    checks.append({'motor':m['motor']+1,'ballCenter':c.tolist(),'axis':n.tolist(),'alphaDeg':m['setting']['a'],'betaDeg':m['setting']['b'],'receiverOverlapMm3':volumes})
    print(checks[-1],flush=True)
    for i,v in volumes.items():assert v<1e-4,f'Receiver overlaps part {i}: {v}'
assembly=cq.Assembly(name=f'SR6_{length}mm_tilted_mount_concept')
for i,shape in enumerate(parts):
    color=cq.Color(.24,.55,.65) if i==38 else cq.Color(.65,.72,.78)
    assembly.add(shape,name=f'part_{i:02d}_{data["parts"][i]["name"]}',color=color)
assembly.export(str(OUT/f'sr6_{length}mm_tilted_mount_concept.step'))
cq.exporters.export(receiver,str(OUT/'receiver_tilted_mount_concept.step'))
(OUT/'design.json').write_text(json.dumps({'compensationMm':length,'configuration':config_key,'zeroOffsetMm':[0,0,0],'ballCentersFixed':True,'padOuterDiameterMm':22,'padThicknessMm':4,'pinClearanceDiameterMm':6.5,'checks':checks,'status':'Concept preview. New receiver shape not yet swept through all motions; machining, strength and assembly access not validated.'},indent=2),encoding='utf-8')
print('STEP export complete; rendering',flush=True)
source_mesh={id(s):data['parts'][i] for i,s in enumerate(original)}

def actor(shape,color,opacity=1):
    source=source_mesh.get(id(shape))
    if source:
        vertices=np.array(source['positions']).reshape(-1,3);tris=np.array(source['indices']).reshape(-1,3)
    else:
        verts,tris=shape.tessellate(.06,.12);vertices=np.array([v.toTuple() for v in verts])
    points=vtk.vtkPoints();points.SetData(numpy_to_vtk(vertices,deep=True))
    cells=vtk.vtkCellArray();cells.ImportLegacyFormat(numpy_to_vtkIdTypeArray(np.column_stack([np.full(len(tris),3),tris]).astype(np.int64).ravel(),deep=True))
    poly=vtk.vtkPolyData();poly.SetPoints(points);poly.SetPolys(cells)
    normals=vtk.vtkPolyDataNormals();normals.SetInputData(poly);normals.SetFeatureAngle(35)
    mapper=vtk.vtkPolyDataMapper();mapper.SetInputConnection(normals.GetOutputPort())
    a=vtk.vtkActor();a.SetMapper(mapper);a.GetProperty().SetColor(*color);a.GetProperty().SetOpacity(opacity);a.GetProperty().SetSpecular(.3);a.GetProperty().SetSpecularPower(30)
    return a
def line(renderer,a,b,color,width=3):
    src=vtk.vtkLineSource();src.SetPoint1(*a);src.SetPoint2(*b)
    mapper=vtk.vtkPolyDataMapper();mapper.SetInputConnection(src.GetOutputPort());obj=vtk.vtkActor();obj.SetMapper(mapper);obj.GetProperty().SetColor(*color);obj.GetProperty().SetLineWidth(width);renderer.AddActor(obj)
def label(renderer,text,x,y,size=21,color=(.87,.93,.96)):
    a=vtk.vtkTextActor();a.SetInput(text);a.GetTextProperty().SetFontSize(size);a.GetTextProperty().SetColor(*color)
    a.GetTextProperty().SetBackgroundColor(.035,.055,.075);a.GetTextProperty().SetBackgroundOpacity(.9)
    a.GetPositionCoordinate().SetCoordinateSystemToNormalizedViewport();a.GetPositionCoordinate().SetValue(x,y);renderer.AddViewProp(a)
def camera(renderer,c,offset,scale):
    cam=renderer.GetActiveCamera();cam.SetFocalPoint(*c);cam.SetPosition(*(np.array(c)+offset));cam.SetViewUp(1,0,0);cam.ParallelProjectionOn();cam.SetParallelScale(scale);renderer.ResetCameraClippingRange()
def receiver_actors(renderer):
    renderer.AddActor(actor(cut_receiver,(.16,.46,.57)))
    for m in mounts:renderer.AddActor(actor(m['pad'],(1,.55,.16)))
def color(i):
    if i in [2,4,6,8,10,12]:return (.13,.18,.22)
    if i in [39,40,41,42,43,44,45,46,47,48]:return (.53,.35,.72)
    if i in [0,1,14,15,16,17,49,58,59]:return (.18,.48,.59)
    if i in [74,75,76,77]:return (.99,.72,.35)
    if i in [68,70]:return (.38,.45,.51)
    return (.67,.76,.82)
window=vtk.vtkRenderWindow();window.SetOffScreenRendering(1);window.SetSize(1800,1100);window.SetMultiSamples(4)
view=vtk.vtkRenderer();view.SetViewport(0,0,.44,1);view.SetBackground(.035,.055,.075);window.AddRenderer(view)
for i,p in enumerate(parts):
    if i!=38:view.AddActor(actor(p,color(i)))
receiver_actors(view)
camera(view,(170,170,-62),(120,520,450),295)
label(view,f'{length} mm / TILTED MOUNT CONCEPT',.04,.95,23)
label(view,'Orange: new mounting seats\nCyan: new pin axis\nAll six ball centers unchanged',.04,.05,20)
for m in mounts:
    c=m['center'];line(view,c-24*m['axis'],c+24*m['axis'],(.2,1,1),4)
for index,m in enumerate(mounts):
    r=vtk.vtkRenderer();r.SetViewport(.445,.5-index*.5,1,1-index*.5);r.SetBackground(.05,.075,.095);window.AddRenderer(r)
    # Crop to the real local receiver neighborhood so the opposite mount does
    # not obstruct this close-up. Hardware and bearing are unscaled originals.
    c=m['center'];box=cq.Solid.makeBox(60,65,60,vector(c-np.array([30,32.5,30])))
    local_receiver=cut_receiver.intersect(box)
    r.AddActor(actor(local_receiver,(.16,.46,.57)));r.AddActor(actor(m['pad'],(1,.55,.16)))
    link=next(l for l in data['spec']['links'] if l['motor']==m['motor'])
    for i in [link['upper_eye'],*m['ids']]:r.AddActor(actor(parts[i],color(i)))
    line(r,c-np.array([0,0,32]),c+np.array([0,0,32]),(.55,.58,.62),2)
    line(r,c-32*m['axis'],c+32*m['axis'],(.1,1,1),3)
    # A fixed-center marker drawn as a small cross on the bore axis.
    for direction in [np.array([1,0,0]),np.array([0,1,0])]:line(r,c-2*direction,c+2*direction,(1,.3,.3),3)
    camera(r,c+np.array([0,-5,0]),(25,50,55*m['side']),36)
    s=m['setting'];label(r,f'M{m["motor"]+1}  |  alpha {s["a"]:+.2f} deg / beta {s["b"]:+.2f} deg',.03,.90,23)
    label(r,'Grey: original axis   /   Cyan: new axis',.03,.045,19)
window.Render();capture=vtk.vtkWindowToImageFilter();capture.SetInput(window);capture.Update()
writer=vtk.vtkPNGWriter();writer.SetFileName(str(OUT/'tilted-mount-preview.png'));writer.SetInputConnection(capture.GetOutputPort());writer.Write();window.Finalize()
print(str(OUT/'tilted-mount-preview.png'),flush=True)
