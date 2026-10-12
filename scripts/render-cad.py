"""Offline VTK renders of generated CAD assets (no desktop automation)."""
import json
from pathlib import Path
import numpy as np
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray

root = Path(__file__).resolve().parents[1]
data = json.loads((root/'public/sr6-cad.json').read_text())
renderer = vtk.vtkRenderer()
renderer.SetBackground(0.045, 0.06, 0.08)
for p in data['parts']:
    points = vtk.vtkPoints()
    points.SetData(numpy_to_vtk(np.array(p['positions']).reshape(-1,3), deep=True))
    faces = np.array(p['indices'], dtype=np.int64).reshape(-1,3)
    cells = vtk.vtkCellArray()
    cells.SetCells(len(faces), numpy_to_vtkIdTypeArray(np.column_stack([np.full(len(faces),3),faces]).ravel(), deep=True))
    poly = vtk.vtkPolyData(); poly.SetPoints(points); poly.SetPolys(cells)
    normals = vtk.vtkPolyDataNormals(); normals.SetInputData(poly); normals.SetFeatureAngle(45)
    mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(normals.GetOutputPort())
    actor = vtk.vtkActor(); actor.SetMapper(mapper)
    color = (0.22,0.25,0.28) if 'socket_cap_bolt' in p['name'] else (0.75,0.8,0.85) if (p['id'] in [20,25,30,35] or 'nut' in p['name']) else (0.9,0.65,0.25) if 54 <= p['id'] <= 57 else (0.43,0.6,0.69)
    actor.GetProperty().SetColor(*color)
    actor.GetProperty().SetSpecular(0.35); actor.GetProperty().SetSpecularPower(30)
    renderer.AddActor(actor)
window = vtk.vtkRenderWindow(); window.SetOffScreenRendering(1)
window.AddRenderer(renderer); window.SetSize(1400,1100)
camera = renderer.GetActiveCamera(); camera.SetViewUp(0,1,0)
camera.ParallelProjectionOn()
for name, position, target, scale in [
    ('assembly-preview.png',(-470,450,650),(130,150,-55),230),
    ('receiver-detail.png',(-140,345,290),(100,261,-56),124),
    ('pitch-pair-preview.png',(530,390,-410),(230,165,-65),180),
]:
    camera.SetPosition(*position); camera.SetFocalPoint(*target); camera.SetParallelScale(scale)
    renderer.ResetCameraClippingRange(); window.Render()
    capture = vtk.vtkWindowToImageFilter(); capture.SetInput(window); capture.Update()
    writer = vtk.vtkPNGWriter(); writer.SetFileName(str(root/'cad'/name))
    writer.SetInputConnection(capture.GetOutputPort()); writer.Write()
    print(name)
window.Finalize()
