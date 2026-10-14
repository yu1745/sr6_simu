"""Render supplied bearing and actual spacer B-reps, without UI automation."""
from pathlib import Path
import cadquery as cq
import numpy as np
import vtk
from vtk.util.numpy_support import numpy_to_vtk,numpy_to_vtkIdTypeArray
root=Path(__file__).resolve().parents[1]
h,b=cq.importers.importStep(str(root/'cad/inputs/SA6TK-jlcfa.step')).val().Solids()
w=cq.importers.importStep(str(root/'cad/SA6TK_M6_profile_washer_H5.step')).val()
def actor(shape,color):
 verts,tris=shape.tessellate(.025,.08);points=vtk.vtkPoints();points.SetData(numpy_to_vtk(np.array([v.toTuple() for v in verts]),deep=True));cells=vtk.vtkCellArray();cells.ImportLegacyFormat(numpy_to_vtkIdTypeArray(np.column_stack([np.full(len(tris),3),tris]).astype(np.int64).ravel(),deep=True));poly=vtk.vtkPolyData();poly.SetPoints(points);poly.SetPolys(cells);normals=vtk.vtkPolyDataNormals();normals.SetInputData(poly);normals.SetFeatureAngle(35);mapper=vtk.vtkPolyDataMapper();mapper.SetInputConnection(normals.GetOutputPort());a=vtk.vtkActor();a.SetMapper(mapper);a.GetProperty().SetColor(*color);a.GetProperty().SetSpecular(.35);a.GetProperty().SetSpecularPower(35);return a
win=vtk.vtkRenderWindow();win.SetOffScreenRendering(1);win.SetSize(1500,800)
for i,title in enumerate(['SA6T/K + PROFILE WASHERS','25 DEG JOINT TILT (NOMINAL)','H5 WASHER / H10 SPACER']):
 r=vtk.vtkRenderer();r.SetViewport(i/3,0,(i+1)/3,1);r.SetBackground(.045,.06,.08);win.AddRenderer(r)
 if i<2:
  r.AddActor(actor(h.rotate((0,0,0),(1,0,0),25 if i else 0),(.36,.55,.65)));r.AddActor(actor(b,(.73,.76,.8)))
  for shape in [w.translate((0,0,4.5)),w.rotate((0,0,0),(1,0,0),180).translate((0,0,-4.5))]:r.AddActor(actor(shape,(.91,.66,.25)))
  target=(0,-8,0);pos=(55,22,55);scale=26
 else:
  d=cq.importers.importStep(str(root/'cad/SA6TK_M6_double_spacer_H10.step')).val()
  r.AddActor(actor(w.translate((0,8,0)),(.91,.66,.25)));r.AddActor(actor(d.translate((0,-8,0)),(.91,.66,.25)));target=(0,0,3);pos=(32,25,35);scale=21
 camera=r.GetActiveCamera();camera.SetPosition(*pos);camera.SetFocalPoint(*target);camera.SetViewUp(0,1,0);camera.ParallelProjectionOn();camera.SetParallelScale(scale);r.ResetCameraClippingRange()
 t=vtk.vtkTextActor();t.SetInput(title);t.GetTextProperty().SetFontSize(18);t.GetTextProperty().SetColor(.85,.9,.94);t.GetPositionCoordinate().SetCoordinateSystemToNormalizedViewport();t.GetPositionCoordinate().SetValue(.035,.94);r.AddViewProp(t)
win.Render();capture=vtk.vtkWindowToImageFilter();capture.SetInput(win);capture.Update();writer=vtk.vtkPNGWriter();writer.SetFileName(str(root/'cad/SA6TK-washer-preview.png'));writer.SetInputConnection(capture.GetOutputPort());writer.Write();win.Finalize()
