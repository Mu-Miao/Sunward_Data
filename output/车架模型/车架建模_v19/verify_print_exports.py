from pathlib import Path
import sys,json
sys.path.append('/Applications/FreeCAD.app/Contents/Resources/lib')
import FreeCAD as App,Part,Mesh
h=Path(__file__).resolve().parent
d=App.openDocument(str(h/'car_assembly_v19.FCStd'))
b=next(o for o in d.Objects if o.Label.startswith('01_') and hasattr(o,'Shape'))
s=b.Shape.common(Part.makeBox(110,130,.1,App.Vector(-55,-65,1.95)))
v,t=s.tessellate(.12)
(h/'floor_layout_mesh.json').write_text(json.dumps(dict(vertices=[round(n,4) for p in v for n in p],triangles=[n for f in t for n in f]),separators=(',',':')))
slices=[]
for prefix in ['01_','02_','10_','13_','14_']:
 o=next(o for o in d.Objects if o.Label.startswith(prefix) and hasattr(o,'Shape'))
 shape=o.Shape.common(Part.makeBox(.1,150,120,App.Vector(-20.05,-75,0)))
 v,t=shape.tessellate(.12)
 slices.append(dict(name=o.Label,color=({'10_':[0.96,0.53,0.16],'13_':[.16,.21,.34],'14_':[.85,.12,.09]}.get(prefix,[0.5,0.57,0.65])),vertices=[round(n,4) for p in v for n in p],triangles=[n for f in t for n in f]))
(h/'battery_section_mesh.json').write_text(json.dumps(slices,separators=(',',':')))
checks=[]
assert d.getObject('HardwareDimensions').PCBPostHeight.Value==20
for name in ['01_lower_base','02_upper_controller_deck']:
 s=Part.read(str(h/'exports'/(name+'.step')))
 m=Mesh.Mesh(str(h/'exports'/(name+'.stl')))
 assert s.isValid() and len(s.Solids)==1 and abs(s.BoundBox.ZMin)<1e-6
 assert m.isSolid()
 if name=='02_upper_controller_deck':
  assert s.isInside(App.Vector(-20,0,.1),1e-6,True)
  assert s.isInside(App.Vector(-35,0,20),1e-6,True)
  assert not s.isInside(App.Vector(-20,0,20),1e-6,True)
 checks.append(dict(file=name,dimensions_mm=[round(s.BoundBox.XLength,4),round(s.BoundBox.YLength,4),round(s.BoundBox.ZLength,4)],bottom_z=s.BoundBox.ZMin,volume_mm3=round(s.Volume,3),solid_count=len(s.Solids),mesh_closed=m.isSolid()))
(h/'print_export_checks.json').write_text(json.dumps(checks,ensure_ascii=False,indent=2))
print(json.dumps(checks,ensure_ascii=False,indent=2))
