from pathlib import Path
import sys,json
sys.path.append('/Applications/FreeCAD.app/Contents/Resources/lib')
import FreeCAD as App,Part
h=Path(__file__).resolve().parent
s=h.parent/'历史版本/车架建模_v18/car_assembly_v18.FCStd'
d=App.openDocument(str(s));b=next(o for o in d.Objects if o.Label.startswith('12_'))
port=App.Vector(-20,-36.3,83.6)
plug=Part.makeCylinder(7,12,port)
# Reference cable sweep: 2.4mm wire, 6mm centre-line bend radius.
a=App.Vector(-20,-36.3,95.6);m=App.Vector(-21.75736,-36.3,99.84264);e=App.Vector(-26,-36.3,101.6)
w=Part.Wire([Part.Arc(a,m,e).toShape(),Part.makeLine(e,App.Vector(-40,-36.3,101.6))])
circle=Part.Wire([Part.makeCircle(1.2,a,App.Vector(0,0,1))]);cable=w.makePipeShell([circle],True,False)
for lift in [0,4,8,12]:
 shape=b.Shape.copy();shape.translate(App.Vector(0,0,lift))
 print(json.dumps(dict(lift_mm=lift,plug_overlap_mm3=plug.common(shape).Volume,plug_gap_mm=plug.distToShape(shape)[0],cable_overlap_mm3=cable.common(shape).Volume,cable_gap_mm=cable.distToShape(shape)[0])))
print('Original C10B bbox',b.Shape.BoundBox)
