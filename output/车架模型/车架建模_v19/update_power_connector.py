"""V19 opposed battery sockets with four equal 80mm standoffs, no straps."""
from pathlib import Path
import sys,json,math,shutil
sys.path.append('/Applications/FreeCAD.app/Contents/Resources/lib')
import FreeCAD as App,Part,Import,Mesh
HERE=Path(__file__).resolve().parent
SOURCE=HERE.parent/'历史版本/车架建模_v18/car_assembly_v18.FCStd'
doc=App.openDocument(str(SOURCE));doc.Label='车架 V19 - 上下电池槽，无绑带'
assembly=next(o for o in doc.Objects if o.TypeId=='App::Part');assembly.Label='V19 下深槽与上反向槽、四等高铜柱'
objects=[o for o in assembly.Group if hasattr(o,'Shape') and o.Shape.Faces]
def obj(p):return next(o for o in objects if o.Label.startswith(p))
def box(l,w,h,x,y,z):return Part.makeBox(l,w,h,App.Vector(x-l/2,y-w/2,z))
def rounded(l,w,h,r,x,y,z):
 s=box(l,w,h,x,y,z)
 e=[e for e in s.Edges if len(e.Vertexes)==2 and abs(e.Vertexes[0].Point.z-e.Vertexes[1].Point.z)>h-.001]
 return s.makeFillet(r,e)
def hole(x,y,z,h=6,d=3.4):return Part.makeCylinder(d/2,h,App.Vector(x,y,z))
def slot_x(l,w,h,x,y,z):return box(l-w,w,h,x,y,z).fuse(hole(x-(l-w)/2,y,z,h,w)).fuse(hole(x+(l-w)/2,y,z,h,w))
def slot_y(l,w,h,x,y,z):return box(w,l-w,h,x,y,z).fuse(hole(x,y-(l-w)/2,z,h,w)).fuse(hole(x,y+(l-w)/2,z,h,w))
original_labels={o.Name:o.Label for o in objects}
original_board_volume=obj('12_').Shape.Volume
original_board_faces=len(obj('12_').Shape.Faces)

dims=doc.getObject('HardwareDimensions');dims.DeckPostHeight=80;dims.FrontDeckPostHeight=80;dims.DeckMountingPadHeight=0
if 'ControllerDeckThickness' not in dims.PropertiesList:dims.addProperty('App::PropertyLength','ControllerDeckThickness','CleanFrame')
dims.ControllerDeckThickness=6
dims.PCBPostHeight=20
dims.SpecificationNote='四根等高M3铜柱柱身80mm，可40+40叠接；直接装到底板上表面Z4，托板底Z84。PCB铜柱20mm。'
anchors=[(-7,-58),(-46,-58),(-7,58),(-46,58)]
posts=sorted([o for o in objects if o.Label.startswith('03_')],key=lambda o:o.Label)
for o,(x,y) in zip(posts,anchors):
 o.Placement.Base.x=x;o.Placement.Base.y=y
 o.setExpression('Placement.Base.z','4 mm')
 o.setExpression('Height','HardwareDimensions.DeckPostHeight')
 o.Label=o.Label.replace('M3x90+6','M3x80+6')
 o.PurchasedSpecification='M3柱身80mm，四根相同；可40+40叠接，螺纹啮合须核对'
pcbposts=[o for o in objects if o.Label.startswith('04_')]
pcbholes=[(o.Placement.Base.x,o.Placement.Base.y) for o in pcbposts]
for o in pcbposts:o.setExpression('Placement.Base.z','4 mm + HardwareDimensions.DeckPostHeight + HardwareDimensions.ControllerDeckThickness')
board=obj('12_')
board.setExpression('Placement.Base.z','4 mm + HardwareDimensions.DeckPostHeight + HardwareDimensions.ControllerDeckThickness + HardwareDimensions.PCBPostHeight - 57 mm')
deck=obj('02_');deck.setExpression('Placement.Base.z',None);deck.Placement=App.Placement()
deckshape=rounded(82,128,6,6,-11,0,0)
for x,y in anchors:
 deckshape=deckshape.cut(hole(x,y,-1,8))
 #1.2mm top counterbore preserves enough exposed6mm male thread for nut.
 deckshape=deckshape.cut(hole(x,y,4.8,2,7))
# Upper battery socket faces downward. Its roof is 0.6mm above the battery;
# walls overlap the top 16mm of the 60mm battery. Wall-to-wall XY clearance
# is 0.6mm per side. The socket is fused to the 4mm deck, not a third part.
upper_socket=rounded(33.2,97.8,16.9,3,-20,0,-16.4)
deckshape=deckshape.fuse(upper_socket)
# Actual 24 x 88.6 x 61.6 mm pack. Top=83.6, socket roof=84.2.
deckshape=deckshape.cut(box(25.2,89.8,16.7,-20,0,-16.5))
deckshape=deckshape.cut(box(8,15,14,-4,0,-16.5))
# Open-ended top notch: no narrow end-wall strip left at the plug outlet.
deckshape=deckshape.cut(rounded(18,26,25,2,-20,-38,-17))
for x,y in pcbholes:
 deckshape=deckshape.cut(hole(x,y,-19,27))
 if x < -5:
  # Two C10B bolts overlap the battery projection. Recess their heads
  # into the socket roof so no screw head protrudes towards the battery.
  deckshape=deckshape.cut(hole(x,y,.2,3.2,6.6))
for y in [-25,25]:deckshape=deckshape.cut(slot_y(24,8,8,-43,y,-1))
deck.Shape=deckshape.removeSplitter()
deck.setExpression('Placement.Base.z','4 mm + HardwareDimensions.DeckPostHeight')
deck.Label='02_controller_deck_redesigned_82x128'

# One continuous printable base with a 20mm-deep battery socket.
base=rounded(110,130,4,8,0,0,0)
for x in [-33.1,-6.9]:
 for y in [-43,43]:base=base.fuse(rounded(6,10,14.5,1,x,y,3.5))
base=base.fuse(rounded(33.2,97.8,4,3,-20,0,18))
rim=rounded(33.2,97.8,20.5,3,-20,0,21.5).cut(box(25.2,89.8,23,-20,0,21))
rim=rim.cut(box(8,15,23,-4,0,22));base=base.fuse(rim)
for x,y in anchors:base=base.cut(hole(x,y,-1))
for sy in [-1,1]:
 base=base.cut(hole(-39,sy*19.5,-1))
 base=base.cut(box(40.4,4.4,6,22.5,sy*58,-1))
 for x in [7.5,37.5]:
  for y in [27,51]:base=base.cut(hole(x,sy*y,-1))
 base=base.cut(slot_x(40,3.4,6,-20,sy*51,-1))
base=base.cut(hole(-39,0,-1,6,32.6)).cut(box(17,32.6,6,-48,0,-1))
for x in [30,42]:
 for y in [-35,0,35]:base=base.cut(slot_y(12,3.4,6,x,y,-1))
for y in [-15,15]:base=base.cut(hole(42,y,-1))
base=base.removeSplitter();assert base.isValid() and len(base.Solids)==1
bobj=obj('01_');bobj.Shape=base
bobj.RotatingClearanceNote='四根直铜柱X=-46/-7，Y=±58，远离电机；底板宽130mm，与轮胎内侧各留5mm。原车头高柱和L梁均取消。'
bobj.WheelSideSlotNote='两侧40x3.4mm槽中心X=-20、Y=±51，较V16向内4mm，为新铜柱安装孔留材料；尺寸保持不变。'
bobj.CableOpeningNote='横立24x88.6x61.6电池；下槽20mm深，内腔25.2x89.8mm，前侧15mm出线开口；上板底面配套反向电池槽。'

battery=obj('10_');battery.Shape=box(24,88.6,61.6,-20,0,22)
# Remove the two purchased strap references entirely: retention now comes
# from the opposing sockets and the screwed four-column upper deck.
for strap in [o for o in objects if o.Label.startswith('11_battery_strap_')]:
 objects.remove(strap);doc.removeObject(strap.Name)
for name,value in [('BatterySocketSideClearance',0.6),('LowerBatterySocketDepth',20),('UpperBatterySocketEngagement',16)]:
 if name not in dims.PropertiesList:dims.addProperty('App::PropertyLength',name,'BatterySockets')
 setattr(dims,name,value)
if 'BatteryRetentionNote' not in dims.PropertiesList:dims.addProperty('App::PropertyString','BatteryRetentionNote','BatterySockets')
dims.BatteryRetentionNote='上下反向槽固定电池；内腔25.2x89.8，单边0.6mm；下槽20mm深、上槽包住16mm，电池顶距上槽顶0.6mm；无绑带。'
# The drawing leaves the jack centre un-dimensioned. Orient the jack
# towards negative Y; use 8mm from the end as an adjustable approximation.
plug=doc.addObject('PartDesign::Feature','PowerPlugReference');assembly.addObject(plug)
plug.Label='13_DC_plug_height12_reference'
plug.Shape=Part.makeCylinder(7,12,App.Vector(-20,-36.3,83.6));objects.append(plug)
a=App.Vector(-20,-36.3,95.6)
e=App.Vector(-26,-36.3,101.6)
a1=Part.Arc(a,App.Vector(-21.7573593129,-36.3,99.8426406871),e).toShape()
f=App.Vector(-35,-36.3,101.6);g=App.Vector(-43,-28.3,101.6)
a2=Part.Arc(f,App.Vector(-40.6568542495,-33.9568542495,101.6),g).toShape()
k=App.Vector(-43,-25,101.6);m=App.Vector(-43,-19,107.6)
a3=Part.Arc(k,App.Vector(-43,-20.7573593129,103.3573593129),m).toShape()
wire=Part.Wire([a1,Part.makeLine(e,f),a2,Part.makeLine(g,k),a3,Part.makeLine(m,App.Vector(-43,-19,120))])
profile=Part.Wire([Part.makeCircle(1.2,a,App.Vector(0,0,1))])
cable=doc.addObject('PartDesign::Feature','PowerCableRouteReference');assembly.addObject(cable)
cable.Label='14_power_cable_route_reference';cable.Shape=wire.makePipeShell([profile],True,False);objects.append(cable)
for n,value in [('PlugExposedHeight',12),('PlugReferenceDiameter',14),('BatteryHeight',61.6),('PowerCableLength',250)]:
 if n not in dims.PropertiesList:dims.addProperty('App::PropertyLength',n,'PowerConnector')
 setattr(dims,n,value)
if 'PowerConnectorNote' not in dims.PropertiesList:dims.addProperty('App::PropertyString','PowerConnectorNote','PowerConnector')
dims.PowerConnectorNote='插头露出12mm实测；直径暂按14mm占位；孔心暂按距端8mm，图纸未标。18x26mm上板端部缺口；电源线250mm，参考直径2.4mm/弯曲半径6mm需试装。'
for o in pcbposts:
 o.Label=o.Label.replace('M3x8','M3x20')
 if 'PurchasedSpecification' not in o.PropertiesList:o.addProperty('App::PropertyString','PurchasedSpecification')
 o.PurchasedSpecification='M3柱身20mm，可10+10叠接；主板安装高度，螺纹需试装。'
doc.recompute()
assert len(deck.Shape.Solids)==1 and deck.Shape.isValid()

overlaps=[];bb=board.Shape.BoundBox
boardbox=Part.makeBox(bb.XLength,bb.YLength,bb.ZLength,App.Vector(bb.XMin,bb.YMin,bb.ZMin))
for i,a in enumerate(objects):
 for c in objects[i+1:]:
  if a==board or c==board:
   other=c if a==board else a
   if other.Label.startswith('04_'):continue
   v=boardbox.common(other.Shape).Volume
  else:v=a.Shape.common(c.Shape).Volume
  if v>.01:overlaps.append(dict(a=a.Label,b=c.Label,volume_mm3=round(v,5)))
bound=Part.makeCompound([o.Shape for o in objects]).BoundBox
report=json.loads((SOURCE.parent/'validation.json').read_text())
clearances=[]
columnshape=Part.makeCompound([o.Shape for o in posts])
for side,sy in [('left',1),('right',-1)]:
 sweep=Part.makeCylinder(32.5,30,App.Vector(22.5,sy*70,24.5),App.Vector(0,sy,0))
 ss=Part.makeCylinder(3,14,App.Vector(22.5,sy*64,24.5),App.Vector(0,sy,0))
 motor=next(o for o in objects if o.Label.startswith('06_') and o.Label.endswith(side))
 entry=dict(side=side,column_to_tire_rotation_gap_mm=round(columnshape.distToShape(sweep)[0],4),column_to_shaft_rotation_gap_mm=round(columnshape.distToShape(ss)[0],4),column_to_motor_gap_mm=round(columnshape.distToShape(motor.Shape)[0],4),base_to_tire_rotation_gap_mm=round(base.distToShape(sweep)[0],4))
 clearances.append(entry)
 assert entry['base_to_tire_rotation_gap_mm']>=5-.001
 assert columnshape.common(sweep).Volume<.01 and columnshape.common(ss).Volume<.01
report.update(version='v19',source=str(SOURCE),architecture='V18结构，实际电池尺寸+顶部插头缺口+抬高C10B',
 base_size_mm=[110,130,4],base_printed_height_mm=round(base.BoundBox.ZLength,4),controller_deck_mm=[82,128,22.4],controller_deck_flat_thickness_mm=6,
 controller_deck_reused=False,deck_reused_without_geometry_change=False,new_printed_parts_count=2,
 main_column_count=4,main_column_height_mm=80,main_column_roots_z=4,main_column_centres_xy_mm=anchors,
 deck_bottom_z=84,upper_socket_bottom_z=round(deck.Shape.BoundBox.ZMin,4),deck_top_z=round(deck.Shape.BoundBox.ZMax,4),
 battery_bottom_z=22,battery_top_z=83.6,battery_deck_gap_mm=0.4,battery_socket_roof_gap_mm=0.6,battery_retention='opposed_sockets_without_straps',lower_socket_depth_mm=20,upper_socket_engagement_mm=16,battery_socket_inner_xy_mm=[25.2,89.8],battery_socket_side_clearance_mm=0.6,pcb_bolt_head_recess_mm=[6.6,3.2],
 pcb_hole_centres_xy_mm=pcbholes,main_deck_hole_top_counterbore_mm=[7,1.2],
 side_slots=dict(count=2,length_mm=40,width_mm=3.4,centers_xy_mm=[[-20,-51],[-20,51]],axis='X前后方向',shift_inward_mm=4),
 outline_long_short_height_mm=[round(bound.YLength,4),round(bound.XLength,4),round(bound.ZLength,4)],
 rotating_clearances=clearances,all_shapes_valid=all(o.Shape.isValid() for o in objects),overlaps=overlaps,
 print_mesh_closed=False,base_single_solid=len(base.Solids)==1,
 change_scope='电池改24x88.6x61.6；上托板加18x26mm插头端部缺口并加厚到6mm；PCB铜柱20mm，主铜柱80mm不变。',
 uncertainties=['插孔中心暂取距长边端8mm，需按实物核对。','插头高12mm来自用户，直径暂取14mm；参考电线径2.4mm和弯曲半径6mm未实测。','PCB柱身20mm可10+10叠接，螺纹与托板刚度需实物试装。','轮毂、编码器局部和35mm延长板仍包含占位。'])
for key in ['deck_original_mm','deck_orientation_front_back_transverse_mm','deck_rotation_deg','front_deck_post_mm','rear_deck_post_mm','front_support_leg_centres_mm','front_support_leg_section_mm','front_support_beam_thickness_mm','front_copper_post_seat_thickness_mm','support_to_wheel_lateral_clearance_mm','support_to_shaft_minimum_clearance_mm','support_to_bracket_lateral_clearance_mm','old_v14_tire_and_shaft_gap_mm']:
 report.pop(key,None)
(HERE/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps({k:v for k,v in report.items() if k in ['version','architecture','base_size_mm','base_printed_height_mm','controller_deck_mm','new_printed_parts_count','main_column_centres_xy_mm','deck_bottom_z','battery_deck_gap_mm','outline_long_short_height_mm','rotating_clearances','overlaps']},ensure_ascii=False,indent=2),flush=True)
assert report['all_shapes_valid'] and not overlaps,overlaps
# Bottom insertion remains clear vertically; upper deck must lift 16.6mm
# off its assembled position to release the top end of the battery.
probe=box(24,88.6,61.6,-20,0,30)
assert base.common(probe).Volume < .01
upper_lift=deck.Shape.copy();upper_lift.translate(App.Vector(0,0,17))
assert upper_lift.common(battery.Shape).Volume < .01
assert abs(base.BoundBox.ZMax-42)<1e-6
assert abs(deck.Shape.BoundBox.ZMin-67.6)<1e-6
assert deck.Shape.isInside(App.Vector(-20,0,84.3),1e-6,True)
assert not deck.Shape.isInside(App.Vector(-20,0,84.1),1e-6,True)
assert not any(o.Label.startswith('11_') for o in objects)
report['power_connector']=dict(exposed_height_mm=12,reference_diameter_mm=14,centre_xy_mm=[-20,-36.3],centre_position_confirmed=False,end_offset_assumed_mm=8,upper_notch_xy_mm=[18,26],upper_notch_centre_xy_mm=[-20,-38],plug_to_c10b_gap_mm=round(plug.Shape.distToShape(board.Shape)[0],4),cable_to_c10b_gap_mm=round(cable.Shape.distToShape(board.Shape)[0],4),route_segment_length_mm=round(wire.Length,4),available_cable_length_mm=250,pcb_post_height_mm=20)
assert plug.Shape.common(board.Shape).Volume<.01 and cable.Shape.common(board.Shape).Volume<.01
assert report['power_connector']['cable_to_c10b_gap_mm']>1
report['battery_capture_checks']={'lower_insertion_path_clear':True,'upper_release_lift_mm':17,'battery_top_vertical_free_play_mm':0.6,'straps_removed':True}
(HERE/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
assert bound.XLength<150 and bound.YLength<=250 and bound.ZLength<=150
assert abs(board.Shape.Volume-original_board_volume)<1e-6
assert len(board.Shape.Faces)==original_board_faces

dims.EnvelopeShortSide=bound.XLength;dims.EnvelopeLongSide=bound.YLength
measurement=doc.getObject('Measurements')
if measurement and 'chassis_width' in measurement.PropertiesList:measurement.chassis_width=130
for o in objects:o.Visibility=True
assembly.Visibility=True
target=HERE/'car_assembly_v19.FCStd';doc.saveAs(str(target))
exports=HERE/'exports';exports.mkdir(exist_ok=True)
Import.export(objects,str(exports/'car_assembly_v19.step'))
meshchecks=[]
for o,name in [(bobj,'01_lower_base'),(deck,'02_upper_controller_deck')]:
 shape=o.Shape.copy()
 if o==deck:shape.rotate(App.Vector(0,0,0),App.Vector(1,0,0),180)
 shape.translate(App.Vector(0,0,-shape.BoundBox.ZMin))
 shape.exportStep(str(exports/(name+'.step')))
 v,t=shape.tessellate(.12);mesh=Mesh.Mesh([(v[a],v[b],v[c]) for a,b,c in t]);assert mesh.isSolid()
 mesh.write(str(exports/(name+'.stl')))
 check=Part.read(str(exports/(name+'.step')));assert check.isValid() and len(check.Solids)==1
 meshchecks.append(dict(file=name,closed=True,step_solid_count=1,printing_orientation='flat deck down, socket opening up' if o==deck else 'base flat bottom down'))
report['print_mesh_closed']=True;report['printed_parts']=meshchecks
(HERE/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
prior=json.loads((SOURCE.parent/'preview_meshes.json').read_text());meshes=[]
for o in objects:
 old=next((p for p in prior if p['name']==original_labels.get(o.Name,o.Label)),None)
 if old is None and o.Label.startswith('02_'):old=next(p for p in prior if p['name'].startswith('02_'))
 if old is None and o.Label.startswith('03_'):old=next(p for p in prior if p['name'].split('_M3')[0]==o.Label.split('_M3')[0])
 entry={k:v for k,v in old.items() if k not in ['vertices','triangles']} if old else dict(name=o.Label,color=([.16,.21,.34] if o==plug else [.85,.12,.09]))
 shape=boardbox if o==board else o.Shape
 v,t=shape.tessellate(.8);entry.update(name=o.Label,vertices=[round(n,4) for p in v for n in p],triangles=[n for f in t for n in f]);meshes.append(entry)
(HERE/'preview_meshes.json').write_text(json.dumps(meshes,separators=(',',':')))
v,t=board.Shape.tessellate(.3)
(HERE/'c10b_mesh.json').write_text(json.dumps(dict(vertices=[round(n,4) for p in v for n in p],triangles=[n for f in t for n in f]),separators=(',',':')))
for filename in ['MG513尺寸原图.jpg','支架尺寸原图.jpg']:shutil.copy2(SOURCE.parent/filename,HERE/filename)
App.closeDocument(doc.Name);saved=App.openDocument(str(target));saved.recompute()
assert saved.getObject('HardwareDimensions').DeckPostHeight.Value==80
print('Saved and reopened '+str(target),flush=True)
