"""Orthographic raster view of the saved CAD tessellation, including full V7 PCB geometry."""
from pathlib import Path
import json,math,sys
from PIL import Image,ImageDraw,ImageFont
import numpy as np
H=Path(__file__).resolve().parent
parts=json.loads((H/'preview_meshes.json').read_text())
board=json.loads((H/'c10b_mesh.json').read_text())
current=next(p for p in parts if p['name'].startswith('12_'))
current.update(board)
floor_view='--floor' in sys.argv
top_view='--top' in sys.argv or floor_view
base_view='--base' in sys.argv or floor_view
deck_view='--deck' in sys.argv
rear_view='--rear' in sys.argv
power_view='--power' in sys.argv
if power_view:parts=[p for p in parts if p['name'].startswith(('02_','04_','10_','12_','13_','14_'))]
section_view='--section' in sys.argv
if section_view:parts=json.loads((H/'battery_section_mesh.json').read_text())
if base_view:parts=[p for p in parts if p['name'].startswith('01_')]
if deck_view:
    parts=[p for p in parts if p['name'].startswith('02_')]
    for m in parts:
        vv=m['vertices'];m['vertices']=[n for i in range(0,len(vv),3) for n in [vv[i],-vv[i+1],90-vv[i+2]]]
if floor_view:parts[0].update(json.loads((H/'floor_layout_mesh.json').read_text()))
yaw=-math.pi/2 if (top_view or section_view) else (-.65 if (rear_view or power_view) else 1.1);el=0 if section_view else (math.pi/2 if top_view else (.18 if power_view else .48));cy=math.cos(yaw);sy=math.sin(yaw);ce=math.cos(el);se=math.sin(el)
def project(x,y,z):
    u=x*cy-y*sy;v=x*sy+y*cy
    return (u,-(z*ce-v*se),v*ce+z*se)
allpts=[project(*m['vertices'][i:i+3]) for m in parts for i in range(0,len(m['vertices']),3)]
lo=[min(p[i] for p in allpts) for i in [0,1]];hi=[max(p[i] for p in allpts) for i in [0,1]]
scale=min(840/(hi[0]-lo[0]),680/(hi[1]-lo[1]))
def screen(p):return (500+(p[0]-(lo[0]+hi[0])/2)*scale,415+(p[1]-(lo[1]+hi[1])/2)*scale)
image=Image.new('RGB',(1000,920),(248,249,251));draw=ImageDraw.Draw(image)
fontpath=next((p for p in [Path('/System/Library/Fonts/PingFang.ttc'),Path('/System/Library/Fonts/STHeiti Medium.ttc')] if p.exists()),None)
font=ImageFont.truetype(str(fontpath),22) if fontpath else ImageFont.load_default()
small=ImageFont.truetype(str(fontpath),18) if fontpath else ImageFont.load_default()
triangles=[]
for m in parts:
    verts=m['vertices'];coords=[project(*verts[i:i+3]) for i in range(0,len(verts),3)]
    for i in range(0,len(m['triangles']),3):
        inds=m['triangles'][i:i+3];a,b,c=[coords[n] for n in inds];pts=[screen(p) for p in [a,b,c]]
        if abs((pts[1][0]-pts[0][0])*(pts[2][1]-pts[0][1])-(pts[1][1]-pts[0][1])*(pts[2][0]-pts[0][0]))<.03:continue
        av,bv,cv=[verts[n*3:n*3+3] for n in inds]
        u=[bv[k]-av[k] for k in range(3)];v=[cv[k]-av[k] for k in range(3)]
        n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
        mag=math.sqrt(sum(x*x for x in n)) or 1
        light=.55+.45*abs((-.3*n[0]-.4*n[1]+.86*n[2])/mag)
        color=tuple(max(0,min(255,round(x*255*light))) for x in m['color'])
        triangles.append(((a[2],b[2],c[2]),pts,color))
# Per-pixel depth prevents long CAD face triangles from being painted over
# closer geometry, especially around socket roofs and narrow motor gaps.
canvas=np.empty((920,1000,3),dtype=np.uint8);canvas[:]=(248,249,251)
zbuffer=np.full((920,1000),-np.inf,dtype=np.float64)
for depths,pts,color in triangles:
    (x0,y0),(x1,y1),(x2,y2)=pts
    xmin=max(0,math.ceil(min(x0,x1,x2)-.5));xmax=min(999,math.floor(max(x0,x1,x2)-.5))
    ymin=max(0,math.ceil(min(y0,y1,y2)-.5));ymax=min(919,math.floor(max(y0,y1,y2)-.5))
    if xmax<xmin or ymax<ymin:continue
    denom=(y1-y2)*(x0-x2)+(x2-x1)*(y0-y2)
    if abs(denom)<1e-9:continue
    xx=np.arange(xmin,xmax+1)[None,:]+.5;yy=np.arange(ymin,ymax+1)[:,None]+.5
    w0=((y1-y2)*(xx-x2)+(x2-x1)*(yy-y2))/denom
    w1=((y2-y0)*(xx-x2)+(x0-x2)*(yy-y2))/denom
    w2=1-w0-w1
    depth=w0*depths[0]+w1*depths[1]+w2*depths[2]
    sub=zbuffer[ymin:ymax+1,xmin:xmax+1]
    mask=(w0>=-1e-6)&(w1>=-1e-6)&(w2>=-1e-6)&(depth>sub)
    sub[mask]=depth[mask]
    canvas[ymin:ymax+1,xmin:xmax+1][mask]=color
image=Image.fromarray(canvas);draw=ImageDraw.Draw(image)
if base_view:
    draw.text((35,25),'V19 · 底板 + 下深电池槽 · 整体打印一件',font=font,fill=(35,40,45))
    draw.text((35,815),'每侧4孔：30 × 24 mm孔距，底板孔径3.4 mm；侧槽40 × 3.4 mm',font=font,fill=(35,40,45))
    draw.text((35,855),'底板 110 × 130 mm，底板厚4 mm，总高42 mm；托盘下方打印需支撑',font=small,fill=(60,65,70))
    output=H/'底板_v19.png'
else:
    draw.text((35,25),'V19 · 四根等高直铜柱 / 下深电池槽 / 上槽插头开口',font=font,fill=(35,40,45))
    draw.text((35,815),'底板前后 110 mm + 35 mm 外伸 = 总长 145 mm（14.5 cm）',font=font,fill=(35,40,45))
    draw.text((35,855),'含35 mm自备延长板：200 × 145 × 130.4 mm；共打印底板、托板两件',font=small,fill=(60,65,70))
    output=H/('后侧效果图_v19.png' if rear_view else '效果图_v19.png')
if top_view and not base_view:
    rear=-55;front=90
    a=screen(project(0,-100,0));b=screen(project(0,100,0))
    y=785
    draw.line([(a[0],y),(b[0],y)],fill=(38,68,91),width=2)
    for x in [a[0],b[0]]:draw.line([(x,y-6),(x,y+6)],fill=(38,68,91),width=2)
    draw.text((370,750),'轮间跨度 200 mm',font=small,fill=(38,68,91))
    pa=screen(project(rear,0,0));pb=screen(project(front,0,0));x=965
    draw.line([(x,pa[1]),(x,pb[1])],fill=(176,49,43),width=2)
    for py in [pa[1],pb[1]]:draw.line([(x-6,py),(x+6,py)],fill=(176,49,43),width=2)
    draw.text((730,62),'车头 ↑ · 含模块总长 145 mm',font=small,fill=(176,49,43))
    base_front=screen(project(55,0,0))[1]
    for xx in range(80,920,12):draw.line([(xx,base_front),(xx+6,base_front)],fill=(70,105,135),width=2)
    draw.text((600,base_front+8),'新车头 · 向前外伸35 mm',font=small,fill=(70,105,135))
    output=H/'长度示意_v19.png'
if top_view and base_view:
    for yy in [-51,51]:
        centre=screen(project(-20,yy,0))
        p=screen(project(-40,yy,0));q=screen(project(0,yy,0))
        xx=centre[0]+22
        draw.line([(xx,p[1]),(xx,q[1])],fill=(174,49,43),width=2)
        for y in [p[1],q[1]]:draw.line([(xx-5,y),(xx+5,y)],fill=(174,49,43),width=2)
        draw.text((xx+8,(p[1]+q[1])/2),'40 mm',font=small,fill=(174,49,43))
    for syy in [-1,1]:
        corners=[screen(project(xx,syy*yy,0)) for xx,yy in [(2.5,19),(42.5,19),(42.5,60),(2.5,60),(2.5,19)]]
        draw.line(corners,fill=(45,88,139),width=2)
        for xx in [7.5,37.5]:
            for yy in [27,51]:
                pp=screen(project(xx,syy*yy,0))
                draw.ellipse((pp[0]-12,pp[1]-12,pp[0]+12,pp[1]+12),outline=(175,40,38),width=2)
        c=screen(project(22.5,syy*40,0))
        draw.text((c[0]-64,c[1]-8),'30 × 24 mm',font=small,fill=(175,40,38))
    draw.text((260,762),'蓝框：底脚在底板下方　红圈：M3固定孔（每侧4个）',font=small,fill=(45,88,139))
    output=H/'孔位俯视_v19.png'
    if floor_view:
        draw.rectangle((0,0,1000,65),fill=(248,249,251))
        draw.text((35,25),'V19 · 底板安装孔位：平面截面示意',font=font,fill=(35,40,45))
        output=H/'底板安装孔位_v19.png'
if deck_view:
    draw.rectangle((0,0,1000,65),fill=(248,249,251))
    draw.rectangle((0,795,1000,920),fill=(248,249,251))
    draw.text((35,25),'V19 · C10B 上托板 · 反向电池槽 · 翻面打印',font=font,fill=(35,40,45))
    draw.text((35,815),'上板包络82 × 128 × 22.4 mm；下向槽包住电池顶部16 mm',font=font,fill=(35,40,45))
    draw.text((35,855),'内腔25.2 × 89.8 mm；单边间隙0.6 mm；平板朝下、槽口朝上打印',font=small,fill=(60,65,70))
    output=H/'主板托板_v19.png'
if section_view:
    draw.rectangle((0,0,1000,65),fill=(248,249,251))
    draw.rectangle((0,795,1000,920),fill=(248,249,251))
    draw.text((35,25),'V19 · 插头与电源线剖面 · C10B铜柱20 mm',font=font,fill=(35,40,45))
    def dimension(z0,z1,x,label):
        y0=screen(project(-20,0,z0))[1];y1=screen(project(-20,0,z1))[1]
        draw.line([(x,y0),(x,y1)],fill=(180,60,40),width=2)
        for y in [y0,y1]:draw.line([(x-7,y),(x+7,y)],fill=(180,60,40),width=2)
        draw.text((x+10,(y0+y1)/2-10),label,font=small,fill=(180,60,40))
    dimension(22,42,833,'下槽20 mm')
    dimension(67.6,83.6,833,'上包16 mm')
    dimension(83.6,95.6,165,'插头12 mm')
    yy=screen(project(-20,0,84.2))[1]
    draw.line([(640,yy),(710,yy-35),(875,yy-35)],fill=(40,85,130),width=2)
    draw.text((713,yy-60),'顶间隙0.6 mm',font=small,fill=(40,85,130))
    draw.text((35,815),'橙色：实际尺寸电池；深蓝：12 mm插头占位；红色：参考电源线',font=font,fill=(35,40,45))
    draw.text((35,855),'插孔中心暂按端部8 mm；插头直径暂按14 mm，实际宽度仍需试装',font=small,fill=(60,65,70))
    output=H/'电池上下槽剖面_v19.png'
if power_view:
    draw.rectangle((0,0,1000,65),fill=(248,249,251))
    draw.rectangle((0,795,1000,920),fill=(248,249,251))
    draw.text((35,25),'V19 · 电池插头与主板间隙 / 电源线绕到板边',font=font,fill=(35,40,45))
    draw.text((35,815),'插头露出12 mm；主板铜柱20 mm；局部参考线路约56 mm',font=font,fill=(35,40,45))
    draw.text((35,855),'电源线250 mm；插头直径暂按14 mm，孔心距端8 mm为估计位置',font=small,fill=(60,65,70))
    output=H/'电源插头走线_v19.png'
image.save(output)
print(output)
