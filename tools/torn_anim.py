"""Ring robot, region 1 cut in two by failures: final RADRA vs. no torn-region handling.
usage: torn_anim.py ring.xml final_log.csv no_torn_log.csv out.mp4
The logs come from RR_LOG=<file>.csv runs of the ring, once as is and once with
split="none" widen="off" e2e="off" gcAck="off" on the <experiment> element."""
import sys, os, csv, collections, shutil, subprocess
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
from animate import Robot, RC, mix, F, FB, FS, encode
from PIL import Image, ImageDraw
cfg,LOG_FIN,LOG_NOTORN,OUT=sys.argv[1:5]
runs=[('Final RADRA (pieces announce themselves)',LOG_FIN),('Without torn-region handling',LOG_NOTORN)]
cell=26; X0=60; Ys=[150,620]
Rs=[Robot(cfg,cell=cell,ox=X0,oy=y) for y in Ys]
R0=Rs[0]
left={m for m in R0.pos if R0.reg[m]==1 and R0.pos[m][0]<11}
t0,t1,dt=13.0e6,40.0e6,80e3; n=int((t1-t0)//dt)
E=[]
for _,f in runs:
    e=collections.defaultdict(list)
    for r in csv.DictReader(open(f)):
        t=int(r['t']); 
        if t<t0 or t>t1: continue
        ev=r['ev']; i=int(r['id']); p=int(r['pid'])
        if ev in('tx','deliver') and p>=16777216: continue
        if ev in('tx','deliver','kill','ctl'): e[int((t-t0)//dt)].append((ev,i))
    E.append(e)
W=X0*2+R0.W*cell; H=Ys[1]+R0.H*cell+150
d_tmp='fr_torn'; shutil.rmtree(d_tmp,ignore_errors=True); os.makedirs(d_tmp)
dead=[set(),set()]; inleft=[0,0]; heat=[collections.Counter(),collections.Counter()]
for k in range(n):
    im=Image.new('RGB',(W,H),'white'); d=ImageDraw.Draw(im)
    d.text((X0,25),"A region torn in two: region 1 (bottom left) is cut by failed modules at 15 s",fill='black',font=FB)
    d.text((X0,65),f"Ring robot, 273 modules, unicast traffic.  t = {(t0+k*dt)/1e6:5.1f} s.  The cut-off piece (outlined) can only be reached the long way round.",fill=(90,90,90),font=FS)
    for s,(R,ev) in enumerate(zip(Rs,E)):
        tx=set()
        for e,i in ev[k]:
            if e=='kill': dead[s].add(i)
            elif e=='tx': tx.add(i); heat[s][i]+=1
            elif e=='deliver' and i in left: inleft[s]+=1
        for m in list(heat[s]): heat[s][m]*=0.85
        for m in R.pos:
            if m in dead[s]: R.box(d,m,(30,30,30)); continue
            c=mix(RC[R.reg[m]],(255,255,255),0.55)
            h=min(1,heat[s].get(m,0)/2.5)
            if h>0.05: c=mix(c,(10,30,80),0.75*h)
            R.box(d,m,c)
            if m in left:
                x,y=R.xy(m); d.rectangle([x,y,x+cell-1,y+cell-1],outline=(200,40,40),width=2)
        d.text((R.ox,R.oy-42),runs[s][0],fill='black',font=F)
        d.text((R.ox+R.W*cell-470,R.oy-40),f"packets delivered into the cut-off piece: {inleft[s]}",fill=(150,30,30),font=FS)
    yy=H-70
    for j,(sw,lab) in enumerate([((10,30,80),"data traffic (darker = busier)"),((30,30,30),"failed module"),(None,"cut-off piece of region 1")]):
        x=X0+j*520
        if sw: d.rectangle([x,yy,x+22,yy+22],fill=sw)
        else: d.rectangle([x,yy,x+22,yy+22],outline=(200,40,40),width=3)
        d.text((x+34,yy),lab,fill=(60,60,60),font=FS)
    im.save(f"{d_tmp}/f{k:05d}.png")
for j in range(50): im.save(f"{d_tmp}/f{n+j:05d}.png")
encode(d_tmp,OUT,25)
print(inleft)
