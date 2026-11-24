"""Render RDRM runs (VisibleSim event log RR_LOG) as videos. Light theme, same colours as the paper figure.
usage: anim.py failures|hotspot out.mp4"""
import sys, os, csv, re, collections, subprocess, shutil, numpy as np
from PIL import Image, ImageDraw, ImageFont
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
RC = {1:(232,197,71),2:(127,166,224),3:(124,192,138),4:(109,193,191),5:(224,130,110),6:(181,138,209)}
def blocks(path):
    out=[]
    for line in open(path):
        m=re.search(r'<block position="(\d+),(\d+),(\d+)"',line)
        if not m: continue
        out.append(((int(m[1]),int(m[3])),int(re.search(r'region="(\d+)"',line)[1]),'fail="true"' in line))
    return out
def font(sz):
    for f in ["/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf","/usr/share/fonts/dejavu/DejaVuSans.ttf"]:
        if os.path.exists(f): return ImageFont.truetype(f,sz)
    return ImageFont.load_default()
F, FB, FS = font(22), font(28), font(17)
def mix(c, w, a): return tuple(int(c[i]*(1-a)+w[i]*a) for i in range(3))
def ylorrd(x):
    stops=[(255,255,204),(254,217,118),(253,141,60),(227,26,28),(128,0,38)]
    x=max(0,min(1,x))*4; i=min(int(x),3); f=x-i
    return mix(stops[i],stops[i+1],f)
class Robot:
    def __init__(s, cfg, cell=16, ox=0, oy=0):
        b=blocks(cfg); s.pos={i+1:x[0] for i,x in enumerate(b)}; s.reg={i+1:x[1] for i,x in enumerate(b)}
        s.fail={i+1 for i,x in enumerate(b) if x[2]}; s.cell=cell; s.ox=ox; s.oy=oy
        s.H=max(z for _,z in s.pos.values())+1; s.W=max(x for x,_ in s.pos.values())+1
    def xy(s,m):
        x,z=s.pos[m]; return s.ox+x*s.cell, s.oy+(s.H-1-z)*s.cell
    def box(s,d,m,col,inset=1):
        x,y=s.xy(m); d.rectangle([x+inset,y+inset,x+s.cell-inset-1,y+s.cell-inset-1],fill=col)
def events(path):
    for r in csv.DictReader(open(path)): yield int(r['t']),r['ev'],int(r['id']),int(r['pid'])
def encode(frames_dir, out, fps):
    subprocess.run(["ffmpeg","-y","-loglevel","error","-framerate",str(fps),"-i",f"{frames_dir}/f%05d.png","-c:v","libx264","-preset","veryfast","-crf","23","-pix_fmt","yuv420p","-vf","pad=ceil(iw/2)*2:ceil(ih/2)*2",out],check=True)

def failures(out):
    """10 % of the 400-module humanoid fails at 15 s; anycast traffic continues; repair messages in purple."""
    cfg=ROOT+'/configs/A__plain__anycast__r10s1__final.xml'; log=ROOT+'/logs/r10.csv'
    R=Robot(cfg,cell=22,ox=60,oy=110); Wd,Ht=R.ox*2+R.W*R.cell+380, R.oy+R.H*R.cell+70
    t0,t1,dt=13.0e6,22.0e6,40e3
    ev=collections.defaultdict(list); killT={}
    for t,e,i,p in events(log):
        if t0<=t<=t1: ev[int((t-t0)//dt)].append((e,i,p))
        if e=='kill': killT[i]=t
    data=ctl=0; dead=set(); recv=0; d_tmp='fr_fail'; shutil.rmtree(d_tmp,ignore_errors=True); os.makedirs(d_tmp)
    ctlheat=collections.Counter(); n=int((t1-t0)//dt)
    for k in range(n):
        t=t0+k*dt; tx=set(); deliv=set(); cnow=collections.Counter()
        for e,i,p in ev[k]:
            if e=='kill': dead.add(i)
            elif e=='tx' and p<16777216: tx.add(i); data+=1
            elif e=='ctl': cnow[i]+=1; ctl+=1
            elif e=='deliver' and p<16777216: deliv.add(i); recv+=1
        for m in list(ctlheat): ctlheat[m]*=0.80
        for m,c in cnow.items(): ctlheat[m]+=c
        im=Image.new('RGB',(Wd,Ht),'white'); d=ImageDraw.Draw(im)
        for m in R.pos:
            if m in dead: R.box(d,m,(40,40,40)); continue
            c=mix(RC[R.reg[m]],(255,255,255),0.55)
            h=min(1,ctlheat.get(m,0)/3)
            if h>0.05: c=mix(c,(110,60,200),0.85*h)
            R.box(d,m,c)
            if m in tx: x,y=R.xy(m); d.ellipse([x+6,y+6,x+R.cell-7,y+R.cell-7],fill=(10,30,80))
            if m in deliv: x,y=R.xy(m); d.rectangle([x+1,y+1,x+R.cell-2,y+R.cell-2],outline=(0,0,0),width=2)
        d.text((R.ox,25),"RDRM, final version: 10 % of the modules fail at 15 s",fill='black',font=FB)
        d.text((R.ox,62),"400-module humanoid, anycast traffic to every other region, VisibleSim",fill=(90,90,90),font=FS)
        X=R.ox+R.W*R.cell+60; Y=R.oy+20
        d.text((X,Y),f"t = {t/1e6:5.2f} s",fill='black',font=FB)
        rows=[("failed modules",f"{len(dead)}"),("data messages",f"{data:,}"),("repair/control msgs",f"{ctl:,}"),("packets delivered",f"{recv:,}")]
        for j,(a,b) in enumerate(rows):
            d.text((X,Y+60+j*44),a,fill=(90,90,90),font=FS); d.text((X+230,Y+56+j*44),b,fill='black',font=F)
        Y2=Y+260
        for j,(sw,lab) in enumerate([((10,30,80),"module sending data"),((110,60,200),"control / repair messages"),((40,40,40),"failed module"),(None,"packet delivered")]):
            yy=Y2+j*34
            if sw: d.rectangle([X,yy,X+20,yy+20],fill=sw)
            else: d.rectangle([X,yy,X+20,yy+20],outline='black',width=2)
            d.text((X+32,yy-1),lab,fill=(60,60,60),font=FS)
        d.text((X,Y2+150),"Packets keep flowing: the gradients\nare repaired locally and the packets\nheld by dead modules are re-sent.",fill=(60,60,60),font=FS)
        im.save(f"{d_tmp}/f{k:05d}.png")
    encode(d_tmp,out,25)

def hotspot(out):
    """Arms and legs stream packets to the head: without and with load-aware forwarding."""
    cfgs=[ROOT+'/configs/F_L__plain__hotspot__none__final__none.xml',ROOT+'/configs/F_L__plain__hotspot__none__final__detour.xml']
    logs=[ROOT+'/logs/hot_none.csv',ROOT+'/logs/hot_detour.csv']
    Rs=[Robot(cfgs[0],cell=18,ox=60,oy=130)]; Rs.append(Robot(cfgs[1],cell=18,ox=60+Rs[0].W*18+160,oy=130))
    Wd=Rs[1].ox+Rs[1].W*18+60; Ht=130+Rs[0].H*18+150
    t0,t1,dt=5.0e6,11.5e6,40e3; n=int((t1-t0)//dt)
    evs=[]
    for lg in logs:
        e=collections.defaultdict(list)
        for t,ev,i,p in events(lg):
            if ev in('tx','deliver') and p<16777216 and t0<=t<=t1: e[int((t-t0)//dt)].append((ev,i))
        evs.append(e)
    load=[collections.Counter(),collections.Counter()]; dl=[0,0]; d_tmp='fr_hot'; shutil.rmtree(d_tmp,ignore_errors=True); os.makedirs(d_tmp)
    for k in range(n):
        im=Image.new('RGB',(Wd,Ht),'white'); d=ImageDraw.Draw(im)
        d.text((60,25),"Load-aware forwarding under a hotspot",fill='black',font=FB)
        d.text((60,62),f"Arms and legs stream 1,600 packets to the head.  t = {(t0+k*dt)/1e6:5.2f} s",fill=(90,90,90),font=FS)
        for s,(R,E) in enumerate(zip(Rs,evs)):
            tx=set()
            for ev,i in E[k]:
                if ev=='tx': load[s][i]+=1; tx.add(i)
                else: dl[s]+=1
            for m in R.pos:
                L=load[s].get(m,0); R.box(d,m,ylorrd(L/800) if L else (230,230,230))
                if m in tx: x,y=R.xy(m); d.ellipse([x+5,y+5,x+R.cell-6,y+R.cell-6],fill=(10,30,80))
            title=["No balancing (lowest id)","With load-aware forwarding"][s]
            d.text((R.ox,R.oy-40),title,fill='black',font=F)
            busiest=max(load[s].values()) if load[s] else 0
            d.text((R.ox,R.oy+R.H*R.cell+15),f"busiest module: {busiest} msgs\ndelivered: {dl[s]:,} / 1,600",fill=(40,40,40),font=FS)
        X=60; Y=Ht-50; W=420
        for j in range(W): d.line([X+j,Y,X+j,Y+14],fill=ylorrd(j/W))
        d.text((X+W+12,Y-3),"data messages sent per module (0 to 800)",fill=(60,60,60),font=FS)
        im.save(f"{d_tmp}/f{k:05d}.png")
    for _ in range(40): im.save(f"{d_tmp}/f{n+_:05d}.png")
    encode(d_tmp,out,25)

if __name__=="__main__":
    {"failures":failures,"hotspot":hotspot}[sys.argv[1]](sys.argv[2])
