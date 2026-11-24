import sys, os; HERE=os.path.dirname(os.path.abspath(__file__)); ROOT=os.path.join(HERE,'..'); sys.path.insert(0,HERE)
from fig_humanoid import *
from PIL import Image
CF=ROOT+'/configs/'; LOG=ROOT+'/logs/'
b=blocks(CF+'A__plain__anycast__r10s1__final.xml')
pos={i+1:x[0] for i,x in enumerate(b)}; reg={i+1:x[1] for i,x in enumerate(b)}; dead={i+1 for i,x in enumerate(b) if x[2]}
ev=events(LOG+'r10.csv')
visits=collections.defaultdict(list); firsttx={}
for r in ev:
    pid=int(r['pid'])
    if r['ev']=='visit': visits[pid].append((int(r['t']),int(r['id'])))
    if r['ev']=='tx' and pid not in firsttx: firsttx[pid]=(int(r['t']),int(r['id']))
def load(f):
    c=collections.Counter(int(r['id']) for r in events(f) if r['ev']=='tx' and int(r['pid'])<16777216); return c
Ln,Ld=load(LOG+'hot_none.csv'),load(LOG+'hot_detour.csv')
vmax=max(max(Ln.values()),max(Ld.values()))
fig,axs=plt.subplots(1,4,figsize=(3.25,1.62),gridspec_kw=dict(wspace=0.04))
# (a) screenshot
im=np.asarray(Image.open(HERE+'/vs_shot.png').convert('RGB')).copy()
g=(im.max(2).astype(int)-im.min(2)<14)&(im.min(2)>215); im[g]=255
im=im[40:835,40:480]
axs[0].imshow(im); axs[0].axis('off')
# (b) failures + paths
base(axs[1],pos,reg,alpha=.45,dead=dead)
for pid,col in ((1853,'#08306b'),(689,'#7a0177'),(785,'#00441b')):
    seq=[firsttx[pid][1]]+[m for _,m in sorted(visits[pid])]
    xs=[pos[m][0]+.46 for m in seq]; zs=[pos[m][1]+.46 for m in seq]
    axs[1].plot(xs,zs,color=col,lw=0.9,solid_joinstyle='round')
    axs[1].plot(xs[0],zs[0],'o',ms=2.5,color=col); axs[1].plot(xs[-1],zs[-1],'*',ms=5,color=col,mec='white',mew=.4)
# (c)(d) load
cm=plt.get_cmap('YlOrRd'); nm=Normalize(0,vmax)
for ax,L in ((axs[2],Ln),(axs[3],Ld)):
    base(ax,pos,reg,tint=lambda m,L=L: cm(nm(L.get(m,0))) if L.get(m,0)>0 else '#e6e6e6')
for ax,t in zip(axs,['(a) VisibleSim','(b) 10% failed','(c) No balancing','(d) Balancing']):
    ax.set_title(t,fontsize=7,pad=1.5)
sm=plt.cm.ScalarMappable(norm=nm,cmap=cm)
cax=fig.add_axes([0.54,0.085,0.40,0.03]); cb=fig.colorbar(sm,cax=cax,orientation='horizontal'); cb.ax.tick_params(labelsize=5.5,length=1.5,pad=1)
cb.ax.set_title('data messages sent per module',fontsize=6,pad=1.5)
fig.subplots_adjust(left=0.005,right=0.995,top=0.93,bottom=0.17)
fig.savefig(HERE+'/fig_humanoid.pdf'); fig.savefig(HERE+'/fig_humanoid.png',dpi=250)
print(vmax,max(Ln.values()),max(Ld.values()))
