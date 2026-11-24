import collections, sys, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt, numpy as np
from matplotlib.patches import Rectangle
from matplotlib.colors import LogNorm, Normalize
from load import *
plt.rcParams.update({"font.family":"serif","font.size":7,"pdf.fonttype":42})
RC={1:"#E8C547",2:"#7FA6E0",3:"#7CC08A",4:"#6DC1BF",5:"#E0826E",6:"#B58AD1"}  # head,body,arms,legs
def base(ax,pos,reg,alpha=1.0,dead=(),tint=None):
    for m,(x,z) in pos.items():
        if m in dead:
            ax.add_patch(Rectangle((x,z),.92,.92,color="#333333",lw=0))
        else:
            c=tint(m) if tint else RC[reg[m]]
            ax.add_patch(Rectangle((x,z),.92,.92,color=c,alpha=alpha,lw=0))
    xs=[p[0] for p in pos.values()]; zs=[p[1] for p in pos.values()]
    ax.set_xlim(min(xs)-.5,max(xs)+1.5); ax.set_ylim(min(zs)-.5,max(zs)+1.5); ax.set_aspect("equal"); ax.axis("off")

def paths_of(ev):
    tx=collections.defaultdict(list); deliv={}
    for r in ev:
        if r['ev']=='tx': tx[int(r['pid'])].append((int(r['t']),int(r['id'])))
        if r['ev']=='visit': tx[int(r['pid'])].append((int(r['t'])+1,-int(r['id'])))
        if r['ev']=='deliver': deliv.setdefault(int(r['pid']),(int(r['t']),int(r['id'])))
    return tx,deliv
