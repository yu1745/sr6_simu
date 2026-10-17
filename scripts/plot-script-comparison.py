"""Plot unfiltered offline reaction traces with the existing Conda environment."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'docs/images'
OUT.mkdir(exist_ok=True)
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
colors=['#138c80','#c45b36','#7477b8']
labels=['V4 septic','Ruckig + PCHIP','Ruckig + linear']
for group,name,window in [('group2','original',(134.8,136.2)),('group1','ver2',(135.3,136.7))]:
    arrays=[np.genfromtxt(ROOT/f'work/{folder}/{group}-{path}-trace.csv',delimiter=',',skip_header=1)
            for folder,path in [('script-comparison-pchip','v4'),('script-comparison-pchip','ruckig'),('script-comparison','ruckig')]]
    fig,axs=plt.subplots(3,2,figsize=(14,9),layout='constrained')
    for a,label,color in zip(arrays,labels,colors):
        t=a[:,0];mask=(t>=window[0])&(t<=window[1]);
        for col,sel in [(0,slice(None,None,10)),(1,mask)]:
            axs[0,col].plot(t[sel],a[sel,7]*100,label=label,color=color,lw=1,alpha=.85)
            axs[1,col].plot(t[sel],a[sel,1],color=color,lw=1,alpha=.85)
            axs[2,col].plot(t[sel],a[sel,6],color=color,lw=1,alpha=.85)
    for i,label in enumerate(['L0 position (%)','Dynamic Fx (N), STEP up','Dynamic Mz (N m)']):
        for ax in axs[i]:ax.set_ylabel(label);ax.grid(alpha=.18)
    axs[0,0].legend(ncol=3,fontsize=9)
    axs[0,0].set_title('Full script (display every 10th sample)')
    axs[0,1].set_title('Zoom: fastest stroke region (all 1 ms samples)')
    for ax in axs[-1]:ax.set_xlabel('Script time (s)')
    fig.suptitle(f'WooHanKyung / {name} — 6010, L0 40 mm, roll & pitch +/-10 deg\nSame script-derived V/A/J; no low-pass filter; reference rigid-body dynamics',fontsize=13)
    fig.savefig(OUT/f'woohan-{name}-comparison.png',dpi=160)
    plt.close(fig)
