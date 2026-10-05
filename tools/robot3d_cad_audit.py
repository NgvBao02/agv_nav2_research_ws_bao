#!/usr/bin/env python3
"""Additional STEP component plates and explicitly axle-anchored comparisons."""
import json,csv,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from robot3d_extract import ROOT,OUT,FIG,render
from robot3d_analysis import save,COL

g=json.loads((OUT/'geometry.json').read_text());cad=list(np.load(OUT/'cad_meshes.npz').values());reg=json.loads((OUT/'registration.json').read_text())
R=np.array(reg['rotation']);t=np.array([0.,0.,-20.])
aligned=[m@R.T+t for m in cad]
pal=[(.49,.67,.74),(.70,.75,.79),(.75,.51,.23),(.26,.35,.43),(.21,.51,.53)]
groups={'cad_structure':[20,21,22,23], 'cad_drive_a':[2]+list(range(4,12))+[25]+list(range(27,34)),
        'cad_drive_b':[3]+list(range(12,20))+[26]+list(range(34,41)),
        'cad_tread_pair':[27,34], 'cad_battery_envelope':[148]}
groups['cad_electronics']=[s['id'] for s in g['step']['solids'] if -70<np.mean([s['bounds_mm'][1],s['bounds_mm'][4]])<0 and 45<np.mean([s['bounds_mm'][2],s['bounds_mm'][5]])<190 and s['volume_mm3']<1e5]
groups['cad_lidar']=[s['id'] for s in g['step']['solids'] if np.mean([s['bounds_mm'][1],s['bounds_mm'][4]])<-87 and abs(np.mean([s['bounds_mm'][2],s['bounds_mm'][5]]))<70]
for name,ids in groups.items():render(name,[aligned[i-1] for i in ids],pal,(1,-1,.7))
extra=list(np.load(OUT/'cad_extra_faces.npz').values()) if (OUT/'cad_extra_faces.npz').exists() else []
wheel_faces=[m for m in extra if abs(m.reshape(-1,3).mean(0)[0])>99 and abs(m.reshape(-1,3).mean(0)[2])<48]
if wheel_faces:
    render('cad_tread_surface',[aligned[26],aligned[33],np.concatenate(wheel_faces)@R.T+t],pal,(1,-1,.7))

b1=np.array(g['step']['solids'][26]['bounds_mm']);b2=np.array(g['step']['solids'][33]['bounds_mm'])
track=float((b1[0]+b1[3]-b2[0]-b2[3])/2);width=float(b1[3]-b1[0]);diam=float(b1[4]-b1[1])
audit=dict(step_track_mm=track,step_tread_width_mm=width,step_tread_diameter_mm=diam,
    urdf_track_mm=254.8,track_difference_mm=254.8-track,relative_track_difference_pct=(254.8/track-1)*100,
    step_tread_solid_ids=[27,34],component_groups=groups,
    axle_anchor_rotation=R.tolist(),axle_anchor_translation_mm=t.tolist(),
    caveat='Solid indices are extraction order, not original BOM part numbers. Axis signs selected by registration; axle anchor z=-20 mm comes from tread bounding-box centers. No scale fitting.',
    bounds_step_aligned_mm=[np.concatenate([m.reshape(-1,3) for m in aligned]).min(0).tolist(),np.concatenate([m.reshape(-1,3) for m in aligned]).max(0).tolist()])
if wheel_faces:
    wp=np.concatenate([m.reshape(-1,3) for m in wheel_faces]);audit['wheel_free_surface_bounds_step_mm']=[wp.min(0).tolist(),wp.max(0).tolist()]
    audit['wheel_free_surface_diameter_y_mm']=float(wp[:,1].max()-wp[:,1].min())
(OUT/'cad_audit.json').write_text(json.dumps(audit,indent=2))
with (OUT/'step_solids.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['solid_id','volume_mm3','area_mm2','xmin_mm','ymin_mm','zmin_mm','xmax_mm','ymax_mm','zmax_mm','cx_mm','cy_mm','cz_mm','triangles'])
    for s in g['step']['solids']:w.writerow([s['id'],s['volume_mm3'],s['area_mm2']]+s['bounds_mm']+s['centroid_mm']+[s['triangles']])
with (OUT/'cad_product_records.txt').open('w') as f:
    for p in g['step']['product_records']:f.write(p.replace('\n',' ')+'\n')

f,axs=plt.subplots(1,2,figsize=(10,4.8))
for ax,b,title,co in [(axs[0],track,'STEP: b = %.1f mm'%track,COL[1]),(axs[1],254.8,'URDF/SDF: b = 254,8 mm',COL[0])]:
    ax.add_patch(Rectangle((-220,-170),440,340,fill=False,ec='gray',lw=2))
    for y in [-b/2,b/2]:ax.add_patch(Rectangle((-42.5,y-15),85,30,fc=co,alpha=.8));ax.axhline(y,c=co,ls='--',alpha=.6)
    ax.annotate('',(85,-b/2),(85,b/2),arrowprops=dict(arrowstyle='<->',lw=1.5));ax.text(93,0,f'{b:.1f} mm',rotation=90,va='center')
    ax.set(xlim=(-245,245),ylim=(-195,195),aspect='equal',title=title,xlabel='x quy ước (mm)',ylabel='y quy ước (mm)')
save(f,'track_difference')
f,axs=plt.subplots(1,2,figsize=(10,5.5));stls=[m*1000 for m in np.load(OUT/'urdf_meshes.npz').values()]
if extra:aligned.append(np.concatenate(extra)@R.T+t)
for ax,dims,title in [(axs[0],(0,1),'Hình chiếu bằng'),(axs[1],(0,2),'Hình chiếu bên - neo theo trục bánh')]:
    for meshes,co,lab in [(aligned,COL[1],'STEP'),(stls,COL[0],'URDF visual')]:
        pts=np.concatenate([m.reshape(-1,3)[::12] for m in meshes]);ax.scatter(pts[:,dims[0]],pts[:,dims[1]],s=.12,alpha=.3,c=co,label=lab,rasterized=True)
    ax.set(aspect='equal',title=title,xlabel='x (mm)',ylabel='y (mm)' if dims[1]==1 else 'z so với trục bánh (mm)');ax.legend(markerscale=12,fontsize=9)
save(f,'cad_urdf_overlay')
print('CAD audit',audit,flush=True)
