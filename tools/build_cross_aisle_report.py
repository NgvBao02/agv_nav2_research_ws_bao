#!/usr/bin/env python3
"""Auditable Vietnamese PDF and publication figures from actual Gazebo trials.

Run with the dedicated PDF venv (system-site-packages enabled). No simulator,
planner, smoother or controller parameters are changed by this report builder.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import html
import io
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys
import textwrap
from datetime import datetime, timezone

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon
from matplotlib import colors as mplcolors
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.platypus import Paragraph, Table, TableStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'docs/warehouse_cross_aisles_5_routes'
FIG = BASE / 'figures'
OUT = ROOT / 'docs/PSTMO_KHO_GIAO_CAT_5_QUY_DAO.pdf'
OLD = ROOT / 'docs/pstmo_bao_cao_toan_dien_assets'
PLANNERS = ['NavFnAStar', 'NavFnDijkstra', 'ThetaStar', 'Smac2D', 'SmacHybrid']
METHODS = ['raw', 'simple', 'savitzky_golay', 'constrained', 'pstmo']
LABEL = dict(zip(METHODS, ['Raw', 'Simple', 'Savitzky-Golay', 'Constrained', 'PSTMO']))
COLOR = dict(zip(METHODS, ['#64748b', '#2563eb', '#d97706', '#9333ea', '#059669']))
STYLES = dict(zip(METHODS, [':', '--', '-.', '--', '-']))
SCENARIOS = yaml.safe_load((BASE / 'scenarios.yaml').read_text())['scenarios']
ROUTES = [s['name'] for s in SCENARIOS]
RID = {s['name']: f'R{i+1:02d}' for i, s in enumerate(SCENARIOS)}
TITLE = {s['name']: s['label'].split(' - ', 1)[1] for s in SCENARIOS}
DESCRIPTIONS = [
    'Tuyến chuẩn giữ nguyên từ mục 7.5 của PSTMO.pdf. Robot xuất phát trong lối phía Nam, chuyển qua giao cắt trung tâm và đi vào lối phía Bắc. Đây là mốc truy vết C21-C25.',
    'Tuyến dài nối hành lang dịch vụ phía Tây với phía Đông qua vùng giao cắt. Chiều dài tăng làm rõ đánh đổi giữa đoạn thẳng hành trình và các chuyển tiếp vào/ra hành lang.',
    'Hai điểm nằm cùng phía Nam nhưng bị các kệ ngăn cách. Robot phải đi lên vùng giao cắt, chuyển ngang và đi xuống lối đích. Hai lần đổi hướng lớn liên tiếp tạo bài toán chữ U có ý nghĩa kiểm tra động học.',
    'Tuyến chéo ngược từ phía Tây Bắc xuống lối phía Đông Nam. Chiều chuyển động và thứ tự các góc khác R01/R02, giúp quan sát độ nhạy theo hướng tiếp cận và vị trí đầu kệ.',
    'Robot đi từ hành lang ngang phía Tây vào lối lấy hàng giữa hai kệ phía Bắc. Tuyến tập trung vào pha rẽ và ổn định hướng sau khi vào lối, thay vì chỉ đánh giá chiều dài đường chéo.',
]
NAVY = '#143247'
TEAL = '#087f8c'
LIGHT = '#edf4f7'
FIGURES = []
REUSE_FIGURES = True
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
    'axes.spines.top': False, 'axes.spines.right': False, 'axes.titleweight': 'bold',
    'axes.labelcolor': NAVY, 'text.color': NAVY, 'axes.titlecolor': NAVY,
    'grid.alpha': .18, 'svg.fonttype': 'none', 'pdf.fonttype': 42})
pixels = np.array(Image.open(ROOT / 'src/vacuum_robot_gazebo/maps/warehouse_cross_aisles.pgm'))
DISTANCE = distance_transform_edt(pixels > 100) * .05
gx, gy = np.meshgrid(np.linspace(-.22,.22,19), np.linspace(-.17,.17,15))
FOOT = np.column_stack((gx.ravel(),gy.ravel()))


def fmt(x, n=3):
    if x is None or not math.isfinite(float(x)):
        return '-'
    return f'{float(x):.{n}f}'.replace('.', ',')


def digest(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def load_records(partial=False):
    records = {}
    missing = []
    for route in ROUTES:
        for planner in PLANNERS:
            for method in METHODS:
                if route == ROUTES[0]:
                    if planner == 'ThetaStar':
                        p = ROOT / f'results/pstmo_execution_theta_star_20260802/warehouse_cross_aisles/{route}_{method}.json'
                    else:
                        p = ROOT / f'results/pstmo_execution_full_20260803/warehouse_cross_aisles/{route}_{planner.lower()}_{method}.json'
                else:
                    p = BASE / 'execution' / route / f'{route}_{planner.lower()}_{method}.json'
                if not p.exists():
                    missing.append(str(p))
                    continue
                try:
                    d = json.loads(p.read_text())
                except json.JSONDecodeError:
                    if partial:continue
                    raise
                if d.get('method') != method or d.get('planner') != planner:
                    raise ValueError(f'identity mismatch: {p}')
                d['_source'] = str(p.relative_to(ROOT))
                d['_cohort'] = 'historic_20260802_03' if route == ROUTES[0] else 'new_20261003'
                records[route,planner,method] = d
    if missing and not partial:
        raise RuntimeError(f'{len(missing)} trials missing; refusing final PDF')
    return records


def diagnostics(route, planner):
    if route == ROUTES[0]:
        p = OLD / f'rviz_cases/warehouse_cross_aisles__{route}__{planner}.json'
        return json.loads(p.read_text())['pstmo_diagnostics']
    p = BASE / 'execution' / route / f'{route}_{planner.lower()}_pstmo.diagnostics.json'
    if not p.exists():
        return {}
    events = json.loads(p.read_text()).get('events',[])
    return next((d for d in reversed(events) if d.get('search_mode')=='hierarchical_alpha_two_trim'),{})


def xy(d, key='selected_path_xy'):
    return np.asarray(d.get(key,[]),dtype=float).reshape((-1,2))


def arc(points):
    return np.r_[0,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))] if len(points) else np.array([])


def curvature(points):
    if len(points)<3:
        return np.zeros(len(points))
    a,b,c=points[:-2],points[1:-1],points[2:]
    ab=b-a;bc=c-b;ac=c-a
    den=np.linalg.norm(ab,axis=1)*np.linalg.norm(bc,axis=1)*np.linalg.norm(ac,axis=1)
    cross=ab[:,0]*ac[:,1]-ab[:,1]*ac[:,0]
    k=np.divide(2*cross,den,out=np.zeros_like(den),where=den>1e-12)
    return np.r_[0,k,0]


def track_errors(points, ref):
    if not len(points) or len(ref)<2:
        return np.zeros(len(points))
    a=ref[:-1];v=np.diff(ref,axis=0);den=(v*v).sum(axis=1)
    errors=[]
    for start in range(0,len(points),200):
        delta=points[start:start+200,None,:]-a[None,:,:]
        t=np.clip(np.divide((delta*v).sum(axis=2),den,out=np.zeros(delta.shape[:2]),where=den>1e-15),0,1)
        errors.extend(np.min(np.linalg.norm(delta-t[:,:,None]*v,axis=2),axis=1))
    return np.array(errors)


def footprint_clearance(poses):
    poses=np.asarray(poses)
    if len(poses)==0:
        return np.array([])
    c=np.cos(poses[:,2,None]);s=np.sin(poses[:,2,None])
    x=poses[:,0,None]+c*FOOT[None,:,0]-s*FOOT[None,:,1]
    y=poses[:,1,None]+s*FOOT[None,:,0]+c*FOOT[None,:,1]
    ix=np.floor((x+6)/.05).astype(int);iy=159-np.floor((y+4)/.05).astype(int)
    outside=(ix<0)|(ix>=240)|(iy<0)|(iy>=160)
    values=np.maximum(DISTANCE[np.clip(iy,0,159),np.clip(ix,0,239)]-.5*np.sqrt(2)*.05,0)
    values[outside]=0
    return values.min(axis=1)


def map_axis(ax, bounds=None, label=True):
    ax.imshow(pixels,cmap=mplcolors.ListedColormap(['#354b5d','#f8fafc']),vmin=0,vmax=254,
        extent=(-6,6,-4,4),origin='upper',interpolation='nearest',zorder=0)
    ax.set_aspect('equal');ax.set_xlim(-6,6);ax.set_ylim(-4,4)
    if bounds:
        ax.set_xlim(bounds[:2]);ax.set_ylim(bounds[2:])
    ax.grid(True,zorder=0)
    if label:
        ax.set_xlabel('x (m)');ax.set_ylabel('y (m)')


def endpoints(ax, d, small=False):
    if not d.get('start'):
        return
    for key,color,marker,letter in [('start','#0d9488','o','S'),('goal','#be185d','*','G')]:
        p=d[key];ax.scatter(p[0],p[1],s=40 if small else 65,c=color,marker=marker,zorder=8,edgecolors='white',linewidths=.8)
        if not small:
            ax.annotate(letter,p[:2],xytext=(5,5),textcoords='offset points',weight='bold',color=color)
        ax.arrow(p[0],p[1],.32*np.cos(p[2]),.32*np.sin(p[2]),head_width=.11,color=color,zorder=7,length_includes_head=True)


def plot_path(ax, d, method, label=True, lw=None):
    p=xy(d)
    if len(p):
        ax.plot(p[:,0],p[:,1],color=COLOR[method],ls=STYLES[method],lw=lw or (2.4 if method=='pstmo' else 1.5),
            label=LABEL[method] if label else None,zorder=5 if method=='pstmo' else 3)


def savefig(fig, name):
    FIG.mkdir(parents=True,exist_ok=True)
    p=FIG/f'{name}.png'
    fig.savefig(p,dpi=220,bbox_inches='tight',facecolor='white')
    fig.savefig(p.with_suffix('.svg'),bbox_inches='tight',facecolor='white')
    plt.close(fig)
    FIGURES.append(str(p.relative_to(ROOT)))
    return p


def asset(name, producer, *args):
    """Reuse the already-audited scientific artwork during editorial revisions."""
    p=FIG/f'{name}.png'
    if REUSE_FIGURES and p.exists() and p.with_suffix('.svg').exists():
        FIGURES.append(str(p.relative_to(ROOT)))
        return p
    return producer(*args)


def geometry_figure(group, diag, name):
    fig=plt.figure(figsize=(12,8.7),layout='constrained')
    gs=fig.add_gridspec(3,3,height_ratios=[1,1,.85])
    main=fig.add_subplot(gs[:2,:2]);map_axis(main)
    for m in METHODS:plot_path(main,group[m],m)
    main.set_title('Đường hình học từ các mẫu ROS Path đã ghi nhận')
    d=group['pstmo'];endpoints(main,d)
    anchors=np.asarray(diag.get('conditioned_polyline',[]))
    if len(anchors):
        main.plot(anchors[:,0],anchors[:,1],'o',ms=3,mfc='white',mec='#15803d',alpha=.8,zorder=6,label='Neo PSTMO')
    main.legend(fontsize=8,loc='upper left',ncol=2,framealpha=.92)
    corners=diag.get('corner_search',[])
    selected=sorted(corners,key=lambda c:abs(c.get('turn_angle',0)),reverse=True)
    centers=[]
    for c in selected:
        p=np.array([c['x'],c['y']])
        if all(np.linalg.norm(p-other)>1 for other in centers):centers.append(p)
        if len(centers)==2:break
    points=xy(d)
    while len(centers)<2:
        centers.append(points[int((len(centers)+1)*max(len(points)-1,0)/3)] if len(points) else np.array([0,0]))
    for i,center in enumerate(centers):
        ax=fig.add_subplot(gs[i,2]);x,y=center
        bounds=(x-1.05,x+1.05,y-1.05,y+1.05);map_axis(ax,bounds)
        main.add_patch(Rectangle((x-1.05,y-1.05),2.1,2.1,fill=False,lw=.8,ec=TEAL,ls='--'))
        for m in METHODS:plot_path(ax,group[m],m,False,1.8 if m=='pstmo' else 1)
        ax.set_title(f'Phóng to vùng chuyển hướng {i+1}',fontsize=10)
    ax=fig.add_subplot(gs[2,:2])
    for m in METHODS:
        p=xy(group[m])
        if len(p):ax.plot(arc(p),curvature(p),color=COLOR[m],lw=1.3,label=LABEL[m])
    ax.set_yscale('symlog',linthresh=1);ax.grid(True);ax.set_xlabel('Chiều dài tích lũy s (m)')
    ax.set_ylabel('κ (m⁻¹), symlog');ax.set_title('Độ cong ba điểm trên mẫu đường gốc',fontsize=10)
    ax=fig.add_subplot(gs[2,2])
    vals=[group[m].get('planned_curvature_energy_1pm',np.nan) for m in METHODS]
    ax.barh([LABEL[m] for m in METHODS],vals,color=[COLOR[m] for m in METHODS]);ax.invert_yaxis()
    ax.set_xscale('log');ax.set_xlabel('Eκ (m⁻¹), log');ax.set_title('Tổng mức uốn hình học',fontsize=10);ax.grid(axis='x')
    return savefig(fig,name)


def execution_figure(group,name):
    fig,axes=plt.subplots(2,3,figsize=(12,7.7),layout='constrained')
    for ax,m in zip(axes.ravel(),METHODS):
        d=group[m];map_axis(ax)
        p=xy(d);e=xy(d,'executed_path_xy')
        if len(p):ax.plot(p[:,0],p[:,1],ls='--',lw=1.6,color='#2563eb',label='Đường kế hoạch')
        if len(e):ax.plot(e[:,0],e[:,1],lw=1.8,color='#dc2626',label='Robot Gazebo')
        endpoints(ax,d,True)
        status='ĐẠT' if d.get('success') else 'KHÔNG ĐẠT'
        ax.set_title(f'{LABEL[m]} | {status}\nT={fmt(d.get("execution_time_s"))} s; RMSE={fmt(d.get("tracking_rmse_m"))} m',fontsize=9)
    ax=axes.ravel()[-1];ax.axis('off')
    ax.plot([],[],ls='--',lw=1.6,color='#2563eb',label='Đường được giao cho FollowPath')
    ax.plot([],[],lw=1.8,color='#dc2626',label='Quỹ đạo ground truth Gazebo')
    ax.legend(loc='upper left',fontsize=10,frameon=False)
    hashes={d.get('raw_path_sha256') for d in group.values() if d.get('raw_path_sha256')}
    ax.text(0,.62,f'Ghép cặp Raw: {"KHỚP" if len(hashes)==1 else "KHÔNG KHỚP"}\n\nS: xuất phát; G: đích.\nMỗi lượt dùng phiên mô phỏng riêng.\nCùng bản đồ, robot và bộ điều khiển.\n\nĐường vẽ từ tọa độ JSON gốc;\nkhông dịch hoặc nắn quỹ đạo.',va='top',transform=ax.transAxes,fontsize=10,linespacing=1.6)
    return savefig(fig,name)


def dynamics_figure(group,name):
    fig,axs=plt.subplots(3,2,figsize=(12,9),layout='constrained')
    for m in METHODS:
        d=group[m];tr=np.asarray(d.get('ground_truth_state_trace',[]),float)
        if len(tr)==0:continue
        stride=max(1,len(tr)//900);t=tr[::stride];p=xy(d)
        color=COLOR[m];lw=1.8 if m=='pstmo' else 1
        axs[0,0].plot(t[:,0],t[:,4],color=color,lw=lw,label=LABEL[m])
        axs[0,1].plot(t[:,0],t[:,5],color=color,lw=lw)
        err=track_errors(t[:,1:3],p)
        axs[1,0].plot(t[:,0],err,color=color,lw=lw)
        clr=footprint_clearance(t[:,1:4]);axs[1,1].plot(t[:,0],clr,color=color,lw=lw)
        loc=np.asarray(d.get('localization_error_trace',[]),float)
        if len(loc):axs[2,0].plot(loc[::stride,0],loc[::stride,1],color=color,lw=lw)
        cmd=np.asarray(d.get('command_velocity_trace',[]),float)
        if len(cmd) and m=='pstmo':
            axs[2,1].plot(cmd[:,0],cmd[:,1],color=TEAL,lw=1,label='v lệnh')
            axs[2,1].plot(t[:,0],t[:,4],color='#dc2626',lw=1,label='v thực Gazebo')
    titles=['Vận tốc tịnh tiến thực','Vận tốc góc thực','Sai lệch ngang tới đường kế hoạch','Khoảng hở footprint thực (hậu kiểm PGM)','Sai số định vị vị trí','PSTMO: lệnh và đáp ứng vận tốc']
    units=['v (m/s)','ω (rad/s)','e (m)','c (m)','e định vị (m)','v (m/s)']
    for ax,title,unit in zip(axs.ravel(),titles,units):
        ax.set_title(title,fontsize=10);ax.set_xlabel('Thời gian mô phỏng t (s)');ax.set_ylabel(unit);ax.grid(True)
    axs[0,0].legend(fontsize=8,ncol=3,loc='upper right');axs[2,1].legend(fontsize=8)
    axs[1,1].axhline(0,color='#dc2626',lw=.8)
    return savefig(fig,name)


def route_overview(route, records, name):
    fig,axes=plt.subplots(2,3,figsize=(12,7.8),layout='constrained')
    for ax,planner in zip(axes.ravel(),PLANNERS):
        map_axis(ax)
        for m in ['raw','pstmo']:plot_path(ax,records[route,planner,m],m)
        d=records[route,planner,'pstmo'];endpoints(ax,d,True)
        ax.set_title(f'{planner}\nL={fmt(d.get("planned_path_length_m"))} m',fontsize=10)
    ax=axes.ravel()[-1];map_axis(ax)
    for planner,color in zip(PLANNERS,['#2563eb','#d97706','#059669','#9333ea','#dc2626']):
        p=xy(records[route,planner,'pstmo'])
        if len(p):ax.plot(p[:,0],p[:,1],color=color,lw=1.5,label=planner)
    ax.set_title('PSTMO theo 5 đường đầu vào',fontsize=10);ax.legend(fontsize=7,loc='upper left')
    return savefig(fig,name)


def construction_figure(route,records,diag,name):
    planner='ThetaStar';d=records[route,planner,'pstmo']
    group=geometry_group(records,route,planner)
    points=xy(group['pstmo']);anchors=np.asarray(diag.get('conditioned_polyline',[]),float)
    fig,axs=plt.subplots(2,2,figsize=(12,8.6),layout='constrained')
    ax=axs[0,0];map_axis(ax)
    plot_path(ax,group['raw'],'raw');plot_path(ax,group['pstmo'],'pstmo')
    if len(anchors):
        ax.plot(anchors[:,0],anchors[:,1],'o--',color='#d97706',ms=4,lw=1,label='Neo sau conditioning')
        for i,p in enumerate(anchors[1:-1],1):ax.annotate(str(i),p,xytext=(4,4),textcoords='offset points',fontsize=8)
    endpoints(ax,d);ax.legend(fontsize=8);ax.set_title('Từ đường đầu vào tới chuỗi neo',fontsize=11)
    candidates=[c for c in diag.get('corner_search',[]) if c.get('selected_trim',0)>0 and c.get('selected_control_fraction',0)>0]
    if not candidates or len(anchors)<3:
        for ax in axs.ravel()[1:]:ax.text(.1,.5,'Không có chuyển tiếp đủ dữ liệu',transform=ax.transAxes)
        return savefig(fig,name)
    corner=max(candidates,key=lambda c:abs(c['turn_angle']));v=np.array([corner['x'],corner['y']])
    k=int(np.argmin(np.linalg.norm(anchors-v,axis=1)));k=min(max(k,1),len(anchors)-2)
    incoming=(v-anchors[k-1]);incoming/=np.linalg.norm(incoming)
    outgoing=anchors[k+1]-v;outgoing/=np.linalg.norm(outgoing)
    trim=corner['selected_trim'];q=trim*corner['selected_control_fraction']
    a=v-trim*incoming;b=v+trim*outgoing
    ctrl=np.array([a,a+q*incoming,a+2*q*incoming,b-2*q*outgoing,b-q*outgoing,b])
    t=np.linspace(0,1,201)
    def bezier(c,t):
        n=len(c)-1
        return sum(math.comb(n,i)*(1-t[:,None])**(n-i)*t[:,None]**i*c[i] for i in range(n+1))
    curve=bezier(ctrl,t);d1=bezier(5*np.diff(ctrl,axis=0),t);d2=bezier(20*np.diff(ctrl,n=2,axis=0),t)
    curv=(d1[:,0]*d2[:,1]-d1[:,1]*d2[:,0])/np.maximum(np.linalg.norm(d1,axis=1)**3,1e-12)
    radius=max(.5,trim*1.3);bounds=(v[0]-radius,v[0]+radius,v[1]-radius,v[1]+radius)
    ax=axs[0,1];map_axis(ax,bounds)
    ax.plot([anchors[k-1,0],v[0],anchors[k+1,0]],[anchors[k-1,1],v[1],anchors[k+1,1]],'--',color='#d97706',label='Polyline')
    ax.plot(ctrl[:,0],ctrl[:,1],'o--',color='#9333ea',ms=4,lw=1,label='Đa giác điều khiển')
    ax.plot(curve[:,0],curve[:,1],color=COLOR['pstmo'],lw=2.5,label='Bézier tái dựng')
    for j,p in enumerate(ctrl):ax.annotate(f'P{j}',p,xytext=(4,7 if j%2==0 else -12),textcoords='offset points',fontsize=8)
    ax.set_title(f'Góc {k}: d={trim:.3f} m; α={corner["selected_control_fraction"]:.3f}',fontsize=11)
    ax.legend(fontsize=7,loc='best')
    ax=axs[1,0];map_axis(ax,bounds)
    plot_path(ax,group['pstmo'],'pstmo')
    poses=np.asarray(d.get('selected_path_poses',[]))
    if len(poses):
        close=poses[np.linalg.norm(poses[:,:2]-v,axis=1)<radius*.95]
        local=np.array([[-.22,-.17],[.22,-.17],[.22,.17],[-.22,.17]])
        for p in close[::max(1,len(close)//10)]:
            c,s=np.cos(p[2]),np.sin(p[2]);R=np.array([[c,-s],[s,c]])
            ax.add_patch(Polygon(local@R.T+p[:2],closed=True,fc='#059669',ec='#047857',alpha=.12,lw=.9))
    ax.set_title('Hình bao robot tại các mẫu đường',fontsize=11)
    ax=axs[1,1];ax.plot(t,curv,color=COLOR['pstmo'],lw=2);ax.scatter([0,1],[curv[0],curv[-1]],color='#d97706',zorder=5)
    ax.axhline(0,color='#64748b',lw=.8);ax.set_xlabel('Tham số Bézier u');ax.set_ylabel('κ giải tích (m⁻¹)');ax.grid(True)
    ax.set_title('Độ cong chuyển tiếp tái dựng: κ(0)=κ(1)=0',fontsize=10)
    return savefig(fig,name)


def construction_page(report,records,route):
    report.start(f'A{ROUTES.index(route)+1} | {RID[route]} - Cơ chế tạo đường',
        'Minh họa với ThetaStar; thuật ngữ và vai trò các bước được giải thích ở mục 1.2.',bookmark='detail_'+route,level=1)
    diag=diagnostics(route,'ThetaStar')
    p=asset(f'{RID[route]}_construction',construction_figure,route,records,diag,f'{RID[route]}_construction')
    report.figure(p,height=365,caption='Đường và footprint lấy từ bản ghi; Bézier minh họa được tái dựng từ d, α và tọa độ đã làm tròn trong diagnostics. Đường tái dựng chỉ giải thích cấu trúc, không thay đường đã chạy trong bảng kết quả.')
    rows=[]
    for planner in PLANNERS:
        dg=diagnostics(route,planner);cs=dg.get('corner_search',[])
        trims=[c['selected_trim'] for c in cs if c.get('selected_trim',0)>0]
        alphas=[c['selected_control_fraction'] for c in cs if c.get('selected_control_fraction',0)>0]
        rows.append([planner,str(dg.get('conditioning_output_points','-')),str(dg.get('g2_transitions','-')),str(dg.get('pivots','-')),
            f'{fmt(min(trims))}-{fmt(max(trims))}' if trims else '-',f'{fmt(min(alphas))}-{fmt(max(alphas))}' if alphas else '-',str(dg.get('dp_states','-'))])
    report.table(['Planner','Neo','G²','Pivot','d chọn min-max (m)','α chọn min-max','Trạng thái DP'],rows,[1.4,.5,.4,.5,1.2,1.2,.7],size=7.2)
    report.para('Với đỉnh V và các hướng đơn vị u, v: A = V − d·u; B = V + d·v; q = α·d. Sáu điểm điều khiển là A, A+q·u, A+2q·u, B−2q·v, B−q·v, B. Hai đạo hàm bậc hai ở đầu/cuối bằng 0, tạo κ = 0 để nối với các đoạn thẳng.',size=8.7)
    report.para('DP chỉ nối hai trạng thái khi dᵢ + dᵢ₊₁ + m ≤ Lᵢ. Chuyển tiếp còn phải qua cổng vùng quét footprint, động học vi sai và ưu thế thời gian. Các hình bao mờ trong hình là mẫu minh họa, không hiển thị toàn bộ các mẫu kiểm tra an toàn nội bộ.',size=8.7)
    report.para('Trong bảng, min-max là khoảng giá trị đã được chọn qua các góc của một đường. Ở điều kiện nối, Lᵢ là chiều dài đoạn giữa hai neo và m là biên đoạn dự phòng.',size=8.2,color='#475569')


def overview_figure(records):
    fig,axs=plt.subplots(2,3,figsize=(12,8),layout='constrained')
    for route,ax in zip(ROUTES,axs.ravel()):
        map_axis(ax);d=records[route,'ThetaStar','pstmo'];p=xy(d)
        if len(p):ax.plot(p[:,0],p[:,1],color=TEAL,lw=2.5)
        endpoints(ax,d);ax.set_title(textwrap.fill(f'{RID[route]} | {TITLE[route]}',32),fontsize=9)
    axs.ravel()[-1].axis('off')
    axs.ravel()[-1].text(.05,.9,'5 tuyến · 1 bản đồ\n5 planner · 5 phương án\n\nR01: dữ liệu gốc 08/2026\nR02-R05: chạy mới 10/2026\n\nMinh họa: đường PSTMO\nvới đầu vào ThetaStar.\nMọi đường dùng tọa độ thật.',va='top',fontsize=12,linespacing=1.7)
    return savefig(fig,'overview_5_routes')


def map3d(records):
    fig=plt.figure(figsize=(12,7.5));ax=fig.add_subplot(111,projection='3d')
    ax.bar3d([-6],[-4],[-.1],[12],[8],[.1],color='#e2e8f0',alpha=.25,shade=True)
    for x in [-3,-1,1,3]:
        for y in [-2.4,2.4]:ax.bar3d([x-.3],[y-1.1],[0],[.6],[2.2],[1.2],color='#5e7e97',alpha=.7,shade=True)
    for route,color in zip(ROUTES,['#059669','#2563eb','#d97706','#9333ea','#dc2626']):
        p=xy(records[route,'ThetaStar','pstmo'])
        if len(p):ax.plot(p[:,0],p[:,1],np.full(len(p),.025),color=color,lw=2.5,label=RID[route])
    ax.set_xlim(-6,6);ax.set_ylim(-4,4);ax.set_zlim(0,2);ax.set_box_aspect((12,8,2))
    ax.view_init(elev=58,azim=-64);ax.set_xlabel('x (m)');ax.set_ylabel('y (m)');ax.set_zlabel('z (m)')
    ax.legend(loc='upper left',ncol=5);fig.tight_layout()
    return savefig(fig,'map_3d_routes')


def dimensions_figure():
    fig,axes=plt.subplots(1,2,figsize=(12,3.8),layout='constrained',gridspec_kw={'width_ratios':[1.6,1]})
    ax=axes[0];map_axis(ax)
    ax.annotate('',xy=(0,1.3),xytext=(0,-1.3),arrowprops={'arrowstyle':'<->','color':'#dc2626','lw':1.7})
    ax.text(.2,0,'2,6 m',color='#dc2626',bbox={'fc':'white','ec':'none'},fontsize=10)
    ax.annotate('',xy=(.7,2.3),xytext=(-.7,2.3),arrowprops={'arrowstyle':'<->','color':TEAL,'lw':1.7})
    ax.text(0,2.7,'1,4 m',color=TEAL,ha='center',bbox={'fc':'white','ec':'none'},fontsize=10)
    ax.set_title('Kích thước từ world SDF / PGM',fontsize=11)
    ax=axes[1];ax.set_aspect('equal');ax.set_xlim(-.35,.4);ax.set_ylim(-.32,.32)
    ax.add_patch(Rectangle((-.22,-.17),.44,.34,fc='#bce5df',ec='#059669',lw=2))
    ax.scatter([0],[0],color=NAVY,s=20);ax.arrow(0,0,.29,0,head_width=.025,color=NAVY,length_includes_head=True)
    ax.text(.29,.025,'x thân',fontsize=9)
    ax.annotate('',xy=(-.22,-.23),xytext=(.22,-.23),arrowprops={'arrowstyle':'<->','color':NAVY})
    ax.text(0,-.28,'0,44 m',ha='center',fontsize=10)
    ax.annotate('',xy=(-.28,-.17),xytext=(-.28,.17),arrowprops={'arrowstyle':'<->','color':NAVY})
    ax.text(-.31,0,'0,34 m',rotation=90,va='center',fontsize=10)
    ax.set_title('Footprint chữ nhật dùng kiểm tra',fontsize=11);ax.set_xlabel('x cục bộ (m)');ax.set_ylabel('y cục bộ (m)');ax.grid(True)
    return savefig(fig,'map_and_footprint_dimensions')


def sampling_figure(records):
    fig,axes=plt.subplots(1,2,figsize=(12,3.4),layout='constrained')
    group=geometry_group(records,ROUTES[0],'ThetaStar')
    for ax,m in zip(axes,['raw','pstmo']):
        points=xy(group[m]);s=arc(points);unique=np.r_[True,np.diff(s)>1e-9];s=s[unique];points=points[unique]
        ns=np.r_[np.arange(0,s[-1],.05),s[-1]]
        new=np.column_stack((np.interp(ns,s,points[:,0]),np.interp(ns,s,points[:,1])))
        ax.plot(s,curvature(points),color='#64748b',lw=1,label='Mẫu gốc')
        ax.plot(ns,curvature(new),color=COLOR['pstmo'],lw=1.4,label='Resample 0,05 m')
        ax.set_title(f'R01 / ThetaStar / {LABEL[m]}',fontsize=11);ax.set_xlabel('s (m)');ax.set_ylabel('κ (m⁻¹)');ax.grid(True);ax.legend(fontsize=8)
    return savefig(fig,'sampling_sensitivity_example')


def aggregate_fig(records, metric, name, ylabel):
    fig,axs=plt.subplots(1,2,figsize=(12,5.5),layout='constrained',gridspec_kw={'width_ratios':[1.1,1]})
    vals=[]
    for r in ROUTES:
        for p in PLANNERS:
            a=records[r,p,'raw'].get(metric);b=records[r,p,'pstmo'].get(metric)
            pair_ok=metric!='execution_time_s' or (records[r,p,'raw'].get('success') and records[r,p,'pstmo'].get('success'))
            vals.append(100*(a-b)/a if a and b is not None and pair_ok else np.nan)
    mat=np.array(vals).reshape(5,5)
    lim=max(10,min(100,np.nanmax(abs(mat))))
    clearance_metric=metric=='planned_footprint_clearance_min_m'
    im=axs[0].imshow(mat,cmap='RdYlGn_r' if clearance_metric else 'RdYlGn',vmin=-lim,vmax=lim,aspect='auto')
    axs[0].set_xticks(range(5),PLANNERS,rotation=30,ha='right');axs[0].set_yticks(range(5),[RID[r] for r in ROUTES])
    for i in range(5):
        for j in range(5):axs[0].text(j,i,(fmt(mat[i,j],1)+'%') if np.isfinite(mat[i,j]) else 'N/A',ha='center',va='center',fontsize=10)
    axs[0].set_title('PSTMO so với Raw: % hao hụt khoảng hở\nSố âm là khoảng hở tăng' if clearance_metric else 'PSTMO so với Raw: % giảm\nSố âm là tăng chỉ số',fontsize=11)
    fig.colorbar(im,ax=axs[0],fraction=.04,pad=.03)
    for j,m in enumerate(METHODS):
        means=[]
        for r in ROUTES:
            v=[records[r,p,m].get(metric) for p in PLANNERS if metric!='execution_time_s' or records[r,p,m].get('success')]
            v=[x for x in v if x is not None]
            means.append(statistics.fmean(v) if v else np.nan)
        axs[1].plot(range(5),means,'o-',color=COLOR[m],label=LABEL[m],lw=2 if m=='pstmo' else 1)
    axs[1].set_xticks(range(5),[RID[r] for r in ROUTES]);axs[1].set_ylabel(ylabel);axs[1].grid(True)
    axs[1].set_title('Trung bình theo planner của mỗi tuyến',fontsize=11);axs[1].legend(fontsize=8)
    return savefig(fig,name)


FONT='/usr/share/fonts/truetype/dejavu/'
for name,file in [('DV','DejaVuSans.ttf'),('DVB','DejaVuSans-Bold.ttf'),('DVI','DejaVuSans-Oblique.ttf')]:
    pdfmetrics.registerFont(TTFont(name,FONT+file))
pdfmetrics.registerFontFamily('DV',normal='DV',bold='DVB',italic='DVI',boldItalic='DVB')
W,H=A4;M=38;CW=W-2*M


class Report:
    def __init__(self,path,refs=None,dry=False):
        self.dry=dry;self.refs=refs or {};self.destinations={};self.figure_pages=[]
        self.c=canvas.Canvas(io.BytesIO() if dry else str(path),pagesize=A4,pageCompression=1)
        self.c.setTitle('PSTMO | Kho có lối giao cắt | Nghiên cứu 5 quỹ đạo')
        self.c.setAuthor('Báo cáo thực nghiệm từ workspace AGV Nav2')
        self.c.setCreator('AGV Nav2 research workspace - measured-data report builder')
        self.c.setSubject('Five routes, 125 Gazebo/Nav2 trials, paired planner and smoother comparisons')
        self.page=0;self.y=H-60;self.sections=[];self.figure_count=0
    def ref(self,key):
        return str(self.refs.get(key,'...'))
    def mark(self,title,key,level=0):
        self.c.bookmarkPage(key);self.c.addOutlineEntry(title.replace('<br/>',' - '),key,level)
        self.destinations[key]=self.page
    def start(self,title,subtitle='',bookmark=None,level=0):
        if self.page:self.c.showPage()
        self.page+=1;c=self.c
        c.setFillColor(colors.HexColor(NAVY));c.rect(0,H-28,W,28,fill=1,stroke=0)
        c.setFont('DVB',8);c.setFillColor(colors.white);c.drawString(M,H-18,'PSTMO  /  NGHIÊN CỨU CHUYÊN SÂU KHO GIAO CẮT')
        c.setFillColor(colors.HexColor('#64748b'));c.setFont('DV',7)
        c.drawString(M,20,'Dữ liệu Gazebo / Nav2 • R01: 08/2026; R02-R05: 10/2026')
        c.drawRightString(W-M,20,str(self.page))
        self.y=H-47
        self.para(title,size=17,bold=True,space=7)
        if subtitle:self.para(subtitle,size=8.5,color='#64748b',space=9)
        if bookmark:
            outline_title={'reader':'1 | Hệ thống và thuật ngữ','protocol':'2 | Thiết kế thí nghiệm và chỉ số',
                'summary':'3 | Kết quả tổng hợp','route_chapter':'4 | Kết quả theo từng tuyến',
                'discussion':'5 | Thảo luận và giới hạn','sources':'C | Nguồn dữ liệu và tái lập'}.get(bookmark,title)
            self.mark(outline_title,bookmark,level)
        self.sections.append({'page':self.page,'title':title,'bookmark':bookmark})
    def para(self,text,size=9.2,bold=False,color=NAVY,space=6):
        style=ParagraphStyle('p',fontName='DVB' if bold else 'DV',fontSize=size,leading=size*1.45,textColor=colors.HexColor(color))
        p=Paragraph(text,style);_,h=p.wrap(CW,1000)
        if self.y-h<36:raise RuntimeError(f'page {self.page} overflow: {text[:70]} y={self.y} h={h}')
        p.drawOn(self.c,M,self.y-h);self.y-=h+space
    def figure(self,path,height=None,caption=''):
        with Image.open(path) as im:ratio=im.height/im.width
        h=height or CW*ratio;w=min(CW,h/ratio);h=w*ratio
        if self.y-h<36:raise RuntimeError(f'figure overflow page {self.page} {path} y={self.y},h={h}')
        if not self.dry:self.c.drawImage(str(path),M+(CW-w)/2,self.y-h,width=w,height=h,mask='auto')
        self.y-=h+4
        if caption:
            self.figure_count+=1
            self.figure_pages.append({'figure':self.figure_count,'page':self.page,'path':str(Path(path).relative_to(ROOT)),'caption':caption})
            self.para(f'Hình {self.figure_count}. {caption}',size=7.5,color='#475569',space=7)
    def table(self,headers,rows,widths=None,size=7.2,links=None):
        style=ParagraphStyle('cell',fontName='DV',fontSize=size,leading=size*1.3,textColor=colors.HexColor(NAVY))
        hstyle=ParagraphStyle('head',parent=style,fontName='DVB',textColor=colors.white)
        data=[[Paragraph(html.escape(str(v)),hstyle) for v in headers]]+[[Paragraph(html.escape(str(v)),style) for v in row] for row in rows]
        if links:
            for i,key in enumerate(links):
                if key:data[i+1][0]=Paragraph(f'<link href="#{key}" color="{TEAL}">{html.escape(str(rows[i][0]))}</link>',style)
        widths=[CW*x/sum(widths) for x in widths] if widths else [CW/len(headers)]*len(headers)
        t=Table(data,colWidths=widths,hAlign='LEFT')
        t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor(NAVY)),('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,colors.HexColor(LIGHT)]),('VALIGN',(0,0),(-1,-1),'MIDDLE'),('BOTTOMPADDING',(0,0),(-1,-1),4),('TOPPADDING',(0,0),(-1,-1),4),('LEFTPADDING',(0,0),(-1,-1),4),('RIGHTPADDING',(0,0),(-1,-1),4),('LINEBELOW',(0,0),(-1,0),.7,colors.HexColor(TEAL))]))
        _,h=t.wrap(CW,1000)
        if self.y-h<36:raise RuntimeError(f'table overflow page {self.page}: y={self.y},h={h}')
        t.drawOn(self.c,M,self.y-h);self.y-=h+9
    def finish(self):
        self.c.save()
        if not self.dry:
            (BASE/'page_index.json').write_text(json.dumps(self.sections,ensure_ascii=False,indent=2))
            (BASE/'figure_page_index.json').write_text(json.dumps(self.figure_pages,ensure_ascii=False,indent=2))


def geometry_rows(group):
    return [[LABEL[m], 'Có' if group[m].get('selected_path_xy') else 'Thiếu',
        fmt(group[m].get('planned_path_length_m')),fmt(group[m].get('planned_max_abs_curvature_1pm')),
        fmt(group[m].get('planned_curvature_energy_1pm')),
        fmt(1000*group[m]['smoothing_time_s']) if group[m].get('smoothing_time_s') is not None else '-',
        fmt(group[m].get('planned_footprint_clearance_min_m')),
        str(group[m].get('planned_footprint_collision_sample_count','-'))] for m in METHODS]


def geometry_group(records,route,planner):
    group={m:dict(records[route,planner,m]) for m in METHODS}
    if route!=ROUTES[0]:return group
    source=json.loads((OLD/f'rviz_cases/warehouse_cross_aisles__{route}__{planner}.json').read_text())
    with (OLD/'benchmark_hinh_hoc_175_luot.csv').open() as f:
        rows={r['method']:r for r in csv.DictReader(f) if r['environment']=='warehouse_cross_aisles' and r['planner']==planner}
    for m in METHODS:
        poses=source['paths'][m]['poses'];met=source['metrics'][m];row=rows[m]
        group[m]['selected_path_xy']=[[p['x'],p['y']] for p in poses]
        for key in ['path_length_m','max_abs_curvature_1pm','curvature_energy_1pm']:
            group[m]['planned_'+key]=met[key]
        group[m]['smoothing_time_s']=met.get('smoothing_time_s',0)
        group[m]['planned_footprint_clearance_min_m']=float(row['footprint_clearance_min_m'])
        group[m]['planned_footprint_collision_sample_count']=int(row['footprint_collision_sample_count'])
    return group


def change(new, old):
    if new is None or old is None or abs(old)<1e-12:return 'không đủ dữ liệu'
    delta=100*(new-old)/old
    return ('tăng ' if delta>0 else 'giảm ')+fmt(abs(delta),2)+'%'


def case_pages(report, records, route, planner):
    group=geometry_group(records,route,planner)
    pst=group['pstmo'];raw=group['raw'];diag=diagnostics(route,planner)
    slug=f'{RID[route]}_{planner}';title=f'A{ROUTES.index(route)+1}.{PLANNERS.index(planner)+1} | {planner}'
    legacy=' | đối chiếu C'+str(21+PLANNERS.index(planner)) if route==ROUTES[0] else ''
    report.start(title+' | Hình học đường đi',RID[route]+' - '+TITLE[route]+legacy,bookmark='case_'+slug,level=2)
    p=asset(slug+'_01_geometry',geometry_figure,group,diag,slug+'_01_geometry')
    report.figure(p,height=375,caption='Đường kế hoạch, hai vùng phóng to và độ cong. Symlog giữ được dấu và phần gần 0; bảng dưới dùng giá trị gốc, không biến đổi thang đo.')
    report.table(['Phương án','Có đường','L (m)','κmax (m⁻¹)','Eκ (m⁻¹)','T smooth (ms)','Hở min (m)','Mẫu va chạm'],geometry_rows(group),[1.45,.6,.65,.8,.8,.85,.8,.8],size=6.8)
    report.para(f'So với Raw, PSTMO {change(pst.get("planned_path_length_m"),raw.get("planned_path_length_m"))} chiều dài; '
        f'{change(pst.get("planned_max_abs_curvature_1pm"),raw.get("planned_max_abs_curvature_1pm"))} đỉnh độ cong và '
        f'{change(pst.get("planned_curvature_energy_1pm"),raw.get("planned_curvature_energy_1pm"))} Eκ. '
        'Eκ biểu diễn mức uốn của đường, không phải điện năng robot.',size=8.8)
    candidates=[m for m in METHODS[1:-1] if group[m].get('planned_curvature_energy_1pm') is not None]
    if candidates:
        best=min(candidates,key=lambda m:group[m]['planned_curvature_energy_1pm'])
        report.para(f'Đối chứng có Eκ thấp nhất: {LABEL[best]} ({fmt(group[best]["planned_curvature_energy_1pm"])} m⁻¹). '
            f'PSTMO {change(pst.get("planned_curvature_energy_1pm"),group[best]["planned_curvature_energy_1pm"])} so với đối chứng này. '
            'Khoảng hở và khả năng bám được đọc cùng với độ mượt để tránh kết luận từ một chỉ số.',size=8.8)
    if diag:
        report.para(f'PSTMO: {diag.get("raw_input_points")} mẫu vào → {diag.get("conditioning_output_points")} neo; '
            f'{diag.get("g2_transitions")} chuyển tiếp G², {diag.get("pivots")} pivot, {diag.get("pass_through_corners")} góc giữ nguyên. '
            f'{diag.get("evaluations")} đánh giá hình dạng; {diag.get("dp_states")} trạng thái DP; {diag.get("compatible_edges")} cạnh tương thích.',size=8.5)
    report.para('Raw có T smooth = 0 vì không qua bộ làm mượt. Thời gian CPU chịu ảnh hưởng tải máy; không dùng chênh lệch ms giữa hai đợt để suy ra ưu thế thuật toán.',size=7.6,color='#64748b')
    if route==ROUTES[0]:
        report.para('R01: hình học và diagnostics lấy từ snapshot RViz của PSTMO.pdf, giữ đúng bảng gốc. Trang thực thi dùng lượt Gazebo lịch sử riêng; các smoother có thể cho sai khác nhỏ giữa hai lần thu.',size=7.3,color='#64748b')

    group={m:records[route,planner,m] for m in METHODS}
    pst=group['pstmo'];raw=group['raw']
    report.start(title+' | Thực thi trong Gazebo',RID[route]+' - '+TITLE[route])
    p=asset(slug+'_02_execution',execution_figure,group,slug+'_02_execution')
    report.figure(p,height=345,caption='Quỹ đạo đo trong mô phỏng (đỏ) và đường kế hoạch (xanh nét đứt) của đủ năm phương án. Đạt/không đạt ở đây là trạng thái thực thi; khác với việc có đường kế hoạch ở trang trước.')
    rows=[]
    for m in METHODS:
        d=group[m]
        rows.append([LABEL[m],'Đạt' if d.get('success') else 'Không đạt',fmt(d.get('execution_time_s')),
            fmt(d.get('traveled_distance_m')),fmt(d.get('final_position_error_m')),
            fmt(math.degrees(d['final_yaw_error_rad']),2) if d.get('final_yaw_error_rad') is not None else '-',
            fmt(d.get('tracking_rmse_m')),fmt(d.get('tracking_max_error_m'))])
    report.table(['Phương án','Kết quả','T chạy (s)','S thực (m)','e đích (m)','e yaw (°)','RMSE (m)','e max (m)'],rows,[1.4,.8,.85,.85,.8,.8,.8,.8],size=6.9)
    successful=[m for m in METHODS if group[m].get('success')]
    if successful:
        winner=min(successful,key=lambda m:group[m]['execution_time_s'])
        time_sentence=(f'PSTMO {change(pst.get("execution_time_s"),raw.get("execution_time_s"))} thời gian so với Raw; '
            if pst.get('success') and raw.get('success') else 'Không so sánh ưu thế thời gian PSTMO-Raw vì có lượt không đạt; ')
        report.para(f'Nhanh nhất trong các lượt đạt: {LABEL[winner]} ({fmt(group[winner]["execution_time_s"])} s). '
            +time_sentence+
            f'RMSE bám đường {change(pst.get("tracking_rmse_m"),raw.get("tracking_rmse_m"))}. '
            'Đây là kết quả một lượt cho mỗi tổ hợp, chưa có khoảng tin cậy theo lặp lại.',size=9)
    hashes={d.get('raw_path_sha256') for d in group.values() if d.get('raw_path_sha256')}
    report.para(f'Ghép cặp đường Raw: {len(hashes)} mã SHA-256 khác nhau cho nhóm năm phương án. '
        +('Đạt điều kiện cùng đầu vào.' if len(hashes)==1 else 'Có sai khác đầu vào; không kết luận so sánh ghép cặp cho nhóm này.'),size=8.5)
    report.para('Một lượt chỉ được đánh dấu đạt khi action điều khiển thành công, robot vào dung sai đích theo ground truth và dừng ổn định. T chạy gồm thời gian action cộng thời gian ổn định vật lý.',size=8.5)
    for m in METHODS:
        d=group[m]
        if not d.get('success'):
            reason=d.get('error') or d.get('controller_error_msg')
            if not reason:
                reason=(f'Action={d.get("controller_action_succeeded")}; dừng={d.get("physically_settled")}; '
                    f'đạt đích ground truth={d.get("ground_truth_goal_reached")}; '
                    f'e vị trí={fmt(d.get("final_position_error_m"),6)} m, ngưỡng={fmt(d.get("ground_truth_position_tolerance_m"),3)} m. '
                    'Giữ nguyên phân loại theo ngưỡng, kể cả khi chỉ vượt rất ít.')
            report.para(f'Lỗi {LABEL[m]}: '+html.escape(str(reason)),size=8,color='#b91c1c')
    report.para('Nguồn: '+html.escape(pst['_source']),size=7.3,color='#64748b')

    report.start(title+' | Động học và quyết định tại góc',RID[route]+' - '+TITLE[route])
    p=asset(slug+'_03_dynamics',dynamics_figure,group,slug+'_03_dynamics')
    report.figure(p,height=367,caption='Trace trạng thái thực và lệnh vận tốc. Hình hiển thị tối đa khoảng 900 mẫu/trace; RMSE trong bảng được lấy từ bộ đánh giá đầy đủ. Khoảng hở thực là hậu kiểm trên PGM tĩnh tại các mẫu hiển thị.')
    if diag:
        corners=diag.get('corner_search',[])
        rows=[]
        for c in corners:
            rows.append([str(c.get('index',0)+1),f'({fmt(c["x"],2)}; {fmt(c["y"],2)})',
                fmt(math.degrees(c.get('turn_angle',0)),2),fmt(c.get('selected_trim')),
                fmt(c.get('selected_control_fraction')),f'{c.get("safe_feasible","-")}/{c.get("evaluations","-")}',str(c.get('states','-'))])
        report.table(['Góc','Tọa độ (m)','θ (°)','d chọn (m)','α chọn','Qua cổng / đánh giá','Trạng thái'],rows,[.4,1.8,.7,.8,.7,1.3,.7],size=6.6)
        report.para(f'Chế độ: {diag.get("preprocessing_mode")} / {diag.get("search_mode")}. '
            f'Hậu kiểm bất biến: {"đạt" if diag.get("final_invariants_verified") else "không xác nhận"}. '
            f'Biên đoạn DP: {fmt(diag.get("segment_margin"))} m. '
            'd = 0 hoặc trường không có giá trị được đọc cùng số pivot/pass-through; không coi đó là một Bézier đã chọn.',size=7.8)
        if route==ROUTES[0]:
            report.para('R01: bảng góc kế thừa snapshot RViz gốc; đồ thị phía trên là trace của lượt thực thi lịch sử. Đây là hai lần thu dữ liệu được lưu riêng.',size=7.4,color='#64748b')
    else:
        report.para('Không có bản ghi diagnostics chi tiết cho lượt này; không suy đoán d, α hoặc trạng thái DP từ hình ảnh.',size=8)


def protocol_pages(report,records):
    report.start('2.1 | Thiết kế thí nghiệm',bookmark='protocol')
    report.para('Phạm vi: một bản đồ kho tĩnh có các lối giao cắt; năm tuyến độc lập; năm bộ lập kế hoạch; năm phương án làm mượt. Tổng cộng 125 lượt thực thi được trình bày: 25 lượt đã có của R01 và 100 lượt chạy mới của R02-R05.',size=10)
    report.table(['Thành phần','Thiết lập'],[
        ['Môi trường','warehouse_cross_aisles; kích thước sàn 12 × 8 m; lưới 0,05 m; gốc (-6; -4) m'],
        ['Kệ hàng','8 khối 0,6 × 2,2 × 1,2 m; tâm x = -3, -1, 1, 3 m; y = ±2,4 m'],
        ['Hành lang','Khe giữa các cột kệ: 1,4 m; hành lang ngang trung tâm: 2,6 m (tính từ SDF)'],
        ['Robot','Footprint chữ nhật 0,44 × 0,34 m; khoảng cách vệt bánh hình học 0,2548 m'],
        ['Planner','NavFnAStar; NavFnDijkstra; ThetaStar; Smac2D; SmacHybrid'],
        ['Phương án','Raw; Nav2 Simple; Savitzky-Golay; Constrained; PSTMO độc lập'],
        ['PSTMO','condition_only + hierarchical_alpha_two_trim; Bézier bậc năm, cổng footprint/động học/thời gian, DP'],
        ['Giới hạn cấu hình','v ≤ 0,30 m/s; |ω| ≤ 0,80 rad/s; tốc độ bánh ≤ 0,36 m/s; gia tốc ngang ≤ 0,18 m/s²'],
        ['Đầu ra','Mỗi lượt lưu JSON, log, đường kế hoạch, ground truth, lệnh vận tốc và sai số định vị'],
        ['Tách phiên','Bốn ma trận mới chạy đồng thời trong các ROS_DOMAIN_ID và GZ_PARTITION riêng; từng lượt khởi động stack mới'],
        ['Thời gian','T di chuyển dùng đồng hồ mô phỏng; T smooth/CPU có thể chịu ảnh hưởng tranh chấp tài nguyên'],
        ['Lặp lại','Một lượt/tổ hợp. Năm planner là năm đầu vào khác nhau, không phải năm lần lặp cùng điều kiện'],
    ],[1,3.1],size=8)
    report.para('Các tuyến được xác định trước khi chạy ma trận thực thi. Một đợt kiểm tra hình học sơ bộ 100 tổ hợp xác nhận khả năng lập đường; số liệu sơ bộ được lưu riêng, không cộng vào 125 lượt thực thi. Không thay world, costmap hoặc tham số thuật toán để tạo lợi thế cho một phương án.',size=8.6)
    report.para('R01 giữ nguyên điểm đầu/đích và số liệu lịch sử. R02-R05 đặt hướng đích dọc theo lối nhận hàng; hướng đầu hoặc được khai báo, hoặc được bộ giải map-aware xác định. Vì các tuyến có tư thế biên khác nhau, chỉ ghép cặp phương pháp trong cùng tuyến và cùng planner.',size=8.6)
    report.para('<b>Tiêu chí đạt:</b> action điều khiển thành công, robot dừng ổn định, sai số vị trí đích ≤ 0,10 m và sai số hướng ≤ 0,15 rad theo Gazebo. Có đường kế hoạch chưa đồng nghĩa thực thi đạt.',size=8.6)
    report.figure(asset('map_and_footprint_dimensions',dimensions_figure),height=170,caption='Kích thước map và hình bao kiểm tra; bề rộng hình học chưa trừ inflation hoặc footprint robot.')

    report.start('2.2 | Chỉ số đánh giá và quy ước đọc hình',bookmark='metrics',level=1)
    report.table(['Ký hiệu','Định nghĩa và đơn vị'],[
        ['L','Tổng khoảng cách Euclid giữa các điểm của đường kế hoạch (m).'],
        ['κmax','Giá trị lớn nhất của |κ| trên chuỗi ba điểm; κ = 2 cross / (a·b·c), đơn vị m⁻¹.'],
        ['Eκ','Tổng κ²·Δs với Δs bằng trung bình hai cạnh kề; đơn vị m⁻¹. Không đo điện năng.'],
        ['T smooth','Thời gian action SmoothPath báo cáo (ms); Raw = 0.'],
        ['T chạy','Thời gian mô phỏng từ nhận đường đến hoàn tất action và dừng ổn định (s).'],
        ['S thực','Quãng đường robot di chuyển theo ground truth Gazebo (m).'],
        ['RMSE, e max','Sai số khoảng cách từ quỹ đạo robot tới đường kế hoạch; dùng hàm benchmark có sẵn (m).'],
        ['e đích, e yaw','Sai số vị trí cuối và sai số góc cuối so với goal, đo từ Gazebo (m, độ).'],
        ['Hở footprint','Hậu kiểm hình chữ nhật 0,44 × 0,34 m trên distance transform PGM; hiệu chỉnh nửa đường chéo ô.'],
        ['Mẫu va chạm','Số mẫu kiểm tra footprint có khoảng hở bằng 0; không đồng nhất với cảm biến va chạm vật lý.'],
        ['% giảm','100 × (đối chứng − PSTMO) / đối chứng; giá trị âm nghĩa là PSTMO tăng chỉ số.'],
        ['G² và pivot','G² là nối liên tục hình học; pivot là quay tại chỗ. G² không tự chứng minh jerk theo thời gian liên tục.'],
    ],[.9,3.4],size=8)
    report.para('Cách đọc hình: đồ thị đường giữ nguyên hệ tọa độ map; xanh đứt là kế hoạch, đỏ là quỹ đạo thực mô phỏng. Các hình phóng to giữ nguyên dữ liệu, chỉ thay giới hạn trục. Độ cong dùng thang symlog khi cần nhìn đồng thời đỉnh nhọn và vùng gần 0; số liệu bảng luôn ở đơn vị gốc.',size=9)
    report.para('Độ cong rời rạc phụ thuộc cách lấy mẫu. Vì vậy báo cáo bổ sung bảng đối chiếu theo metric tịnh tiến resample 0,05 m ở phần tổng hợp. Không diễn giải riêng một đỉnh κ rất lớn của đường Raw thành độ cong vật lý mà robot thực sự đã chạy.',size=9)
    report.para('R01 có hai nguồn lịch sử như báo cáo gốc: snapshot RViz cung cấp hình học/bảng góc; các lượt Gazebo cung cấp đường thực thi và động học. Phần tổng hợp dùng metric đường thực thi cho cả 125 lượt. Vì thế một số giá trị R01, nhất là Constrained và T smooth, có thể khác nhẹ bảng snapshot gốc; không trộn hai lần đo để tạo một lượt mới.',size=8.5)
    report.para('Phạm vi kết luận: dữ liệu mô phỏng tĩnh trên năm tuyến chọn trước hỗ trợ so sánh trong map này. Chưa có thử nghiệm lặp nhiều seed, vật cản động, robot thật, tải thay đổi hoặc phép đo Wh. Khoảng xoay từ yaw robot ban đầu tới hướng cạnh đầu vẫn là giới hạn được nêu trong PSTMO.pdf.',size=9)
    report.figure(asset('sampling_sensitivity_example',sampling_figure,records),height=155,caption='Ví dụ ảnh hưởng của bước lấy mẫu tới độ cong rời rạc; đường liên tục và cách biểu diễn mẫu cần được phân biệt.')


def audit(records):
    checks=[];issues=[];manifest=[]
    for route in ROUTES:
        for planner in PLANNERS:
            group=[records[route,planner,m] for m in METHODS]
            hashes={d.get('raw_path_sha256') for d in group if d.get('raw_path_sha256')}
            paired=len(hashes)==1 and all(d.get('raw_path_sha256') for d in group)
            checks.append({'route':route,'planner':planner,'raw_pairing_valid':paired,'raw_hashes':sorted(hashes)})
            if not paired:issues.append(f'Raw hash mismatch: {route}/{planner}')
            for d in group:
                if d.get('success'):
                    for field in ['controller_action_succeeded','physically_settled','ground_truth_goal_reached']:
                        if not d.get(field):issues.append(f'Inconsistent success {route}/{planner}/{d["method"]}: {field}')
                p=ROOT/d['_source'];manifest.append({'path':d['_source'],'sha256':digest(p),'cohort':d['_cohort']})
    source_paths=[
        'src/vacuum_robot_gazebo/config/nav2_params.yaml',
        'src/vacuum_robot_gazebo/config/real_robot_profile.yaml',
        'src/vacuum_robot_gazebo/worlds/warehouse_cross_aisles.sdf',
        'src/vacuum_robot_gazebo/maps/warehouse_cross_aisles.pgm',
        'src/vacuum_robot_gazebo/maps/warehouse_cross_aisles.yaml',
        'src/adaptive_pivot_g2_nav2/src/adaptive_pivot_g2_smoother.cpp',
        'src/adaptive_pivot_g2/src/hierarchical_shape_search.cpp',
        'src/adaptive_pivot_g2/src/path_optimization.cpp',
        'src/adaptive_pivot_g2_benchmark/adaptive_pivot_g2_benchmark/execution_trial.py',
        'docs/warehouse_cross_aisles_5_routes/scenarios.yaml',
        'tools/run_cross_aisle_study.py','tools/build_cross_aisle_report.py',
    ]
    for path in source_paths:manifest.append({'path':path,'sha256':digest(ROOT/path),'cohort':'source'})
    for planner in PLANNERS:
        p=OLD/f'rviz_cases/warehouse_cross_aisles__{ROUTES[0]}__{planner}.json'
        manifest.append({'path':str(p.relative_to(ROOT)),'sha256':digest(p),'cohort':'historic_geometry_snapshot'})
    p=OLD/'benchmark_hinh_hoc_175_luot.csv'
    manifest.append({'path':str(p.relative_to(ROOT)),'sha256':digest(p),'cohort':'historic_geometry_snapshot'})
    for route in ROUTES[1:]:
        for planner in PLANNERS:
            p=BASE/'execution'/route/f'{route}_{planner.lower()}_pstmo.diagnostics.json'
            dg=diagnostics(route,planner)
            if not dg or not dg.get('final_invariants_verified'):
                issues.append(f'Missing or unverified diagnostics: {route}/{planner}')
            manifest.append({'path':str(p.relative_to(ROOT)),'sha256':digest(p),'cohort':'new_diagnostics'})
    for (route,planner,method),d in records.items():
        points=xy(d)
        if not len(points):continue
        kval=curvature(points);seg=np.linalg.norm(np.diff(points,axis=0),axis=1)
        values={'planned_path_length_m':float(seg.sum()),'planned_max_abs_curvature_1pm':float(abs(kval).max()),
            'planned_curvature_energy_1pm':float(np.sum(kval[1:-1]**2*(seg[:-1]+seg[1:])*.5))}
        for key,value in values.items():
            if not math.isclose(value,d[key],rel_tol=1e-8,abs_tol=1e-8):
                issues.append(f'Metric mismatch {route}/{planner}/{method}: {key}')
    payload={'generated_at':datetime.now(timezone.utc).isoformat(),'record_count':len(records),
        'git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'success_count':sum(d.get('success',False) for d in records.values()),'pairing':checks,'issues':issues,'files':manifest}
    (BASE/'audit_manifest.json').write_text(json.dumps(payload,indent=2,ensure_ascii=False))
    fields=['route','route_id','planner','method','cohort','success','execution_time_s','traveled_distance_m','tracking_rmse_m','tracking_max_error_m','final_position_error_m','final_yaw_error_rad','planned_path_length_m','planned_max_abs_curvature_1pm','planned_curvature_energy_1pm','planned_translation_max_abs_curvature_1pm','planned_translation_curvature_energy_1pm','planned_footprint_clearance_min_m','planned_footprint_collision_sample_count','collision_monitor_interventions','smoothing_time_s','raw_path_sha256','selected_path_sha256','source']
    with (BASE/'all_125_trials.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for (route,planner,method),d in records.items():
            row={key:d.get(key) for key in fields};row.update(route=route,route_id=RID[route],planner=planner,method=method,cohort=d['_cohort'],source=d['_source']);writer.writerow(row)
    return payload


def aggregate_pages(report,records,manifest):
    report.start('3.1 | Khả năng hoàn tất và thời gian di chuyển',bookmark='summary')
    report.para(f'{len(records)} lượt được lưu; {manifest["success_count"]} lượt đạt. '
        f'{sum(p["raw_pairing_valid"] for p in manifest["pairing"])}/25 nhóm có cùng SHA-256 Raw giữa năm phương án. '
        'R01 được giữ như mốc lịch sử; các kết luận chính về đợt mở rộng cần đọc riêng R02-R05.',size=10)
    p=asset('aggregate_time',aggregate_fig,records,'execution_time_s','aggregate_time','T chạy trung bình (s)')
    report.figure(p,height=260,caption='Mỗi ô là một tuyến × planner; đường bên phải là trung bình theo năm planner, chỉ tính các lượt đạt. Ô giảm thời gian chỉ có ý nghĩa hiệu suất khi hai lượt đều hoàn tất.')
    rows=[]
    for route in ROUTES:
        row=[RID[route]]
        for m in METHODS:
            ds=[records[route,p,m] for p in PLANNERS];ok=[d['execution_time_s'] for d in ds if d.get('success') and d.get('execution_time_s') is not None]
            row.append(f'{fmt(statistics.fmean(ok)) if ok else "-"} ({len(ok)}/5)')
        rows.append(row)
    report.table(['Tuyến','Raw','Simple','Savitzky-Golay','Constrained','PSTMO'],rows,[.55,1,1,1,1,1],size=7.5)
    report.para('Bảng trên: T chạy trung bình (s), kèm số lượt đạt/5. Các ô có 4/5 không cùng tập planner với ô 5/5; dùng bảng ghép cặp bên dưới để so sánh thời gian. Mỗi planner là một đầu vào, không phải một lần lặp.',size=8.5)
    rows=[]
    for m in METHODS[:-1]:
        pairs=[(records[r,p,m],records[r,p,'pstmo']) for r in ROUTES[1:] for p in PLANNERS]
        pairs=[(a,b) for a,b in pairs if a.get('success') and b.get('success')]
        diff=[b['execution_time_s']-a['execution_time_s'] for a,b in pairs]
        gains=[100*(a['execution_time_s']-b['execution_time_s'])/a['execution_time_s'] for a,b in pairs]
        rows.append([LABEL[m],str(len(diff)),str(sum(x<-.001 for x in diff)),str(sum(x>.001 for x in diff)),fmt(statistics.fmean(diff)) if diff else '-',fmt(statistics.fmean(gains),2)+'%' if gains else '-'])
    report.table(['Đối chứng R02-R05','Cặp đạt','PSTMO nhanh hơn','PSTMO chậm hơn','ΔT TB (s)','% giảm TB'],rows,[1.4,.7,1,1,1,1],size=7)
    report.para('Bảng ghép cặp: chỉ tính khi cả hai lượt đạt; ΔT = T PSTMO - T đối chứng. ΔT âm và % giảm dương đều nghĩa là PSTMO nhanh hơn. TB là trung bình các cặp; N/A trên hình là cặp không đủ điều kiện.',size=8.2)

    report.start('3.2 | Độ mượt và khoảng hở tới vật cản',bookmark='shape_summary',level=1)
    p=asset('aggregate_energy',aggregate_fig,records,'planned_curvature_energy_1pm','aggregate_energy','Eκ trung bình (m⁻¹)')
    report.figure(p,height=252,caption='So sánh mức uốn hình học trên đường kế hoạch gốc. Trung bình hình học dùng đủ 5 planner, kể cả đường của lượt thực thi không đạt. Eκ không phải điện năng.')
    p=asset('aggregate_clearance',aggregate_fig,records,'planned_footprint_clearance_min_m','aggregate_clearance','Hở footprint tối thiểu (m)')
    report.figure(p,height=252,caption='Trung bình hình học dùng đủ 5 planner. Giá trị giảm khoảng hở dương nghĩa là biên dự phòng nhỏ đi, không phải cải thiện. Đọc cùng kiểm tra va chạm và sai số bám.')
    report.para('Một đường ngắn và ít uốn có thể tiến gần đầu kệ hơn. Khả năng tránh va chạm ở mức mô phỏng được kiểm tra qua footprint kế hoạch và trace thực; không suy ra an toàn vận hành từ riêng việc Eκ giảm.',size=9)


def sampling_appendix(report,records):
    report.start('B | Đối chiếu độ cong theo cách lấy mẫu',bookmark='sampling')
    rows=[]
    for r in ROUTES:
        for p in PLANNERS:
            a=records[r,p,'raw'];b=records[r,p,'pstmo']
            rows.append([RID[r],p,fmt(a.get('planned_max_abs_curvature_1pm')),fmt(b.get('planned_max_abs_curvature_1pm')),
                fmt(a.get('planned_translation_max_abs_curvature_1pm')),fmt(b.get('planned_translation_max_abs_curvature_1pm')),
                fmt(b.get('planned_translation_curvature_energy_1pm'))])
    report.table(['Tuyến','Planner','κ Raw gốc','κ PSTMO gốc','κ Raw 0,05','κ PSTMO 0,05','Eκ PSTMO 0,05'],rows,[.55,1.4,.9,.9,.9,.9,1],size=6.9)
    report.para('Các cột κ có đơn vị m⁻¹; Eκ có đơn vị m⁻¹. Cột “0,05” dùng metric tịnh tiến resample của benchmark; pivot được tách thành marker. Khác biệt giữa hai cách đo cần được giữ khi so với bảng trong PSTMO.pdf, vì bảng gốc đo trên tọa độ ROS Path gốc.',size=8.5)


def route_intro(report,records,route):
    i=ROUTES.index(route)
    report.start(f'4.{i+1} | {RID[route]} - {TITLE[route]}',bookmark='route_chapter' if i==0 else route,level=0 if i==0 else 1)
    if i==0:report.mark(f'4.1 | {RID[route]} - {TITLE[route]}',route,1)
    report.para(DESCRIPTIONS[i],size=10)
    d=records[route,'ThetaStar','pstmo']
    start=d.get('start',SCENARIOS[i]['start']);goal=d.get('goal',SCENARIOS[i]['goal'])
    report.table(['Tư thế','x (m)','y (m)','Yaw (rad)','Yaw (°)'],[
        ['Xuất phát',fmt(start[0]),fmt(start[1]),fmt(start[2]),fmt(math.degrees(start[2]),2)],
        ['Đích',fmt(goal[0]),fmt(goal[1]),fmt(goal[2]),fmt(math.degrees(goal[2]),2)],
    ],[1,.8,.8,1,1],size=8.5)
    p=asset(f'{RID[route]}_route_overview',route_overview,route,records,f'{RID[route]}_route_overview')
    report.figure(p,height=345,caption='Raw và PSTMO của từng planner; ô cuối chồng năm đường PSTMO. Các khác biệt hành lang là do đầu vào planner, không phải dịch chuyển hình cho dễ nhìn.')
    ds=[records[route,p,m] for p in PLANNERS for m in METHODS]
    report.para(f'Tuyến này có {sum(d.get("success",False) for d in ds)}/25 lượt đạt. '
        f'Nguồn hướng đầu: {html.escape(str(d.get("initial_heading_source","scenario_explicit")))}. '
        +('Số liệu kế thừa tháng 8/2026; không được tính là lần chạy mới.' if i==0 else 'Số liệu chạy mới trong đợt tháng 10/2026.'),size=9)
    if i==2:
        report.para('Tuyến chữ U kiểm tra khả năng nối hai pha quay vào/ra giao cắt trong khi đích nằm cùng phía xuất phát. Đọc mức giảm tốc ở hai đầu của pha chuyển ngang cùng với khoảng hở đầu kệ; chiều dài ngắn hơn chưa đủ bảo đảm thời gian ngắn hơn.',size=9)
    report.para('Trang tiếp theo tổng hợp điều cần rút ra từ tuyến này. Hình phóng to, đường thực thi và bảng từng góc được tra tại phụ lục A'+str(i+1)+', trang '+report.ref('detail_'+route)+'.',size=9)


def conclusion_pages(report,records,manifest):
    report.start('5.1 | Thảo luận từ năm tình huống',bookmark='discussion')
    new=[d for (r,p,m),d in records.items() if r!=ROUTES[0]]
    report.para(f'Đợt mở rộng có 4 tuyến, 20 cặp tuyến-planner và 100 lượt thực thi, trong đó {sum(d.get("success",False) for d in new)} lượt đạt. '
        'R01 cung cấp mốc lịch sử cùng bản đồ. Bảng sau đặt năm tình huống cạnh nhau; mỗi dòng chỉ lấy các cặp Raw-PSTMO mà cả hai lượt đều đạt.',size=10)
    rows=[]
    for r in ROUTES:
        pairs=[(records[r,p,'raw'],records[r,p,'pstmo']) for p in PLANNERS]
        ok=[(a,b) for a,b in pairs if a.get('success') and b.get('success')]
        wins=sum(b['execution_time_s']<a['execution_time_s'] for a,b in ok)
        emean=statistics.fmean([100*(a['planned_curvature_energy_1pm']-b['planned_curvature_energy_1pm'])/a['planned_curvature_energy_1pm'] for a,b in ok if a.get('planned_curvature_energy_1pm',0)>0]) if ok else None
        rows.append([RID[r],TITLE[r],f'{wins}/{len(ok)}',fmt(emean,2)])
    report.table(['Tuyến','Tình huống','Nhanh hơn / cặp đạt','Giảm Eκ TB (%)'],rows,[.5,2.5,1,1],size=8.5)
    report.para('Eκ trong bảng này dùng cùng tập cặp đạt với thời gian. Các bảng hình học ở mục 3-4 dùng đủ năm đường mỗi tuyến, nên mẫu tính trung bình có thể khác ở R03 và R04.',size=8.7)
    report.para('<b>1. Hiệu quả hình học phụ thuộc đầu vào.</b> Các đường Raw của NavFn và ThetaStar có nhiều mẫu đổi hướng nhỏ; hai planner Smac tạo đầu vào có cấu trúc khác. Mức giảm Eκ lớn cần được đọc cùng giá trị ban đầu và bảng lấy mẫu ở phụ lục B. Tổng mức uốn giảm vẫn có thể đi kèm κmax tăng, như một số trường hợp Smac trong mục 4.',size=10,space=12)
    n,w,g=paired_comparison(records,ROUTES[1:],'simple')
    report.para(f'<b>2. Đường ít uốn hơn chưa quyết định phương án nhanh nhất.</b> Khi so với Simple trên đợt bổ sung, PSTMO nhanh hơn ở {w}/{n} cặp cùng đạt, với mức giảm trung bình {fmt(g,2)}%. Các bảng từng tuyến cho thấy phương án nhanh nhất thay đổi theo planner. Vì vậy, kết quả so với Raw cần được đặt cạnh cả ba bộ làm mượt đối chứng.',size=10,space=12)
    reduced=sum(records[r,p,'pstmo']['planned_footprint_clearance_min_m']<records[r,p,'raw']['planned_footprint_clearance_min_m'] for r in ROUTES for p in PLANNERS)
    report.para(f'<b>3. Khoảng hở là một đánh đổi cần giữ trong kết luận.</b> PSTMO có khoảng hở kế hoạch nhỏ hơn Raw ở {reduced}/25 đầu vào. Để đánh giá chất lượng một tuyến, cần đọc đồng thời hình phóng to đầu kệ, sai số bám và khoảng hở trên quỹ đạo thực thi trong phụ lục A.',size=10,space=12)
    report.para('Ba lớp hệ thống giải thích cách đọc các kết quả này: planner quyết định đường đầu vào; PSTMO chọn hình dạng nối góc; controller quyết định vận tốc và sai số khi bám đường. Bộ dữ liệu quan sát được sự khác nhau của đầu ra tổng thể, chưa tách riêng đóng góp nhân quả của từng lớp.',size=9.5)
    report.para('Hai lượt không đạt được giữ trong mẫu số tỷ lệ thành công. Trang tiếp theo giải thích đúng ngưỡng gây không đạt và phạm vi kết luận có thể sử dụng khi viết bài báo.',size=9.5,color=TEAL)


def source_pages(report,records,manifest):
    report.start('C.1 | Nguồn dữ liệu và cách tái lập',bookmark='sources')
    report.para('Tài liệu tham chiếu chính: docs/PSTMO.pdf, mục 7.5, trang 81-91. Báo cáo hiện tại giữ R01 và bổ sung R02-R05 trên cùng world và cấu hình nguồn. Toàn bộ dữ liệu mới nằm trong docs/warehouse_cross_aisles_5_routes/.',size=9.5)
    report.table(['Tệp / thư mục','Vai trò'],[
        ['scenarios.yaml','Danh sách năm tuyến đã chọn; tọa độ và hướng đích.'],
        ['execution/<scenario>/*.json','100 kết quả mới; kế hoạch, trace và chỉ số thực thi.'],
        ['execution/<scenario>/*.log','Log launch, planner, smoother, controller, lỗi và retry hạ tầng nếu có.'],
        ['*.diagnostics.json','Diagnostics PSTMO thu thụ động từ topic trong đúng phiên thực thi.'],
        ['geometry_preflight.csv / .json','100 tổ hợp sơ bộ; không cộng vào ma trận thực thi chính.'],
        ['all_125_trials.csv','Bảng gộp 25 lượt cũ + 100 lượt mới, kèm đường dẫn nguồn.'],
        ['audit_manifest.json','Kiểm tra ghép cặp Raw và SHA-256 của dữ liệu, mã nguồn.'],
        ['figures/*.png / *.svg','Hình độ phân giải cao và bản vector để đưa vào bài báo.'],
        ['page_index.json / figure_index.json / figure_page_index.json','Chỉ mục trang, danh sách ảnh và ánh xạ từng hình tới trang PDF.'],
        ['tools/run_cross_aisle_study.py','Chạy ma trận gốc, thêm recorder thụ động; không sửa thuật toán.'],
        ['tools/build_cross_aisle_report.py','Tính hình từ JSON và xuất PDF tiếng Việt.'],
    ],[1.8,2.8],size=8)
    report.para('Tái lập sau khi source /opt/ros/jazzy/setup.bash và install/setup.bash: chạy tools/run_cross_aisle_study.py với tên scenario và base-domain-id riêng. Trước khi chạy lại nên chọn thư mục kết quả mới để bảo toàn đợt gốc; script ma trận không tự coi một lượt lặp là dữ liệu đã công bố.',size=8.8)
    report.para('PDF và hình được tạo trực tiếp từ dữ liệu đo trong Gazebo, không dùng ảnh sinh AI. Hình 3D là dựng hình khoa học theo kích thước SDF; không phải ảnh chụp GUI Gazebo. Nguồn R01 tiếp tục trỏ đến các JSON lịch sử đã dùng trong PSTMO.pdf.',size=8.8)
    report.para('Git HEAD nguồn: '+manifest['git_head'],size=7.8)
    report.para('Kết quả kiểm tra: '+ ('không phát hiện bất nhất cấu trúc trong các tiêu chí đã kiểm tra.' if not manifest['issues'] else html.escape('; '.join(manifest['issues']))),size=8.8)

    report.start('C.2 | SHA-256 các thành phần thí nghiệm',bookmark='hashes',level=1)
    rows=[]
    for f in manifest['files']:
        if f['cohort']=='source':rows.append([f['path'],f['sha256']])
    report.table(['Tệp','SHA-256'],rows,[1.45,1.8],size=6.4)
    report.para('Manifest JSON lưu thêm hash đầy đủ cho cả 125 tệp kết quả. Hash xác nhận nội dung tệp dùng trong báo cáo; không thay thế kiểm chứng khoa học hoặc kiểm thử trên phần cứng.',size=9)


def paired_comparison(records,routes,method='raw'):
    pairs=[(records[r,p,method],records[r,p,'pstmo']) for r in routes for p in PLANNERS]
    ok=[(a,b) for a,b in pairs if a.get('success') and b.get('success')]
    gains=[100*(a['execution_time_s']-b['execution_time_s'])/a['execution_time_s'] for a,b in ok]
    return len(ok),sum(b['execution_time_s']<a['execution_time_s']-.001 for a,b in ok),statistics.fmean(gains) if gains else None


def reading_pages(report,records):
    report.start('Tóm tắt nghiên cứu',bookmark='abstract')
    report.para('Câu hỏi của báo cáo là: trên một bản đồ kho có lối giao cắt, PSTMO thay đổi hình dạng đường đi như thế nào, robot bám đường đó ra sao, và thay đổi ấy có giúp giảm thời gian di chuyển hay không?',size=11,space=14)
    report.para('Nghiên cứu gồm năm tuyến: một tuyến gốc R01 và bốn tuyến bổ sung R02-R05. Mỗi tuyến được khảo sát với năm bộ lập kế hoạch và năm phương án xử lý đường. Bộ dữ liệu có 125 lượt thực thi trong Gazebo, gồm 25 lượt lịch sử và 100 lượt bổ sung.',size=10)
    n,w,g=paired_comparison(records,ROUTES[1:])
    ns,ws,gs=paired_comparison(records,ROUTES[1:],'simple')
    report.table(['Điểm cần nắm','Kết quả và cách hiểu'],[
        ['Khả năng hoàn tất','123/125 lượt đạt; riêng đợt bổ sung đạt 98/100. Hai lượt còn lại vượt ngưỡng sai số vị trí đích.'],
        ['Thời gian so với Raw',f'Trong đợt bổ sung, PSTMO nhanh hơn ở {w}/{n} cặp cùng đạt. Trung bình mức giảm theo cặp là {fmt(g,2)}%. Raw là đường chưa làm mượt.'],
        ['Thời gian so với Simple',f'PSTMO nhanh hơn ở {ws}/{ns} cặp cùng đạt; mức giảm trung bình là {fmt(gs,2)}%. Kết quả phụ thuộc đường đầu vào và đối chứng.'],
        ['Hình dạng và khoảng hở',f'PSTMO giảm Eκ so với Raw ở {sum(records[r,p,"pstmo"]["planned_curvature_energy_1pm"]<records[r,p,"raw"]["planned_curvature_energy_1pm"] for r in ROUTES for p in PLANNERS)}/25 đầu vào trong bộ thực thi. Mức uốn, đỉnh độ cong và khoảng hở là ba chỉ số riêng, cần đọc cùng nhau.'],
        ['Phạm vi bằng chứng','Một lần chạy cho mỗi tổ hợp trên map tĩnh. Các số liệu mô tả bộ thí nghiệm hiện tại; chưa đo độ lặp lại hoặc hiệu quả trên robot thật.'],
    ],[1,3],size=9.1)
    report.para('Mạch đọc được tổ chức theo câu hỏi: hiểu hệ thống và chỉ số (mục 1-2), xem kết quả chung (mục 3), xem điều rút ra từ từng tuyến (mục 4), rồi đối chiếu giới hạn kết luận (mục 5). Phụ lục A giữ toàn bộ hình, bảng và bản ghi xử lý góc theo từng bộ lập kế hoạch.',size=10,space=14)
    report.para('Phiên bản biên tập ngày 05/10/2026 giữ bộ dữ liệu và toàn bộ 92 hình của bản trước. Các trang giải thích và bảng tổng hợp theo tuyến được bổ sung để người đọc có thể theo dõi lập luận trước khi tra số liệu chi tiết.',size=9,color='#475569')


def contents_page(report):
    report.start('Mục lục và lộ trình đọc',bookmark='contents')
    entries=[
        ('1. Làm quen với hệ thống và thuật ngữ','reader'),
        ('2. Thiết kế thí nghiệm, chỉ số và bản đồ','protocol'),
        ('3. Kết quả tổng hợp: thời gian, độ mượt, khoảng hở','summary'),
        *[(f'4.{i+1}. {RID[r]} - {TITLE[r]}',r) for i,r in enumerate(ROUTES)],
        ('5. Thảo luận và giới hạn kết luận','discussion'),
        ('A. Hồ sơ chi tiết của 25 nhóm tuyến - planner','appendix'),
        ('B. Đối chiếu độ cong theo cách lấy mẫu','sampling'),
        ('C. Nguồn dữ liệu, tái lập và mã kiểm tra','sources'),
    ]
    report.table(['Nội dung (bấm để chuyển đến mục)','Trang'],[[label,report.ref(key)] for label,key in entries],[4,.45],size=9,links=[key for _,key in entries])
    report.para('<b>Đọc lần đầu.</b> Đi theo mục 1-5. Mỗi tuyến trong mục 4 có hai trang: tình huống di chuyển và kết quả cần rút ra.',size=10,space=10)
    report.para('<b>Đọc để kiểm tra số liệu.</b> Dùng bảng tra phụ lục A ở trang '+report.ref('appendix')+'. Mỗi nhóm có ba trang liên tiếp: hình học đường đi; robot thực thi trong Gazebo; động học và quyết định tại góc.',size=10,space=10)
    report.para('<b>Đọc để sử dụng hình.</b> Mọi hình có số thứ tự và chú thích. Tệp figure_page_index.json nối số hình, trang PDF và tên ảnh PNG/SVG; số liệu nguồn được dẫn trong từng nhóm và phụ lục C.',size=10)
    report.para('Các dấu trang PDF được chia theo mục, tuyến và bộ lập kế hoạch. Trong các bảng, dấu "-" là không áp dụng hoặc thiếu điều kiện so sánh; N/A trên bản đồ màu có cùng ý nghĩa.',size=9,color='#475569')


def orientation_pages(report):
    report.start('1.1 | Thuật ngữ cần biết trước khi đọc',bookmark='reader')
    report.para('Một tuyến là một cặp tư thế xuất phát - đích trên bản đồ. Với cùng tuyến, mỗi bộ lập kế hoạch có thể tạo một đường khác nhau; mỗi đường đó lại được xử lý bằng năm phương án để so sánh.',size=10)
    report.table(['Thuật ngữ trong hình/bảng','Hiểu theo vai trò trong nghiên cứu'],[
        ['Planner / bộ lập kế hoạch','Tạo đường ban đầu qua vùng trống của bản đồ. Năm tên trong báo cáo: NavFnAStar, NavFnDijkstra, ThetaStar, Smac2D, SmacHybrid.'],
        ['Raw','Giữ nguyên đường do planner tạo ra, làm mốc so sánh.'],
        ['Smoother / bộ làm mượt','Xử lý hình dạng đường trước khi giao cho bộ điều khiển. Bốn phương án: Simple, Savitzky-Golay, Constrained và PSTMO.'],
        ['Controller / bộ điều khiển','Nhận đường kế hoạch và sinh lệnh vận tốc để robot bám đường.'],
        ['Đường kế hoạch / quỹ đạo thực thi','Đường kế hoạch là dãy điểm mục tiêu. Quỹ đạo thực thi là các vị trí robot đã đi qua trong mô phỏng.'],
        ['Ground truth','Vị trí, hướng và vận tốc do Gazebo cung cấp, dùng làm giá trị tham chiếu cho đánh giá thực thi.'],
        ['Footprint / hình bao robot','Hình chữ nhật biểu diễn vùng robot chiếm chỗ trên mặt phẳng; dùng khi kiểm tra khoảng hở tới vật cản.'],
        ['Yaw, v, ω','Yaw là hướng robot; v là vận tốc tịnh tiến; ω là vận tốc góc. Đơn vị đi kèm từng bảng hoặc trục hình.'],
        ['RMSE / sai số bám điển hình','Căn trung bình bình phương khoảng cách từ robot tới đường kế hoạch. Nhỏ hơn nghĩa là bám sát hơn theo chỉ số này.'],
        ['κmax và Eκ','κmax là đỉnh độ cong rời rạc; Eκ cộng mức uốn dọc đường. Eκ có đơn vị m⁻¹ và không phải điện năng.'],
        ['Diagnostics / bản ghi xử lý','Thông tin nội bộ về các góc, ứng viên và tham số mà PSTMO đã chọn.'],
        ['Snapshot RViz','Bản lưu dữ liệu tại thời điểm quan sát bằng RViz. R01 có bộ snapshot hình học riêng và bộ thực thi Gazebo riêng.'],
        ['SHA-256 / mã kiểm tra','Mã dùng để kiểm tra nội dung tệp và xác nhận năm phương án nhận đúng cùng một đường Raw.'],
    ],[1.1,3],size=8.8)
    report.para('<b>Một lượt chạy:</b> một tuyến + một planner + một phương án. Vì vậy 5 × 5 × 5 = 125 lượt; năm planner không phải năm lần chạy lặp cùng điều kiện.',size=10)

    report.start('1.2 | Từ bản đồ đến robot chuyển động',bookmark='pipeline',level=1)
    report.para('Kết quả được giải thích theo chuỗi xử lý dưới đây. Mỗi bước trả lời một câu hỏi riêng; điều này giúp phân biệt đường đẹp về hình học với chuyển động tốt khi thực thi.',size=10)
    report.table(['Bước','Đầu ra','Câu hỏi được kiểm tra'],[
        ['1. Bản đồ + tư thế đầu/đích','Tình huống di chuyển R01-R05','Robot cần qua hành lang và giao cắt nào?'],
        ['2. Planner','Đường Raw','Đường ban đầu có cấu trúc góc và cách lấy mẫu ra sao?'],
        ['3. Raw hoặc một smoother','Đường giao cho điều khiển','Chiều dài, độ cong và khoảng hở thay đổi thế nào?'],
        ['4. Controller + Gazebo','Vị trí, vận tốc, thời gian thực thi','Robot có bám được đường và hoàn tất nhiệm vụ không?'],
        ['5. Bộ đánh giá','Bảng số liệu và phân loại đạt','Các phương án khác nhau trên cùng đầu vào như thế nào?'],
    ],[1.2,1.35,2],size=9)
    report.para('<b>PSTMO xử lý góc như thế nào?</b> Trước hết, bước conditioning rút gọn dãy mẫu thành các điểm neo. Tại một góc, thuật toán xét đoạn nối cong Bézier bậc năm; d là khoảng cắt trên hai cạnh kề, còn α điều chỉnh vị trí các điểm điều khiển thông qua q = α·d.',size=10,space=10)
    report.para('Các ứng viên phải đáp ứng điều kiện hình bao robot, động học và tiêu chí thời gian. Quy hoạch động (DP) chọn tổ hợp các trạng thái góc có thể nối liên tiếp mà không dùng chồng phần cạnh còn lại. Các bảng phụ lục ghi số ứng viên, số trạng thái và tham số được chọn.',size=10,space=10)
    report.table(['Ký hiệu xử lý góc','Ý nghĩa khi tra phụ lục'],[
        ['Neo','Điểm đại diện của đường sau bước conditioning.'],
        ['Chuyển tiếp G²','Đoạn nối bảo đảm liên tục hình học đến độ cong; tại chỗ nối với đoạn thẳng, độ cong bằng 0.'],
        ['Pivot / pass-through','Lần lượt là quay tại chỗ / giữ góc theo cách xử lý mà bản ghi báo cáo.'],
        ['θ, d (m), α, DP','θ là góc đổi hướng; d là khoảng cắt; α là hệ số hình dạng; DP chọn tổ hợp trạng thái. Bảng d/α mô tả đường đã chọn.'],
    ],[1,3.2],size=9)
    report.para('G² mô tả hình học của đoạn nối. Độ êm theo thời gian còn phụ thuộc vận tốc và bộ điều khiển; muốn đánh giá phải xem thêm v(t), ω(t) và sai số bám ở trang thứ ba của mỗi nhóm phụ lục.',size=9.5)


def route_summary(report,records,route):
    i=ROUTES.index(route);group={p:{m:records[route,p,m] for m in METHODS} for p in PLANNERS}
    report.start(f'4.{i+1} | {RID[route]} - Kết quả và cách diễn giải',bookmark='route_result_'+route,level=2)
    n,w,g=paired_comparison(records,[route])
    report.para(f'<b>Kết quả thời gian.</b> PSTMO nhanh hơn Raw ở {w}/{n} cặp mà cả hai lượt đều đạt. Mức giảm thời gian trung bình theo các cặp này là {fmt(g,2)}%. Bảng sau cho biết kết quả thuộc về đầu vào nào.',size=10)
    rows=[]
    for p in PLANNERS:
        a=group[p]['raw'];b=group[p]['pstmo'];ok=a.get('success') and b.get('success')
        gain=100*(a['execution_time_s']-b['execution_time_s'])/a['execution_time_s'] if ok else None
        eg=100*(a['planned_curvature_energy_1pm']-b['planned_curvature_energy_1pm'])/a['planned_curvature_energy_1pm']
        delta=1000*(b['planned_footprint_clearance_min_m']-a['planned_footprint_clearance_min_m'])
        rows.append([p,fmt(a['execution_time_s'])+(' *' if not a.get('success') else ''),fmt(b['execution_time_s'])+(' *' if not b.get('success') else ''),fmt(gain,2),fmt(eg,2),fmt(delta,1)])
    report.table(['Planner','T Raw (s)','T PSTMO (s)','Giảm T (%)','Giảm Eκ (%)','Δ hở (mm)'],rows,[1.4,1,1,.9,1,1],size=8)
    report.para('Giảm T/Eκ dương là chỉ số giảm; Δ hở = hở PSTMO - hở Raw, nên Δ hở dương là khoảng hở tăng. Dấu * là lượt không đạt: thời gian vẫn được lưu nhưng không tính ưu thế. Eκ và khoảng hở dùng đường kế hoạch của đủ năm planner.',size=8.7)
    report.para('<b>Đặt PSTMO cạnh đủ các đối chứng.</b> Bảng dưới giữ năm phương án. Cột nhanh nhất đếm số nhóm planner mà phương án có thời gian nhỏ nhất trong các lượt đạt; mỗi nhóm chỉ là một quan sát.',size=9.5)
    rows=[]
    wins={m:0 for m in METHODS}
    for p in PLANNERS:
        eligible=[m for m in METHODS if group[p][m].get('success')]
        if eligible:wins[min(eligible,key=lambda m:group[p][m]['execution_time_s'])]+=1
    for m in METHODS:
        ds=[group[p][m] for p in PLANNERS];ok=[d for d in ds if d.get('success')]
        rows.append([LABEL[m],f'{len(ok)}/5',fmt(statistics.fmean(d['planned_curvature_energy_1pm'] for d in ds)),fmt(statistics.fmean(d['tracking_rmse_m'] for d in ok)) if ok else '-',str(wins[m])+'/5'])
    report.table(['Phương án','Lượt đạt','Eκ TB (m⁻¹)','RMSE TB (m)','Nhanh nhất'],rows,[1.25,.7,1.1,1.1,.8],size=8.3)
    report.para('Eκ TB dùng đủ năm đường kế hoạch. RMSE TB chỉ dùng các lượt đạt; khi mẫu số khác nhau, tra bảng theo từng planner để đối chiếu công bằng.',size=8.5)
    higher=[p for p in PLANNERS if group[p]['pstmo']['planned_max_abs_curvature_1pm']>group[p]['raw']['planned_max_abs_curvature_1pm']]
    less_clear=[p for p in PLANNERS if group[p]['pstmo']['planned_footprint_clearance_min_m']<group[p]['raw']['planned_footprint_clearance_min_m']]
    report.para(f'<b>Đánh đổi cần đọc cùng kết quả.</b> Khoảng hở kế hoạch giảm ở {len(less_clear)}/5 đầu vào. '
        +(f'Đỉnh độ cong κmax tăng ở {", ".join(higher)}; Eκ thấp hơn vẫn có thể đi kèm một đỉnh cong lớn hơn.' if higher else 'Đỉnh độ cong κmax của PSTMO không tăng so với Raw ở cả năm đầu vào.'),size=9.5)
    failed=[(p,m,d) for p in PLANNERS for m,d in group[p].items() if not d.get('success')]
    for p,m,d in failed:
        report.para(f'<b>Lượt không đạt:</b> {p} / {LABEL[m]}, sai số đích {fmt(d["final_position_error_m"],6)} m so với ngưỡng {fmt(d["ground_truth_position_tolerance_m"],6)} m. Xem phân tích chung ở mục 5.2.',size=9.1,color='#b91c1c')
    if i==0:report.para('Nguồn R01 trên trang này là bộ thực thi lịch sử. Bảng hình học trong phụ lục A1 giữ snapshot của tài liệu gốc; hai lần thu được ghi rõ để người đọc đối chiếu đúng.',size=8.8)
    report.para('Tra minh chứng: phụ lục A'+str(i+1)+' bắt đầu trang '+report.ref('detail_'+route)+'. Thứ tự planner và số trang từng nhóm có trong bảng tra trang '+report.ref('appendix')+'.',size=9,color=TEAL)


def limits_page(report,records):
    report.start('5.2 | Hai lượt không đạt và giới hạn kết luận',bookmark='limits',level=1)
    report.para('Tiêu chí hoàn tất yêu cầu đồng thời: action điều khiển báo thành công, robot dừng ổn định, sai số vị trí đích không vượt 0,10 m và sai số hướng không vượt 0,15 rad. Kiểm tra đích dùng trạng thái tham chiếu của Gazebo.',size=10)
    rows=[]
    for (r,p,m),d in records.items():
        if d.get('success'):continue
        rows.append([RID[r],p,LABEL[m],fmt(d['final_position_error_m'],6),fmt(d['final_yaw_error_rad'],6),fmt(1000*(d['final_position_error_m']-d['ground_truth_position_tolerance_m']),3)])
    report.table(['Tuyến','Planner','Phương án','e vị trí (m)','e hướng (rad)','Vượt vị trí (mm)'],rows,[.6,1.2,.8,1,1,1],size=8)
    report.para('Cả hai lượt đã hoàn tất action và dừng, nhưng vượt ngưỡng vị trí. Vì vậy, phân loại vẫn là không đạt. Số lẻ được giữ đến sáu chữ số ở đây để tránh hiểu nhầm khi bảng chi tiết làm tròn 0,100234 m thành 0,100 m.',size=10)
    report.para('Đây là mô tả điều kiện gây không đạt theo bộ đánh giá. Các bản ghi hiện tại chưa đủ để quy nguyên nhân sâu hơn cho riêng bộ làm mượt, bộ điều khiển hay sai số định vị.',size=9.5,space=12)
    report.table(['Kết luận có thể rút ra','Giới hạn đi kèm'],[
        ['So sánh hình dạng đường trên cùng đầu vào','25/25 nhóm có cùng mã Raw; độ cong vẫn phụ thuộc bước lấy mẫu, xem phụ lục B.'],
        ['So sánh thời gian trên các cặp cùng đạt','Mỗi tổ hợp chỉ có một lượt. Chưa có phương sai qua lặp lại, khoảng tin cậy hay kiểm định ý nghĩa thống kê.'],
        ['Mô tả khoảng hở và sai số trong mô phỏng','PGM là bản đồ tĩnh; khoảng hở hậu kiểm không thay cho đo tiếp xúc vật lý hoặc thử nghiệm robot thật.'],
        ['So sánh trong một bản đồ với nhiều tuyến','Chưa khảo sát vật cản động, nhiễu định vị có kiểm soát, thay đổi tải hoặc nhiều bản đồ.'],
        ['Ghi nhận T smooth và Eκ','T smooth chịu ảnh hưởng tải máy; Eκ không phải điện năng. Chưa có đo Wh.'],
        ['Mô tả nối G² tại các góc được xử lý','Không suy ra jerk theo thời gian liên tục; pha xoay ban đầu tới hướng cạnh đầu vẫn là giới hạn nêu trong PSTMO.pdf.'],
    ],[1.5,2.6],size=9)
    report.para('Để mở rộng mức độ kết luận cho bài báo, các bước phù hợp là lặp lại cùng điều kiện, báo cáo phân bố sai số/thời gian, rồi khảo sát nhiễu và tải có kiểm soát. Những phép thử này chưa thuộc bộ dữ liệu hiện tại.',size=9.5)


def appendix_guide(report):
    report.start('A | Tra cứu hồ sơ chi tiết',bookmark='appendix')
    report.para('Phụ lục này giữ đủ 25 nhóm tuyến - planner. Mỗi tuyến bắt đầu bằng một trang minh họa cơ chế PSTMO và bảng tham số chung, tiếp theo là năm nhóm phân tích. Dùng bảng sau để nhảy tới nhóm cần đối chiếu.',size=10)
    rows=[]
    for r in ROUTES:
        pages=[report.ref('case_'+RID[r]+'_'+p) for p in PLANNERS]
        rows.append([RID[r],report.ref('detail_'+r),*[p+'-'+str(int(p)+2) if p.isdigit() else '...' for p in pages]])
    report.table(['Tuyến','Cơ chế','NavFnAStar','NavFnDijkstra','ThetaStar','Smac2D','SmacHybrid'],rows,[.5,.55,1,1.2,1,1,1],size=7.6,links=['detail_'+r for r in ROUTES])
    report.table(['Trang trong nhóm','Nội dung được giữ','Cách đọc'],[
        ['1. Hình học','Đường chồng, hai vùng phóng to, đồ thị độ cong, Eκ; bảng L, κmax, Eκ, T smooth, khoảng hở và mẫu va chạm.','Có đường chỉ xác nhận đường kế hoạch tồn tại. Đọc đỉnh cong và khoảng hở cùng mức uốn tổng.'],
        ['2. Thực thi','Đủ năm đường kế hoạch/quỹ đạo Gazebo; trạng thái, T chạy, S thực, sai số đích/hướng và sai số bám.','Đạt là thỏa tiêu chí thực thi. Đối chiếu Raw và PSTMO chỉ khi cùng đầu vào và cả hai lượt đạt.'],
        ['3. Động học và góc','Sáu đồ thị theo thời gian; bảng tọa độ góc, θ, d, α, ứng viên và trạng thái.','Xem biến thiên tốc độ tại góc, sai số bám và khoảng hở; bảng góc giải thích lựa chọn hình dạng.'],
    ],[.9,2,1.7],size=9)
    report.para('<b>Hai nguồn của R01.</b> Hình học và bản ghi góc dùng snapshot RViz của PSTMO.pdf; đường thực thi và đồ thị thời gian dùng các lượt Gazebo lịch sử. Cả hai nguồn được giữ, có chú thích trong từng nhóm. Tổng hợp ở phần chính dùng bộ thực thi cho tất cả các tuyến.',size=10)
    report.para('<b>Quy ước hình.</b> Hình học dùng màu theo phương án; trong hình thực thi, xanh đứt là đường kế hoạch và đỏ là quỹ đạo đo trong Gazebo. Thang symlog hiển thị được giá trị dương, âm và gần 0; thang log dùng khi các giá trị chênh nhau lớn.',size=10)
    report.para('Đồ thị động học hiển thị tối đa khoảng 900 mẫu mỗi chuỗi để dễ quan sát. Các chỉ số bảng được lấy từ bộ đánh giá đầy đủ. Khoảng hở trên đồ thị là hậu kiểm PGM tại các mẫu hiển thị.',size=9.5)


def compose_report(report,records,manifest):
    report.start('KHO CÓ LỐI GIAO CẮT<br/>Nghiên cứu năm quỹ đạo',bookmark='cover')
    report.para('PSTMO • ROS 2 Navigation2 • Robot vi sai • Gazebo',size=12,color=TEAL,space=14)
    report.para('BÁO CÁO THỰC NGHIỆM - BẢN BIÊN TẬP LẠI',size=10,bold=True)
    p=asset('map_3d_routes',map3d,records)
    report.figure(p,height=315,caption='Dựng hình theo kích thước world SDF; năm đường PSTMO với đầu vào ThetaStar được đặt trên đúng tọa độ map. Đây là hình khoa học dựng từ dữ liệu.')
    report.table(['5 tuyến','5 planner','5 phương án','125 lượt'],[['1 tuyến gốc + 4 tuyến mới','5 bộ lập kế hoạch','Raw + 4 bộ làm mượt','25 lịch sử + 100 bổ sung']],size=9)
    report.para('Từ câu hỏi nghiên cứu đến minh chứng: cách tạo đường, kết quả thực thi, các đánh đổi về độ cong và khoảng hở, cùng hồ sơ chi tiết cho từng tuyến.',size=11)
    report.para('Dữ liệu bổ sung: 03/10/2026 · Dữ liệu gốc: 02-03/08/2026<br/>Biên tập lại: 05/10/2026 · Workspace: agv_nav2_research_ws_bao',size=8.5,color='#64748b')
    reading_pages(report,records)
    contents_page(report)
    orientation_pages(report)
    protocol_pages(report,records)
    report.start('2.3 | Năm tuyến trên cùng bản đồ',bookmark='routes',level=1)
    p=asset('overview_5_routes',overview_figure,records)
    report.figure(p,height=370,caption='Tổng quan năm tuyến; hình minh họa dùng PSTMO/ThetaStar. Mỗi tuyến vẫn được kiểm tra với đủ năm planner và năm phương án.')
    for r,desc in zip(ROUTES,DESCRIPTIONS):report.para(f'<b>{RID[r]}.</b> {desc}',size=8.5)
    aggregate_pages(report,records,manifest)
    for r in ROUTES:
        route_intro(report,records,r)
        route_summary(report,records,r)
    conclusion_pages(report,records,manifest)
    limits_page(report,records)
    appendix_guide(report)
    for r in ROUTES:
        construction_page(report,records,r)
        for p in PLANNERS:
            if not report.dry:print('Typesetting',RID[r],p,flush=True)
            case_pages(report,records,r,p)
    sampling_appendix(report,records)
    source_pages(report,records,manifest)


def build(preview=False):
    FIG.mkdir(parents=True,exist_ok=True)
    records=load_records(partial=preview)
    if preview:
        report=Report(BASE/'preview.pdf')
        report.start('A | Xem thử bố cục',bookmark='preview')
        ready=[(r,p) for r in ROUTES for p in PLANNERS if all((r,p,m) in records for m in METHODS)]
        for r,p in ready[:2]+[x for x in ready if x[0]!=ROUTES[0]][:2]:
            report.start(RID[r]+' '+p,bookmark='preview_'+r+p,level=1)
            case_pages(report,records,r,p)
        report.finish();print('Preview',report.page,'pages',flush=True);return
    manifest=audit(records)
    if manifest['issues']:
        raise RuntimeError('Audit failed; refusing final PDF: '+ '; '.join(manifest['issues']))
    draft=Report(OUT,dry=True)
    compose_report(draft,records,manifest)
    draft.finish()
    FIGURES.clear()
    report=Report(OUT,refs=draft.destinations)
    compose_report(report,records,manifest)
    if report.destinations!=draft.destinations:raise RuntimeError('Pagination changed between passes')
    if report.figure_count!=92 or len(set(FIGURES))!=92:raise RuntimeError('Expected all 92 original figures')
    report.finish()
    (BASE/'figure_index.json').write_text(json.dumps(FIGURES,ensure_ascii=False,indent=2))
    print(json.dumps({'pdf':str(OUT),'pages':report.page,'figures':report.figure_count,'records':len(records),'successes':manifest['success_count']},ensure_ascii=False),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--preview',action='store_true')
    parser.add_argument('--regenerate-figures',action='store_true',help='Recompute scientific plots instead of reusing the verified artwork')
    args=parser.parse_args();REUSE_FIGURES=not args.regenerate_figures;build(args.preview)
