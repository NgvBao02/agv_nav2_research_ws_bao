#!/usr/bin/env python3
"""Build the Vietnamese PSTMO II/III manuscript from audited, immutable data.

Run from any directory using a Python environment with python-docx, matplotlib,
numpy, Pillow, PyYAML and lxml. No experiment is launched and no input is edited.
"""
from __future__ import annotations

import csv
import hashlib
import io
import json
import math
import re
from copy import deepcopy
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyArrowPatch, Rectangle
import numpy as np
from PIL import Image
import yaml
from lxml import etree
from docx import Document
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.style import WD_STYLE_TYPE
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt, RGBColor
from latex2mathml.converter import convert as latex_to_mathml
import mathml2omml

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'Viet_bao/journal/ver4'
FIG = OUT / 'figures'
TEMPLATE = ROOT / 'Viet_bao/final_bao_ICEEIS/IEEE Xplore conference-template-a4.docx'
MAIN = ROOT / 'docs/warehouse_cross_aisles_5_routes/all_125_trials.csv'
COHORT = ROOT / 'results/pstmo_journal_repeatability_20261011/analysis'
COL = (595.3 - 2 * 45.35 - 18) / 72 / 2
METHODS = ['raw', 'simple', 'savitzky_golay', 'constrained', 'pstmo']
LABELS = ['Đường gốc', 'Simple', 'SG', 'Constrained', 'PSTMO']
PLANNERS = ['NavFnAStar', 'NavFnDijkstra', 'ThetaStar', 'Smac2D', 'SmacHybrid']
SHORT = ['A*', 'Dijkstra', 'Theta*', 'Smac 2D', 'Hybrid']
COLORS = ['#555555', '#0072B2', '#D55E00', '#9B59B6', '#009E73']
LINES = [':', '--', '-.', '--', '-']
ROUTES = ['cross_aisle_transfer', 'wide_diagonal_transfer', 'south_u_transfer',
          'reverse_diagonal_transfer', 'cross_to_picking_turn']
SOURCES: dict[str, str] = {}
FIGURES: dict[str, dict] = {}
TABLES: dict[str, tuple] = {}


def fingerprint(path):
    path = Path(path)
    SOURCES[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()


def read_csv(path):
    fingerprint(path)
    with path.open(encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))


ROWS = read_csv(MAIN)
for r in ROWS:
    for k, v in list(r.items()):
        if k == 'success':
            r[k] = v.lower() in ('true', '1')
        elif k.endswith(('_m', '_s', '_1pm', '_rad')):
            r[k] = float(v)
assert len(ROWS) == 125 and sum(r['success'] for r in ROWS) == 123
fingerprint(COHORT / 'aggregate.json')
AGG = json.loads((COHORT / 'aggregate.json').read_text())
BLOCKS = read_csv(COHORT / 'block_differences.csv')
TRIALS = read_csv(COHORT / 'trial_metrics.csv')
assert AGG['trial_count'] == 250 and not AGG['audit_issues']
assert AGG['valid_raw_pair_count'] == 125
DATA = {}
for row in ROWS:
    p = ROOT / row['source']
    fingerprint(p)
    DATA[row['route'], row['planner'], row['method']] = json.loads(p.read_text())
for p in ['src/vacuum_robot_gazebo/config/nav2_params.yaml',
          'docs/warehouse_cross_aisles_5_routes/scenarios.yaml',
          'src/adaptive_pivot_g2/src/quintic_transition.cpp',
          'src/adaptive_pivot_g2/src/path_optimization.cpp',
          'src/adaptive_pivot_g2_nav2/src/adaptive_pivot_g2_smoother.cpp',
          'tools/analyze_journal_repeatability.py']:
    fingerprint(ROOT / p)


def values(route, method, key, success=False):
    return [r[key] for r in ROWS if r['method'] == method and
            (route is None or r['route'] == route) and (not success or r['success'])]


def mean(route, method, key, success=False):
    return float(np.mean(values(route, method, key, success)))


def pair(route, planner, key):
    a = next(r for r in ROWS if (r['route'], r['planner'], r['method']) ==
             (route, planner, 'pstmo'))
    b = next(r for r in ROWS if (r['route'], r['planner'], r['method']) ==
             (route, planner, 'constrained'))
    return a[key] - b[key] if a['success'] and b['success'] else None


def setup_plots():
    for path in Path('/home/linh-pham/.local/share/fonts/pstmo-times').glob('*.TTF'):
        font_manager.fontManager.addfont(str(path))
    plt.rcParams.update({'font.family': 'serif', 'font.serif': ['Times New Roman', 'DejaVu Serif'],
        'font.size': 8.5, 'axes.labelsize': 8.5, 'axes.titlesize': 9,
        'xtick.labelsize': 8, 'ytick.labelsize': 8, 'legend.fontsize': 8,
        'axes.spines.top': False, 'axes.spines.right': False, 'axes.linewidth': .55,
        'grid.linewidth': .4, 'svg.fonttype': 'none', 'pdf.fonttype': 42,
        'savefig.facecolor': 'white', 'mathtext.fontset': 'stix'})


def savefig(fig, name, description, sources=None):
    fig.savefig(FIG / f'{name}.png', dpi=450)
    svg_path=FIG / f'{name}.svg'
    fig.savefig(svg_path)
    # Matplotlib emits spaces before line endings inside path data. Normalize
    # generated SVGs so repository whitespace checks remain deterministic.
    svg_path.write_text('\n'.join(line.rstrip() for line in
        svg_path.read_text(encoding='utf-8').splitlines())+'\n',encoding='utf-8')
    FIGURES[name] = {'description': description, 'width_in': fig.get_figwidth(),
                     'height_in': fig.get_figheight(), 'sources': sources or []}
    plt.close(fig)


MAP = ROOT / 'src/vacuum_robot_gazebo/maps/warehouse_cross_aisles.pgm'
fingerprint(MAP)
fingerprint(MAP.with_suffix('.yaml'))
MAPINFO = yaml.safe_load(MAP.with_suffix('.yaml').read_text())
MAPIMAGE = np.asarray(Image.open(MAP))
RES = MAPINFO['resolution']
ORIGIN = MAPINFO['origin']
EXTENT = [ORIGIN[0], ORIGIN[0] + MAPIMAGE.shape[1] * RES,
          ORIGIN[1], ORIGIN[1] + MAPIMAGE.shape[0] * RES]


def map_axes(ax):
    ax.imshow(MAPIMAGE, cmap='gray', vmin=0, vmax=255, extent=EXTENT, origin='upper',
              interpolation='nearest', alpha=.65)
    ax.set(xlim=(-6, 6), ylim=(-4, 4), xlabel='x (m)', ylabel='y (m)')
    ax.set_aspect('equal')
    ax.grid(False)


def figures():
    setup_plots()
    fig, ax = plt.subplots(figsize=(COL, 2.28))
    ax.axis('off'); ax.set(xlim=(0, 10), ylim=(0, 10))
    labels = ['Đường vào + bản đồ chi phí + vùng bao', 'Tiền xử lý; phát hiện góc',
              'Ứng viên Bézier G² / quay tại chỗ an toàn',
              'Lọc động học và thời gian; chấm điểm',
              'Quy hoạch động; kiểm tra cuối',
              'Đường tham chiếu → RPP → robot; ghi quỹ đạo thực']
    for i, label in enumerate(labels):
        y = 9.35 - i * 1.65
        ax.add_patch(Rectangle((.12, y-.57), 9.75, 1.14, fill=False, lw=.7))
        ax.text(5, y, label, ha='center', va='center', fontsize=8.1)
        if i < len(labels)-1:
            ax.annotate('', (5,y-1.05), (5,y-.6), arrowprops={'arrowstyle':'->','lw':.65})
    fig.subplots_adjust(left=.02,right=.98,top=.98,bottom=.02)
    savefig(fig, 'pipeline', 'Sơ đồ khái niệm; không phải dữ liệu đo.')

    fig, ax = plt.subplots(figsize=(COL, 2.65))
    c=np.array([0.,0.]); u=np.array([1.,0.]); w=np.array([0.,1.]); d=1.; q=.3
    pts=np.array([-d*u,-d*u+q*u,-d*u+2*q*u,d*w-2*q*w,d*w-q*w,d*w])
    t=np.linspace(0,1,250)
    curve=sum(math.comb(5,j)*(1-t[:,None])**(5-j)*t[:,None]**j*pts[j] for j in range(6))
    ax.plot([-1.35,0,0],[0,0,1.35],':',color=COLORS[0],lw=1.2,label='Đường gấp khúc')
    ax.plot(pts[:,0],pts[:,1],'o--',color='#0072B2',ms=3,lw=.6,label='Đa giác điều khiển')
    ax.plot(curve[:,0],curve[:,1],color=COLORS[-1],lw=1.5,label='Bézier bậc năm')
    for i,p in enumerate(pts):
        label = rf'$P_{i}$' if i not in (0,5) else (r'$A=P_0$' if i==0 else r'$B=P_5$')
        ax.annotate(label,p,xytext=(-14,-14) if i<3 else (6,0),textcoords='offset points',fontsize=8)
    ax.text(.04,-.09,'C');ax.text(-1.3,.08,'u →');ax.text(.06,1.25,'w ↑')
    ax.annotate('',(0,-.28),(-1,-.28),arrowprops={'arrowstyle':'<->','lw':.65})
    ax.text(-.5,-.34,r'$d$',ha='center',va='top',fontsize=8)
    ax.annotate('',(.36,1),(.36,0),arrowprops={'arrowstyle':'<->','lw':.65})
    ax.text(.43,.5,r'$d$',va='center',fontsize=8)
    ax.set(xlim=(-1.45,.65),ylim=(-.49,1.5));ax.set_aspect('equal');ax.axis('off')
    ax.legend(loc='upper left',bbox_to_anchor=(-.22,1.02),frameon=False,fontsize=7.7)
    fig.subplots_adjust(left=.15,right=.98,top=.98,bottom=.03)
    savefig(fig,'bezier','Sơ đồ hình học với d=1, α=0,3; không phải quỹ đạo thử nghiệm.')

    fig,ax=plt.subplots(figsize=(COL,2.75));map_axes(ax)
    routecols=['#0072B2','#D55E00','#009E73','#CC79A7','#332288']
    for i,route in enumerate(ROUTES):
        j=DATA[route,'ThetaStar','raw'];p=np.array(j['selected_path_xy'])
        ax.plot(p[:,0],p[:,1],color=routecols[i],lw=1.0,label=f'R{i+1:02}')
        ax.plot(*j['start'][:2],'o',color=routecols[i],ms=4)
        ax.plot(*j['goal'][:2],'*',color=routecols[i],ms=7)
    ax.legend(loc='upper center',bbox_to_anchor=(.5,1.2),ncol=5,frameon=False,columnspacing=.6,handlelength=1)
    fig.subplots_adjust(left=.15,right=.98,bottom=.17,top=.82)
    savefig(fig,'routes','Raw ThetaStar; chấm tròn: xuất phát; sao: đích; bản đồ occupancy gốc.')

    fig,ax=plt.subplots(figsize=(COL,1.95));ax.set_aspect('equal');ax.axis('off')
    ax.add_patch(Rectangle((-.22,-.17),.44,.34,fill=False,lw=1))
    for y in [-.2548/2,.2548/2]:ax.add_patch(Rectangle((-.04,y-.023),.08,.046,color='#777777'))
    ax.plot(0,0,'k+',ms=7);ax.arrow(0,0,.15,0,width=.001,head_width=.012,color='k')
    ax.text(.14,.018,'x');ax.text(-.03,.02,'O')
    for a,b,label,xy in [((-.22,-.24),(.22,-.24),'0,44 m',(0,-.28)),
                         ((.29,-.17),(.29,.17),'0,34 m',(.36,0)),
                         ((-.12,-.1274),(-.12,.1274),'b = 0,2548 m',(-.13,.23))]:
        ax.add_patch(FancyArrowPatch(a,b,arrowstyle='<->',mutation_scale=6,lw=.7))
        ax.text(*xy,label,ha='center',va='center',fontsize=8)
    ax.set(xlim=(-.35,.5),ylim=(-.34,.29));fig.subplots_adjust(left=.01,right=.99,top=.98,bottom=.02)
    savefig(fig,'robot','Sơ đồ mặt bằng footprint và track động học; không phải ảnh CAD/Gazebo.')

    for ri,route in enumerate(ROUTES,1):
        fig,axs=plt.subplots(2,1,figsize=(COL,4.3),gridspec_kw={'height_ratios':[1,1]})
        for ax in axs:map_axes(ax)
        for mi,m in enumerate(METHODS):
            j=DATA[route,'ThetaStar',m];p=np.array(j['selected_path_xy'])
            for ax in axs:ax.plot(p[:,0],p[:,1],LINES[mi],color=COLORS[mi],lw=1.1,label=LABELS[mi])
        base=DATA[route,'ThetaStar','pstmo'];p=np.array(base['selected_path_xy'])
        raw=np.array(DATA[route,'ThetaStar','raw']['selected_path_xy'])
        # Deterministic zoom: point of greatest PSTMO distance to Raw vertices.
        dist=np.min(np.sum((p[:,None,:]-raw[None,:,:])**2,axis=2),axis=1)
        center=p[int(np.argmax(dist))]; span=1.15
        axs[1].set(xlim=(center[0]-span,center[0]+span),ylim=(center[1]-.72,center[1]+.72))
        axs[0].add_patch(Rectangle((center[0]-span,center[1]-.72),2*span,1.44,fill=False,lw=.7))
        for ax in axs:
            ax.plot(*base['start'][:2],'ko',ms=3);ax.plot(*base['goal'][:2],'k*',ms=6)
        axs[0].set_title(f'(a) R{ri:02} / Theta* — toàn tuyến',pad=3)
        axs[1].set_title('(b) Phóng to vùng khác biệt hình học',pad=3)
        handles,labels=axs[0].get_legend_handles_labels()
        fig.legend(handles,labels,loc='upper center',ncol=3,frameon=False,bbox_to_anchor=(.52,1.005),columnspacing=.8,handlelength=1.6)
        fig.subplots_adjust(left=.16,right=.98,bottom=.10,top=.84,hspace=.52)
        savefig(fig,f'r{ri:02}_paths',f'R{ri:02}, ThetaStar, năm đường đã lập; zoom chọn bằng quy tắc cố định.',
                [str(next(r['source'] for r in ROWS if (r['route'],r['planner'],r['method'])==(route,'ThetaStar',m))) for m in METHODS])

        fig,axs=plt.subplots(2,1,figsize=(COL,3.65),sharex=True)
        x=np.arange(5)
        for ax,key,scale,ylab in [(axs[0],'execution_time_s',1,'Thời gian (s)'),
                                 (axs[1],'tracking_rmse_m',1000,'RMSE (mm)')]:
            for mi,m in enumerate(METHODS):
                rr=[next(r for r in ROWS if (r['route'],r['planner'],r['method'])==(route,p,m)) for p in PLANNERS]
                v=[r[key]*scale for r in rr]
                bars=ax.bar(x+(mi-2)*.15,v,width=.14,color=COLORS[mi],label=LABELS[mi],alpha=.9)
                for b,r in zip(bars,rr):
                    if not r['success']:
                        b.set_hatch('////'); b.set_alpha(.3)
                        ax.text(b.get_x()+b.get_width()/2,b.get_height(),'×',ha='center',va='bottom',fontsize=11)
            ax.set_ylabel(ylab);ax.grid(axis='y',alpha=.3);ax.set_axisbelow(True)
            ax.set_ylim(0,ax.get_ylim()[1]*1.08)
        axs[1].set_xticks(x,SHORT,rotation=15,ha='right')
        axs[0].legend(ncol=3,loc='lower center',bbox_to_anchor=(.5,1.01),frameon=False,columnspacing=.7,handlelength=1.1)
        fig.subplots_adjust(left=.17,right=.98,bottom=.15,top=.86,hspace=.20)
        savefig(fig,f'r{ri:02}_metrics',f'R{ri:02}, toàn bộ 25 lượt; cột gạch/× là lượt không đạt và không được gộp vào trung bình thành công.')

    fig,axs=plt.subplots(2,1,figsize=(COL,3.75))
    for ax,key,label in [(axs[0],'execution_time_s','(a) Δ thời gian (s)'),
                         (axs[1],'tracking_rmse_m','(b) Δ RMSE (mm)')]:
        arr=np.array([[pair(r,p,key) if pair(r,p,key) is not None else np.nan for p in PLANNERS] for r in ROUTES])
        if key=='tracking_rmse_m':arr*=1000
        lim=np.nanmax(abs(arr));ax.imshow(arr,cmap='RdBu_r',vmin=-lim,vmax=lim,aspect='auto')
        for i in range(5):
            for j in range(5):
                v=arr[i,j];ax.text(j,i,'—' if np.isnan(v) else f'{v:+.1f}',ha='center',va='center',fontsize=8,
                                  color='white' if abs(v)>lim*.6 else 'black')
        ax.set_xticks(range(5),SHORT,fontsize=7.5);ax.set_yticks(range(5),[f'R{i:02}' for i in range(1,6)])
        ax.set_title(label,pad=4)
    fig.subplots_adjust(left=.11,right=.98,bottom=.07,top=.94,hspace=.40)
    savefig(fig,'matrix_delta','PSTMO − Constrained; chỉ cặp cùng đạt; màu xanh là chênh lệch âm; không có CI từ ma trận một lượt.')

    fig,axs=plt.subplots(2,1,figsize=(COL,3.5))
    route=ROUTES[1]
    map_axes(axs[0])
    selected=np.array(DATA[route,'ThetaStar','pstmo']['selected_path_xy'])
    raw=np.array(DATA[route,'ThetaStar','raw']['selected_path_xy'])
    delta=np.min(np.sum((selected[:,None,:]-raw[None,:,:])**2,axis=2),axis=1)
    center=selected[int(np.argmax(delta))]
    for m,color in [('constrained',COLORS[3]),('pstmo',COLORS[4])]:
        j=DATA[route,'ThetaStar',m];plan=np.array(j['selected_path_xy']);gt=np.array(j['ground_truth_state_trace'])
        label='Constrained' if m=='constrained' else 'PSTMO'
        axs[0].plot(plan[:,0],plan[:,1],'--',color=color,lw=1.0,label=label+' tham chiếu')
        axs[0].plot(gt[:,1],gt[:,2],'-',color=color,lw=1.0,label=label+' thực')
        axs[1].plot(gt[:,0],gt[:,5],color=color,lw=.65,label=label)
    axs[0].set(xlim=(center[0]-1.15,center[0]+1.15),ylim=(center[1]-.68,center[1]+.68))
    axs[0].legend(loc='lower center',bbox_to_anchor=(.5,1.01),ncol=2,frameon=False,fontsize=7.5,columnspacing=.7)
    axs[1].set(xlabel='Thời gian mô phỏng tương đối (s)',ylabel='ω thực (rad/s)')
    axs[1].axhline(0,color='gray',lw=.5);axs[1].grid(alpha=.25)
    fig.subplots_adjust(left=.20,right=.98,bottom=.13,top=.86,hspace=.48)
    savefig(fig,'execution_trace','R02/ThetaStar, ma trận khảo sát: đường tham chiếu và quỹ đạo thực; không lọc đường ω(t).',
            [next(r['source'] for r in ROWS if (r['route'],r['planner'],r['method'])==(route,'ThetaStar',m)) for m in ['constrained','pstmo']])

    fig,axs=plt.subplots(3,1,figsize=(COL,3.9))
    for ax,key,scale,lab in zip(axs,['execution_time_s','tracking_rmse_m','executed_curvature_energy_1pm'],[1,1000,1],
                              ['Δ thời gian (s)','Δ RMSE (mm)',r'Δ $E_{\kappa,exec}$ ($m^{-1}$)']):
        subset=[b for b in BLOCKS if b['metric']==key]
        order={(r,p):i*5+j for i,r in enumerate(ROUTES) for j,p in enumerate(PLANNERS)}
        subset.sort(key=lambda b:order[b['scenario'],b['planner']])
        yy=np.array([float(b['pstmo_minus_constrained'])*scale for b in subset])
        ax.axhline(0,color='black',lw=.7);ax.scatter(range(25),yy,s=12,color=COLORS[-1],zorder=3)
        for boundary in [4.5,9.5,14.5,19.5]:ax.axvline(boundary,color='#bbbbbb',lw=.5)
        a=AGG['metrics'][key];avg=a['mean_block_difference_pstmo_minus_constrained']*scale
        ax.axhline(avg,color=COLORS[3],lw=.9,ls='--')
        ax.set_xticks([2,7,12,17,22],[f'R{i:02}' for i in range(1,6)])
        ax.set_ylabel(lab);ax.grid(axis='y',alpha=.25)
    fig.subplots_adjust(left=.21,right=.98,bottom=.07,top=.98,hspace=.33)
    savefig(fig,'repeat_blocks','25 hiệu ứng trung bình trong khối; thứ tự bộ lập kế hoạch A*, Dijkstra, Theta*, Smac2D, Hybrid; không phải 125 quan sát độc lập.',
            ['results/pstmo_journal_repeatability_20261011/analysis/block_differences.csv'])

    fig,axs=plt.subplots(2,1,figsize=(COL,3.7))
    for ri,route in enumerate(ROUTES):
        xs=[];ys=[];ts=[];es=[]
        for p in PLANNERS:
            b=lambda k: next(float(b['pstmo_minus_constrained']) for b in BLOCKS if b['metric']==k and b['scenario']==route and b['planner']==p)
            xs.append(100*b('planned_footprint_clearance_min_m'));ys.append(b('planned_curvature_energy_1pm'))
            ts.append(b('execution_time_s'));es.append(1000*b('final_position_error_m'))
        axs[0].scatter(xs,ys,s=23,color=routecols[ri],marker=['o','s','^','D','v'][ri],label=f'R{ri+1:02}')
        axs[1].scatter(ts,es,s=23,color=routecols[ri],marker=['o','s','^','D','v'][ri])
    for ax in axs:ax.axhline(0,color='gray',lw=.6);ax.axvline(0,color='gray',lw=.6);ax.grid(alpha=.2)
    axs[0].set(xlabel='Δ khoảng hở nhỏ nhất (cm)',ylabel=r'Δ $E_{\kappa,ref}$ ($m^{-1}$)')
    axs[1].set(xlabel='Δ thời gian (s)',ylabel='Δ sai số đích (mm)')
    axs[0].legend(ncol=5,loc='lower center',bbox_to_anchor=(.5,1.01),frameon=False,columnspacing=.45,handletextpad=.2)
    fig.subplots_adjust(left=.22,right=.98,bottom=.13,top=.89,hspace=.49)
    savefig(fig,'tradeoffs','Mỗi điểm là một khối tuyến–bộ lập kế hoạch của bộ lặp; chênh lệch PSTMO − Constrained.')

    fig,ax=plt.subplots(figsize=(COL,2.6))
    for mi,m in enumerate(['constrained','pstmo']):
        rr=[r for r in TRIALS if r['method']==m]
        yy=[float(r['final_position_error_m']) for r in rr]
        # Jitter is deterministic for display only, not a stochastic trial seed.
        xx=[mi+((i%13)-6)*.027 for i in range(len(rr))]
        ax.scatter(xx,yy,s=9,color=COLORS[3+mi],alpha=.55)
    ax.axhline(.1,color='black',ls='--',lw=.7,label='Ngưỡng 0,10 m')
    ax.set_yscale('log');ax.set_xticks([0,1],['Constrained','PSTMO']);ax.set_ylabel('Sai số vị trí cuối (m, log)')
    ax.legend(frameon=False,loc='upper center',fontsize=8);ax.grid(axis='y',alpha=.3)
    fig.subplots_adjust(left=.2,right=.98,bottom=.14,top=.98)
    savefig(fig,'endpoints','Toàn bộ 250 lượt; điểm nằm dưới ngưỡng vị trí vẫn phải thỏa điều kiện tác vụ, hướng và trạng thái dừng.')


def tables():
    TABLES['params']=(['Thông số PSTMO','Giá trị'],[
        ['Khoảng cắt d / α thô','0,02–0,80 m / 0,1:0,1:0,5'],
        ['Bước kiểm tra / xuất đường','0,02 / 0,05 m'],
        ['Ngưỡng góc / biên đoạn','5° / 0,05 m (tự suy ra)'],
        ['Biên lợi thế / cửa sổ thời gian','0,15 / 10,0 s'],
        ['Trọng số chi phí / góc / độ cong','0,15 / 0,10 / 0,75'],
        ['Hệ số chuẩn hóa độ cong','1 m⁻¹'],
        ['Vận tốc thẳng / góc cực đại','0,30 m/s / 0,80 rad/s'],
        ['Vận tốc bánh / gia tốc ngang cực đại','0,36 m/s / 0,18 m/s²'],
        ['Gia tốc / giảm tốc / gia tốc góc','0,35 / 0,45 m/s² / 1,20 rad/s²'],
        ['Tiền xử lý','Điều hòa dữ liệu, không rút gọn đường'],
        ['Tìm ứng viên','Phân cấp theo α với hai khoảng cắt']], [1.82,1.56])
    TABLES['robot']=(['Đại lượng','Giá trị'],[
        ['Hệ phần mềm','ROS 2 Jazzy, Nav2, Gazebo'],
        ['Vùng bao điều hướng','0,44 × 0,34 m'],
        ['Bán kính bánh dẫn động','0,0425 m'],
        ['Khoảng cách bánh trong PSTMO','0,2548 m'],
        ['Khoảng cách bánh hiệu dụng trong mô phỏng','0,2834 m'],
        ['Độ phân giải bản đồ','0,05 m/ô'],
        ['RPP: tần số / khoảng nhìn trước','20 Hz / 0,50 m'],
        ['Vận tốc đặt / xoay hướng','0,30 m/s / 0,25 rad/s'],
        ['Bộ kiểm tra đích Nav2: vị trí / hướng','0,06 m / 0,10 rad'],
        ['Hậu kiểm: vị trí / hướng','0,10 m / 0,15 rad'],
        ['Hậu kiểm dừng: |v| / |ω|','≤ 0,01 m/s / ≤ 0,02 rad/s']], [1.8,1.58])
    TABLES['scenarios']=(['Tuyến','Xuất phát (x, y) m','Đích (x, y) m'],[
        [f'R{i+1:02}',', '.join(f'{v:.1f}' for v in DATA[r,'ThetaStar','raw']['start'][:2]),
         ', '.join(f'{v:.1f}' for v in DATA[r,'ThetaStar','raw']['goal'][:2])] for i,r in enumerate(ROUTES)], [.46,1.46,1.46])
    TABLES['baselines']=(['Phương án','Cấu hình đáng chú ý'],[
        ['Đường gốc','Không hậu xử lý; giữ đường của bộ lập kế hoạch'],
        ['Simple','Tối đa 1.000 vòng; dung sai 10⁻¹⁰; tinh chỉnh hai lần'],
        ['SG','Savitzky–Golay; tinh chỉnh hai lần'],
        ['Constrained','Trọng số trơn 200.000; chi phí 0,015; khoảng cách và độ cong bằng 0; tối đa 70 vòng'],
        ['PSTMO','Tham số trong Bảng I; cùng RPP và vùng bao điều hướng']], [.88,2.5])
    for i,r in enumerate(ROUTES,1):
        rows=[]
        for m,label in zip(METHODS,LABELS):
            n=sum(x['success'] for x in ROWS if x['route']==r and x['method']==m)
            rows.append([label,str(n)+'/5',f"{mean(r,m,'execution_time_s',True):.2f}",
                f"{1000*mean(r,m,'tracking_rmse_m',True):.1f}",
                f"{mean(r,m,'planned_curvature_energy_1pm'):.2f}",
                f"{100*mean(r,m,'planned_footprint_clearance_min_m'):.1f}"])
        TABLES[f'r{i:02}']=(['Phương án','Đạt','T (s)','e (mm)','Eκ,ref (m⁻¹)','c (cm)'],rows,[.76,.33,.49,.51,.79,.50])
        rows=[]
        for p,label in zip(PLANNERS,SHORT):
            vals=[pair(r,p,k) for k in ['execution_time_s','tracking_rmse_m','planned_curvature_energy_1pm']]
            rows.append([label]+['—' if v is None else f'{v*(1000 if j==1 else 1):+.2f}' for j,v in enumerate(vals)])
        TABLES[f'r{i:02}_paired']=(['Bộ lập kế hoạch','ΔT (s)','Δe (mm)','ΔEκ,ref (m⁻¹)'],rows,[1.10,.65,.68,.95])
    rows=[]
    for m,label in zip(METHODS,LABELS):
        n=sum(r['success'] for r in ROWS if r['method']==m)
        rows.append([label,f'{n}/25',f"{mean(None,m,'execution_time_s',True):.2f}",
                     f"{1000*mean(None,m,'tracking_rmse_m',True):.1f}",f"{mean(None,m,'planned_curvature_energy_1pm'):.2f}"])
    TABLES['pooled']=(['Phương án','Đạt','T (s)','e (mm)','Eκ,ref (m⁻¹)'],rows,[.82,.45,.58,.61,.92])
    keys=['execution_time_s','tracking_rmse_m','executed_curvature_energy_1pm']
    names=['T (s)','e (mm)','Eκ,exec (m⁻¹)']; scales=[1,1000,1]
    rows=[]; means=[]
    for k,n,s in zip(keys,names,scales):
        a=AGG['metrics'][k];lo,hi=a['mean_block_difference_bootstrap_95_ci']
        rows.append([n,f"{a['mean_block_difference_pstmo_minus_constrained']*s:.3f}",f'[{lo*s:.3f}; {hi*s:.3f}]',f"{a['holm_adjusted_p_primary_family']:.2e}"])
        means.append([n,f"{a['constrained_mean']*s:.3f}",f"{a['pstmo_mean']*s:.3f}",f"{a['blocks_pstmo_lower']}/25"])
    TABLES['primary']=(['Chỉ số','Δ khối','Khoảng tin cậy 95%','p Holm'],rows,[.84,.58,1.32,.64])
    TABLES['means']=(['Chỉ số','Constrained','PSTMO','Khối giảm'],means,[.88,.93,.73,.84])
    rows=[]
    for p,label in zip(PLANNERS,SHORT):
        vals=[np.mean([float(b['pstmo_minus_constrained'])*s for b in BLOCKS
                       if b['planner']==p and b['metric']==k]) for k,s in zip(keys,scales)]
        rows.append([label]+[f'{v:+.3f}' for v in vals])
    TABLES['by_planner']=(['Bộ lập kế hoạch','ΔT (s)','Δe (mm)','ΔEκ,exec'],rows,[1.12,.66,.68,.92])
    rows=[]
    for k,n,s in [('planned_curvature_energy_1pm','Eκ,ref (m⁻¹)',1),
        ('actual_p95_abs_jerk_mps3','Độ giật p95 (m/s³)',1),('traveled_distance_m','Quãng đường (m)',1),
        ('final_position_error_m','Sai số đích (mm)',1000),('planned_footprint_clearance_min_m','Khoảng hở (mm)',1000),
        ('smoothing_time_s','Thời lượng hậu xử lý (ms)',1000)]:
        a=AGG['metrics'][k];lo,hi=a['mean_block_difference_bootstrap_95_ci']
        rows.append([n,f"{a['mean_block_difference_pstmo_minus_constrained']*s:+.3f}",f'[{lo*s:+.3f}; {hi*s:+.3f}]'])
    TABLES['secondary']=(['Chỉ số phụ','Δ khối','Khoảng tin cậy 95%'],rows,[1.15,.65,1.58])
    TABLES['failures']=(['Tuyến/bộ lập kế hoạch','Phương án/lần','Trạng thái','Sai số đích (m)'],[
        ['R05 / A*','P / 4','Kết thúc trước đích','1,580262'],['R05 / Dijkstra','P / 4','Kết thúc trước đích','0,685907'],
        ['R05 / Smac2D','C / 3','Kết thúc trước đích','3,465366'],['R05 / Smac2D','P / 1','Kết thúc trước đích','1,148750'],
        ['R05 / Hybrid','C / 4','Kết thúc trước đích','3,655099'],['R04 / Hybrid','P / 4','Vượt ngưỡng vị trí','0,101583'],
        ['R04 / Theta*','C / 4','Vượt ngưỡng vị trí','0,100358']], [1.12,.70,1.00,.64])


def normalized_template():
    """Normalize a copy of the retained ISO Strict template for python-docx."""
    namespaces={
        'http://purl.oclc.org/ooxml/wordprocessingml/main':'http://schemas.openxmlformats.org/wordprocessingml/2006/main',
        'http://purl.oclc.org/ooxml/officeDocument/relationships':'http://schemas.openxmlformats.org/officeDocument/2006/relationships',
        'http://purl.oclc.org/ooxml/officeDocument/math':'http://schemas.openxmlformats.org/officeDocument/2006/math',
        'http://purl.oclc.org/ooxml/drawingml/main':'http://schemas.openxmlformats.org/drawingml/2006/main',
        'http://purl.oclc.org/ooxml/drawingml/wordprocessingDrawing':'http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing',
        'http://purl.oclc.org/ooxml/drawingml/picture':'http://schemas.openxmlformats.org/drawingml/2006/picture',
    }
    b=io.BytesIO()
    with ZipFile(TEMPLATE) as zin, ZipFile(b,'w',ZIP_DEFLATED) as zout:
        for name in zin.namelist():
            data=zin.read(name)
            if name.endswith(('.xml','.rels')):
                s=data.decode('utf-8')
                for old,new in namespaces.items():s=s.replace(old,new)
                s=re.sub(r'((?:w|m):[A-Za-z]+=")(-?\d+(?:\.\d+)?)pt"',
                         lambda m:m.group(1)+str(round(float(m.group(2))*20))+'"',s)
                s=s.replace('w:val="start"','w:val="left"').replace('w:val="end"','w:val="right"')
                data=s.encode('utf-8')
            zout.writestr(name,data)
    b.seek(0);return Document(b)


def no_number(p):
    pp=p._p.get_or_add_pPr();num=OxmlElement('w:numPr');v=OxmlElement('w:numId');v.set(qn('w:val'),'0');num.append(v);pp.append(num)


def style_font(st, size):
    st.font.name='Times New Roman';st.font.size=Pt(size);st.font.color.rgb=RGBColor(0,0,0)
    rp=st.element.get_or_add_rPr();fonts=rp.find(qn('w:rFonts'))
    if fonts is not None:
        for k in ['ascii','hAnsi','cs','eastAsia']:fonts.set(qn('w:'+k),'Times New Roman')
    lang=OxmlElement('w:lang');lang.set(qn('w:val'),'vi-VN');rp.append(lang)


def create_doc():
    doc=normalized_template()
    sections=doc.element.body.findall('.//'+qn('w:sectPr'))
    bodysect=deepcopy(next(s for s in sections if s.find(qn('w:cols')) is not None and s.find(qn('w:cols')).get(qn('w:num'))=='2'))
    for e in list(bodysect):
        if e.tag in [qn('w:headerReference'),qn('w:footerReference'),qn('w:titlePg'),qn('w:type')]:bodysect.remove(e)
    for e in list(doc.element.body):doc.element.body.remove(e)
    doc.element.body.append(bodysect)
    for name,size in [('Normal',10),('Body Text',10),('Heading 1',10),('Heading 2',10),('Heading 3',10),('figure caption',8),('table head',8)]:
        st=doc.styles[name];style_font(st,size)
        if name.startswith('Heading'):st.font.bold=False
    body=doc.styles['Body Text'];body.paragraph_format.widow_control=True
    body.paragraph_format.keep_together=False
    body.paragraph_format.line_spacing=1.06
    body.paragraph_format.space_after=Pt(4)
    for name,base,size in [('Scientific Table','Body Text',8),('Scientific Note','Body Text',8),('Scientific Equation','Body Text',10),('Scientific Reference','Body Text',8)]:
        st=doc.styles.add_style(name,WD_STYLE_TYPE.PARAGRAPH) if name not in doc.styles else doc.styles[name]
        st.base_style=doc.styles[base];style_font(st,size)
        st.paragraph_format.first_line_indent=Pt(0)
        st.paragraph_format.space_after=Pt(3)
        st.paragraph_format.line_spacing=1.0
    doc.styles['Scientific Table'].paragraph_format.alignment=WD_ALIGN_PARAGRAPH.CENTER
    doc.styles['Scientific Table'].paragraph_format.space_after=Pt(0)
    doc.styles['Scientific Note'].paragraph_format.alignment=WD_ALIGN_PARAGRAPH.JUSTIFY
    doc.styles['Scientific Reference'].paragraph_format.left_indent=Pt(12)
    doc.styles['Scientific Reference'].paragraph_format.first_line_indent=Pt(-12)
    doc.core_properties.title='PSTMO Phương pháp và thực nghiệm trên năm tuyến kho giao cắt'
    doc.core_properties.subject='Phần II và III của bản thảo journal phiên bản 4'
    doc.core_properties.author=''
    doc.core_properties.keywords='PSTMO; Nav2; ROS 2; G2; fixed-input repeatability'
    mathpr=doc.settings.element.find(qn('m:mathPr'))
    if mathpr is None:
        mathpr=OxmlElement('m:mathPr');doc.settings.element.append(mathpr)
    mf=mathpr.find(qn('m:mathFont'))
    if mf is None:mf=OxmlElement('m:mathFont');mathpr.append(mf)
    mf.set(qn('m:val'),'DejaVu Math TeX Gyre')
    return doc


def table(doc, key, caption, number):
    heads, rows, widths=TABLES[key]
    widths=[w*COL/sum(widths) for w in widths]
    p=doc.add_paragraph(f'BẢNG {roman(number)}. {caption.upper()}',style='table head');no_number(p)
    p.paragraph_format.keep_with_next=True
    t=doc.add_table(rows=1,cols=len(heads));t.autofit=False
    t.alignment=1
    for c,w in zip(t.columns,widths):c.width=Inches(w)
    for cells,data in [(t.rows[0].cells,heads)]+[(t.add_row().cells,r) for r in rows]:
        for i,(c,txt) in enumerate(zip(cells,data)):
            c.width=Inches(widths[i]);c.vertical_alignment=1
            p=c.paragraphs[0];p.style=doc.styles['Scientific Table'];add_scientific_text(p, display_cell(txt))
            p.paragraph_format.keep_with_next=False
            if i==0:p.paragraph_format.alignment=WD_ALIGN_PARAGRAPH.LEFT
            tcPr=c._tc.get_or_add_tcPr()
            margins=OxmlElement('w:tcMar')
            for side,v in [('top',44),('bottom',44),('left',38),('right',38)]:
                e=OxmlElement('w:'+side);e.set(qn('w:w'),str(v));e.set(qn('w:type'),'dxa');margins.append(e)
            tcPr.append(margins)
        trpr=cells[0]._tc.getparent().get_or_add_trPr();trpr.append(OxmlElement('w:cantSplit'))
    for c in t.rows[0].cells:
        for r in c.paragraphs[0].runs:r.font.bold=True
    # All manuscript tables fit one column; keep each complete table together.
    for row in t.rows[:-1]:
        for c in row.cells:
            for p in c.paragraphs:p.paragraph_format.keep_with_next=True
    header=OxmlElement('w:tblHeader');t.rows[0]._tr.get_or_add_trPr().append(header)
    pr=t._tbl.tblPr;borders=OxmlElement('w:tblBorders')
    for side in ['top','left','bottom','right','insideH','insideV']:
        e=OxmlElement('w:'+side);e.set(qn('w:val'),'single');e.set(qn('w:sz'),'4');e.set(qn('w:color'),'777777');borders.append(e)
    pr.append(borders)
    p=doc.add_paragraph();p.paragraph_format.space_after=Pt(0);p.paragraph_format.space_before=Pt(0);p.paragraph_format.line_spacing=Pt(2);p.add_run().font.size=Pt(2)


def roman(n):
    out=''
    for v,s in [(10,'X'),(9,'IX'),(5,'V'),(4,'IV'),(1,'I')]:
        while n>=v:out+=s;n-=v
    return out


def equation(doc, text, n):
    # Real editable Office Math fractions, scripts, radicals and operators.
    from docx.enum.text import WD_TAB_ALIGNMENT
    lines=text.split(' || ')
    for i,line in enumerate(lines):
        p=doc.add_paragraph(style='Scientific Equation')
        p.paragraph_format.alignment=WD_ALIGN_PARAGRAPH.LEFT
        p.paragraph_format.tab_stops.add_tab_stop(Pt(14.4),WD_TAB_ALIGNMENT.CLEAR)
        p.paragraph_format.tab_stops.add_tab_stop(Inches(COL/2),WD_TAB_ALIGNMENT.CENTER)
        p.paragraph_format.tab_stops.add_tab_stop(Inches(COL),WD_TAB_ALIGNMENT.RIGHT)
        p.paragraph_format.keep_together=True;p.paragraph_format.keep_with_next=i<len(lines)-1
        p.paragraph_format.space_before=Pt(3 if i==0 else 0)
        p.paragraph_format.space_after=Pt(6 if i==len(lines)-1 else 2)
        omml=mathml2omml.convert(latex_to_mathml(line))
        omml=omml.replace('<m:oMath>', '<m:oMath xmlns:m="http://schemas.openxmlformats.org/officeDocument/2006/math" xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">',1)
        om=etree.fromstring(omml.encode())
        # mathml2omml omits the mandatory degree slot on square roots.
        # Add it explicitly for consistent Word and LibreOffice import.
        for rad in om.iter(qn('m:rad')):
            pr=OxmlElement('m:radPr');hide=OxmlElement('m:degHide');hide.set(qn('m:val'),'1');pr.append(hide)
            rad.insert(0,pr);rad.insert(1,OxmlElement('m:deg'))
        for node in om.iter(qn('m:nary')):
            pr=node.find(qn('m:naryPr'))
            if pr is None:pr=OxmlElement('m:naryPr');node.insert(0,pr)
            for part in ['sub','sup']:
                e=node.find(qn('m:'+part))
                if e is None or not ''.join(e.itertext()).strip():
                    h=OxmlElement('m:'+part+'Hide');h.set(qn('m:val'),'1');pr.append(h)
        for r in om.iter(qn('m:r')):
            rp=OxmlElement('w:rPr');fonts=OxmlElement('w:rFonts')
            for key in ['ascii','hAnsi']:fonts.set(qn('w:'+key),'DejaVu Math TeX Gyre')
            rp.append(fonts);sz=OxmlElement('w:sz');sz.set(qn('w:val'),'18');rp.append(sz)
            r.insert(1 if r.find(qn('m:rPr')) is not None else 0,rp)
        p.add_run('\t');p._p.append(om)
        if i==len(lines)-1:p.add_run(f'\t({n})')


def display_cell(value):
    text=str(value)
    text=text.replace('Smac2D','Smac 2D')
    if re.fullmatch(r'[0-9]+\.[0-9]+e-[0-9]+',text):
        mantissa, exponent=text.split('e-')
        sup=str(int(exponent)).translate(str.maketrans('0123456789','⁰¹²³⁴⁵⁶⁷⁸⁹'))
        return mantissa.replace('.',',')+' × 10⁻'+sup
    # Only data cells use decimal dots. Parameter descriptions already follow
    # Vietnamese notation (e.g. 200.000 is two hundred thousand).
    if re.fullmatch(r'[+−-]?\d+\.\d+',text) or re.fullmatch(r'\[[+−-]?\d+\.\d+;\s*[+−-]?\d+\.\d+\]',text):
        return text.replace('.',',')
    return text


def add_scientific_text(paragraph, text):
    # Keep mathematical subscripts editable without exposing implementation names.
    forms={'Eκ,ref':('E','κ,ref'),'Eκ,exec':('E','κ,exec'),
           'Eκ':('E','κ'),'vL':('v','L'),'vR':('v','R'),
           'dmax':('d','max'),'dmin':('d','min'),'cmax':('c','max'),
           'ωpeak':('ω','peak'),'ωmax':('ω','max'),'Escale':('E','scale'),
           'Tmin':('T','min'),'Tp':('T','p'),'δT':('δ','T'),
           'ρc':('ρ','c'),'ρω':('ρ','ω'),'ρE':('ρ','E'),
           'Rb':('R','b'),'db':('d','b')}
    pattern='('+ '|'.join(re.escape(k) for k in sorted(forms,key=len,reverse=True))+')'
    for part in re.split(pattern,text):
        if part in forms:
            base,sub=forms[part]
            paragraph.add_run(base).italic=True
            run=paragraph.add_run(sub);run.font.subscript=True
        elif part:
            paragraph.add_run(part)


def manuscript():
    doc=create_doc();nf=nt=ne=0
    source=OUT/'manuscript.md';fingerprint(source);fingerprint(TEMPLATE)
    chunks=re.split(r'\n\s*\n',source.read_text(encoding='utf-8').strip())
    for chunk in chunks:
        chunk=chunk.strip()
        if chunk.startswith('<!--'):continue
        if chunk.startswith('@figure '):
            _,name,caption=chunk.split(' ',2);nf+=1
            p=doc.add_paragraph();p.paragraph_format.first_line_indent=Pt(0);p.paragraph_format.alignment=WD_ALIGN_PARAGRAPH.CENTER
            p.paragraph_format.space_before=Pt(3);p.paragraph_format.space_after=Pt(0);p.paragraph_format.keep_with_next=True
            p.add_run().add_picture(str(FIG/f'{name}.png'),width=Inches(COL))
            alt=p._p.find('.//'+qn('wp:docPr'));alt.set('descr',caption)
            p=doc.add_paragraph(f'Hình {nf}. {caption}',style='figure caption');no_number(p)
            p.paragraph_format.keep_together=True;p.paragraph_format.keep_with_next=False
            FIGURES[name]['number']=nf;FIGURES[name]['caption']=caption
        elif chunk.startswith('@table '):
            _,key,caption=chunk.split(' ',2);nt+=1;table(doc,key,caption,nt)
        elif chunk.startswith('@equation '):
            ne+=1;equation(doc,chunk[len('@equation '):],ne)
        elif chunk=='@page':
            doc.add_page_break()
        elif chunk.startswith('# '):
            p=doc.add_paragraph(chunk[2:],style='Heading 1');no_number(p)
            p.paragraph_format.keep_with_next=True
        elif chunk.startswith('## '):
            p=doc.add_paragraph(chunk[3:],style='Heading 2');no_number(p)
            p.paragraph_format.keep_with_next=True
        elif chunk.startswith('### '):
            p=doc.add_paragraph(chunk[4:],style='Heading 3');no_number(p)
            p.paragraph_format.keep_with_next=True
        elif chunk.startswith('@note '):
            add_scientific_text(doc.add_paragraph(style='Scientific Note'),chunk[6:])
        elif re.match(r'^\[\d+\]',chunk):
            doc.add_paragraph(chunk,style='Scientific Reference')
        else:
            add_scientific_text(doc.add_paragraph(style='Body Text'),chunk.replace('\n',' '))
    path=OUT/'PSTMO_JOURNAL_VER4_PHAN_II_III.docx'
    # The original template ends the two-column body with a continuous
    # one-column section. Retain that device to balance the final page.
    from docx.enum.section import WD_SECTION_START
    tail=doc.add_section(WD_SECTION_START.CONTINUOUS)
    tail._sectPr.find(qn('w:cols')).set(qn('w:num'),'1')
    used={v for e in doc.element.iter() for k,v in e.attrib.items()
          if k.startswith('{http://schemas.openxmlformats.org/officeDocument/2006/relationships}')}
    removable={'header','footer','footnotes','endnotes','comments','image','hyperlink'}
    for rid,rel in list(doc.part.rels.items()):
        if rid not in used and rel.reltype.rsplit('/',1)[-1] in removable:
            del doc.part.rels[rid]
    fingerprint(Path(__file__).resolve())
    fingerprint(ROOT/'Viet_bao/journal/ver3/manuscript.md')
    fingerprint(ROOT/'Viet_bao/journal/ver3/PSTMO_JOURNAL_VER3_PHAN_II_III.docx')
    fingerprint(ROOT/'Viet_bao/bao_Rev_ecit_2026/ver5/REV_ECIT2026_PSTMO_VER5.docx')
    doc.save(path)
    report={'docx':path.name,'template':str(TEMPLATE.relative_to(ROOT)),
            'counts':{'figures':nf,'tables':nt,'equations':ne,'main_trials':125,'repeat_trials':250},
            'evidence':{'main_success':123,'repeated_success':243,'both_success_pairs':118,'blocks':25},
            'figures':FIGURES,'source_sha256':SOURCES,'docx_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
            'scope':'Sections II and III only; inherited IEEE conference A4 template, not venue-specific journal acceptance certification.'}
    (OUT/'provenance.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'output':str(path),'counts':report['counts']},ensure_ascii=False))


if __name__=='__main__':
    OUT.mkdir(parents=True,exist_ok=True);FIG.mkdir(exist_ok=True)
    figures();tables();manuscript()
