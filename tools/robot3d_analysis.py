#!/usr/bin/env python3
"""Compute auditable maneuver metrics and engineering figures from local data."""
import csv,json,math,subprocess,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle,Polygon
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'docs/robot_3d_report';FIG=OUT/'figures'
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,'grid.alpha':.2,'figure.facecolor':'white','axes.titleweight':'bold'})
COL=['#087f8c','#e78631','#7654ac'];LABEL={'forward':'Tiến thẳng','reverse':'Lùi thẳng','pivot_ccw':'Quay trái tại chỗ','pivot_cw':'Quay phải tại chỗ','arc_left':'Chạy cung trái','arc_right':'Chạy cung phải','brake':'Dừng từ 0,30 m/s'}
def save(fig,name):
    fig.tight_layout(pad=1.4);fig.savefig(FIG/(name+'.png'),dpi=210,bbox_inches='tight');plt.close(fig)
def local_pose(d):
    y=np.unwrap(d['yaw']);c,s=np.cos(y[0]),np.sin(y[0]);dx=d['x']-d['x'][0];dy=d['y']-d['y'][0]
    return np.column_stack((c*dx+s*dy,-s*dx+c*dy)),y-y[0]
def load(key):return np.genfromtxt(OUT/'motion'/(key+'.csv'),delimiter=',',names=True,dtype=None,encoding='utf-8')

def motion():
    trials=json.loads((OUT/'motion/trials.json').read_text());data={k:load(k) for k in ['truth','odom','imu','joints','scan','commands']}
    metrics=[];detailed={}
    for tr in trials:
        k=tr['trial'];a={key:d[d['trial']==k] for key,d in data.items()};g=a['truth'];o=a['odom'];t=g['t']-tr['command_start']
        xy,ang=local_pose(g);oxy,oang=local_pose(o)
        vel=np.gradient(xy,g['t'],axis=0);speed=np.linalg.norm(vel,axis=1)
        signed=vel[:,0]*np.cos(ang)+vel[:,1]*np.sin(ang);omega=np.gradient(ang,g['t'])
        oi=np.column_stack([np.interp(g['t'],o['t'],oxy[:,j]) for j in range(2)]);oa=np.interp(g['t'],o['t'],oang)
        err=np.linalg.norm(xy-oi,axis=1);ae=ang-oa
        steady=(g['t']>=tr['command_start']+1.5)&(g['t']<=tr['command_stop']-.3)
        braking=g['t']>=tr['command_stop'];stopidx=np.flatnonzero(braking & (speed<.01) & (np.abs(omega)<.02))
        stop_time=float(g['t'][stopidx[0]]-tr['command_stop']) if len(stopidx) else None
        # Discrete path length after zero command, interpolating exact command stop.
        stopxy=np.array([np.interp(tr['command_stop'],g['t'],xy[:,j]) for j in range(2)])
        after=np.vstack([stopxy,xy[braking]])
        metric=dict(trial=k,case=tr['case'],repeat=tr['repeat'],samples=len(g),
            path_length_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),
            endpoint_x_m=float(xy[-1,0]),endpoint_y_m=float(xy[-1,1]),yaw_change_rad=float(ang[-1]),
            steady_v_mps=float(signed[steady].mean()),steady_w_radps=float(omega[steady].mean()),
            odom_endpoint_error_m=float(err[-1]),odom_yaw_endpoint_error_rad=float(ae[-1]),
            odom_xy_rmse_m=float(np.sqrt(np.mean(err**2))),
            stopping_distance_m=float(np.linalg.norm(np.diff(after,axis=0),axis=1).sum()),stopping_time_s=stop_time,
            max_lateral_speed_mps=float(np.max(np.abs(-vel[:,0]*np.sin(ang)+vel[:,1]*np.cos(ang)))))
        metrics.append(metric);detailed[k]=(tr,a,t,xy,ang,signed,omega,err,ae)
    with (OUT/'motion/metrics.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(metrics[0]));w.writeheader();w.writerows(metrics)
    summary=[]
    for case,label in LABEL.items():
        ms=[m for m in metrics if m['case']==case];sm=dict(case=case,n=len(ms))
        for key in ['path_length_m','yaw_change_rad','steady_v_mps','steady_w_radps','odom_endpoint_error_m','odom_yaw_endpoint_error_rad','stopping_distance_m','stopping_time_s']:
            vals=[m[key] for m in ms if m[key] is not None];sm[key]={'mean':float(np.mean(vals)),'sd':float(np.std(vals,ddof=1))}
        summary.append(sm)
        f,axs=plt.subplots(2,2,figsize=(10,7.2))
        for rep in range(1,4):
            tr,a,t,xy,ang,v,w,err,ae=detailed[f'{case}_{rep}'];co=COL[rep-1]
            axs[0,0].plot(t,v,color=co,label=f'Lần {rep}')
            axs[0,1].plot(t,w,color=co)
            axs[1,0].plot(t,ang,color=co)
            axs[1,1].plot(xy[:,0],xy[:,1],color=co,label=f'Lần {rep}')
        axs[0,0].step([-1,0,tr['nominal_duration'],tr['nominal_duration']+3],[0,tr['v'],0,0],where='post',ls='--',color='#323e4b',label='Lệnh')
        axs[0,1].step([-1,0,tr['nominal_duration'],tr['nominal_duration']+3],[0,tr['w'],0,0],where='post',ls='--',color='#323e4b')
        for ax in axs.flat[:3]:ax.set_xlabel('Thời gian từ lúc phát lệnh (s)');ax.axvline(tr['nominal_duration'],ls=':',color='gray')
        axs[0,0].set_ylabel('v từ ground truth (m/s)');axs[0,1].set_ylabel('ω từ ground truth (rad/s)')
        axs[1,0].set_ylabel('Góc quay tích lũy (rad)');axs[1,1].set(xlabel='x cục bộ (m)',ylabel='y cục bộ (m)',aspect='equal')
        if tr['w']==0:
            axs[0,1].set_ylim(-.005,.005);axs[1,0].set_ylim(-.005,.005)
            axs[0,1].text(.03,.9,'Gần zero; nhiễu số ~10⁻¹¹',transform=axs[0,1].transAxes,fontsize=9)
            axs[1,1].set_ylim(-.22,.22)
        if tr['v']==0:
            axs[0,0].set_ylim(-.005,.005)
        axs[0,0].legend(fontsize=9);f.suptitle(label+' - đáp ứng động học, n = 3',fontsize=16)
        save(f,'motion_'+case)
        f,axs=plt.subplots(2,2,figsize=(10,7.2))
        for rep in range(1,4):
            tr,a,t,xy,ang,v,w,err,ae=detailed[f'{case}_{rep}'];co=COL[rep-1]
            axs[0,0].plot(t,err*1000,color=co,label=f'Lần {rep}');axs[0,1].plot(t,np.degrees(ae),color=co)
            j=a['joints'];axs[1,0].plot(j['t']-tr['command_start'],j['left_vel'],color=co,lw=1)
            axs[1,0].plot(j['t']-tr['command_start'],j['right_vel'],color=co,lw=1,ls='--')
            im=a['imu'];axs[1,1].plot(im['t']-tr['command_start'],im['wz'],color=co,lw=.6,alpha=.7)
        for ax in axs.flat:ax.set_xlabel('Thời gian từ lúc phát lệnh (s)');ax.axvline(tr['nominal_duration'],ls=':',color='gray')
        axs[0,0].set_ylabel('Sai lệch odom - GT (mm)');axs[0,1].set_ylabel('Δyaw GT - odom (độ)')
        axs[1,0].set_ylabel('Tốc độ bánh (rad/s)');axs[1,1].set_ylabel('IMU gyro z (rad/s)')
        axs[1,0].set_title('Nét liền: trái; nét đứt: phải',fontsize=10);axs[0,0].legend(fontsize=9)
        if tr['w']==0:
            axs[0,0].set_ylim(0,.01);axs[0,1].set_ylim(-.001,.001)
            axs[0,0].text(.03,.88,'Lệch < 0,001 mm trong mô phỏng',transform=axs[0,0].transAxes,fontsize=8)
        f.suptitle(label+' - odometry, bánh xe và IMU',fontsize=16);save(f,'sensors_'+case)
    (OUT/'motion/summary.json').write_text(json.dumps(summary,indent=2))
    rates={}
    for key,d in data.items():
        dt=np.diff(d['t']);dt=dt[dt>1e-8]
        rates[key]=dict(samples=len(d),median_hz=float(1/np.median(dt)),mean_period_s=float(np.mean(dt)),p95_period_s=float(np.percentile(dt,95)))
    (OUT/'motion/rates.json').write_text(json.dumps(rates,indent=2))
    f,axs=plt.subplots(1,2,figsize=(10,4.6));xs=np.arange(len(summary))
    axs[0].bar(xs,[s['odom_endpoint_error_m']['mean']*1000 for s in summary],color=COL[0]);axs[0].set_ylabel('Sai lệch vị trí cuối (mm)')
    axs[1].bar(xs,[np.degrees(s['odom_yaw_endpoint_error_rad']['mean']) for s in summary],color=COL[1]);axs[1].set_ylabel('Δyaw cuối GT - odom (độ)')
    for ax in axs:ax.set_xticks(xs,[LABEL[s['case']] for s in summary],rotation=35,ha='right',fontsize=8)
    save(f,'odom_summary')
    f,axs=plt.subplots(1,2,figsize=(10,4.8))
    for case in ('forward','reverse','arc_left','arc_right'):
        tr,a,t,xy,*_=detailed[case+'_1'];axs[0].plot(xy[:,0],xy[:,1],label=LABEL[case])
    axs[0].set(xlabel='x cục bộ (m)',ylabel='y cục bộ (m)',aspect='equal');axs[0].legend(fontsize=9)
    sc=json.loads((OUT/'motion/scan_snapshot.json').read_text());angles=sc['angle_min']+np.arange(len(sc['ranges']))*sc['angle_increment'];ran=np.array([np.nan if x is None else x for x in sc['ranges']])
    axs[1].scatter(ran*np.cos(angles),ran*np.sin(angles),s=3,c=COL[1]);axs[1].scatter([0],[0],c=COL[0]);axs[1].set(xlabel='x trong frame laser (m)',ylabel='y trong frame laser (m)',aspect='equal',title='Một bản tin /scan thực tế')
    save(f,'paths_scan')
    print('Analyzed',len(metrics),'trials',rates,flush=True)

def engineering():
    fig,axs=plt.subplots(1,2,figsize=(10,5.3))
    ax=axs[0];ax.add_patch(Rectangle((-.22,-.17),.44,.34,fc='#cbd7df',ec=COL[0],lw=2))
    for y in (-.1274,.1274):ax.add_patch(Rectangle((-.0425,y-.015),.085,.03,fc='#253c4d'))
    for x in (-.15,.15):
        for y in (-.09,.09):ax.add_patch(Circle((x,y),.006,color=COL[1]))
    ax.annotate('',(-.22,-.22),(.22,-.22),arrowprops=dict(arrowstyle='<->'));ax.text(0,-.245,'440 mm',ha='center')
    ax.annotate('',(.27,-.17),(.27,.17),arrowprops=dict(arrowstyle='<->'));ax.text(.30,0,'340 mm',rotation=90,va='center')
    ax.scatter([0],[0],s=30,c='#b51c44');ax.text(.015,.012,'base_link',fontsize=9)
    ax.set(xlim=(-.30,.36),ylim=(-.29,.24),aspect='equal',xlabel='x (m)',ylabel='y (m)',title='Footprint và bố trí tiếp xúc')
    ax=axs[1];ax.add_patch(Rectangle((-.22,.06),.44,.10,fc='#cbd7df',ec=COL[0]))
    ax.add_patch(Circle((0,.0425),.0425,color='#253c4d'));ax.axhline(0,c='black')
    ax.axhline(.15142,c=COL[1],ls='--',label='Mặt phẳng laser: 151,42 mm')
    ax.scatter([0],[.0297],c=COL[2],label='IMU: 29,70 mm',zorder=4)
    ax.scatter([-.005588],[.057241],c='red',marker='x',label='COM toàn mô hình SDF')
    ax.set(xlim=(-.29,.29),ylim=(-.015,.23),aspect='equal',xlabel='x (m)',ylabel='Cao độ so với sàn (m)',title='Cao độ danh định - không tải thêm')
    ax.legend(loc='upper center',fontsize=8);save(fig,'geometry_layout')
    f,ax=plt.subplots(figsize=(10,5.3));ax.axis('off')
    labels=['base_footprint\nz = -42,50 mm','laser\nz = +108,92 mm','imu_link\nz = -12,80 mm','left_motor\ny = +93,40 mm','right_motor\ny = -93,40 mm','left_wheel\ny = +127,40 mm','right_wheel\ny = -127,40 mm']
    ax.text(.04,.5,'base_link\n(root)',ha='center',va='center',bbox=dict(boxstyle='round,pad=.8',fc='#dceef0',ec=COL[0]),transform=ax.transAxes)
    for i,l in enumerate(labels):
        yy=.94-i*.143;ax.annotate(l,xy=(.13,.5),xytext=(.50,yy),xycoords='axes fraction',textcoords='axes fraction',va='center',bbox=dict(boxstyle='round,pad=.35',fc='#eef2f6',ec='#afbfc9'),arrowprops=dict(arrowstyle='<-',color=COL[0]))
        ax.text(.86,yy,'continuous' if i>4 else 'fixed',transform=ax.transAxes,color=COL[1] if i>4 else '#536575',va='center')
    save(f,'tf_tree')
    f,axs=plt.subplots(1,2,figsize=(10,4.8));w=np.linspace(-.8,.8,401)
    for b,lab,co in [(.2548,'b hình học = 0,2548 m',COL[0]),(.2834,'b hiệu dụng = 0,2834 m',COL[1])]:
        vmax=np.minimum(.30,.36-abs(w)*b/2)
        axs[0].plot(w,vmax,label=lab,color=co);axs[0].plot(w,-vmax,color=co)
    axs[0].set(xlabel='ω (rad/s)',ylabel='v (m/s)',title='Miền ghép giới hạn: |v bánh| ≤ 0,36 m/s');axs[0].legend(fontsize=8)
    v=np.linspace(.01,.30,150)
    axs[1].plot(v,v/.8,label='Từ |ω| ≤ 0,80',color=COL[0]);axs[1].plot(v,v*v/.18,label='Từ a ngang ≤ 0,18',color=COL[1]);axs[1].axhline(.35,ls='--',color=COL[2],label='SmacHybrid: 0,35 m')
    axs[1].set(xlabel='v (m/s)',ylabel='Bán kính cung (m)',title='Các ràng buộc KHÁC NHAU');axs[1].legend(fontsize=8);save(f,'velocity_envelope')
    f,axs=plt.subplots(1,2,figsize=(10,5.2));corn=np.array([[.22,.17],[.22,-.17],[-.22,-.17],[-.22,.17]])
    for angle in np.linspace(0,np.pi/2,10):
        R=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]]);axs[0].add_patch(Polygon(corn@R.T,fill=False,ec=COL[0],alpha=.45))
    radius=np.hypot(.22,.17);axs[0].add_patch(Circle((0,0),radius,fill=False,ec=COL[1],lw=2,ls='--'))
    axs[0].set(xlim=(-.34,.34),ylim=(-.34,.34),aspect='equal',xlabel='x (m)',ylabel='y (m)',title=f'Quay tại chỗ: bán kính quét {radius*1000:.1f} mm')
    theta=np.linspace(0,np.pi/2,120);R=.50;path=np.c_[R*np.sin(theta),R*(1-np.cos(theta))]
    axs[1].plot(*path.T,c=COL[1],lw=2)
    for k in np.linspace(0,len(theta)-1,9).astype(int):
        a=theta[k];rot=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]]);axs[1].add_patch(Polygon(corn@rot.T+path[k],fill=False,ec=COL[0],alpha=.5))
    axs[1].set(aspect='equal',xlabel='x (m)',ylabel='y (m)',title='Ví dụ hình học: cung R = 0,50 m');axs[1].autoscale();save(f,'swept_footprint')
    f,axs=plt.subplots(1,2,figsize=(10,4.5));speed=np.linspace(0,.3,150)
    for a in (.35,.45):axs[0].plot(speed,speed**2/(2*a),label=f'|a| = {a:.2f} m/s²')
    axs[0].plot(speed,speed**2/.9+speed*.25,ls='--',label='a = 0,45; trễ giả định 0,25 s')
    axs[0].set(xlabel='Vận tốc ban đầu (m/s)',ylabel='Quãng dừng lý tưởng (m)',title='Mô hình giải tích, KHÔNG là chứng nhận');axs[0].legend(fontsize=8)
    b=np.array([.2548,.2834]);axs[1].bar(['Hình học','Hiệu dụng'],b*1000,color=COL[:2]);axs[1].set(ylabel='Khoảng cách bánh (mm)',title='Không đổi hình học khi hiệu chỉnh odom')
    for i,v in enumerate(b):axs[1].text(i,v*1000+4,f'{v*1000:.1f}',ha='center')
    axs[1].set_ylim(0,320);save(f,'braking_calibration')
    s=ET.parse(ROOT/'src/vacuum_robot_gazebo/models/vacuum_robot/model.sdf').getroot().find('model')
    items=[]
    for link in s.findall('link'):
        inert=link.find('inertial')
        if inert is None:continue
        i=inert.find('inertia');I=np.array([[float(i.findtext('i'+a+b)) if i.find('i'+a+b) is not None else float(i.findtext('i'+b+a)) for b in 'xyz'] for a in 'xyz'])
        pos=np.fromstring(link.findtext('pose','0 0 0 0 0 0'),sep=' ')[:3];center=np.fromstring(inert.findtext('pose','0 0 0 0 0 0'),sep=' ')[:3]+pos
        items.append(dict(link=link.get('name'),mass=float(inert.findtext('mass')),COM_floor_m=center.tolist(),inertia=I.tolist(),eigenvalues=np.linalg.eigvalsh(I).tolist()))
    mass=sum(p['mass'] for p in items);com=sum(p['mass']*np.array(p['COM_floor_m']) for p in items)/mass
    I=np.zeros((3,3))
    for p in items:
        d=np.array(p['COM_floor_m'])-com;I+=np.array(p['inertia'])+p['mass']*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
    out=dict(parts=items,total_mass_kg=mass,COM_floor_m=com.tolist(),total_inertia_at_COM=I.tolist(),principal_inertias=np.linalg.eigvalsh(I).tolist())
    (OUT/'mass_properties.json').write_text(json.dumps(out,indent=2))
    f,axs=plt.subplots(1,2,figsize=(10,4.7));axs[0].bar(['Thân','Bánh trái','Bánh phải'],[p['mass'] for p in items],color=[COL[0],COL[1],COL[2]]);axs[0].set(ylabel='Khối lượng khai báo (kg)',title='SDF: tổng 5,00 kg')
    axs[1].bar(['I₁','I₂','I₃'],out['principal_inertias'],color=COL);axs[1].set(ylabel='Quán tính chính (kg·m²)',title='Đã quy về COM toàn robot');save(f,'mass_inertia')
    print('Engineering properties',out,flush=True)
if __name__=='__main__':
    engineering()
    if '--engineering-only' not in sys.argv:motion()
