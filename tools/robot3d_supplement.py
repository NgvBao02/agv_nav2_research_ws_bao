#!/usr/bin/env python3
import json,subprocess,hashlib,csv,math,sys,platform
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import xml.etree.ElementTree as ET
from robot3d_analysis import ROOT,OUT,FIG,COL,save,load

f,ax=plt.subplots(figsize=(10,5.8));ax.axis('off')
nodes=[(.08,.85,'STEP\n542 solid + mặt rời'),(.39,.85,'STL + URDF\nVisual / TF'),(.73,.85,'SDF\nVật lý / cảm biến'),
       (.08,.52,'/robot_description\nrobot_state_publisher'),(.39,.52,'/joint_states\n/odom + /tf'),(.73,.52,'/scan + /imu/data\nROS-Gazebo bridge'),
       (.08,.16,'RViz2\nQuan sát, KHÔNG mô phỏng lực'),(.39,.16,'AMCL + costmap\nNav2 planner/controller'),(.73,.16,'/cmd_vel\nGazebo DiffDrive')]
for x,y,t in nodes:ax.text(x,y,t,transform=ax.transAxes,ha='left',va='center',fontsize=10,bbox=dict(boxstyle='round,pad=.55',fc='#edf3f5',ec=COL[0]))
for a,b in [(0,1),(1,3),(2,4),(2,5),(3,6),(4,6),(5,7),(7,8),(8,2)]:
    x,y,_=nodes[a];xx,yy,_=nodes[b];ax.annotate('',xy=(xx+.08,yy+.08),xytext=(x+.08,y-.08),xycoords='axes fraction',arrowprops=dict(arrowstyle='->',color='#7c919b',connectionstyle='arc3,rad=.04'))
save(f,'model_pipeline')
f,axs=plt.subplots(1,2,figsize=(10,4.8));names=['truth','odom','imu','joints','scan'];labels=['GT','Odom','IMU','Joint','LiDAR'];rates=json.loads((OUT/'motion/rates.json').read_text())
axs[0].bar(labels,[1/rates[n]['mean_period_s'] for n in names],color=COL[0]);axs[0].set(ylabel='Hz từ 1 / khoảng cách mẫu trung bình',title='Tốc độ ghi nhận, có cả các pha reset');axs[0].set_yscale('log')
for n,label in [('truth','Ground truth'),('imu','IMU')]:
    d=load(n);dt=np.diff(d['t'])*1000;dt=dt[(dt>0)&(dt<50)];axs[1].hist(dt,bins=np.arange(0,51,1.5),histtype='step',lw=1.6,label=label)
axs[1].set(xlabel='Khoảng cách timestamp (ms)',ylabel='Số khoảng',title='Lượng tử hóa theo bước vật lý 3 ms');axs[1].legend();save(f,'sample_rates')
f,axs=plt.subplots(1,2,figsize=(10,4.8));support=np.array([[-.15,-.09],[0,-.1274],[.15,-.09],[.15,.09],[0,.1274],[-.15,.09]])
axs[0].add_patch(Polygon(support,fc='#d4ebeb',ec=COL[0]));axs[0].plot(*np.r_[support,support[:1]].T,'o',color=COL[0]);axs[0].scatter([-.005588],[-.000493],c=COL[1],marker='x',s=80,label='COM SDF')
axs[0].set(xlim=(-.24,.24),ylim=(-.19,.19),aspect='equal',xlabel='x (m)',ylabel='y (m)',title='Đa giác tiếp xúc danh định');axs[0].legend()
payload=np.linspace(0,3,100)
for z in (.10,.20,.30):axs[1].plot(payload,(5*.05724066944+payload*z)/(5+payload)*1000,label=f'Cao độ tải giả định {z:.2f} m')
axs[1].set(xlabel='Khối lượng thêm GIẢ ĐỊNH (kg)',ylabel='Cao độ COM tổng (mm)',title='Ví dụ giải tích, không xác nhận tải cho phép');axs[1].legend(fontsize=8);save(f,'support_payload')
d=json.loads((OUT/'warehouse_demo.json').read_text());gt=np.array(d['truth']);f,axs=plt.subplots(2,1,figsize=(10,7),gridspec_kw={'height_ratios':[2.2,1]})
world=ET.parse(ROOT/'src/vacuum_robot_gazebo/worlds/warehouse_cross_aisles.sdf').getroot()
from matplotlib.patches import Rectangle
for co in world.findall('.//collision'):
    box=co.find('geometry/box/size')
    if box is None:continue
    dims=np.fromstring(box.text,sep=' ');pos=np.fromstring(co.findtext('pose','0 0 0 0 0 0'),sep=' ')
    if dims[2]<.1:continue
    axs[0].add_patch(Rectangle((pos[0]-dims[0]/2,pos[1]-dims[1]/2),dims[0],dims[1],fc='#9ba9b4',alpha=.8))
axs[0].plot(gt[:,1],gt[:,2],color=COL[0],lw=2,label='Ground truth');axs[0].scatter([gt[0,1]],[gt[0,2]],color=COL[1],label='Bắt đầu');axs[0].scatter([5],[0],marker='*',s=100,color=COL[2],label='Goal map (quy ước trùng world)')
axs[0].set(xlim=(-6,6),ylim=(-4,4),aspect='equal',xlabel='world x (m)',ylabel='world y (m)',title='Chạy minh họa Nav2 / ThetaStar');axs[0].legend(fontsize=8,loc='upper center')
axs[1].plot(gt[:,0]-gt[0,0],gt[:,2]*1000,color=COL[0],label='Lệch ngang so với y = 0');axs[1].set(xlabel='Thời gian kể từ mẫu đầu (s)',ylabel='y ground truth (mm)');axs[1].legend(fontsize=8)
save(f,'warehouse_trajectory')
stats=dict(status=d['status'],recorded_duration_s=float(gt[-1,0]-gt[0,0]),path_length_m=float(np.linalg.norm(np.diff(gt[:,1:3],axis=0),axis=1).sum()),goal_error_m=float(np.linalg.norm(gt[-1,1:3]-[5,0])),end_yaw_deg=float(np.degrees(gt[-1,3])),samples=len(gt),note='Duration includes pre-action discovery and post-action observations, not action-only latency. Goal error assumes map/world alignment of generated warehouse.')
(OUT/'warehouse_metrics.json').write_text(json.dumps(stats,indent=2))
tests=subprocess.run([sys.executable,'-m','pytest',str(ROOT/'src/vacuum_robot_gazebo/test/test_robot_description.py'),'-q'],capture_output=True,text=True)
(OUT/'logs/robot_description_tests.txt').write_text(tests.stdout+tests.stderr)
snapshot=dict(python=sys.version,platform=platform.platform(),test_exit_code=tests.returncode,git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
    sources=[])
paths=[ROOT/'file_3D/Xe.step']+list((ROOT/'tools').glob('robot3d_*.py'))
paths+=list((ROOT/'src/vacuum_robot_gazebo').glob('config/*.yaml'))
paths+=[ROOT/'src/vacuum_robot_gazebo/urdf/vacuum_robot.urdf',ROOT/'src/vacuum_robot_gazebo/models/vacuum_robot/model.sdf',ROOT/'src/vacuum_robot_gazebo/launch/simulation.launch.py']
paths+=list((ROOT/'src/vacuum_robot_gazebo/models/vacuum_robot/meshes').glob('*.stl'))
paths+=[ROOT/'src/vacuum_robot_gazebo/worlds/open_arena.sdf',ROOT/'src/vacuum_robot_gazebo/worlds/warehouse_cross_aisles.sdf']
for p in paths:snapshot['sources'].append(dict(path=str(p.relative_to(ROOT)),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),bytes=p.stat().st_size))
(OUT/'provenance.json').write_text(json.dumps(snapshot,indent=2))
print(stats,flush=True)
