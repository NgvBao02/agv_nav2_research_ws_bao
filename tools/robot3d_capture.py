#!/usr/bin/env python3
"""Capture actual Gazebo/RViz windows, with saved camera configurations."""
import argparse, ctypes, json, math, os, re, signal, subprocess, tempfile, time
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[1]; OUT=ROOT/'docs/robot_3d_report'
SHOTS=OUT/'screenshots'; CONFIG=OUT/'rviz'; LOG=OUT/'logs'
for d in (SHOTS,CONFIG,LOG): d.mkdir(parents=True,exist_ok=True)

def window(pattern):
    txt=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
    found=[]
    for line in txt.splitlines():
        if re.search(pattern,line):
            m=re.search(r'(0x[0-9a-f]+).*? (\d+)x(\d+)\+',line)
            if m: found.append((int(m[2])*int(m[3]),m[1]))
    if not found: raise RuntimeError('Window not found '+pattern)
    return max(found)[1]

def resize(w):
    lib=ctypes.CDLL('libX11.so.6'); lib.XOpenDisplay.restype=ctypes.c_void_p
    display=lib.XOpenDisplay(None)
    lib.XMoveResizeWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_int,ctypes.c_uint,ctypes.c_uint]
    lib.XMoveResizeWindow(display,int(w,16),30,45,1700,940)
    lib.XRaiseWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong];lib.XRaiseWindow(display,int(w,16))
    lib.XSync.argtypes=[ctypes.c_void_p,ctypes.c_int];lib.XSync(display,0)
    lib.XCloseDisplay.argtypes=[ctypes.c_void_p];lib.XCloseDisplay(display)

def capture(w,name,metadata):
    with tempfile.TemporaryDirectory(prefix='robot3d_window_') as tmp:
        raw=Path(tmp)/'window.xwd'
        subprocess.run(['xwd','-silent','-id',w,'-out',str(raw)],check=True)
        subprocess.run(['ffmpeg','-y','-loglevel','error','-i',str(raw),str(SHOTS/(name+'.png'))],check=True)
    entry=dict(file='screenshots/'+name+'.png',captured_local=time.strftime('%Y-%m-%dT%H:%M:%S%z'),window=w,**metadata)
    with (OUT/'capture_manifest.jsonl').open('a') as f: f.write(json.dumps(entry)+'\n')
    print('Captured',name,flush=True)

def gazebo():
    w=window('"gz-sim-gui"');resize(w);time.sleep(1)
    cameras=[('gz_iso_front',(0.80,-.82,.66),(0,0,.09)),
             ('gz_iso_rear',(-.80,.82,.60),(0,0,.09)),
             ('gz_front',(1.10,0,.18),(0,0,.08)),
             ('gz_side',(0,-1.10,.22),(0,0,.07)),
             ('gz_top',(.001,0,1.35),(0,0,0)),
             ('gz_wheel_detail',(.33,-.52,.20),(0,-.10,.05)),
             ('gz_arena',(6,-6,7),(0,0,0))]
    for name,pos,target in cameras:
        if name!='gz_arena':
            pos=tuple(target[i]+.55*(pos[i]-target[i]) for i in range(3))
            pos=(pos[0]-3,pos[1]-2,pos[2]);target=(target[0]-3,target[1]-2,target[2])
        dx,dy,dz=[target[i]-pos[i] for i in range(3)]
        yaw=math.atan2(dy,dx);pitch=-math.atan2(dz,math.hypot(dx,dy))
        q=( -math.sin(yaw/2)*math.sin(pitch/2),math.cos(yaw/2)*math.sin(pitch/2),
            math.sin(yaw/2)*math.cos(pitch/2),math.cos(yaw/2)*math.cos(pitch/2))
        request='pose: {position: {x: %f y: %f z: %f} orientation: {x: %f y: %f z: %f w: %f}}'%(*pos,*q)
        result=subprocess.run(['gz','service','-s','/gui/move_to/pose','--reqtype','gz.msgs.GUICamera',
            '--reptype','gz.msgs.Boolean','--timeout','4000','--req',request],capture_output=True,text=True)
        if result.returncode or 'true' not in result.stdout: raise RuntimeError(result.stdout+result.stderr)
        time.sleep(2)
        capture(w,name,dict(application='Gazebo Sim 8',world='open_arena',camera=pos,target=target,service=request))

def rviz():
    base=yaml.safe_load((ROOT/'src/vacuum_robot_gazebo/rviz/urdf_check.rviz').read_text())
    views=[('rviz_iso',.55,.8,1.05,False,False),('rviz_rear',.45,3.9,1.05,False,False),
        ('rviz_top',1.5707,0,1.05,False,False),('rviz_front',.04,0,1.05,False,False),
        ('rviz_side',.07,-1.5708,1.05,False,False),('rviz_collision',.55,.8,1.05,True,False),
        ('rviz_tf',.55,.8,1.2,False,True),('rviz_scan',1.5707,0,17.,False,False)]
    for name,pitch,yaw,distance,collision,tf in views:
        if os.environ.get('ROBOT3D_CAPTURE_ONLY') and name!=os.environ['ROBOT3D_CAPTURE_ONLY']:continue
        c=yaml.safe_load(yaml.safe_dump(base));vm=c['Visualization Manager']
        vm['Views']['Current'].update(Pitch=pitch,Yaw=yaw,Distance=distance)
        vm['Displays'][1].update({'Collision Enabled':collision,'Visual Enabled':not collision})
        vm['Displays'][2].update({'Enabled':tf,'Show Names':tf,'Marker Scale':.22})
        if tf:
            vm['Displays'][1]['Alpha']=.28
            vm['Displays'][2]['Frames']={'All Enabled':False,**{n:{'Value':n in ['base_link','laser','left_wheel','right_wheel']} for n in ['base_link','base_footprint','laser','imu_link','left_motor','right_motor','left_wheel','right_wheel','odom']}}
            vm['Views']['Current']['Distance']=1.1
        if name=='rviz_scan':
            vm['Displays'].append({'Class':'rviz_default_plugins/LaserScan','Name':'LaserScan','Enabled':True,
                'Topic':{'Value':'/scan','Reliability Policy':'Best Effort','Durability Policy':'Volatile','Depth':5},
                'Size (m)':.04,'Style':'Points','Color Transformer':'FlatColor','Color':'255; 80; 20'})
        c['Window Geometry']={'Width':1700,'Height':940,'X':30,'Y':45}
        path=CONFIG/(name+'.rviz');path.write_text(yaml.safe_dump(c,sort_keys=False))
        with (LOG/(name+'.log')).open('w') as log:
            proc=subprocess.Popen(['python3',str(ROOT/'src/vacuum_robot_gazebo/scripts/sanitized_rviz.py'),'-d',str(path),
                '--ros-args','-p','use_sim_time:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                time.sleep(9)
                if proc.poll() is not None: raise RuntimeError('RViz exited: '+str(path))
                w=window('"rviz2"');resize(w);time.sleep(1)
                capture(w,name,dict(application='RViz2',configuration=str(path.relative_to(ROOT)),world='open_arena'))
            finally:
                os.killpg(proc.pid,signal.SIGINT)
                try: proc.wait(10)
                except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGTERM);proc.wait(5)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('kind',choices=['gazebo','rviz']);a=p.parse_args()
    globals()[a.kind]()
