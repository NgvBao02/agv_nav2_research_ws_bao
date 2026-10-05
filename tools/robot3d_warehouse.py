#!/usr/bin/env python3
"""Collect integration screenshots and one actual Nav2 traversal in cross aisles."""
import json,math,os,signal,subprocess,time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry,Path as NavPath
from geometry_msgs.msg import Twist
from robot3d_capture import ROOT,OUT,SHOTS,CONFIG,LOG,window,resize,capture

def camera(name,pos,target):
    dx,dy,dz=[target[i]-pos[i] for i in range(3)];yaw=math.atan2(dy,dx);pitch=-math.atan2(dz,math.hypot(dx,dy))
    q=(-math.sin(yaw/2)*math.sin(pitch/2),math.cos(yaw/2)*math.sin(pitch/2),math.sin(yaw/2)*math.cos(pitch/2),math.cos(yaw/2)*math.cos(pitch/2))
    req='pose: {position: {x: %f y: %f z: %f} orientation: {x: %f y: %f z: %f w: %f}}'%(*pos,*q)
    subprocess.run(['gz','service','-s','/gui/move_to/pose','--reqtype','gz.msgs.GUICamera','--reptype','gz.msgs.Boolean','--timeout','4000','--req',req],check=True,capture_output=True)
    time.sleep(2);w=window('"gz-sim-gui"');resize(w);time.sleep(1)
    capture(w,name,dict(application='Gazebo Sim 8',world='warehouse_cross_aisles',camera=pos,target=target))

def config(name,local=False):
    c=yaml.safe_load((ROOT/'src/vacuum_robot_gazebo/rviz/urdf_check.rviz').read_text());vm=c['Visualization Manager']
    vm['Global Options']['Fixed Frame']='map';vm['Displays'][0]['Cell Size']=1.;vm['Displays'][0]['Plane Cell Count']=20
    vm['Displays'][0]['Reference Frame']='<Fixed Frame>';vm['Displays'][2]['Enabled']=False
    def topic(t):return {'Value':t,'Reliability Policy':'Reliable','Durability Policy':'Transient Local','Depth':1}
    vm['Displays']+=[{'Class':'rviz_default_plugins/Map','Name':'Map kho giao cat','Enabled':True,'Topic':topic('/map'),'Alpha':.85,'Color Scheme':'map'},
        {'Class':'rviz_default_plugins/Map','Name':'Global costmap','Enabled':True,'Topic':topic('/global_costmap/costmap'),'Alpha':.40,'Color Scheme':'costmap'},
        {'Class':'rviz_default_plugins/LaserScan','Name':'LaserScan','Enabled':True,'Topic':{'Value':'/scan','Reliability Policy':'Best Effort','Durability Policy':'Volatile','Depth':5},'Size (m)':.035,'Style':'Points','Color Transformer':'FlatColor','Color':'255; 80; 20'},
        {'Class':'rviz_default_plugins/Path','Name':'Nav2 global plan','Enabled':True,'Topic':{'Value':'/plan','Reliability Policy':'Reliable','Durability Policy':'Volatile','Depth':5},'Color':'0; 180; 230','Line Style':'Lines','Line Width':.05},
        {'Class':'rviz_default_plugins/Polygon','Name':'Footprint','Enabled':True,'Topic':{'Value':'/local_costmap/published_footprint','Reliability Policy':'Reliable','Durability Policy':'Volatile','Depth':5},'Color':'255; 210; 0'}]
    vm['Views']['Current'].update(Pitch=1.5707,Yaw=0.,Distance=18. if not local else 2.2,
        **{'Target Frame':'<Fixed Frame>' if not local else 'base_link','Focal Point':{'X':0.,'Y':0.,'Z':0.}})
    c['Window Geometry']={'Width':1700,'Height':940,'X':30,'Y':45}
    p=CONFIG/(name+'.rviz');p.write_text(yaml.safe_dump(c,sort_keys=False));return p

class Demo(Node):
    def __init__(self):
        super().__init__('robot3d_warehouse_demo',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.rows=[];self.plans=[];self.latest=None
        self.create_subscription(Odometry,'/ground_truth/odom',self.odom,qos_profile_sensor_data)
        self.create_subscription(NavPath,'/plan',self.plan,10)
        self.client=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.selector=self.create_publisher(String,'/planner_selector',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
    def odom(self,m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        self.latest=[m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z)]
        self.rows.append(self.latest)
    def plan(self,m):self.plans.append(dict(frame=m.header.frame_id,points=[[p.pose.position.x,p.pose.position.y] for p in m.poses]))

def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='184' and os.environ.get('GZ_PARTITION')=='robot3d_warehouse'
    previous=OUT/'warehouse_demo.json'
    if previous.exists() and not (OUT/'warehouse_demo_attempt1.json').exists():previous.rename(OUT/'warehouse_demo_attempt1.json')
    camera('gz_warehouse_overview',(9,-10,12),(0,0,0));camera('gz_warehouse_top',(.001,0,15),(0,0,0));camera('gz_warehouse_robot',(-4.60,-.4,.40),(-5,0,.09))
    for local in (True,False):
        name='rviz_warehouse_local' if local else 'rviz_warehouse_overview';p=config(name,local)
        with (LOG/(name+'.log')).open('w') as log:
            proc=subprocess.Popen(['python3',str(ROOT/'src/vacuum_robot_gazebo/scripts/sanitized_rviz.py'),'-d',str(p),'--ros-args','-p','use_sim_time:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                time.sleep(10);w=window('"rviz2"');resize(w);time.sleep(1)
                capture(w,name,dict(application='RViz2',world='warehouse_cross_aisles',configuration=str(p.relative_to(ROOT))))
                if not local:
                    rclpy.init();n=Demo();n.client.wait_for_server(timeout_sec=30)
                    n.selector.publish(String(data='ThetaStar'))
                    for _ in range(30):rclpy.spin_once(n,timeout_sec=.1)
                    goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=n.get_clock().now().to_msg();goal.pose.pose.position.x=5.;goal.pose.pose.orientation.w=1.
                    start=n.latest;future=n.client.send_goal_async(goal);rclpy.spin_until_future_complete(n,future,timeout_sec=20)
                    handle=future.result();assert handle and handle.accepted
                    result=handle.get_result_async();deadline=time.monotonic()+240;captured=False
                    while not result.done() and time.monotonic()<deadline:
                        rclpy.spin_once(n,timeout_sec=.1)
                        if n.latest and n.latest[1]>-1. and not captured:
                            capture(w,'rviz_warehouse_navigation',dict(application='RViz2',world='warehouse_cross_aisles',truth_sample=n.latest));captured=True
                    if not result.done():
                        handle.cancel_goal_async();status='timeout'
                    else:status=result.result().status
                    for _ in range(50):rclpy.spin_once(n,timeout_sec=.05)
                    capture(w,'rviz_warehouse_arrival',dict(application='RViz2',world='warehouse_cross_aisles',action_status=status,truth_sample=n.latest))
                    (OUT/'warehouse_demo.json').write_text(json.dumps(dict(start=start,end=n.latest,status=status,planner='ThetaStar (selected on /planner_selector)',goal_map=[5,0,0],truth=n.rows,plans=n.plans),indent=2))
                    print('Warehouse action status',status,'end',n.latest,flush=True);n.destroy_node();rclpy.shutdown()
            finally:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(8)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(5)
if __name__=='__main__':main()
