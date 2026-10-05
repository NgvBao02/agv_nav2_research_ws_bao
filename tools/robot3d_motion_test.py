#!/usr/bin/env python3
"""Isolated simulation-only step responses; raw streams and trial manifest."""
import csv,json,math,os,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu,JointState,LaserScan
from rosgraph_msgs.msg import Clock
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'docs/robot_3d_report/motion'
OUT.mkdir(parents=True,exist_ok=True)

def stamp(s): return s.sec+s.nanosec*1e-9
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class Recorder(Node):
    def __init__(self):
        super().__init__('robot3d_measurement',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.sim=None;self.trial='warmup';self.files={};self.writers={};self.counts={};self.last={}
        self.create_subscription(Clock,'/clock',lambda m:setattr(self,'sim',stamp(m.clock)),10)
        for topic,key in [('/ground_truth/odom','truth'),('/odom','odom')]:
            self.create_subscription(Odometry,topic,lambda m,k=key:self.odom(m,k),qos_profile_sensor_data)
        self.create_subscription(Imu,'/imu/data',self.imu,qos_profile_sensor_data)
        self.create_subscription(JointState,'/joint_states',self.joints,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
    def row(self,key,fields,vals):
        if key not in self.files:
            self.files[key]=(OUT/(key+'.csv')).open('w');self.writers[key]=csv.writer(self.files[key]);self.writers[key].writerow(['trial']+fields)
        self.writers[key].writerow([self.trial]+vals);self.counts[key]=self.counts.get(key,0)+1
    def odom(self,m,key):
        p=m.pose.pose.position;q=m.pose.pose.orientation;t=m.twist.twist
        self.row(key,['t','x','y','z','yaw','vx','vy','wz'],[stamp(m.header.stamp),p.x,p.y,p.z,yaw(q),t.linear.x,t.linear.y,t.angular.z])
        self.last[key]=[stamp(m.header.stamp),p.x,p.y,yaw(q)]
    def imu(self,m):
        self.row('imu',['t','ax','ay','az','wx','wy','wz','yaw'],[stamp(m.header.stamp),m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,yaw(m.orientation)])
    def joints(self,m):
        d={n:i for i,n in enumerate(m.name)}
        def val(arr,n): return arr[d[n]] if n in d and len(arr)>d[n] else float('nan')
        self.row('joints',['t','left_pos','right_pos','left_vel','right_vel'],[stamp(m.header.stamp)]+[val(a,n) for a in (m.position,m.velocity) for n in ('left_wheel_joint','right_wheel_joint')])
    def scan(self,m):
        vals=[v for v in m.ranges if math.isfinite(v)]
        self.row('scan',['t','n','finite','min','max','angle_min','angle_max','angle_increment'],[stamp(m.header.stamp),len(m.ranges),len(vals),min(vals,default=float('nan')),max(vals,default=float('nan')),m.angle_min,m.angle_max,m.angle_increment])
        if not (OUT/'scan_snapshot.json').exists():
            (OUT/'scan_snapshot.json').write_text(json.dumps(dict(t=stamp(m.header.stamp),frame=m.header.frame_id,angle_min=m.angle_min,angle_increment=m.angle_increment,ranges=[x if math.isfinite(x) else None for x in m.ranges])))
    def command(self,v,w):
        m=Twist();m.linear.x=v;m.angular.z=w;self.pub.publish(m)
        self.row('commands',['t','v','w'],[self.sim,v,w])
    def interval(self,duration,v=0.,w=0.):
        start=self.sim;last=-1
        deadline=time.monotonic()+max(30,duration*15)
        while self.sim-start<duration:
            rclpy.spin_once(self,timeout_sec=.01)
            if self.sim-last>=.05:self.command(v,w);last=self.sim
            if time.monotonic()>deadline: raise RuntimeError('Simulation clock stalled')
    def close(self):
        self.command(0.,0.)
        for f in self.files.values():f.close()

def main():
    if os.environ.get('ROS_DOMAIN_ID')!='183' or os.environ.get('GZ_PARTITION')!='robot3d_report':
        raise RuntimeError('Must use isolated report simulation domain/partition')
    rclpy.init();n=Recorder();deadline=time.monotonic()+30
    while n.sim is None or 'truth' not in n.last:
        rclpy.spin_once(n,timeout_sec=.1)
        if time.monotonic()>deadline: raise RuntimeError('No simulation ground truth')
    cases=[('forward',.20,0.,5.),('reverse',-.20,0.,5.),('pivot_ccw',0.,.50,5.),('pivot_cw',0.,-.50,5.),
           ('arc_left',.15,.30,6.),('arc_right',.15,-.30,6.),('brake',.30,0.,4.)]
    manifest=[]
    try:
        for rep in range(1,4):
            for name,v,w,duration in cases:
                n.trial='reset';n.interval(1.)
                cmd=['gz','service','-s','/world/open_arena/set_pose','--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','3000','--req',
                     'name: "vacuum_robot" position: {x: -3 y: -2 z: 0.002} orientation: {w: 1}']
                subprocess.run(cmd,check=True,capture_output=True);n.interval(1.5)
                n.trial=f'{name}_{rep}';n.interval(1.)
                start=n.sim;n.interval(duration,v,w);stop=n.sim;n.interval(3.);end=n.sim
                manifest.append(dict(trial=n.trial,case=name,repeat=rep,v=v,w=w,command_start=start,command_stop=stop,end=end,
                    nominal_duration=duration,reset_pose=[-3,-2,.002,0],world='open_arena'))
                (OUT/'trials.json').write_text(json.dumps(manifest,indent=2))
                print(n.trial,'complete',n.last['truth'],flush=True)
    finally:
        n.close();(OUT/'counts.json').write_text(json.dumps(n.counts,indent=2));n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
