#!/usr/bin/env python

""" triogo_track.py - Version 1.0 @2017

    A basic app of the using odometry data to move trio robot
    along magnetic trajectory.
    
    o/s: 
        func to install lamp indicator
        func to have corrective plan for off-track during running
        func to display no conn, low bat, wrong rfid, off track, etc
        provide tutorial of bluetooth control for operators
"""

import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion, PoseArray, Pose
from math import radians, sqrt, pow, pi, copysign, asin, degrees
import tf, PyKDL, numpy as np

dispMsg= String(); dispMsg.data = "      "
termN=0; lastN=termN; lastResp=str(termN)*2

""" translate useful msgs """
def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]
        
def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res

def blob_digitize(scan):
    scan= [x * (x<1.5) for x in scan]
    edges= np.histogram(scan,8)[1][::2]
    return np.digitize(scan, edges) * (np.array(scan)!=0)

def blob_neighbors(scan):
    labels = blob_digitize(scan)
    nearest= np.diff(labels)
    nearest= (np.insert(nearest,0,1)==0) + (np.append(nearest,1)==0)
    return (labels!=0) * nearest

def blob_detect(scan):
        scan= [x * (x<1.5) for x in scan]
        rpmask = np.array([0] + scan[:-1]) + np.array(scan[1:] + [0])
        return (rpmask!=0.0) * (np.array(scan)!=0)
    
class point():
    def __init__(self,x,y,z):
        self.x= x; self.y= y, self.z= z

# remote plan object self should be navSensors()
def do_stop(self):
    '''modify lastN as current pose'''
    global lastResp, termN, lastN    
    if lastResp=="A_" and lastN==termN:
        lastN = 5
        self.OLED_write("stopAA")
    else:
        lastN = termN
        self.OLED_write("stop" + str(termN)*2)  

def do_hold(self):
    global lastResp
    lastResp= plan_resp(self) 
    self.OLED_write("hold" + lastResp)  

def do_goto(self):
    '''action in defined plan'''
    global lastResp     
    lastResp= plan_resp(self)
    self.OLED_write("goto" + lastResp) 

def do_arrival(self, home=False):
    global lastResp, termN, lastN 
    charge= lastResp=="A_" and termN==0

    if home:
        #set forward motion from home
        lastN = 0; termN= 0
        lastResp= "0_"
        set_move(False)
        self.OLED_write("hold0_")
        return 0

    elif charge or lastResp in "_0":
        #set backward motion to home
        lastN = 1; termN= 0
        lastResp= "_0"
        self.OLED_write("hold_0")
        return plan_goal(self)        
    
    elif lastResp=="0_":
        #first position just after home
        lastN = 1; termN= lastN
        lastResp= "_A"

    #normal paths
    elif lastResp[1]=="A":
        lastResp= "A_"
    elif lastResp[0]=="A":
        lastResp= "_A"
    elif lastResp in ("1_", "2_", "3_", "4_", "_1", "_2", "_3", "_4"):
        termN = lastN
        lastResp= "_A"

    else:
        termN = lastN
        print("unknown arrival(__)")
        lastResp= "__"
        
    do_stop(self)
    set_move(False)
    return plan_goal(self)

def send_action(self):
    global lastResp, termN, lastN
    teleop_cmd= self.teleop_cmd
    OLED_write = self.OLED_write
    
    if teleop_cmd=="_term_" and not on_move:
        if dispMsg.data[0:4] in('term','stop'):
            OLED_write("term"+str(termN)*2)

        else:
            if dispMsg.data[0:4]=='hold':
                #sim arrival
                do_arrival(self)

            else:
                do_hold(self)

    elif teleop_cmd=="_togg_":
        if dispMsg.data[0:4]=='term' or is_pushstop():
            OLED_write("stop" + str(termN)*2) 
        elif dispMsg.data[0:4] in ('hold', 'stop'):
            #toggle run
            do_goto(self)
        else:
            #toggle halt
            do_hold(self)

    elif teleop_cmd=="_incN_":
        if dispMsg.data[0:4]=='term':
            termN= min(termN + 1, 4)
            lastN = termN
            OLED_write("term" + str(termN)*2)

        else:
            if is_stop():
                termN= min(termN+1, 4)
                OLED_write("stop." + str(termN)) 
            elif lastResp[0]=="A":
                OLED_write("holdA_")
            elif lastResp[0]=="_":
                OLED_write("hold"+plan_resp(self))
            else:
                OLED_write(dispMsg.data)

    elif teleop_cmd=="_decN_":
        if dispMsg.data[0:4]=='term':
            termN= max(0, termN-1)
            lastN = termN
            OLED_write("term" + str(termN)*2)

        else:
            if is_stop():  
                termN= max(0, termN-1)                
                OLED_write("stop." + str(termN))
            elif lastResp[1]=="A":
                OLED_write("hold_A")
            elif lastResp[1]=="_":
                OLED_write("hold"+lastResp)
            else:
                OLED_write(dispMsg.data)

def plan_resp(self):
    global lastResp, termN, lastN
    resp= lastResp
    if termN==0 or termN!=lastN and lastN!=5:
        if (termN>=lastN):
            resp= str(termN) + "_"
        else:
            resp= "_" + str(termN)
    else:
        #normal plan for N -> 5 or 5 -> N
        if (termN>=lastN):
            resp= str(termN) + "A"
        else:
            resp= "A" + str(termN)
    return resp

def plan_goal(self):
    global lastResp, termN, lastN
    goal_dist = self.goal_dist
    if termN!=lastN:
        dist= sum(goal_dist[:termN+1]) - sum(goal_dist[:lastN+1]) 
    else:
        dist= sum(goal_dist) - sum(goal_dist[:termN+1])
    resp= plan_resp(self)
    print(round(dist,1), termN, lastN, resp)
    if resp[1] in ("A", "_"):
        return abs(dist)   
    if resp[0] in ("A", "_"):
        return -abs(dist)

def plan_next(self):
    global termN    
    if is_stop() and termN!=0:
        termN= max(0, termN-1)  
        do_goto(self)
    else:
        do_hold(self)

def set_move(flag):
    global on_move
    on_move= (flag!=0)
        
def is_move():
    global on_move
    return on_move
    
def is_station(station):
    global lastN
    return (lastN==station)

def is_gohome():
    global termN
    return (is_station(1) and termN==0)
    
def is_pushstop():
    return (dispMsg.data[:5]=='stop.')
    
def is_stop():
    return (dispMsg.data[:4]=='stop')
    
def is_goto():
    return (dispMsg.data[:4]=='goto')
    
class navSensors():
    """ TRIOGO interface sensors """	
    #ags info
    ags_imu, ags_missctn= ([0, 0], [0, 0])
    ags_carrier, ags_bar = ([0, 0], [False, False])
    ags_drift, ags_acute = (0.0, 0.0)
    
    #ags location
    scan= {'back':[],'front':[],'hud_F':[], 'hud_B':[], 'frame':[]}
    align_stamp, align_pose, align_angle = (None, None, None)
    tagrf= {'frame':"",'stamp':None,'pose':None}
    tagza= {'frame':"",'stamp':None,'pose':None}
    tagzb= {'frame':"",'stamp':None,'pose':None}

    def __init__(self, heading):
        # Give the node a name
        rospy.init_node('navi', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.get_params()
        
        # How fast will we check the odometry values?
        self.r = rospy.Rate(25)
        
        #init msgs
        rospy.Subscriber("teleop_joystick", Header, self.callback_teleop_joy)
        rospy.Subscriber("chatter", String, self.callback_msg_ags16)
        rospy.Subscriber("locater", Header, self.callback_msg_rfid)
        rospy.Subscriber("scan", LaserScan, self.callback_msg_scan)

        # Publisher to control the robot's speed
        self.heading = heading
        self.tele_oled = rospy.Publisher('/teleop_OLED', String, queue_size=2)
        self._ags_status= rospy.Publisher('/ags_status', String, queue_size=2)
        self.ags_scan= rospy.Publisher('/ags_scan', String, queue_size=2)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)        
        self.track_poses = rospy.Publisher('/track_poses',PoseArray, queue_size=5)
        
        # Initialize the tf listener and give tf some time to fill its buffer
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1)
        
        # The base frame is base_footprint and odom_combined and countercheck
        self.base_frame = '/base_footprint'
        self.odom_frame = '/odom_combined'
        try:
            self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(0.6))
            rospy.loginfo("odom Listening")
            
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("Cannot find transform between /odom and /base_footprint")
            rospy.signal_shutdown("tf Exception")  


    ''' fine toning parameters '''
    def get_params(self):
        # Set the parameters for the target square
        # goal: a A 4 3 2 1, beacon: a b c d e f
        # base length: 150/30 or 5mm per unit
        rf_code = ['F4C010', 'F4C011', 'F4C012', 'F4C013', 'F4C014','F4C015']
        self.base_length = rospy.get_param("~base_length", 560.0/5)
        self.goal_dist = rospy.get_param("~goal_dist", [0, 0.6 , 2.4, 2.4, 2.4, 5.0])
        self.rf_beacon = rospy.get_param("~rf_beacon", [0, 1.8, 2.4, 2.4, 2.4, 5.0])
        self.rf_id = rospy.get_param("~rf_id", rf_code)
        self.semi_trig = rospy.get_param("~semi_trig", 0.5)
        self.start_trig = rospy.get_param("~start_trig", 0.5)
        self.align_acute = rospy.get_param("~align_acute", 0.25) # (0.25, 0.20)
        self.align_drift = rospy.get_param("~align_drift", 1.2) # 0.20
        self.align_Adist = rospy.get_param("~align_Adist", 0.2)
        self.align_Bdist = rospy.get_param("~align_Bdist", 0.2)
        self.test_distance = rospy.get_param("~test_distance", 3.5)

    ''' key AI '''
    def auto_steering(self, error, ags_status, scan_object):
        offtrack =  (ags_status[0:4]=='off_')
        on_align = 'on_A1' if error > 0 else 'on_B1'

        #rotation in z-axis counterclockwise +ve in forward motion
        #rotation in z-axis clockwise +ve in reverse motion
        spinz = self.ags_acute / self.base_length * copysign(self.align_acute, error)
        spinz+= self.ags_drift / self.base_length * self.align_drift
        #motion factor against the path
        move_factor = (scan_object != 'block' and not offtrack)
        move_factor *= is_goto()
        move_factor *= 0.5 if ags_status == on_align else 1.0
        move_factor *= 0.5 if abs(error) < self.semi_trig else 1.0
        move_factor *= 0.6 if abs(self.ags_drift) < 20 else 1.0
        move_factor *= 0.8 if scan_object=='impact' else 1.0

        return (spinz, move_factor)

    def auto_tracking(self, error, ags_status, scan_object, onhold):
        track_acute = self.align_acute * 1.6

        move_semipass = (abs(error) < 0.15)
        onbar = (ags_status==onhold)
        onbar |= (ags_status[0:4]=='bar_' and onhold=='bar_')
        offtrack = (ags_status[0:4]=='off_') and (onhold[0:3]=='on_')
        
        if ags_status=="on_track":
            spinz = self.ags_acute / self.base_length * copysign(track_acute, error)
        else:
            spinz = 0
        move_factor= (scan_object != 'block') 
        move_factor *= is_goto()      
        move_factor*= (not move_semipass) * 0.5 + 0.5
        move_stop = onbar or offtrack        

        return (spinz, move_factor, move_stop)

    def off_tracking(self,normdir=1):
        return (0, 0)
       
    ''' sensors coding '''
    def OLED_write(self, disp_cmd):
        ''' publish command to OLED box '''
        dispMsg.data= disp_cmd
        self.tele_oled.publish(dispMsg)
    
    def callback_teleop_joy(self, msg):
        ''' response action on "teleop_joystick" '''
        self.teleop_cmd= msg.frame_id 
        send_action(self)   
    
    def callback_msg_ags16(self, msg):
        msg_id= msg.data[0:4]
        msg_data= msg.data[4:]
        y= list("{0:b}".format(int(msg_data,16)))
        y= map(int, y[::-1])
        k= 0 if msg_id=="V4A1" else -1
        k= 1 if msg_id=="V4B1" else k
        if(k > -1) and not rospy.is_shutdown():
            #tracklen= max(np.arange(len(y))*y)- y.index(1) + 1
            if (sum(y)):
                self.ags_imu[k]= sum([(i+1)*x for i, x in enumerate(y)])*2/sum(y) - 17
                if self.ags_carrier[k]==0:
                    self.ags_bar[k]= False
                if not self.ags_bar[k] and (sum(y) > 13):
                    self.align_stamp = rospy.Time(0)
                self.ags_bar[k] |= (sum(y) > 13)

            else:
                self.ags_imu[k]= None
                self.ags_bar[k]= False

            self.ags_carrier[k]+= 1

    def callback_msg_scan(self, msg):
        ranges= msg.ranges
        #(rmin, rmax)= (msg.range_min, msg.range_max)
        self.scan['back']= ranges[-30:] + ranges[0:65] 
        self.scan['front']= ranges[350:435]
        self.scan['hud_B']= ranges[-50:] + ranges[0:85]
        self.scan['hud_F']= ranges[330:455]

    def callback_msg_rfid(self, msg):
        if msg.frame_id[0:4] == self.heading:
            self.tagrf['frame']= msg.frame_id[4:]
            self.tagrf['stamp']= rospy.Time(0)
            self.tagrf['pose'], _ = self.get_odom(self.tagrf['stamp'])

    def zero_imu_measured(self):
        self.ags_drift, self.ags_acute = (0, 0)  

    def markzab(self):
        ''' reference lines for strip '''
        tpose= PoseArray()
        if self.tagza['pose'] is not None:
            if self.tagzb['pose'] is not None:
                p1= Pose(); p2=Pose()               
                p1.position, p2.position = (self.tagza['pose'], self.tagzb['pose'])
                tpose.poses.append(p1)
                tpose.poses.append(p2)
                self.track_poses.publish(tpose)
    
    def get_ags_status(self):
        (imu_A1, imu_B1)= tuple(self.ags_imu)
        (bar_A1, bar_B1)= tuple(self.ags_bar)
        (carrier_A1, carrier_B1)= tuple(self.ags_carrier)
        self.ags_carrier= [0, 0]        
        if (carrier_A1>0) and (carrier_B1>0):
            self.ags_missctn[0]= 0
            if imu_A1 is not None and imu_B1 is not None:
                self.ags_drift= imu_A1 + imu_B1
                self.ags_acute= imu_A1 - imu_B1

            if imu_A1 is None and imu_B1 is None:
                self.zero_imu_measured()
                return "off_track"
                
            nbar_A1= (imu_A1 is not None and not bar_A1)
            nbar_B1= (imu_B1 is not None and not bar_B1)
            if bar_A1 and bar_B1:
                self.zero_imu_measured()
                return "off_track"
            if bar_A1 and nbar_B1:
                self.align_pose, _ = self.get_odom(self.align_stamp)
                return "on_A1"
            if bar_B1 and nbar_A1:
                self.align_pose, _ = self.get_odom(self.align_stamp)
                return "on_B1"
            
            if bar_A1 and imu_B1 is None:
                return "bar_A1"
            if bar_B1 and imu_A1 is None:
                return "bar_B1"

            if imu_A1 is None:
                self.zero_imu_measured()
                return "off_A1"
            elif imu_B1 is None:
                self.zero_imu_measured()
                return "off_B1"
            else:
                return "on_track"

        else:
            self.ags_missctn[0] += 1
            self.ags_missctn[1]= max(self.ags_missctn)

            if self.ags_missctn[0] > 2:
                self.zero_imu_measured()
            
            if self.ags_missctn[0] < 4:
                return "miss_carrier"
            else:
                if self.ags_missctn[0]==self.ags_missctn[1]:
                    info= str(self.ags_missctn[0]) + " timesteps!"
                    rospy.loginfo("AGS loss_carrier at most " + info)
                return "loss_carrier"

    def get_scan_object(self, normdir=1):
        #interaction of radar
        key= 'back' if normdir < 0 else 'front'
        rpscan= self.scan[key]
        rpmask = blob_neighbors(rpscan) 

        key= 'hud_B' if normdir < 0 else 'hud_F'  
        hudscan = self.scan[key]
        hudmask = blob_neighbors(hudscan)
        hudmask *= (np.array(hudscan)>0.2)
        hudmask *= (np.array(hudscan)<0.5)
        scan= np.array(rpscan) * (np.array(rpscan)>0.2) 
        scan *= (np.array(rpscan)<1.4) * rpmask
        print(hudmask)

        if is_stop():
            print([hudmask])
        else:
            print([scan.round(1)])
            
        if np.any((scan < 0.7) * rpmask) + np.any(hudmask):
            return "block"
        elif np.any((scan < 1.0) * rpmask):
            return "impact"
        elif np.any((scan < 1.4) * rpmask):
            return "nearby"
        
        else:
            return ""

    def get_odom(self, stamp=None):
        # Get the current transform between the odom and base frames
        try:
            stamp = rospy.Time(0) if stamp is None else stamp
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, stamp)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

        
    def ags_status(self, odom_angle=None):
        navi = self
        ags_status= navi.get_ags_status()

        if ags_status=="on_track" and odom_angle is not None:
            run_angle = odom_angle + asin(navi.ags_acute/navi.base_length)                
            if navi.align_angle is None:   navi.align_angle = run_angle
            navi.align_angle= navi.align_angle * 0.6 + run_angle *0.4
            navi.align_angle = normalize_angle(navi.align_angle)
           
        run_angle= degrees(navi.align_angle) if navi.align_angle is not None else 0
        ags_info = navi.tagrf['frame'] + "  "
        ags_info += "track %sdeg" % (round(run_angle, 1))
        navi._ags_status.publish(str(ags_status) + ags_info)
        return ags_status        

    def scan_object(self, normdir=1):
        ags_scan = self.get_scan_object(normdir)
        info = str(ags_scan)
        self.ags_scan.publish(str(info))
        return ags_scan
        
        
if __name__ == '__main__':
    """ TRIOGO main """
    try:
        navi= navSensors(heading="F400")

        # stop any previous motion
        navi.cmd_vel.publish(Twist())
        do_arrival(navi)
        #navi.move_right_angle(normdir=-1)
        
        while not rospy.is_shutdown():        
            ags_status= navi.ags_status()
            normdir= 1 if is_goto() else -1
            scan_object= navi.scan_object(-normdir)                  
            navi.r.sleep()
            
        # Stop the robot when we are done
        navi.cmd_vel.publish(Twist())
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
