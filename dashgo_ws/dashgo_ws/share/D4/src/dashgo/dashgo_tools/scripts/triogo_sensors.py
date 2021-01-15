#!/usr/bin/env python

""" 
triogo_sensors.py
created by clarence PEMS@2017

A basic app of the using sensors data to move trio robot

update (Oct 2017):
    echo 'termAA' for button cmd '_stAA_'
    goto home and set termN=0 for button cmd '_stBB_'
    use is_move to disable do_ZBA_autotimer after human interface
    add topic teleop_e31 for station remote go "__PING__"
    add button cmd "usrX" as if multi stations
    from A to any combination of stations 1 to 4.


triogo_station.logic 
usrX state(N):
    condition: navi in stop mode and not zero in termN
    if usr4 lastN>4: set termN = 4
    elseif usr3 and lastN>3: set termN = 3
    elseif usr2 and lastN>2: set termN = 2
    elseif usr1 and lastN>1: set termN = 1
    assert direction: goal<0 if lastN!=termN else goal>0


arrival:
    state   previous  arrival  stop  goto/hold
    charge   A0       _0       ..     _0
    ZBB      _0       0_       ..     0_
    pose     0_       _A       11     1A
    ZBA      NA       A_       AA     AN
    ZAB      AN       _A       NN     NA
    other    ..       __       NN     ..

"""

import rospy
from rosserial_arduino.msg import Adc
from std_msgs.msg import String, Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion, PoseArray, Pose
from math import pi, copysign, degrees, radians, sin
import tf, PyKDL, numpy as np

dispMsg= String(); on_move= False
dispMsg.data = "      "
termN=0; lastN=termN; lastResp=str(termN)*2

""" translate useful msgs """
def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]

def pdist(s,m):
    # usage  pdist(r-p,q-p)
    pvec= np.cross(np.array(s), np.array(m))
    return round(pvec/np.sqrt(np.dot(m,m)),3)

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


class navSensors():
    """ TRIOGO interface sensors """	
    # ags info
    ags_imu, ags_missctn= ([0, 0], [0, 0])
    ags_carrier, ags_bar = ([0, 0], [False, False])
    ags_drift, ags_acute = (0.0, 0.0)
    odom_posxy, odom_drift = ([0,0], 0)
    low_batt = False
    
    # ags location
    scan= {'back':[],'front':[],'hud_F':[], 'hud_B':[]}
    align_stamp, align_pose = (None, None)
    align_hud, align_angle = (None, None)
    tagrf= {'frame':"",'stamp':None,'pose':None}
    tagza= {'frame':"",'stamp':None,'pose':None}
    tagzb= {'frame':"",'stamp':None,'pose':None}

    def __init__(self, heading):
        # node name is ZBA_autoback
        rospy.init_node('navi', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.get_params()
        rospy.loginfo("autoback %s sec" % (self.ZBA_autoback))

        # init msgs
        rospy.Subscriber("teleop_joystick", Header, self.callback_teleop_joy)
        rospy.Subscriber("chatter", String, self.callback_msg_ags16)
        rospy.Subscriber("locater", Header, self.callback_msg_rfid)
        rospy.Subscriber("scan", LaserScan, self.callback_msg_scan)
        rospy.Subscriber("batmon", Adc, self.callback_adc_lbat)
        rospy.Subscriber("teleop_e31", Header, self.callback_msg_e31)        

        # Publisher to control the robot's speed
        self.heading = heading
        self.tele_oled = rospy.Publisher('/teleop_OLED', String, queue_size=2)
        self.ags_status= rospy.Publisher('/ags_status', String, queue_size=2)
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
            rospy.loginfo("odom Listening and push button to start")
            
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("Cannot find transform between /odom and /base_footprint")
            rospy.signal_shutdown("tf Exception")  


    def get_params(self):
        """ fine toning parameters """
        rf_code = ['F4C020','F4C021', 'F4C022', 'F4C023', 'F4C024', 'F4C025']
        # settings on AGV tracking
        # goal: a 1 2 3 4 A, beacon: a b c d e f
        # base length: 150/30 or 5mm per unit
        self.base_length = rospy.get_param("~base_length", 560.0/5)
        self.rf_id = rospy.get_param("~rf_id", rf_code)
        self.lbat_trig = rospy.get_param("~lbat_trig", 1000)
        self.semi_trig = rospy.get_param("~semi_trig", 0.5)
        self.align_blob = rospy.get_param("~align_blob", 0.20)
        self.align_acute = rospy.get_param("~align_acute", 0.25) # (0.25, 0.20)
        self.align_drift = rospy.get_param("~align_drift", 1.2) # 1.2
        self.align_Adist = rospy.get_param("~align_Adist", 0.2) # 0.2
        self.align_Bdist = rospy.get_param("~align_Bdist", 0.2) # 0.2
        # setting of AGV locations
        self.goal_dist = rospy.get_param("~goal_dist", [0, 0.6 , 3.2, 1.7, 2.0, 8.7])
        self.rf_beacon = rospy.get_param("~rf_beacon", [0, 1.6, 1.3, 0.0, 0.0, 0.0])
        self.ZTEST_distance = rospy.get_param("~ZTEST_distance", 0.0) # 3.5
        self.ZPASS_atleast = rospy.get_param("~ZPASS_atleast", 1.2) # 1.2
        self.ZBA_autoback = rospy.get_param("~ZBA_autoback", 10) # 10
        self.ZBP_autoback = rospy.get_param("~ZBP_autoback", 0) # disable
        self.ZPP_angle = rospy.get_param("~ZPP_angle", -86.0) # -ve anticlockwsie
        self.ZPP_dist = rospy.get_param("~ZPP_dist", 0.6)
        self.ZPB_dist = rospy.get_param("~ZPB_dist", 1.9)

    def auto_steering(self, error, ags_status, scan_object):
        """ key AI """
        offtrack =  (ags_status[0:4]=='off_')
        on_align = 'on_A1' if error > 0 else 'on_B1'

        # rotation in z-axis counterclockwise +ve in forward motion
        # rotation in z-axis clockwise +ve in reverse motion
        spinz = self.ags_acute / self.base_length * copysign(self.align_acute, error)
        spinz+= self.ags_drift / self.base_length * self.align_drift
        # motion factor against the path
        move_factor = (scan_object != 'block' and not offtrack)
        move_factor *= (ags_status!="loss_carrier")
        move_factor *= is_goto()
        move_factor *= 0.5 if ags_status == on_align else 1.0
        move_factor *= 0.5 if abs(error) < self.semi_trig else 1.0
        move_factor *= 0.5 if scan_object=='impact' else 1.0
        move_factor *= 0.7 if abs(self.ags_drift) > 20 else 1.0

        return (spinz, move_factor)

    def auto_tracking(self, error, ags_status, scan_object, onhold):
        move_semipass = (abs(error) < 0.15)

        onbar = (ags_status==onhold)
        onbar |= (ags_status[0:4]=='bar_' and onhold=='bar_')
        offtrack = (ags_status[0:4]=='off_') and (onhold[0:3]=='on_')
        
        if ags_status=="on_track":
            spinz = self.ags_acute / self.base_length * copysign(self.align_acute, error)
            spinz+= self.ags_drift / self.base_length * self.align_drift
        else:
            spinz = 0
        move_factor= (scan_object != 'block') 
        move_factor *= is_goto()      
        move_factor*= (not move_semipass) * 0.5 + 0.5
        move_stop = onbar or offtrack        

        return (spinz, move_factor, move_stop)

    def off_tracking(self, scan_object):
        # corrent angle between baseline and orientation
        off_angle= -self.get_blob_angle()
        blob_factor = 0.2 if abs(off_angle) > 6 else 0.0
        blob_factor *= (scan_object != 'block')
        blob_factor *= is_goto()

        return (off_angle, blob_factor)

    def blob_tracking(self, error, ags_status, scan_object):
        move_semipass = (abs(error) < 0.15)
        blob_angle= self.get_blob_angle()
        blob_acute = radians(blob_angle-90) if blob_angle!=0 else 0.0
        blob_acute = blob_acute + pi if blob_angle<0 else blob_acute
        blob_acute = sin(blob_acute)*6       

        if ags_status!="on_track":
            spinz = blob_acute * copysign(self.align_blob, error)
        else:
            spinz = 0
        move_factor= (scan_object != 'block') 
        move_factor *= is_goto()      
        move_factor*= (not move_semipass) * 0.5 + 0.5

        return (spinz, move_factor)

    """ sensors coding """
    def OLED_write(self, disp_cmd):
        """ publish command to OLED box """
        if disp_cmd[0:4] not in {"WAKE"}:
            dispMsg.data= disp_cmd
            self.tele_oled.publish(dispMsg)
        else:
            # special cmd to control lamp and buzzer
            alertMsg= String()
            alertMsg.data= disp_cmd
            self.tele_oled.publish(alertMsg)

    def callback_adc_lbat(self, adc):
        adc0_batmon = adc.adc0
        if adc0_batmon!= 0:
            if adc0_batmon < self.lbat_trig:
                self.low_batt = True
            
    def callback_teleop_joy(self, msg):
        """ response action on "teleop_joystick" """
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
            # tracklen= max(np.arange(len(y))*y)- y.index(1) + 1
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
        # (rmin, rmax)= (msg.range_min, msg.range_max)
        self.scan['front']= ranges[300:385] 
        self.scan['back']= ranges[-80:] + ranges[:15]
        self.scan['hud_F']= ranges[290:395] 
        self.scan['hud_B']= ranges[-90:] + ranges[:25] 

    def callback_msg_e31(self, msg):
        if msg.frame_id[4:10]=='_CALL_':
            if not is_goto():
                do_goto(self)

    def callback_msg_rfid(self, msg):
        if msg.frame_id[0:4] == self.heading:
            self.mark_rftag(msg.frame_id[4:])
            

    def zero_imu_measured(self):
        self.ags_drift, self.ags_acute = (0, 0)  

    def zero_rftag(self):
        self.mark_rftag('')
        
    def is_rftag_lastN(self):
        global termN
        return (self.tagrf['frame']==self.rf_id[termN])
        
    def mark_rftag(self,frame):
        self.tagrf['frame']= frame
        self.tagrf['stamp']= rospy.Time(0)
        self.tagrf['pose'], _ = self.get_odom(self.tagrf['stamp'])
        
    def markzab(self):
        """ reference lines for strip """
        tpose= PoseArray()
        if self.tagza['pose'] is not None:
            if self.tagzb['pose'] is not None:
                p1= Pose(); p2=Pose()               
                p1.position, p2.position = (self.tagza['pose'], self.tagzb['pose'])
                tpose.poses.append(p1)
                tpose.poses.append(p2)
                self.track_poses.publish(tpose)

    def get_blob_angle(self):
        isready = is_move()
        isready*= self.tagza['pose'] is not None
        isready*= self.tagzb['pose'] is not None
        isready*= self.align_angle is not None
        if isready:
            pos1, pos2 = (self.tagza['pose'], self.tagzb['pose'])
            odom_pos = self.odom_posxy
            m = np.array([pos2.x - pos1.x, pos2.y - pos1.y])
            s = np.array([odom_pos[0] - pos1.x, odom_pos[1] - pos1.y])
            self.odom_drift = pdist(s, m)
            track_angle = np.arctan2(pos1.y-pos2.y, pos1.x-pos2.x)
            odom_angle = self.align_angle
            off_angle = normalize_angle(odom_angle -track_angle)
            off_angle += pi if off_angle < -pi/2 else 0
            off_angle += -pi if off_angle > pi/2 else 0
            return round(degrees(off_angle),1)
        else:
            return 0.0

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

    def get_scan_block(self, normdir=1):
        # interaction of radar hud
        try:
            key= 'hud_B' if normdir < 0 else 'hud_F'  
            hudscan = self.scan[key]
            hudmask = (np.array(hudscan)>0.2)
            hudmask *= (np.array(hudscan)<0.6)
                
            if np.any(hudmask):
                return "block"
            else:
                return ""
        
        except:
            rospy.loginfo("laser hudscan failure!")

    def get_scan_object(self, normdir=1):
        # interaction of radar scan
        try:
            key= 'back' if normdir < 0 else 'front'
            rpscan= np.array(self.scan[key])
 
            key= 'hud_B' if normdir < 0 else 'hud_F'  
            hudscan = self.scan[key]
            hudmask = (np.array(hudscan)>0.2)
            hudmask *= (np.array(hudscan)<0.5)
            rpmask = (rpscan>0.2) * (rpscan<1.5) 
                
            if np.any((rpscan < 0.70)*rpmask) + np.any(hudmask):
                return "block"
            elif np.any((rpscan < 1.3)*rpmask):
                return "impact"
            elif np.any((rpscan < 1.5)*rpmask):
                return "nearby"
            
            else:
                return ""
        
        except:
            rospy.loginfo("laser scan failure!")

    """
    def get_scan_block_flash(self, normdir=1):
        #interaction of radar hud
        try:
            key= 'hud_B' if normdir < 0 else 'hud_F'  
            hudscan = self.scan[key]
            hudmask = blob_neighbors(hudscan)
            hudmask *= (np.array(hudscan)>0.2)
            hudmask *= (np.array(hudscan)<0.6)
                
            if np.any(hudmask):
                return "block"
            else:
                return ""
        
        except:
            rospy.loginfo("laser hudscan failure!")

    def get_scan_object_flash(self, normdir=1):
        #interaction of radar scan
        try:
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
                
            if np.any((scan < 0.70) * rpmask) + np.any(hudmask):
                return "block"
            elif np.any((scan < 1.3) * rpmask):
                return "impact"
            elif np.any((scan < 1.5) * rpmask):
                return "nearby"
            
            else:
                return ""
        
        except:
            rospy.loginfo("laser scan failure!")
     """            

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

# remote plan object self should be navSensors()
def do_stop(self):
    """modify lastN as current pose after arrival"""
    global lastResp, termN, lastN    
    if lastResp[0]=="A" and lastN==termN:
        lastN = 5
        self.OLED_write("stopAA")
    else:
        lastN = termN
        self.OLED_write("stop" + str(termN)*2)  

def do_hold(self):
    global lastResp
    set_move(False)
    lastResp= plan_resp(self) 
    self.OLED_write("hold" + lastResp)  

def do_goto(self):
    """action in defined plan"""
    global lastResp
    set_move(True)    
    lastResp= plan_resp(self)
    self.OLED_write("goto" + lastResp) 

def do_arrival(self, home=False):
    global lastResp, lastN, termN 
    charge= lastResp=="A0" and termN==0

    if home:
        # set forward motion from home
        lastN = 0; termN= 0
        lastResp= "0_"
        set_move(False)
        self.OLED_write("hold0_")
        return 0
    elif charge or lastResp is "_0":
        # set backward motion to home
        lastN = 1; termN= 0
        lastResp= "_0"
        self.OLED_write("hold_0")
        return plan_goal(self)        
    elif lastResp=="0_":
        # first position just after home
        lastN = 1; termN= lastN
        lastResp= "_A"
    # normal paths
    elif lastResp[1]=="A":
        lastResp= "A_"
    elif lastResp[0]=="A":
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

    if teleop_cmd=="_stBB_":
        # set termN=0 as if goto charge
        termN, lastResp = (0, "A0")
        self.OLED_write("term11")
        rospy.sleep(0.2)
        plan_goal(self,True)

    elif teleop_cmd=="_stAA_":
        # reset flags as if staion is on AA
        plan_startAA(self)
        set_move(False)
        rospy.sleep(0.2)
        plan_goal(self,True)

    elif teleop_cmd=="_togg_":
        if is_term():
            do_goto(self)
            
        elif dispMsg.data[0:4] in ('hold', 'stop'):
            if not self.low_batt or not is_station(5):
                # toggle run
                do_goto(self)
            else:
                # low batt halt at station 5
                do_hold(self)            
        else:
            # toggle hold
            do_hold(self)
    elif teleop_cmd=="_usr4_":
        set_usrX(4)
    elif teleop_cmd=="_usr3_":
        set_usrX(3)
    elif teleop_cmd=="_usr2_":
        set_usrX(2)
    elif teleop_cmd=="_usr1_":
        set_usrX(1)
    elif teleop_cmd=="_CALL_":
        if dispMsg.data[0:4] in ('stop', 'hold'):
            if not self.low_batt or not is_station(5):
                # call run
                do_goto(self)        

def plan_resp(self):
    global lastResp, termN, lastN
    resp= lastResp
    if termN==0 and lastN==0:
        resp = "0_"
    elif termN==0 and lastN==1:
        resp = "_0"
    elif lastResp=="A0" and termN==0:
        resp = "A0"
    else:
        # normal plan for BA or AB
        if (termN>=lastN):
            resp= str(termN) + "A"
        else:
            resp= "A" + str(termN)
    return resp

def plan_goal(self, verbal=False):
    global termN, lastN
    goal_dist = self.goal_dist
    if termN!=lastN:
        dist= sum(goal_dist[:termN+1]) - sum(goal_dist[:lastN+1]) 
    else:
        dist= sum(goal_dist) - sum(goal_dist[:termN+1]) + 0.2
    resp= plan_resp(self)
    if verbal:
        print(round(dist,1), termN, lastN, resp)
    return dist

def plan_next(self):
    global termN    
    if is_stop() and termN!=0:
        termN= max(0, termN-1)  
        do_goto(self)
    else:
        do_hold(self)

def plan_startAA(self):
    # self.OLED_write("termAA")
    global lastResp, termN, lastN
    lastN = 5; termN= 1
    lastResp= "A1"
    self.OLED_write("termAA")

def set_usrX(N):
    global termN
    if not is_goto():
        if lastN>=N:
            termN = N
            return True
        else:
            return False
    else:
        return False

def set_move(flag):
    global on_move
    on_move= (flag!=0)

def set_align(station):
    # state last move resp before arrival
    global lastResp, termN, lastN
    if station!=5:
        if termN==0 and station==1:
            lastResp = '_0'
            lastN = 1
        else:
            lastResp = 'A_'
            termN = station
            lastN = station
    else:
        lastResp = '_A'
        lastN = termN
     
def get_WAKEn():
    return "WAKEn"+str(lastN)
        
def is_move():
    global on_move
    return on_move
    
def is_station(station):
    global lastN
    return (lastN==station)

def is_backhome():
    global termN
    return (is_station(1) and termN==0)

def is_term():
    return (dispMsg.data[:4]=='term')
   
def is_stop():
    return (dispMsg.data[:4]=='stop')
    
def is_goto():
    return (dispMsg.data[:4]=='goto')

