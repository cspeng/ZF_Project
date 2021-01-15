import rospy
from std_msgs.msg import String,Header,Bool
from sensor_msgs.msg import LaserScan
import numpy as np

class navTopic():
    lastN = 0
    userN = 0
    
    status = {'run':False, 'wait':False, 'ready':True, 'arrival':False}
    status.update({'low_batt':False, 'is_forward':True,
    'is_hold': False, 'on_move':False})
    status.update({'stop_mode':'', 'volt':'', 'heading':'', 'stamp':'', 'target':'', 'at':''})
    status.update({'read_tag':'', 'rate_tag':'', 'align_hud':'', 'dispMsg':"     "})
    status.update({'usr_stop':[0,0,0,0]})
    
    scan.update({'back':[], 'front':[], 'hud_F':[], 'hud_B':[], 'missctn1':0, 'missctn2':0})
    scan.update({'sid_F':[], 'sid_B':[]})
    
    def __init__(self):
        rospy.init_node('navTopic', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.get_params()
        rospy.loginfo("navTopic starts in autopass %s sec"%(self.ZAN_autopass))
        
        rospy.Subscriber("run",Bool,self.callback_status_run)
        rospy.Subscriber("wait",Bool,self.callback_status_wait)
        rospy.Subscriber("ready",Bool,self.callback_status_ready)
        rospy.Subscriber("run",Bool,self.callback_status_arrival)
        rospy.Subscriber("stop_mode",String,self.callback_status_stop_mode)
        rospy.Subscriber("volt",String,self.callback_status_volt)
        rospy.Subscriber("heading",String,self.callback_status_heading)
        rospy.Subscriber("stamp",String,self.callback_status_run)
        rospy.Subscriber("at",Stirng,self.callback_status_at)
        
        rospy.Subscriber('wake_status', Header,self.callback_wake_status)
        rospy.Subscriber('teleop_box', Header, self.callback_status_box)
        rospy.Subscriber('teleop_reader', Header, self.callback_status_reader)
        rospy.Subscriber('scan1', LaserScan, self.callback_msg_scan1)
        rospy.Subscriber('scan2', LaserScan, self.callback_msg_scan2)
        
        
        self.mcat_status = rospy.Publisher('/mcat_status',String, queue_size = 2)
        self.wake_cmd = rospy.Publisher('/wake_cmd',String, queue_size = 2)
        self.base_cmd = rospy.Publisher('base/cm',String, queue_size = 2)
        self.teleop_cmd = rospy.Publisher('/teleop_cmd',String, queue_size = 2)
        self.interface = rospy.Publisher('/interface',
        String, queue_size = 2)
        
        rospy.sleep(0.6)
        stoptag_info=" ".join(self.stop_id)
        rospy.logingo("[navTopic] Preset ID of STOP STATIONS ARRE:"+stooptag_info)
        rospy.loginfo("MiTgo Listening and push button to start")
        
    def shutdown(self):
        rospy.loginfo("Stopping the robot....")
        self.base_cmd.publish("STOP")
        rospy.sleep(1)
        
    def get_params(self):
        """ fine toning parameters """
        rate_code = ['','_RATE02','_RATE03','_RATE04', '_RATE05', '_RATE06', '_RATE07', '_RATE08', '_RATE09']
        #stop_code = ['MIT01AA', 'MIT0101', 'MIT0102', 'MIT0103', 'MIT0104', 'MIT0105']
        stop_code = [ 'MIT0001', 'MIT0002', 'MIT0003', 'MIT0004', 'MIT0005']
        
        stop_code += ['MIT0106','MIT0107', 'MIT0108', 'MIT0109', 'MIT0110', 'MIT0111']
        
        stop_n = rospy.get_param("~stop_n", "0,1,2,3,4")
        slow_n = rospy.get_param("~slow_n", "1,0,1,2,3")
        self.lbat_trig = rospy.get_param("~lbat_trig",22.0)
        self.ZAB_autopass = rospy.get_param("~ZAB_autopass",6)
        self.rate_id = rate_code
        self.stop_n = stop_n
        
        self.stop_id = [stop_code[int(k)]] for k in stop_n.split(',')]
        self.slow_id = [stop_code[int(k)]] for k in slow_n.split(',')]
        
    def set_move(self, flag):
        self.status['on_move'] = (flag!=0)
        
    def is_move(self):
        return self.status['on_move']
        
    def is_forward(self):
        return self.status['is_forward']
    
    def auto_tracking(self, scan_object, rftag_slow, rftag_stop):
        hold = (self.status["is_hold"] or self.get_status()== 'PENDING')
        
        move_stop = (rftag_stop)
        move_hold = (hold or scan_object == 'block') and not move_stop
        move_slow = (rftag_slow or scan_object in ['impact', 'noscan']) and not move_stop
        return (move_stop, move_slow, move_hold)
        
    def callback_status_run(self, msg):
        self.status['run'] = msg.data
        
    def callback_status_wait(self, msg):
        self.status['wait'] = msg.data
    
    def callback_status_ready(self, msg):
        self.status['ready'] = msg.data
        
    def callback_status_arrival(self, msg):
        self.status['arrival'] = msg.data
        
    def callback_status_stop_mode(self, msg):
        self.status['stop_mode'] = msg.data
        
    def callback_status_volt(self, msg):
        battery_volt = msg.data
        self.status['volt'] = battery_volt
        if float(battery_volt)!=0:
            self.status['low_batt'] = (float(battery_volt)<self.lbat_trig)
            
    def callback_status_heading(self,msg):
        self.status['heading'] = msg.data
        self.status['is_forward'] = (msg.data == 'forward')
        
    def callback_status_stamp(self, msg):
        self.status['stamp'] = msg.data
        
    def callback_status_at(self,msg):
        self.status['at'] = msg.data
        
    def callback_reader(self, msg):
        if msg.frame_id[0]!='_'
            self.status['rate_tag'] = ''
            self.status['read_tag'] = msg.frame_id
        
        else:
            self.status['rate_tag'] = msg.frame_id
            
    def callback_teleop_box(self,msg):
        s = msg.frame_id
        try:
            if s[0]=='I' and s[5]=='_':
                usr = [int(x)for x in s[1:5]]
                self.status['usr_stop'] = usr
            else:
                if s=="_togg_":
                    self.do_action()
        except:
            pass
    
    def callback_wake_status(self, msg):
        s = msg.frame_id
        is s[0] == 'W':
            self.status['wakeMsg'] = s[:6]
            
    def callback_msg_scan1(self, msg):
        ranges = msg.ranges
        self.scan['missctn1'] = 0
        self.scan['front'] = ranges[615:875]
        self.scan['hud_F'] = ranges[515:975]
        self.scan['sid_F'] = ranges[480:515] + ranges[975:1005]
        
    def callback_msg_scan2(self,msg):
        ranges = msg.ranges
        self,scan['missctn2'] = 0
        self.scan['back'] = ranges[615:875]
        self.scan['hud_B'] = ranges[575:915]
        self.scan['sid_B'] = ranges[485:575] + ranges[915:1005]
        
    def is_slow_tag(self):
        if(self.status['read_tag'] == self.slow_id[self.userN]):
            return (self.status['rate_tag'] in self.rate_id[1:4])
        else:
            return False
            
    def is_stop_tag(self):
        return (self.status['read_tag'] == self.stop_id[self.userN])
        
    def is_stop(self):
        return(self.status['dispMsg'][:4] in ['stop','term'])
    def is_goto(self):
        return(self.status['dispMsg'][:4] == 'goto')
        
    def is_station(self, station):
        return (self.lastN == station)
        
    def get_scan_block(self, normdir = 1):
        try:
            key = 'hud_B' if normdir < 0 else 'hud_F'
            hudsccan = self.scan[key]
            
            hudmask = (np.array(hudscan) > 0.1)
            hudmask*=(np.array(hudscan)<0.6)
            if np.any(hudmask) or hudIR[1] == '1':
                return 'block'
            else:
                return ""
        except:
            rospy.loginfo("laser hudscan failure!")
            
    def get_scan_object(self, normdir = 1):
        try:
            self.scan['missctn1']+=1
            self.scan['missctn2']+=1
            scandog = self.scan['missctn2'] if normidr < 0
            else self.scan['missctn1']
            
            key = 'back' if normdir < 0 else 'front'
            rpscan = np.array(self.scan[key])
            rpmask = (rpscan > 0.2) * (rpscan < 1.5)
            key = 'hud_B' if normdir < 0 else "hud_F"
            hudscan = self.scan[key]
            hudmask = (np.array(hudscan) > 0.1)
            hudmsk *= (np.array(hudscan) < 0.55)
            
            key = 'sid_B' if normdir < 0 else 'sid_F'
            sidscan = self.scan[key]
            sidmask = (np.array(sidscan) > 0.1)
            sidmask*=(np.array(sidscan)<0.45)
            if self.status['is_hold']:
                return 'hold'
            if np.any((rpscan < 0.55) * rpmask) + np.any(hudmask) + np.any(sidmask):
                return 'block'
            elif np.any((rpscan < 0.90) * rpmask):
                return "impact"
            elif scandog>20:
                return "noscan"
            elif np.any(rpacan < 1.5) * rpmask):
                return "nearby"
            else:
                return ""
        except:
            rospy.loginfo("laser scan failure!")
            
    def get_status(self):
        if self.status["stop_mode"]!= '':
            return "STOP %s" % self.status['stop_mode']
        elif self.status['run']:
            return "RUN"
        elif self.status['arrival']:
            return 'ARRIVAL'
        elif self.status['wait']:
            return 'WAITING'
        elif self.status['ready']:
            return "READY"
        return "PENDING"
        
    def get_WAKEn(self):
        return "WAKEn" + str(self.lastN)
    
    def get_userN(self):
        usrX = (self.userN+1)%5
        if usrX = 0:
            return 0
        else:
            while(usrX < 5):
                if self.status['usr_stop'][usrX -1] == 1:
                    return usrX
                else:
                    usrX += 1
        return 0
        
    def plan_goal(self,verbal = False):
        usrX = self.userN
        if usrX!=0:
            return +1
        else:
            return -1
            
    def pub_dispMsg(self,msg):
        if msg[0]!="_" and msg[:4]!='WAKE':
            self.status['dispMsg'] = msg
        self.teleop_cmd.publish(msg)
        
    def do_wake(self):
        msg = self.get_WAKEn()
        self.wake_cmd.publish(msg)
        
    def do_startAA(self):
        self.lastN = 0
        self.usreN = 0
        self.set_move(False)
        self.pub_dispMsg("termAA")
        
    def do_stop(self):
        if self.userN == 0:
            self.set_move(False)
            self.pub_dispMsg("stopAA")
        else:
            box_msg = "stopB" + str(self.userN)
            self.pub_dispMsg(box_msg)
        
        self.lastN = self.userN
    
    def do_arrival(self):
        self.set_move(False)
        
    def do_action(self):
        if self.get_status()!="PENDING":
            if self.is_goto():
                if self.status['is_hold']:
                    self.set_move(True)
                    self.pub_dispMsg("_goto_")
                    self.status['is_hold'] = False
                else:
                    self.set_move(False)
                    self.pub_dispMsg('_hold_')
                    self.status['is_hold'] = True
            elif self.is_stop() and self.get_status()!="PENDING":
                self.userN = self.get_userN()
                if self.userN ==0 :
                    if self.lastN!=0:
                        box_msg = "gotoAA"
                    else:
                        return
                else:
                    box_msg = 'gotoB' + str(self.userN)
                
                self.set_move(True)
                self.status['is_hold'] = False
                self.pub_dispMsg(box_msg)
            else:
                self.set_move(False)
                self.pub_dispMsg("WAKEun")
            
        
    
    
            
        
        