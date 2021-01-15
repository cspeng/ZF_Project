#!/usr/bin/env python
""" 
mitgo_traffic.py
created by clarence PEMS@2020

    A basic app of the using odometry data to move trio robot
    along magnetic trajectory.
    
    flow:
        init lamp: write("WAKEff")
        init disp: do_hold
        init beep: write("WAKEn3")
        Pose/N1 on_track: ZAB/R0  if goal<0 else N2
        Pose/N3 on_A1: tagzb
        Pose/N6 on_track: ZBA/D0
        D0 on_track and is_goto: ZAB/R0 if goal<0 else D1
        D4 is_move: tagza
        D5 is_goto: ZBA/D0 if goal>0 else ZAB/R0
        D6: ZAB/R0
        R0 on_track: ZBA/D0 if goal>0 else R1
        R1 on_track: write("goto_0")
        R4 is_move: tagzb
        R5: ZBA/D0 if goal > 0 else R6
        R6: ZBP/P0
 """
PKG= 'dashgo_tools'
import roslib; roslib.load_manifest(PKG)
import rospy
from std_msgs.msg import String, Bool

from mitgo_topic import navTopic
from mitgo_topic import dispMsg, get_WAKEn
from mitgo_topic import plan_goal, plan_startAA

from mitgo_topic import is_goto, is_move, is_term, is_station, is_stop
from mitgo_topic import do_goto, do_arrival, do_hold, set_move


class NavGO():
    """ TRIOGO navigation """
    def __init__(self, navi):
        self.navi= navi
        self.zone, self.gear = ("forward", "D0")
        self.do_autoTimer = 0
        
        # How fast will we check the odometry values?
        self.r = rospy.Rate(25)
        
        # stop any previous motion
        self.navi.base_cmd.publish('STOP')
        rospy.sleep(0.2)

        # stop any previous indications 
        self.navi.teleop_box('WAKEff')
        rospy.sleep(0.1)
        plan_startAA(self.navi)    
        self.navi.teleop_box('termAA')
        
        self._status = ''
        self.interface = rospy.Publisher('/interface',String,queue_size=2)

        while not rospy.is_shutdown():
            if self.zone=="forward":
                self.do_ZAB()
            elif self.zone=="backward":
                self.do_ZBA()
                    
            self.r.sleep()
            
        # Stop the robot when we are done
        self.navi.base_cmd.publish('STOP')

    def mcat_status(self):
        navi = self.navi
        self._status= navi.get_status()
        self.interface_info(self._status)

        ags_info = " %s %s  " % (self.zone, self.gear)

        ags_info += navi.tagrf['frame']
        ags_info += " on_tag  " if navi.is_rftag_lastN() else "  "
        ags_info += "LowBat" if navi.low_batt else "  "         
        ags_info += "%s  " % (dispMsg.data)
        navi.ags_status.publish(str(self._status) + ags_info)
        
        return self._status     

    def interface_info(self, _status):    ###pid
        WAKE_data = ""
        navi = self.navi
        if dispMsg.data == "WAKElb":
            WAKE_data = "WAKElb"
        elif _status[0:3] == "off":
            WAKE_data = "WAKEof"
        elif str(navi.align_hud) == "block":
            WAKE_data ="WAKEbc"
        else:
            WAKE_data ="______"
        InterfaceData = "$AGV,"+str(navi.voltage_value)
        InterfaceData += "," + dispMsg.data + "," + WAKE_data + "\r\n"

        self.interface.publish(InterfaceData)

    def scan_object(self, goal='forward'):
        navi = self.navi
        ags_scan = navi.get_scan_object(goal)    ###pid
        navi.align_hud = str(ags_scan)
        return ags_scan

    def scan_block(self, goal='forward'):
        navi = self.navi
        ags_scan = navi.get_scan_block(goal)    ###pid
        navi.align_hud = str(ags_scan)
        return ags_scan


    """ go to target at slow speed """
    def move_slow(self, goal='forward', speed='RATE4'):
        self.move_drive(goal, speed)
 
    """ go to target at param speed protocol """
    def move_drive(self, goal='forward', speed='RATE8'):
        navi = self.navi
        move_stop, rftag_slow, rftag_stop = (False, False, False)
        
        # set STOP to the base
        navi.base_cmd.publish('STOP')
        rospy.sleep(0.1)
        navi.base_cmd.publish(speed)
        rospy.sleep(0.1)
        move_speed = speed
 
        while not move_stop and not rospy.is_shutdown():
            self.r.sleep()
            mcat_status= self.mcat_status()
            scan_object= self.scan_block(goal) 

            # check speed tag
            if not rftag_slow:
                speedtag = navi.rftag_speed()
                if speedtag!='':    rftag_slow = True

            # check stop tag 
            if not rftag_stop:
                if navi.is_rftag_stop():
                    print "pass stop tag at %s" % navi.rftag_at()
                    rftag_stop = True

            move_stop, move_slow, move_hold = navi.auto_tracking(mcat_status, scan_object, rftag_slow, rftag_stop)
            #move_stop |= is_term()      # override by term button 

            # check heading of the base
            if navi.is_forward():
                if goal=='backward':    
                    navi.base_cmd.publish('REVERSE')
                    rospy.sleep(0.1)
            else:
                if goal=='forward':    
                    navi.base_cmd.publish('REVERSE')
                    rospy.sleep(0.1)   

            # set speed normal or slow
            if move_speed!='RATE4':
                if move_slow:
                    move_speed=='RATE4'
                    navi.base_cmd.publish(move_speed)
                    rospy.sleep(0.1)
            else:
                if not move_slow:
                    move_speed = speed
                    navi.base_cmd.publish(move_speed)
                    rospy.sleep(0.1)

            if move_stop:
                navi.base_cmd.publish('STOP')
            else:
                if is_start():
                    if move_hold:
                    navi.base_cmd.publish('STOP')
                else:
                    navi.base_cmd.publish('START')
        
        return mcat_status


    def do_ZBA(self):
        gear= self.gear
        mcat_status= self.mcat_status()
        
        # R gearing
        if gear=="R0":
            if mcat_status=="on_track" and is_goto():
                # override reverse mode by 'termAA' 
                if plan_goal(self.navi)>0:  
                    self.gear = "R6"
                else:
                    self.gear = "R1"

        if gear=="R1":
            do_goto(self.navi)
            self.gear= "R2"

        elif gear=="R2":
            # starting goal
            self.move_drive(goal='backward')
            self.gear= "R3"

        elif gear=="R3":
            if mcat_status=="on_track":
                self.gear= "R4"
            
        elif gear=="R4":
            if is_move():
                do_arrival(self.navi)
                rospy.sleep(1.6)
            if not self.navi.low_batt:
                # conti when not low battery
                self.do_autoTimer = 0
                self.gear= "R5"
            else:
                self.navi.teleop_cmd("WAKElb")
                rospy.sleep(1.6)  


        elif gear=="R5": 
            ### autostart to AA for testing
            """
            if is_stop() and self.navi.ZBA_autoback!=0: 
                if is_station(5):
                    rospy.sleep(1)
                    self.do_autoTimer += 1
                    if self.do_autoTimer > self.navi.ZBA_autoback * 2:
                        do_goto(self.navi)
		    """
            if is_goto():    
                # button to go
                if plan_goal(self.navi)<0:
                    self.zone, self.gear = ("ZBA", "R0")
                else:            
                    self.gear="R6"

        elif gear=="R6":
            self.zone, self.gear = ("ZAB", "D0")

    def do_ZAB(self):
        gear= self.gear
        mcat_status= self.mcat_status()

        # D gearing        
        if gear=="D0":
            if mcat_status=="on_track" and is_goto():
                # override forward mode by 'term11' 
                if plan_goal(self.navi)<0:
                    self.zone, self.gear = ("ZBA", "R0")
                else:            
                    self.gear = "D1"

        if gear=="D1":
            self.navi.teleop_cmd("gotoB1") 
            do_goto(self.navi)
            self.gear= "D2"

        elif gear=="D2":
            # starting goal
            self.move_drive(goal='forward')
            self.gear= "D3"

        elif gear=="D3":
            self.do_autoTimer = 0
            if mcat_status=="on_track":
                self.gear= "D4"
  
        elif gear=="D4":
            if is_move():
                do_arrival(self.navi)
                rospy.sleep(0.6)
                #self.navi.OLED_write(get_WAKEn())
            self.do_autoTimer = 0
            self.gear= "D5"

        elif gear=="D5":
            #autoback to home with do_autoTimer
            if is_stop() and self.navi.ZBA_autoback!=0: 
                if not is_station(5):
                    rospy.sleep(1)
                    self.do_autoTimer += 1
                    if self.do_autoTimer > self.navi.ZBA_autoback:
                        do_goto(self.navi)
                        
            if is_goto(): 
                # button to go
                set_move(True)
                if plan_goal(self.navi)>0:  
                    self.zone, self.gear = ("ZBA", "R6")   
                else:
                    self.gear="D6"     

        elif gear=="D6":
            set_move(False)
            self.zone, self.gear = ("ZBA", "R0")

if __name__ == '__main__':
    """ TRIOGO main """
    try:
        navi= navTopic(heading="F5D0")
        NavGO(navi)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
