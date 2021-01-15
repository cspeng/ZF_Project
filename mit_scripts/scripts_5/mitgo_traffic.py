#!/usr/bin/env python
""" 
mitgo_traffic.py
created by clarence PEMS@2020

    A basic app of using rfid tag to move trio robot
    along magnetic trajectory.
    
    flow:
        D0 on_track and is_goto: ZBA/R0 if goal<0 else D1
        D4 is_move: tagza
        D5 is_goto: ZBA/D0 if goal>0 else ZBA/R0
        D6: ZBA/R0
        R0 on_track: ZAB/D0 if goal>0 else R1
        R1 on_track: write("goto_0")
        R4 is_move: tagzb
        R5: ZAB/D0 if goal > 0 else R6
        R6: ZAB/D0
 """
PKG= 'mitgo_tools'
import roslib; roslib.load_manifest(PKG)
import rospy
from std_msgs.msg import String, Bool

from mitgo_topic import navTopic


class NavGO():
    """ TRIOGO navigation """
    def __init__(self, navi):
        self.navi= navi
        self.zone, self.gear = ("forward", "D0")
        self.test_toggle = True
        self.do_autoTimer = 0
        self.ping_cnt = 0
        
        # How fast will we check the odometry values?
        self.r = rospy.Rate(25)
        
        # stop any previous motion
        self.navi.base_cmd.publish('STOP')
        rospy.sleep(0.2)

        # stop any previous indications 
        self.navi.pub_dispMsg('WAKEff')
        rospy.sleep(0.1)
        self.navi.do_startAA()    
        
        self._status = ''
        self._scanobject = ''
        self.interface = rospy.Publisher('/interface',String,queue_size=2)
        self.fresh_ping()
        
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
        self.ping_cnt = (self.ping_cnt + 1) %12
        if self.ping_cnt==0:
            navi.base_cmd.publish('STATUS')
            rospy.sleep(0.1)
        if navi.is_stop():
            ags_scan = self.scan_object(1) 
            
        self._status= navi.get_status()
        self.interface_info(self._status)

        ags_info = " %s %s  " % (self.zone, self.gear)
        ags_info += navi.status['align_hud']
        ags_info += " on_tag  " if navi.is_stop_tag() else "  "
        ags_info += navi.status['read_tag']
        ags_info += "LowBat" if navi.status['low_batt'] else "  "         
        ags_info += "%s  " % (navi.status['dispMsg'])
        navi.mcat_status.publish(str(self._status) + ags_info)
        
        return self._status

    def interface_info(self, _status):    ###pid
        navi = self.navi

        WAKE_data = ""
        if navi.status['low_batt']:    
            WAKE_data = "WAKElb"
        elif str(navi.status['align_hud']) == "block":   
            WAKE_data ="WAKEbc"
        else:   
            WAKE_data ="______"
        InterfaceData = "$AGV,"+str(navi.status['volt'])
        InterfaceData += "," + navi.status['dispMsg'] + "," + WAKE_data + "\r\n"

        self.interface.publish(InterfaceData)

    def scan_object(self, normdir=1):
        navi = self.navi
        ags_scan = navi.get_scan_object(normdir)    ###pid
        navi.status['align_hud'] = str(ags_scan)
        return ags_scan

    def scan_block(self, normdir=1):
        navi = self.navi
        ags_scan = navi.get_scan_block(normdir)    ###pid
        navi.status['align_hud'] = str(ags_scan)
        return ags_scan

    def fresh_ping(self):
        navi.pub_dispMsg("_PING_")

    """ go to target at slow speed """
    #def move_slow(self, goal='forward', speed='RATE3'):
        #sself.move_drive(goal, speed)
  
    def set_drive(self, speed):
        navi.base_cmd.publish('STOP')
        rospy.sleep(0.25)
        #cmd = 'setID' + str(self.navi.userN)
        #navi.base_cmd.publish(cmd)        
        #rospy.sleep(0.2)
        navi.base_cmd.publish(speed)
        rospy.sleep(0.25)
    def set_direction(self, goal):
        if navi.is_forward():
            if goal=='backward':    
                navi.base_cmd.publish('REVERSE')
        else:
            if goal=='forward':    
                navi.base_cmd.publish('REVERSE')
        rospy.sleep(0.2) 
    
    def set_branchInstructons(self, indication):
        if navi.is_turnlf():
            if indication == 'turnrg':
                navi.base_cmd.publish('branch_instruction')
        else:
            if indication == 'turnlf':
                navi.base_cmd.publish('branch_instruction')
        
        rospy.sleep(0.2)

    def set_arrival(self):
        navi.base_cmd.publish('setLTAG')
        rospy.sleep(0.2)
        navi.base_cmd.publish('RATE3')
        rospy.sleep(0.2)
        navi.base_cmd.publish('setRTAG')
        rospy.sleep(0.2)
        navi.base_cmd.publish('STATUS')
        rospy.sleep(0.2)
    def set_parking(self):
        #cmd = "saveID" + str(self.navi.userN)
        #navi.base_cmd.publish(cmd)
        #rospy.sleep(0.2)
        self.navi.do_stop()

    """ go to target at param speed protocol """
    def move_drive(self, goal='forward', indication = 'turnlf',speed='RATE6'):
        navi = self.navi
        move_stop, rftag_slow, rftag_stop, rftag_turn = (False, False, False, False)
        
        # set target to the base
        move_speed = speed
        self.set_drive(speed=move_speed)

        # check heading of the base
        self.set_direction(goal)
        
        self.set_branchInstructons(indication)
        
        navi.base_cmd.publish('START')
        rospy.sleep(0.2)
        is_last_hold = False
 
        while not move_stop and not rospy.is_shutdown():
            self.r.sleep()
            mcat_status= self.mcat_status()
            normdir = -1 if goal=='backward' else 1
            scan_object= self.scan_object(normdir) 

            # check speed tag
            if not rftag_slow:
                if navi.is_slow_tag(): 
                    rftag_slow = True

            # check stop tag 
            if not rftag_stop:
                if navi.is_stop_tag():
                    print "pass stop tag at %s" % navi.status['read_tag']
                    rftag_stop = True

            #check turn_tag
            
            if not rftag_turn:
                if navi.is_turn_tag() and goal == 'forward':
                    self.set_branchInstructons('turnrg')
                    rftag_turn = True
                
            move_stop, move_slow, move_hold = navi.auto_tracking(scan_object, rftag_slow, rftag_stop)

            # set speed normal or slow
            if (move_speed!='RATE3') and (move_slow or move_hold):
                move_speed = 'RATE3'
                navi.base_cmd.publish(move_speed)
                rospy.sleep(0.15)
                navi.base_cmd.publish(move_speed)
                rospy.sleep(0.15)

            if (move_speed!=speed) and not move_slow and not move_hold:
                move_speed = speed
                navi.base_cmd.publish(move_speed)
                rospy.sleep(0.2)

            if move_stop:
                navi.base_cmd.publish('STOP')
                
            else:
                if mcat_status=="RUN":
                    if move_hold and not is_last_hold:
                        is_last_hold = True
                        navi.base_cmd.publish('STOP')
                        navi.teleop_cmd.publish('_hold_')
                        rospy.sleep(0.2)
                if mcat_status=="READY":
                    if not move_hold and is_last_hold:
                        is_last_hold = False
                        navi.base_cmd.publish('START')
                        navi.teleop_cmd.publish('_goto_')
                        rospy.sleep(0.2)
        rospy.sleep(0.1)
        navi.base_cmd.publish('STOP')
        rospy.sleep(0.2)   

        if rftag_stop:
            self.set_parking()
        
        return mcat_status


    def do_ZBA(self):
        gear= self.gear
        mcat_status= self.mcat_status()
        
        # R gearing
        if gear=="R0":
            if mcat_status!="PENDING" and self.navi.is_goto():
                # override reverse mode by 'termAA' 
                if self.navi.plan_goal()>0:  
                    self.gear = "R6"
                else:
                    self.gear = "R1"

        if gear=="R1":
            self.fresh_ping()
            self.gear= "R2"

        elif gear=="R2":
            # starting goal
            self.move_drive(goal='backward')
            self.gear= "R3"

        elif gear=="R3":
            if mcat_status!="PENDING":
                self.gear= "R4"
            
        elif gear=="R4":
            self.navi.do_arrival()
            self.fresh_ping()
            rospy.sleep(1.6)
            
            if not self.navi.status['low_batt']:
                # conti when not low battery
                self.do_autoTimer = 0
                self.gear= "R5"
            else:
                self.navi.teleop_cmd("WAKElb")
                rospy.sleep(1.6)  


        elif gear=="R5": 
            ### outostart from AA for testing
            if self.navi.is_stop() and self.navi.ZAB_autopass !=0:
                if self.navi.is_station(0) and self.test_toggle:
                    rospy.sleep(1)
                    self.do_autoTimer += 1
                    if self.do_autoTimer > self.navi.ZAB_autopass * 2:
                        self.navi.do_action()
                        
            if self.navi.is_goto():    
                # button to go
                if self.navi.plan_goal()<0:
                    self.zone, self.gear = ("backward", "R0")
                else:            
                    self.gear="R6"

        elif gear=="R6":
            self.zone, self.gear = ("forward", "D0")

    def do_ZAB(self):
        gear= self.gear
        mcat_status= self.mcat_status()

        # D gearing        
        if gear=="D0":
            if mcat_status!="PENDING" and self.navi.is_goto():
                # override forward mode by 'term11' 
                if self.navi.plan_goal()<0:
                    self.zone, self.gear = ("backward", "R0")
                else:            
                    self.gear = "D1"

        if gear=="D1":
            self.fresh_ping()
            self.gear= "D2"

        elif gear=="D2":
            # starting goal
            self.move_drive(goal='forward')
            self.gear= "D3"

        elif gear=="D3":
            self.do_autoTimer = 0
            if mcat_status!="PENDING":
                self.gear= "D4"
  
        elif gear=="D4":
            self.navi.do_arrival()
            self.fresh_ping()
            rospy.sleep(0.6)
            self.navi.do_wake()       ###pub string to wake boards
            #self.navi.OLED_write(get_WAKEn())
            self.do_autoTimer = 0
            self.gear= "D5"

        elif gear=="D5":
            #autoback to home with do_autoTimer
            if self.navi.is_stop() and self.navi.ZAB_autopass!=0:
                if not self.navi.is_station(0):
                    rospy.sleep(1)
                    self.do_autoTimer += 1
                    if self.do_autoTimer > self.navi.ZAB_autopass:
                        self.navi.do_action()

            if self.navi.is_goto(): 
                # button to go
                if self.navi.plan_goal()>0:  
                    self.zone, self.gear = ("backward", "R6")
                else:
                    self.gear="D6"

        elif gear=="D6":
            self.zone, self.gear = ("backward", "R0")
        


if __name__ == '__main__':
    """ TRIOGO main """
    try:
        navi= navTopic()
        NavGO(navi)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")

