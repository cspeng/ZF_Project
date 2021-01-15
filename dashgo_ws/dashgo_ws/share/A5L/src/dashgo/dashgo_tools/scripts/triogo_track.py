#!/usr/bin/env python
""" 
triogo_track.py
created by clarence PEMS@2017

    A basic app of the using odometry data to move trio robot
    along magnetic trajectory.
    
    o/s:
        alert Red when ags_status is "loss_carrier" over 100
		add teleop "usrX" as if goto multi stations
			
pinz,   flow:
        init zone: Search/N0
        init lamp: write("WAKEff")
        init disp: do_hold
        init beep: write("WAKEn3")
        N0 on_track and is_term: ZAB/R0
        N0 on_track and holdA1: ZAB/R0
        N0 on_track and is_goto: ZAB/R0
        N0 is_goto and off_track: N1
        N0 is_goto and bar_: N2
        N7: Pose/N0
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
        P0 on_track: P3
        P4 move_blob? wrong indication on dir
        P5 is_move: do_goto if autotimer>autoback        
        P5 is_goto: Search/N0

"""
PKG= 'dashgo_tools'
import roslib; roslib.load_manifest(PKG)
import rospy
from triogo_sensors import navSensors, dispMsg, normalize_angle, get_WAKEn
from triogo_sensors import plan_goal, plan_resp, plan_startAA
from triogo_sensors import is_goto, is_move, is_term, is_station,is_backhome,is_stop
from triogo_sensors import do_goto, do_arrival, do_hold, set_move, set_align
from geometry_msgs.msg import Twist
from math import radians, sqrt, pow, copysign, asin, degrees


class NavGO():
    """ TRIOGO navigation """
    def __init__(self, navi):
        self.navi= navi
        self.zone, self.gear = ("Search", "N0")
        self.do_autoTimer = 0

        # How fast will we check the odometry values?
        self.r = rospy.Rate(25)

        # stop any previous motion
        self.navi.cmd_vel.publish(Twist())
        # stop any previous indications 
        self.navi.OLED_write("WAKEff")
        rospy.sleep(0.4)
        do_hold(self.navi)
        rospy.sleep(0.4)
        self.navi.OLED_write("WAKEn3") 
        
        while not rospy.is_shutdown():
            if self.zone=="Search":
                self.do_search()
           
            elif self.zone=="Pose":
                self.do_pose()

            elif self.zone=="ZBA":
                self.do_ZBA()
                    
            elif self.zone=="ZAB":
                self.do_ZAB()
                
            elif self.zone=="ZBP":
                self.do_ZBP()
                    
            self.r.sleep()
            
        # Stop the robot when we are done
        self.navi.cmd_vel.publish(Twist())

    def ags_status(self):
        navi = self.navi
        ags_status= navi.get_ags_status()

        ags_info = " %s %s  " % (self.zone, self.gear)
        ags_info += "%s/%s  " % (navi.ags_acute, navi.ags_drift)
        ags_info += navi.tagrf['frame'] + " %s  " % str(navi.is_rftag_lastN()*1)
        ags_info += "track %s drift %s " % (navi.align_hud, navi.odom_drift)
        ags_info += "%sdeg  " % (navi.get_blob_angle())
        ags_info += "LowBat %s  " % (navi.low_batt)         
        ags_info += "%s  " % (dispMsg.data)
        navi.ags_status.publish(str(ags_status) + ags_info)
        return ags_status        

    def scan_object(self, normdir=1):
        navi = self.navi
        ags_scan = navi.get_scan_object(normdir)
        navi.align_hud = str(ags_scan)
        navi.ags_scan.publish(navi.align_hud)
        return ags_scan

    def scan_block(self, normdir=1):
        navi = self.navi
        ags_scan = navi.get_scan_block(normdir)
        navi.align_hud = str(ags_scan)
        navi.ags_scan.publish(navi.align_hud)
        return ags_scan


    """ motion parts """    
    def move_drive(self, speed=0.5):
        """ get D gearing motion """
        navi= self.navi
        
        odom_pose, move_tol = (None, 0.015)
        goal, align_Adist = (plan_goal(navi, verbal=True), navi.align_Adist)

        # keep goal positive
        on_exempt = (goal<0); goal = abs(goal)
        if navi.ZTEST_distance!=0:    goal = navi.ZTEST_distance
        move_stop, move_alignpass = (on_exempt, False)
        ags_status = ''
        
        # Stop the robot by default
        move_cmd = Twist()
        navi.cmd_vel.publish(move_cmd)
        odom_pose, _ = navi.get_odom()
        x_start, y_start= (odom_pose.x, odom_pose.y)
        offtick_time= rospy.get_rostime().to_sec() - 2.0
        
        while not move_stop and not rospy.is_shutdown():
            self.r.sleep()
            odom_pose, navi.align_angle = navi.get_odom()
            navi.odom_posxy = [odom_pose.x, odom_pose.y]

            ags_status= self.ags_status()
            scan_object= self.scan_object(normdir=1) 

            distance = sqrt(pow((odom_pose.x - x_start), 2) +
                            pow((odom_pose.y - y_start), 2))
                            
            # error is positive for D gear
            error =  goal - distance

            # alignpass A1 is valid after travel 1.2m            
            if ags_status=='on_A1' and distance > navi.ZPASS_atleast:    
                move_alignpass = True
                lasterror= error - align_Adist
            
            if move_alignpass:
                error -= lasterror

            # turn -ve clockwise when triogo on left
            spinz, move_factor = navi.auto_steering(error, ags_status, scan_object)
                     
            if move_alignpass:
                move_factor = 0.25 * (move_factor>0)
           
            move_stop = abs(error) < move_tol
            move_stop |= is_term()      # override by term button
            offtrack = (ags_status[0:4]=='off_') 
            
            if offtrack:    offtick_time= rospy.get_rostime().to_sec()
            elapsed_time = rospy.get_rostime().to_sec() - offtick_time
            retrack_hold = (elapsed_time<0.4)

            if move_stop:
                move_cmd= Twist()

            else:
                if not offtrack and not retrack_hold:
                    # If normal, move in the appropriate direction
                    move_cmd.linear.x = copysign(speed * move_factor, error)
                    move_cmd.angular.z = spinz * move_factor
                else:
                    off_angle, blob_factor = navi.off_tracking(scan_object)
                    move_cmd.linear.x = 0
                    move_cmd.angular.z = copysign(blob_factor, off_angle)

            navi.cmd_vel.publish(move_cmd)
        
        return move_alignpass    

    def move_reverse(self, speed=0.5):
        """ get R gearing motion """
        navi= self.navi
        
        odom_pose, move_tol = (None, 0.015)
        goal, align_Bdist = (plan_goal(navi, verbal=True), navi.align_Bdist)

        # keep goal positive
        on_exempt = (goal>0); goal= abs(goal)
        if navi.ZTEST_distance!=0:    goal = navi.ZTEST_distance
        move_stop, move_alignpass, rftag_lastNpass = (on_exempt, False, False)
        ags_status = ''

        # Stop the robot by default
        move_cmd = Twist()
        navi.cmd_vel.publish(move_cmd)
        # zero rf tag at postion A
        navi.zero_rftag()
        # init variables before loop
        odom_pose, _ = navi.get_odom()
        x_start, y_start= (odom_pose.x, odom_pose.y)
        offtick_time= rospy.get_rostime().to_sec() - 2.0        

        while not move_stop and not rospy.is_shutdown():
            self.r.sleep()
            odom_pose, navi.align_angle = navi.get_odom()
            navi.odom_posxy = [odom_pose.x, odom_pose.y]
            
            ags_status= self.ags_status()
            scan_object= self.scan_object(normdir=-1) 

            distance = sqrt(pow((odom_pose.x - x_start), 2) +
                            pow((odom_pose.y - y_start), 2))
                            
            # error is negative for R gear
            error = distance - goal

            # alignpass B1 is valid after travel 1.2m
            if ags_status=='on_B1' and distance > navi.ZPASS_atleast:    
                move_alignpass = True
                lasterror= error + align_Bdist
            
            # detect rftag_lastN and set rftag_lastNpass 
            if not (rftag_lastNpass or move_alignpass):
                if navi.is_rftag_lastN():
                    print "pass rftag at %s" % str(error)
                    rftag_lastNpass = True
                    lasterror= error + align_Bdist                
  
            if move_alignpass or rftag_lastNpass:
                error -= lasterror
             
            # turn +ve anticlockwise when triogo on left
            spinz, move_factor = navi.auto_steering(error, ags_status, scan_object)
            
            if move_alignpass or rftag_lastNpass:
                # set angular speed at 0.25
                move_factor = 0.25 * (move_factor>0)
            
            move_stop = abs(error) < move_tol
            move_stop |= is_term()      # override by term button
            offtrack = (ags_status[0:4]=='off_')

            if offtrack:    offtick_time= rospy.get_rostime().to_sec()
            elapsed_time = rospy.get_rostime().to_sec() - offtick_time
            retrack_hold = (elapsed_time<0.4)
            
            if move_stop:
                move_cmd= Twist()

            else:
                if not offtrack and not retrack_hold:
                    # If normal, move in the appropriate direction
                    move_cmd.linear.x = copysign(speed * move_factor, error)
                    move_cmd.angular.z = spinz * move_factor
                else:
                    off_angle, blob_factor = navi.off_tracking(scan_object)
                    move_cmd.linear.x = 0
                    move_cmd.angular.z = copysign(blob_factor, off_angle)
                
            navi.cmd_vel.publish(move_cmd)

        return move_alignpass    

    def move_blob(self, goal, speed=0.25):
        odom_pose, move_tol = (None, 0.01)
        move_stop = False
        
        # Stop the robot
        move_cmd = Twist()
        self.navi.cmd_vel.publish(move_cmd)
        odom_pose, self.navi.align_angle = self.navi.get_odom()
        x_start, y_start= (odom_pose.x, odom_pose.y)
        normdir, ags_status = (copysign(1, speed), '')
        
        while not move_stop and not rospy.is_shutdown():
            self.r.sleep()
            odom_pose, self.navi.align_angle = self.navi.get_odom()
            ags_status= self.ags_status()
            scan_object= self.scan_block(normdir) 

            distance = sqrt(pow((odom_pose.x - x_start), 2) +
                            pow((odom_pose.y - y_start), 2))
                            
            # How close is goal?
            error =  (goal - distance) * normdir

            spinz, move_factor= self.navi.blob_tracking(error, ags_status, scan_object)
            
            if (abs(error) < move_tol) or move_stop:
                move_cmd= Twist()
                move_stop = True
            else:
                # If not, move in the appropriate direction
                move_cmd.linear.x = copysign(speed * move_factor, error)
                move_cmd.angular.z = spinz * move_factor
       
            self.navi.cmd_vel.publish(move_cmd)
        
        return ags_status
    
    
    """ get P and N gearing motion """
    def move_slow(self, goal, speed=0.25, onhold=''):
        odom_pose, move_tol = (None, 0.01)
        move_stop = False
        
        # Stop the robot
        move_cmd = Twist()
        self.navi.cmd_vel.publish(move_cmd)
        self.navi.align_angle = None
        odom_pose, _ = self.navi.get_odom()
        x_start, y_start= (odom_pose.x, odom_pose.y)
        normdir, ags_status = (copysign(1, speed), '')
        
        while not move_stop and not rospy.is_shutdown():
            self.r.sleep()
            odom_pose, _= self.navi.get_odom()
            ags_status= self.ags_status()
            scan_object= self.scan_block(normdir) 

            distance = sqrt(pow((odom_pose.x - x_start), 2) +
                            pow((odom_pose.y - y_start), 2))
                            
            # How close is goal?
            error =  (goal - distance) * normdir

            spinz, move_factor, move_stop = self.navi.auto_tracking(error, 
                                            ags_status, scan_object, onhold)
      
            move_stop |= is_term()      # override by term button            
            if (abs(error) < move_tol) or move_stop:
                move_cmd= Twist()
                move_stop = True
            else:
                # If not, move in the appropriate direction
                move_cmd.linear.x = copysign(speed * move_factor, error)
                move_cmd.angular.z = spinz * move_factor
       
            self.navi.cmd_vel.publish(move_cmd)
        
        return ags_status

    """ spin motion """
    def move_angular(self, goal, speed=0.4):
        odom_angle, move_goal, move_tol = (None, radians(goal), radians(0.6))
       
        # clear robot motion
        move_cmd = Twist()
        self.navi.cmd_vel.publish(move_cmd)  
        self.navi.align_angle = None
        _, odom_angle = self.navi.get_odom()
        last_angle, turn_angle = (odom_angle, 0)

        trig_goal= max(abs(move_goal) * 0.3, radians(18))
        error = move_goal - turn_angle
               
        while abs(error) > move_tol:
            if rospy.is_shutdown():
                return
            # speed should be positive
            move_factor= 0.3 if abs(error) < trig_goal else 1.0
            move_cmd.angular.z = copysign(speed * move_factor, error)
            self.navi.cmd_vel.publish(move_cmd)
            self.r.sleep()
         
            # Get the current rotation angle from tf                   
            _, odom_angle = self.navi.get_odom()
            self.ags_status()
            
            # Add to our total angle so far
            delta_angle = normalize_angle(odom_angle - last_angle)
            turn_angle += delta_angle

            # How close is goal?
            error = move_goal - turn_angle
            last_angle = odom_angle
                            
        # clear robot motion
        self.navi.cmd_vel.publish(Twist())

    # get right angle turn in anticlockwise 
    # if normdir>0, otherwise clockwise
    def move_right_angle(self, normdir=1):
        ZPP_angle = self.navi.ZPP_angle * normdir
        return self.move_angular(ZPP_angle, speed=0.6)        

    # get tracking orientation
    def move_orient(self):
        # Is parallel to track?
        for i in range(16):
            self.r.sleep()   
        attack = self.navi.ags_acute
        
        # attack-spin to align track
        if attack > 8:
            attack_angle= asin(attack / self.navi.base_length) * 0.4
            for i in range(8):
                self.r.sleep()              
            self.move_angular(degrees(attack_angle))


    """ matrix move_base plan """
    def do_ZBP(self):
        gear= self.gear
        ags_status= self.ags_status()
        
        # P gearing
        if gear=="P0":
            if ags_status=="on_track":
                do_goto(self.navi)    
                self.gear= "P2"

        elif gear =="P2":
            if ags_status=="on_track":
                self.gear= "P3"
       
        elif gear=="P3":
            self.move_orient()
            self.move_right_angle(normdir=1)
            self.gear= "P4"
      
        elif gear=="P4":
            ags_status = self.move_blob(self.navi.ZPP_dist, -0.25)
            self.navi.OLED_write("WAKEn2") 
            rospy.sleep(0.4)
            do_arrival(self.navi, home=True)
            rospy.sleep(0.6)
            self.navi.OLED_write("WAKEon") 
            self.do_autoTimer = 0
            set_move(True)
            self.gear= "P5"            

        elif gear=="P5":
            # autoback to home with do_autoTimer
            if is_move() and self.navi.ZBP_autoback!=0:
                rospy.sleep(1)
                self.do_autoTimer += 1
                if self.do_autoTimer > self.navi.ZBP_autoback:
                    do_goto(self.navi) 

            if is_goto(): 
                # button to go
                self.gear = "P6"                  
            
        elif gear=="P6":
            # charging position or start next search
            self.zone, self.gear = ("Search", "N0")            
    
    def do_search(self):
        gear= self.gear
        ags_status= self.ags_status()
        
        # N gearing
        if gear=="N0":
            if ags_status=="on_track" and is_term():
                self.zone, self.gear = ("ZAB", "R0")

            if ags_status=="on_track" and dispMsg.data=="holdA1":
                self.zone, self.gear = ("ZAB", "R0")     

            if ags_status=="on_track" and is_goto():
                plan_startAA(self.navi)
                self.zone, self.gear = ("ZAB", "R0")

            if is_goto():
                if ags_status=="off_track":
                    self.gear= "N1"
                if ags_status[0:4]=="bar_":
                    self.gear= "N2"
        
        elif gear=="N1":
            ags_status = self.move_slow(self.navi.ZPP_dist, onhold='bar_')
            self.gear= "N2"

        if gear=="N2":
            if is_term():
                self.zone, self.gear = ("ZAB", "R0")
            if ags_status=="bar_A1":
                self.gear= "N3"
            if ags_status=="bar_B1":
                self.gear= "N4"
                
        elif gear=="N3":
            if ags_status=="bar_A1":
                self.move_slow(0.2)
            self.gear= "N5"

        elif gear=="N4":
            if ags_status=="bar_B1":
                self.move_slow(0.3, -0.25)
            self.gear= "N5"
            
        elif gear=="N5":
            self.move_right_angle(normdir=-1)
            self.gear= "N6"

        elif gear=="N6":
            if ags_status=="on_track":
                self.gear= "N7"

        elif gear=="N7":
            self.zone, self.gear = ("Pose", "N0")

    def do_pose(self):
        gear= self.gear
        ags_status= self.ags_status()
        
        # N gearing
        if gear=="N0":
            if ags_status=="on_track":
                self.gear= "N1"
                
        elif gear=="N1":
            if self.navi.ZPB_dist!=0:
                # tracking orientation
                self.move_orient()
                # try run short distance onto station 1
                ags_status = self.move_slow(self.navi.ZPB_dist, onhold='on_A1')
            self.gear= "N2"

        if gear=="N2":
            if ags_status=="on_A1":
                self.gear= "N3"
            else:
                self.gear= "N4"
                
        elif gear=="N3":
            self.move_slow(0.2)
            do_arrival(self.navi)  
            # tracking A1 pose
            stamp= rospy.get_rostime()
            pose, _ = self.navi.get_odom()
            self.navi.tagzb = {'frame':plan_resp(self.navi),'stamp':stamp,'pose':pose}
            self.navi.markzab()
            self.gear= "N5"
            
        elif gear=="N4":
            do_arrival(self.navi)
            self.gear= "N5"

        elif gear=="N5":
            # rospy.sleep(1.6)
            self.gear= "N6"
            
        elif gear=="N6":
            if ags_status=="on_track":
                do_goto(self.navi)
                self.zone, self.gear = ("ZBA", "D0")

    def do_ZBA(self):
        gear= self.gear
        ags_status= self.ags_status()
        
        # D gearing
        if gear=="D0":
            if ags_status=="on_track" and is_goto():
                # override reverse mode by 'termAA' 
                if plan_goal(self.navi)<0:  
                    self.gear = "D6"
                else:
                    self.gear = "D1"

        if gear=="D1":
            do_goto(self.navi)
            # tracking orientation
            self.move_orient()
            self.gear= "D2"

        elif gear=="D2":
            # starting goal
            move_alignpass = self.move_drive()
            if move_alignpass:    set_align(5)
            self.gear= "D3"

        elif gear=="D3":
            if ags_status=="on_track":
                self.gear= "D4"
            
        elif gear=="D4":
            if is_move():
                do_arrival(self.navi)
                # tracking B1 pose
                stamp= rospy.get_rostime()
                pose, _ = self.navi.get_odom()
                self.navi.tagza = {'frame':plan_resp(self.navi),'stamp':stamp,'pose':pose}
                if is_station(5):    self.navi.markzab()
                rospy.sleep(1.6)
            if not self.navi.low_batt:
                # conti when not low battery
                self.gear= "D5"
            else:
                self.navi.OLED_write("WAKElb")
                rospy.sleep(1.6)  


        elif gear=="D5": 
            if is_goto():    
                # button to go
                if plan_goal(self.navi)>0:
                    self.zone, self.gear = ("ZBA", "D0")
                else:            
                    self.gear="D6"
    
        elif gear=="D6":
            self.zone, self.gear = ("ZAB", "R0")

    def do_ZAB(self):
        gear= self.gear
        ags_status= self.ags_status()

        # R gearing        
        if gear=="R0":
            if ags_status=="on_track" and is_goto():
                # override forward mode by 'term11' 
                if plan_goal(self.navi)>0:
                    self.zone, self.gear = ("ZBA", "D0")
                else:            
                    self.gear = "R1"

        if gear=="R1":
            self.navi.OLED_write("goto_0") 
            do_goto(self.navi)
            # tracking orientation
            self.move_orient()
            self.gear= "R2"

        elif gear=="R2":
            # starting goal
            move_alignpass = self.move_reverse()
            # already include backhome logic
            if move_alignpass:    set_align(1)
            self.gear= "R3"

        elif gear=="R3":
            self.do_autoTimer = 0
            if ags_status=="on_track":
                self.gear= "R4"
  
        elif gear=="R4":
            if is_move():
                do_arrival(self.navi)
                # tracking A1 pose
                stamp= rospy.get_rostime()
                pose, _ = self.navi.get_odom()
                self.navi.tagzb = {'frame':plan_resp(self.navi),'stamp':stamp,'pose':pose}
                rospy.sleep(0.6)
                if not is_station(5):    self.navi.markzab()
                if not is_station(5):    self.navi.OLED_write(get_WAKEn())
            #check return to charging site
            if is_backhome() and is_station(1):
                if self.navi.ZBP_autoback==0:
                    rospy.sleep(1.0)
                self.zone, self.gear = ("ZBP", "P0") 
            else:
                self.do_autoTimer = 0
                self.gear= "R5"

        elif gear=="R5":
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
                if plan_goal(self.navi)<0:  
                    self.zone, self.gear = ("ZBA", "D6")   
                else:
                    self.gear="R6"     

        elif gear=="R6":
            set_move(False)
            self.zone, self.gear = ("ZBA", "D0")

if __name__ == '__main__':
    """ TRIOGO main """
    try:
        navi= navSensors(heading="F400")
        NavGO(navi)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")

