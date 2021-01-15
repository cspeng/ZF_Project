#!/usr/bin/env python
"""
    --- laserscan reader ---
    
"""
PKG= 'dashgo_tools'
import roslib; roslib.load_manifest(PKG)
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Header
import numpy as  np

def blob_digitize(scan):
    scan= [x * (x<1.5) for x in scan]
    edges= np.histogram(scan,8)[1][::2]
    return np.digitize(scan, edges) * (np.array(scan)!=0)

def blob_neighbors(scan):
    labels = blob_digitize(scan)
    nearest= np.diff(labels)
    nearest= (np.insert(nearest,0,1)==0) + (np.append(nearest,1)==0)
    return (labels!=0) * nearest

class naviTest():
    """ TRIOGO sensing test """

    scan = {'back':[],'front':[],'hud_F':[], 'hud_B':[]}
    alt = False

    def __init__(self, heading):
        # Give the node a name
        rospy.init_node('navi', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        
        self.heading = heading
        rospy.Subscriber("scan", LaserScan, self.callback_msg_scan)
        #rospy.Subscriber("teleop_e31", Header, self.callback_msg_e31)     
        rospy.Subscriber("teleop_joystick", Header, self.callback_msg_joy)

    def get_scan_block(self, normdir=1):
        #interaction of radar hud
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
    
    def callback_msg_scan(self, msg):
        ranges= msg.ranges
        #(rmin, rmax)= (msg.range_min, msg.range_max)
        self.scan['front']= ranges[-20:] + ranges[0:20] 
        self.scan['back']= ranges[160:200]
        self.scan['hud_F']= ranges[-35:] + ranges[0:35]
        self.scan['hud_B']= ranges[145:215]

    def callback_msg_joy(self, msg):
        if msg.frame_id[:6]=='_CALL_':
            self.alt = not self.alt

    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the test...")
        rospy.sleep(1)

    def run(self):
        self.r = rospy.Rate(2)
        while not rospy.is_shutdown():
            if not self.alt:
                hud, rp =  ('hud_F', 'front')
                print(self.get_scan_object(1))
            else:
                hud, rp =  ('hud_B', 'back')
                print(self.get_scan_object(-1))

            rpscan, hudscan =  (self.scan[rp], self.scan[hud])
            scanmask = (np.array(rpscan)>0.2) 
            scanmask *= (np.array(rpscan)<1.4) 
            hudmask = (np.array(hudscan)>0.2)
            hudmask *= (np.array(hudscan)<0.5)
            print('# scan')
            print(scanmask*1)
            print('# hud')
            print(hudmask*1)
            print("\n")

            self.r.sleep()

if __name__ == '__main__':
    """ TRIOGO """
    try:
        test= naviTest(heading="F5D0")
        test.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
        
