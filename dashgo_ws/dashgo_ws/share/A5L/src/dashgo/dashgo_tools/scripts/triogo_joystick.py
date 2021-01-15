#!/usr/bin/env python
"""
triogo teleop communication
created by clarence PEMS@2017

teleop joystick interface
    change into term    _term_  serect stick for 2 seconds
	dec station N       _decN_  push rear
	inc station N       _incN_  push front
   toggle goto/hold    _togg_  press button

display cmd interface
    term states <> park where N = 1, 2, 3, 4, 5
    "N    --  "    holdN_
    "N --     "    hold_N
    "N -- -- A"    hold__
    "N    -- A"    hold_A
    "N --    A"    holdA_

    "N  > >   "    gotoN_
    "N  < <   "    goto_N
    "1 2 3 4 A"    tremNN
    "N.     .A"    stopNN
    ".       N"    stop.N
    " ERROR N "    -err-N
    " NO CONN "    -err-6
    " RECHARGE"    -err-7
    " LOW BAT "    -err-7
    "WRONG  ID"    -err-8
    "OFF TRACK"    -err-9

	gotoAN states <> holdA_ where N = 0, 1, 2, 3, 4
	"N     < A"
	"N    <  A"
	"N   <   A"
	"N  <    A"
	"N <     A"

	gotoNA states <> hold_A where N = 1, 2, 3, 4
	"N >     A"
	"N  >    A"
	"N   >   A"
	"N    >  A"
	"N     > A"

"""
import rospy, serial
from std_msgs.msg import String, Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseArray

is_shutdown = False
COM=7
tty='/dev/ttyCOM4'
timeout= 0.01
MF= "F5"; MR= "A0"
termN=0; lastN=termN; lastResp=str(termN)*2

#ser = serial.Serial(port=COM-1, baudrate=9600, timeout=timeout)
ser = serial.Serial(tty, 9600, timeout=timeout)
msg= Header()
strip = Marker()

def callback_teleop_OLED(data):
    OLED_write(data.data)

def callback_track_poses(data):
    pos0, pos1 = tuple(data.poses[0:2])
    pos0, pos1 = (pos0.position, pos1.position)
    if len(strip.points):
        p0 = strip.points[0]; p1= strip.points[1]
        p0.x, p0.y, p0.z = (pos0.x, pos0.y, pos0.z)
        p1.x, p1.y, p1.z = (pos1.x, pos1.y, pos1.z)       

def joystick_publish(self, teleop_cmd):
    #print ["0x%02X" % ord(x) for x in teleop_cmd]
    msg.seq += 1
    msg.stamp= rospy.Time.now()
    msg.frame_id= teleop_cmd
    self.publish(msg)

def OLED_write(disp_cmd):
    init= [0x02] + [ord(x) for x in MF + MR][0:4]
    data= init + [ord(x) for x in disp_cmd][0:6] + [0x03]
    #print ["0x%02X" % x for x in data]
    #ser.write(data) for version 3.1
    data_str= "".join([chr(x) for x in data])     #pyserial 2.6
    ser.write(data_str)

def init_strip():
    strip.header.frame_id = "/odom_combined";
    strip.header.stamp = rospy.Time.now();
    strip.ns = "triogo_joystick";
    strip.action = Marker.ADD;
    # %Tag(ID)%
    strip.id= 0;
    strip.type = Marker.LINE_STRIP;    
    strip.scale.x = 0.06;
    strip.pose.orientation.w = 1.0;
    strip.color.r, strip.color.g = (1.0, 1.0)
    strip.color.a = 1.0;

    # setup coordinates of strip
    p1 = Point(); p2 = Point()
    p1.x, p1.y, p1.z = (0.0, 0.0 , 0.0)
    strip.points.append(p1)
    p2.x, p2.y, p2.z = (5.0, 0.0 , 0.0)
    strip.points.append(p2)    


def comm_listern():
    tele_pub = rospy.Publisher('teleop_joystick', Header, queue_size=2)  
    mark_pub = rospy.Publisher('track_strip', Marker, queue_size=10)      

    init_strip(); mark_pub.publish(strip)
    
    OLED_write("stop00"); msg.frame_id= "stop"+lastResp

    while not (rospy.is_shutdown() or is_shutdown):
        sync= ser.read(1)
        sync= "0x%02X" % ord(sync) if len(sync) else "0x00"
        if sync=="0x02":
            floorID= ser.read(4)
            if floorID==MF + MR:
                data= ser.read(6)
                sync= ser.read(1)
                sync= "0x%02X" % ord(sync) if len(sync) else ""
                if len(data)==6 and sync=="0x03":
                    # pub response
                    joystick_publish(tele_pub, data)

        mark_pub.publish(strip)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        rospy.init_node('triogo_joystick', anonymous=True)        
        rospy.Subscriber("teleop_OLED", String, callback_teleop_OLED)
        rospy.Subscriber("track_poses", PoseArray, callback_track_poses)
        rospy.loginfo("subscribe teleop_OLED on Triogo-%s" % MF)
        rospy.loginfo("publish teleop_joystick on Triogo-%s" % MF)
        comm_listern()

    except rospy.ROSInterruptException:
        rospy.loginfo("teleop is close")
    
    except serial.serialutil.SerialException:
        rospy.loginfo("teleop conn is loss")
    
    finally:
        is_shutdown = True
        rospy.loginfo("Triogo-%s is shutdown" % (MF+MR))        
        ser.close()
