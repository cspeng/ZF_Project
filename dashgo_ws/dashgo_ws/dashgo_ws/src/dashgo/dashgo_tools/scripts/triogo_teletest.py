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
    term states <> park where N = 1, 2, 3, 4
    "N    --  "    holdN_
    "N --     "    hold_N
    "N -- -- A"    hold__
    "N    --  "    hold_A
    "N --     "    holdA_  

    "N  > >  A"    gotoNA
    "N  < <  A"    gotoAN   
    "  beep   "    WAKEnN

    N = 0, 1, 2, 3, 4
    "N  > >   "    gotoN_
    "N  < <   "    goto_N 
    "N.     .A"    stopNN    
    
    "   > >    "    term11
    "   < <    "    termAA

    "         "    WAKEff
    " YELLOW  "    WAKEon
    " LOW BAT "    WAKElb
"""

import rospy, serial
from std_msgs.msg import String, Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseArray

timeout= 0.01
is_shutdown = False
tty1='/dev/ttyCOM4'
# tty2 = '/dev/ttyCOM3'
tty2 = '/dev/ttyebyte'

tele_e31, tele_joy= None * 2
MF, MR= ("F4", "J1")
# MF= "F5"; MR= "D0"

CHANNEL, SOLE = (0x50, "00")
LINK, ZONE = (0X20, "Z.")

termN=0; lastN=termN; lastResp=str(termN)*2

#ser = serial.Serial(port=COM-1, baudrate=9600, timeout=timeout)
ser = serial.Serial(tty1, 9600, timeout=timeout)
ser2 = serial.Serial(tty2, 9600, timeout=timeout)
msg= Header()
msg2= Header()
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
    global tele_e31, tele_joy
    tele_joy = rospy.Publisher('teleop_joystick', Header, queue_size=2)  
    tele_e31 = rospy.Publisher('teleop_e31', Header, queue_size=2)
    mark_pub = rospy.Publisher('track_strip', Marker, queue_size=10)      

    init_strip()
    mark_pub.publish(strip)
    
    OLED_write("stop00")
    msg.frame_id= "stop"+lastResp
    e31remote_PING()

    while not (rospy.is_shutdown() or is_shutdown):
        joystick_read()
        e31remote_read()
        mark_pub.publish(strip)
        rospy.sleep(0.1)


def joystick_read():
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
                joystick_publish(data)

        
def e31remote_PING(e31_cmd='_PING_'):
    init= [0xFF, 0xFF, LINK] + [ord(x) for x in ZONE + SOLE][0:4]
    data= init + [ord(x) for x in e31_cmd][0:6]
    # print ["0x%02X" % x for x in data]
    # ser.write(data) for version 3.1
    data_str= "".join([chr(x) for x in data])     #pyserial 2.6
    ser2.write(data_str)


def e31remote_read():
    sync = ser2.read(1)
    if sync == 'Z':
        if ser2.read(1)=='.':
            zoneID = 'Z.' + ser2.read(2)
            data = ser2.read(6)
            if len(data) == 6:
                # pub response
                e31remote_publish(zoneID+data)


def joystick_publish(teleop_cmd):
    #print ["0x%02X" % ord(x) for x in teleop_cmd]
    msg.seq += 1
    msg.stamp= rospy.Time.now()
    msg.frame_id= teleop_cmd
    tele_joy.publish(msg)


def e31remote_publish(teleop_cmd):
    # print ["0x%02X" % ord(x) for x in teleop_cmd]
    msg2.seq += 1
    msg2.stamp= rospy.Time.now()
    msg2.frame_id= teleop_cmd
    tele_e31.publish(msg2)


if __name__ == '__main__':
    try:
        rospy.init_node('triogo_joystick', anonymous=True)        
        rospy.Subscriber("teleop_OLED", String, callback_teleop_OLED)
        rospy.Subscriber("track_poses", PoseArray, callback_track_poses)
       
        rospy.loginfo("subscribe teleop_OLED on Triogo-%s" % (MF+MR))
        rospy.loginfo("publish teleop_joystick on Triogo-%s" % (MF+MR))
        rospy.loginfo("publish teleop_e31 on Triogo teleop %s" % (ZONE+SOLE))

        comm_listern()

    except rospy.ROSInterruptException:
        rospy.loginfo("teleop is close on Triogo")
    
    except serial.serialutil.SerialException:
        rospy.loginfo("teleop conn is loss on Trigo")
    
    finally:
        is_shutdown = True
        rospy.loginfo("Triogo-%s is shutdown" % (MF+MR))        
        ser.close()
        ser2.close()
