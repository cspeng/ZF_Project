#!/usr/bin/env python
"""
triogo teleop communication
created by clarence PEMS@2017

display cmd interface
    term states where N = 1, 2, 3, 4
    "N    --  "    holdN_
    "N --     "    hold_N
    "N -- -- A"    hold__
    "N    --  "    hold_A
    "N --     "    holdA_  

    "N  > >  A"    gotoNA
    "N  < <  A"    gotoAN   
    "  beep   "    WAKEnN

    include park state where N = 0, 1, 2, 3, 4
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

is_shutdown = False
#tty2 = '/dev/ttyCOM3'
tty2 = '/dev/port2'
timeout = 0.01
CHANNEL, SOLE = (0x50, "00")
LINK, ZONE = (0X20, "Z.")

ser2 = serial.Serial(tty2, 9600, timeout=timeout)
msg = Header()


def e31remote_publish(self, teleop_cmd):
    # print ["0x%02X" % ord(x) for x in teleop_cmd]
    msg.seq += 1
    msg.stamp= rospy.Time.now()
    msg.frame_id= teleop_cmd
    self.publish(msg)


def e31remote_PING(e31_cmd='_PING_'):
    init= [0xFF, 0xFF, LINK] + [ord(x) for x in ZONE + SOLE][0:4]
    data= init + [ord(x) for x in e31_cmd][0:6]
    # print ["0x%02X" % x for x in data]
    # ser.write(data) for version 3.1
    data_str= "".join([chr(x) for x in data])     #pyserial 2.6
    ser2.write(data_str)


def comm_listern():
    tele_pub = rospy.Publisher('teleop_e31', Header, queue_size=2)

    while not (rospy.is_shutdown() or is_shutdown):
        sync = ser2.read(1)
        if sync == 'Z':
            if ser2.read(1)=='.':
                zoneID = 'Z.' + ser2.read(2)
                data = ser2.read(6)
                if len(data) == 6:
                    # pub response
                    e31remote_publish(tele_pub, zoneID+data)

        rospy.sleep(0.1)


def callback_teleop_OLED(data):
    global disp_cmd
    if data.data == 'stopAA':
        e31remote_PING()


if __name__ == '__main__':
    try:
        rospy.init_node('triogo_e31port', anonymous=True)
        rospy.Subscriber("teleop_OLED", String, callback_teleop_OLED)
        rospy.loginfo("publish teleop_e31 on Triogo teleop %s" % (ZONE+SOLE))
        e31remote_PING()
        comm_listern()

    except rospy.ROSInterruptException:
        rospy.loginfo("teleop is close")

    except serial.serialutil.SerialException:
        rospy.loginfo("teleop conn is loss")

    finally:
        is_shutdown = True
        rospy.loginfo("Triogo teleop %s is shutdown" % (ZONE+SOLE))
        ser2.close()
