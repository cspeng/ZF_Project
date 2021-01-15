#!/usr/bin/env python
"""
iron teleop communication
created by clarence PEMS@2018
"""
from __future__ import print_function
PKG= 'iron_socket'
import roslib; roslib.load_manifest(PKG)
import rospy, serial
from std_msgs.msg import String, Header

is_shutdown = False
tty='/dev/ttyCOM4'
timeout= 0.01

MF= "F5"; MR= "A0"    #not used in coding
equipID="CAM+"

#ser = serial.Serial(port=COM-1, baudrate=9600, timeout=timeout)
ser = serial.Serial(tty, 9600, timeout=timeout)
msg= Header()

def callback_teleop_remote(data):
    remote_write(data.data)

def remote_write(disp_cmd):
    init= [ord(x) for x in equipID]
    data= init + [ord(x) for x in disp_cmd][0:6]
    #print(["0x%02X" % x for x in data])
    #ser.write(data) for version 3.1
    data_str= "".join([chr(x) for x in data])     #pyserial 2.6
    ser.write(data_str)

def status_publish(self, teleop_cmd):
    #print(["0x%02X" % ord(x) for x in teleop_cmd])
    msg.seq += 1
    msg.stamp= rospy.Time.now()
    msg.frame_id= teleop_cmd
    self.publish(msg)

def comm_listern():
    tele_pub = rospy.Publisher('tele_status', Header, queue_size=2)  

    while not (rospy.is_shutdown() or is_shutdown):
        sync= ser.read(1)
        sync= "0x%02X" % ord(sync) if len(sync) else "0x00"
        if sync=="0x02":
            mID= ser.read(4)
            if mID==equipID:
                data= ser.read(6)
                sync= ser.read(1)
                sync= "0x%02X" % ord(sync) if len(sync) else ""
                if len(data)==6 and sync=="0x03":
                    # pub response
                    status_publish(tele_pub, data)

        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        rospy.init_node('iron_sensors', anonymous=True)        
        rospy.Subscriber("teleop_remote", String, callback_teleop_remote)
        rospy.loginfo("subscribe /teleop_remote on Trio %s-%s" % (MF,equipID))
        rospy.loginfo("publish /teleop_status on Trio %s-%s" % (MF,equipID))
        comm_listern()

    except rospy.ROSInterruptException:
        rospy.loginfo("teleop is close")
    
    except serial.serialutil.SerialException:
        rospy.loginfo("teleop conn is loss")
    
    finally:
        is_shutdown = True
        rospy.loginfo("Trio %s-%s is shutdown" % (MF,equipID))        
        ser.close()
