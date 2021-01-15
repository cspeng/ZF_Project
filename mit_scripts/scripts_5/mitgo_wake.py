#!/usr/bin/env python
"""
script: mitgo_wake.py
remark: buzzer wake-up box communication with mitgo
created by: clarence PEMS@2020
remote command:  _PING_ RESUME WAKEn1 WAKEn2 WAKEn3 WAKEn4 
"""

from __future__ import print_function
PKG= 'mitgo_tools'
import roslib; roslib.load_manifest(PKG)
import rospy, serial
from std_msgs.msg import String, Header

is_shutdown = False
tty='/dev/port4'
timeout= 0.01

equipID="MIT+"
abnormal_mode  = ["urgent","buffer","off_track"]
ser = serial.Serial(tty, 9600, timeout=timeout)
msg= Header()
count  = Header()
def callback_teleop_remote(data):
    remote_write(data.data)

def callback_status_resume(data):
    if(abnormal_mode[0] in data.data) or (abnormal_mode[1] in data.data) or (abnormal_mode[2] in data.data):
        if(count.seq%50 == 0):               ###2s
            remote_write("RESUME")
        count.seq += 1
       # print(count.seq)
    

def remote_write(disp_cmd):
    init= [ord(x) for x in equipID]
    data= init + [ord(x) for x in disp_cmd][0:6]
    data_str= "".join([chr(x) for x in data])
    ser.write(data_str)
    

def status_publish(self, teleop_cmd):
    msg.seq += 1
    msg.stamp= rospy.Time.now()
    msg.frame_id= teleop_cmd
    self.publish(msg)

def comm_listern():
    tele_pub = rospy.Publisher('wake_status', Header, queue_size=2)  

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
        rospy.init_node('mitgo_wake', anonymous=True)        
        rospy.Subscriber("wake_cmd", String, callback_teleop_remote)
        rospy.Subscriber("mcat_status", String, callback_status_resume)
        rospy.loginfo("subscribe /wake_cmd on mitgo %s" % (equipID))
        rospy.loginfo("publish /wake_status on mitgo %s" % (equipID))
        comm_listern()

    except rospy.ROSInterruptException:
        rospy.loginfo("mitgo wake int is close")
    
    except serial.serialutil.SerialException:
        rospy.loginfo("mitgo wake conn is loss")
    
    finally:
        is_shutdown = True
        rospy.loginfo("mitgo wake %s is shutdown" % (equipID))        
        ser.close()
