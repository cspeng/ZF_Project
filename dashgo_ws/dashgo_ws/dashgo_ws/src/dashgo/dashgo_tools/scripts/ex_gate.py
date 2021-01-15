#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import String, Header
import serial
import time

COM='COM10'
tty='/dev/ttyCOM4'
timeout= 0.01

MF = "F0"; MR = "Z0"
is_shutdown = False

ser = serial.Serial(tty, 9600, timeout=timeout)
msg = Header()

def callback_telemsg_GATE(data):
    GATE_write(data.data)

def CMD_publish(self, teleop_cmd):
    #print ["0x%02X" % ord(x) for x in teleop_cmd]
    msg.seq += 1
    msg.stamp= rospy.Time.now()
    msg.frame_id= teleop_cmd
    self.publish(msg)

def GATE_write(disp_cmd):
    init= [0x02] + [ord(x) for x in MF + MR][0:4]
    data= init + [ord(x) for x in disp_cmd][0:8] + [0x03]
    #print ["0x%02X" % x for x in data]
    #ser.write(data) for version 3.1
    data_str= "".join([chr(x) for x in data])     #pyserial 2.6
    ser.write(data_str)
    
def comm_listern():   
    CMD_pub = rospy.Publisher('telecmd_GATE', Header, queue_size=2)  
    GATE_write("_MOT-dn_")

    while not (rospy.is_shutdown() or is_shutdown):
        sync= ser.read(1)
        sync= "0x%02X" % ord(sync) if len(sync) else "0x00"
        if sync=="0x02":
            floorID= ser.read(4)
            if floorID==MF + MR:
                data= ser.read(8)
                sync= ser.read(1)
                sync= "0x%02X" % ord(sync) if len(sync) else ""
                if len(data)==8 and sync=="0x03":
                    #print(data)
                    CMD_publish(CMD_pub, data)

        time.sleep(0.02)

if __name__ == '__main__':
    try:
        rospy.init_node('ex_gate', anonymous=True)   
        rospy.Subscriber("telemsg_GATE", String, callback_telemsg_GATE)
	
        rospy.loginfo("publish telecmd_GATE on Triogo-%s" % MF+MR)	
        print("ex_gate server is started.")
        comm_listern()

    except rospy.ROSInterruptException:
        print("GATE conn is close")

    except serial.serialutil.SerialException:
        print("GATE conn is loss")

    finally:
        is_shutdown = True
        rospy.loginfo("GATE on Triogo-%s is shutdown" % (MF+MR))        
        ser.close()

