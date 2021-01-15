#!/usr/bin/env python
"""
triogo teleop station panel
created by clarence PEMS@2020

teleop joystick interface
  >>> rfdata indicator: flash LED_BUILTIN on command with valid heading(remoteStr)
  >>> rfdata data: I0000_, _togg_
  >>> rfdata cmd: _PING_, WAKEff, WAKEon, WAKElb
  >>> cmd stop: termAA, termBB, stopAA, stopBB, stopB1, stopB2, stopB3, stopB4
  >>> cmd run: gotoAA, gotoBB, gotoB1, gotoB2, gotoB3, gotoB4, _goto_, _hold_
  >>>
"""
import rospy, serial
from std_msgs.msg import String, Header

is_shutdown = False
COM=7
#tty='/dev/ttyACM0'    #usb port from micro module
tty='/dev/port1'   #usb port thro AS32 rf-serial
timeout= 0.01
MRF = "Z.17"
termN=0; lastN=termN; lastResp=str(termN)*2

#ser = serial.Serial(port=COM-1, baudrate=9600, timeout=timeout)
ser = serial.Serial(tty, 9600, timeout=timeout)
msg= Header()


def cmd_write(cmd_data='_PING_'):
    init= [ord(x) for x in MRF][0:4]
    data= init + [ord(x) for x in cmd_data][0:6]
    # print ["0x%02X" % x for x in data]
    # ser.write(data) for version 3.1
    data_str= "".join([chr(x) for x in data])     #pyserial 2.6
    print(data_str)
    ser.write(data_str)

def callback_cmd_write(data):
    if len(data.data) == 6:
        cmd_write(data.data)

def box_publish(self,box_data):
    # print ["0x%02X" % ord(x) for x in box_data]
    msg.seq += 1
    msg.stamp= rospy.Time.now()
    msg.frame_id= box_data
    self.publish(msg)

def comm_listern():
    tele_pub = rospy.Publisher('teleop_box', Header, queue_size=2)

    while not (rospy.is_shutdown() or is_shutdown):
        sync = ser.read(1)
        if sync == 'R':
            if ser.read(1)=='.':
                zoneID = 'R.' + ser.read(2)
                data = ser.read(6)
                if len(data) == 6:
                    # pub response
                    box_publish(tele_pub, data)

        rospy.sleep(0.1)


if __name__ == '__main__':
    try:
        rospy.init_node('mitgo_panel', anonymous=True)        
        rospy.Subscriber("teleop_cmd", String, callback_cmd_write)
        rospy.loginfo("subscribe teleop_cmd on mitogo-%s" % MRF)
        rospy.loginfo("publish teleop_box on mitogo-%s" % MRF)
        cmd_write('_PING_')
        comm_listern()

    except rospy.ROSInterruptException:
        rospy.loginfo("teleop is close")
    
    except serial.serialutil.SerialException:
        rospy.loginfo("teleop conn is loss")
    
    finally:
        is_shutdown = True
        rospy.loginfo("mitogo-%s is shutdown" % (MRF))        
        ser.close()
