#!/usr/bin/env python
"""
mitgo teleop RFID Reader
created by clarence PEMS@2020


"""
import rospy, serial
from std_msgs.msg import String, Header

is_shutdown = False
tty='/dev/ttyUSB0'
timeout= 0.01

ser = serial.Serial(tty, 19200, timeout=timeout,parity=serial.PARITY_EVEN, rtscts=1)
msg= Header()


def reader_publish(self, reader_data):
    #print ["0x%02X" % ord(x) for x in reader_data]
    msg.seq += 1
    msg.stamp= rospy.Time.now()
    msg.frame_id= reader_data
    self.publish(msg)

def comm_listern():
    tele_pub = rospy.Publisher('teleop_reader', Header, queue_size=2)  

    while not (rospy.is_shutdown() or is_shutdown):
        sync= ser.read(1)
        sync= "0x%02X" % ord(sync) if len(sync) else "0x00"
        if sync=="0x02":
            floorID= ser.read(4)
            data= ser.read(6)
            crc= ser.read(2)
            if len(data)==6:
                # pub response
                reader_publish(tele_pub, floorID + data)

        rospy.sleep(0.1)


if __name__ == '__main__':
    try:
        rospy.init_node('mitgo_reader', anonymous=True)
        rospy.loginfo("publish teleop_reader on mitgo")
        comm_listern()

    except rospy.ROSInterruptException:
        rospy.loginfo("teleop_reader is close")
    
    except serial.serialutil.SerialException:
        rospy.loginfo("teleop_reader conn is loss")
    
    finally:
        is_shutdown = True
        rospy.loginfo("mitgo-reader is shutdown")
        ser.close()
