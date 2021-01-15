#!/usr/bin/env python
"""
mitgo teleop RFID Reader
created by clarence PEMS@2020


"""
import rospy, serial
import modbus_tk.defines as cst
import modbus_tk.modbus_rtu as modbus_rtu
from std_msgs.msg import String, Header

is_shutdown = False
tty='/dev/portb3'
timeout= 0.01

ser = serial.Serial(tty, baudrate = 19200,bytesize = 8,parity = serial.PARITY_EVEN,stopbits = serial.STOPBITS_ONE)


master = modbus_rtu.RtuMaster(ser)
master.set_timeout(0.025)

#master.execute(0x02,cst.WRITE_SINGLE_REGISTER,0x00,output_value = 0x03) //auto read

msg= Header()



def reader_publish(self, reader_data):
    print (reader_data)
    #reader_data = ["%02X" % ord(x) for x in reader_data]
    #reader_data = "".join(reader_data)
    #reader_data = reader_data[::-1]
    msg.seq += 1
    msg.stamp= rospy.Time.now()
    msg.frame_id= reader_data
    self.publish(msg)

def comm_listern():
    isRepeat = False
    lastIDdata = 0
    tele_pub = rospy.Publisher('teleop_reader', Header, queue_size=2)  
    
    while not (rospy.is_shutdown() or is_shutdown):
        try:
            
            TMPx = master.execute(0x02, cst.READ_HOLDING_REGISTERS,0x0E,7)
            id_data = ["%04X"%x for x in TMPx[1:4]]
            id_data = "".join(id_data)
            floorID = id_data[:6]
            '''
            if(floorID[:4] == "0FFF"):
                if(lastIDdata!=id_data[6:10]):
                    lastIDdata = id_data[6:10]
                    isRepeat = False
                else:
                    isRepeat = True
            '''        
            if(isRepeat == False):
                if(floorID == "0FFF00"):
                    data = "MIT" + id_data[6:10]
                elif(floorID == "0FFFF0"):
                    data = "_RATE02"
                elif(floorID == "0FFFFF"):
                    data = "BRANCH" + id_data[6:10]
                else:
                    data = "00"
                #print(data)
              
                reader_publish(tele_pub, data)
            '''
            sync= ser.read(1)
            sync= "0x%02X" % ord(sync) if len(sync) else "0x00"
            if sync=="0x02":
                ser.read(4)
                data= ser.read(3)
                floorID = ser.read(2)
                crc= ser.read(2)
                if len(data)==3:
                    #data = data[::-1]
                    # pub response
                    reader_publish(tele_pub, floorID + data)
            '''
            rospy.sleep(0.01)
        except Exception as e:
            print(e)
    

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
