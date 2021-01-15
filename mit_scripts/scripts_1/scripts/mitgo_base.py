#!/usr/bin/env python
"""
mitgo teleop Base Communiction
created by clarence PEMS@2020

parse status label form base:
run  06 +05 0C 00 xx bin(xxxx x1xx) xx xx xx xx [xx]*20
wait  06 +05 0C 00 xx bin(1xxx xxxx) xx xx xx xx [xx]*20
ready  06 +05 0C 00 xx xx bin(xxxx xxx1) xx xx xx [xx]*20
arrival  06 +05 0C 00 xx xx bin(xxxx xx1x) xx xx xx [xx]*20
backward  06 +05 0C 00 xx xx bin(xx1x xxxx) xx xx xx [xx]*20
forward  06 +05 0C 00 xx xx bin(xx0x xxxx) xx xx xx [xx]*20

# get stamp ID where has already passed ID card
stamp_ID1  06 +05 0C 00 xx xx xx bin(0000 0001) xx xx [xx]*20
stamp_ID2  06 +05 0C 00 xx xx xx bin(0000 0010) xx xx [xx]*20
stamp_ID3  06 +05 0C 00 xx xx xx bin(0000 0011) xx xx [xx]*20
stamp_ID4  06 +05 0C 00 xx xx xx bin(0000 0100) xx xx [xx]*20
stamp_IDA  06 +05 0C 00 xx xx xx bin(0000 0101) xx xx [xx]*20

# get target ID where show the stop point
target_ID1  06 +05 0C 00 xx xx xx xx xx bin(0000 0001) xx [xx]*20
target_ID2  06 +05 0C 00 xx xx xx xx xx bin(0000 0010) xx [xx]*20
target_ID3  06 +05 0C 00 xx xx xx xx xx bin(0000 0011) xx [xx]*20
target_ID4  06 +05 0C 00 xx xx xx xx xx bin(0000 0100) xx [xx]*20
target_IDA  06 +05 0C 00 xx xx xx xx xx bin(0000 0101) xx [xx]*20

# get last arrival ID where show the last stop point
at_ID1  06 +05 0C 00  xx xx xx xx xx xx bin(0000 0001) [xx]*20
at_ID2  06 +05 0C 00  xx xx xx xx xx xx bin(0000 0010) [xx]*20
at_ID3  06 +05 0C 00  xx xx xx xx xx xx bin(0000 0111) [xx]*20
at_ID4  06 +05 0C 00  xx xx xx xx xx xx bin(0000 0100) [xx]*20
at_IDA  06 +05 0C 00  xx xx xx xx xx xx bin(0000 0101) [xx]*20

urgent_stop(abnormal reusme)  06 +05 0C  00 xx xx xx xx 08 xx.[xx]*20
buffer_stop(abnormal reusme)  06 +05 0C  00 xx xx xx xx 0A.xx.[xx]*20
obstacle_stop(object impact)  06 +05 0C  00 xx xx xx xx 4C.xx [xx]*20

volt_262  06 +05 0C 00  20 C1 12 05 0A 00 05 00 00 00 00 00 00 00 (06 01) 00 00 00 00 00 00 00 00 1A 01
measured battery voltage = (0106)/10


send command to base:-
05 05 00 00 00 00 00 00 00 00 00 00 00 05 00    STATUS
05 05 00 01 00 00 00 00 00 00 00 00 00 06 00    HOLD
05 05 00 02 00 00 00 00 00 00 00 00 00 07 00    SLOW
05 05 00 20 00 00 00 00 00 00 00 00 00 25 00    RESUME
05 05 00 40 00 00 00 00 00 00 00 00 00 45 00    START
05 05 00 80 00 00 00 00 00 00 00 00 00 85 00    STOP
05 05 00 00 02 00 00 00 00 00 00 00 00 07 00    REVERSE

set speed rate command:
05 05 00 00 04 00 00 00 00 00 00 00 00 09 00    RATE1
05 05 00 00 08 00 00 00 00 00 00 00 00 0D 00    RATE2
05 05 00 00 0C 00 00 00 00 00 00 00 00 11 00    RATE3
05 05 00 00 10 00 00 00 00 00 00 00 00 15 00    RATE4
05 05 00 00 14 00 00 00 00 00 00 00 00 19 00    RATE5
05 05 00 00 18 00 00 00 00 00 00 00 00 1D 00    RATE6
05 05 00 00 1C 00 00 00 00 00 00 00 00 21 00    RATE7
05 05 00 00 20 00 00 00 00 00 00 00 00 25 00    RATE8
05 05 00 00 24 00 00 00 00 00 00 00 00 29 00    RATE9
05 05 00 00 28 00 00 00 00 00 00 00 00 2D 00    RATEa
05 05 00 00 2C 00 00 00 00 00 00 00 00 31 00    RATEb
05 05 00 00 30 00 00 00 00 00 00 00 00 35 00    RATEc
05 05 00 00 34 00 00 00 00 00 00 00 00 39 00    RATEd
05 05 00 00 38 00 00 00 00 00 00 00 00 3D 00    RATEe
05 05 00 00 3C 00 00 00 00 00 00 00 00 41 00    RATEf

05 05 00 00 40 00 00 00 00 00 00 00 00 45 00    switch to read external data
05 05 00 00 80 00 00 00 00 00 00 00 00 85 00    switch to write external data

set target command:
05 05 00 00 00 01 00 00 00 00 00 00 00 06 00    setID1
05 05 00 00 00 02 00 00 00 00 00 00 00 07 00    setID2
05 05 00 00 00 03 00 00 00 00 00 00 00 08 00    setID3
05 05 00 00 00 04 00 00 00 00 00 00 00 09 00    setID4
05 05 00 00 00 05 00 00 00 00 00 00 00 0A 00    setID0
05 05 00 00 00 00 04 00 00 00 00 00 00 09 00    clearID (valid when stop)

save arrival command:
05 05 00 00 00 01 00 00 00 00 00 00 00 06 00    saveID1
05 05 00 00 00 02 00 00 00 00 00 00 00 07 00    saveID2
05 05 00 00 00 03 00 00 00 00 00 00 00 08 00    saveID3
05 05 00 00 00 04 00 00 00 00 00 00 00 09 00    saveID4
05 05 00 00 00 05 00 00 00 00 00 00 00 0A 00    saveID0

"""

import rospy, serial
from std_msgs.msg import String, Header, Bool

is_shutdown = False
tty='/dev/portb4'
timeout= 0.01

#ser = serial.Serial(tty, 38400, timeout=timeout)
ser = serial.Serial(tty, baudrate = 38400,bytesize = 8,parity = serial.PARITY_ODD,stopbits = serial.STOPBITS_ONE,timeout = timeout )
msg= Header()

PING_ARRAY = [05, 05, 00] + [00]*10 + [05, 00]


def cmd_write(cmd_data=PING_ARRAY):
    #data_str= "".join([chr(x) for x in cmd_data])     #pyserial 2.6
    print(cmd_data)
    #ser.write(data_str)
    ser.write(cmd_data)

def callback_cmd_label(data):
    label = data.data
    if label=="STATUS":
        c = PING_ARRAY
    elif label=="HOLD":
        c = [05, 05, 00, 0x01, 00, 00, 00, 00, 00, 00, 00, 00, 00, 0x06, 00]
    elif label=="SLOW":
        c = [05, 05, 00, 0x02, 00, 00, 00, 00, 00, 00, 00, 00, 00, 0x07, 00]
    elif label=="RESUME":
        c = [05, 05, 00, 0x20, 00, 00, 00, 00, 00, 00, 00, 00, 00, 0x25, 00]
    elif label=="START":
        c = [05, 05, 00, 0x40, 00, 00, 00, 00, 00, 00, 00, 00, 00, 0x45, 00]
    elif label=="STOP":
        c = [05, 05, 00, 0x80, 00, 00, 00, 00, 00, 00, 00, 00, 00, 0x85, 00]
    elif label=="REVERSE":
        c = [05, 05, 00, 00, 0x02, 00, 00, 00, 00, 00, 00, 00, 00, 0x07, 00]
        
    elif label=="RATE1":
        c = [05, 05, 00, 00, 0x04, 00, 00, 00, 00, 00, 00, 00, 00, 0x09, 00]
    elif label=="RATE2":
        c = [05, 05, 00, 00, 0x08, 00, 00, 00, 00, 00, 00, 00, 00, 0x0D, 00]
    elif label=="RATE3":
        c = [05, 05, 00, 00, 0x0C, 00, 00, 00, 00, 00, 00, 00, 00, 0x11, 00]
    elif label=="RATE4":
        c = [05, 05, 00, 00, 0x10, 00, 00, 00, 00, 00, 00, 00, 00, 0x15, 00]
    elif label=="RATE5":
        c = [05, 05, 00, 00, 0x14, 00, 00, 00, 00, 00, 00, 00, 00, 0x19, 00]
    elif label=="RATE6":
        c = [05, 05, 00, 00, 0x18, 00, 00, 00, 00, 00, 00, 00, 00, 0x1D, 00]
    elif label=="RATE7":
        c = [05, 05, 00, 00, 0x1C, 00, 00, 00, 00, 00, 00, 00, 00, 0x21, 00]
    elif label=="RATE8":
        c = [05, 05, 00, 00, 0x20, 00, 00, 00, 00, 00, 00, 00, 00, 0x25, 00]
    elif label=="RATE9":
        c = [05, 05, 00, 00, 0x24, 00, 00, 00, 00, 00, 00, 00, 00, 0x29, 00]
    elif label=="RATEa":
        c = [05, 05, 00, 00, 0x28, 00, 00, 00, 00, 00, 00, 00, 00, 0x2D, 00]
    elif label=="RATEb":
        c = [05, 05, 00, 00, 0x2C, 00, 00, 00, 00, 00, 00, 00, 00, 0x31, 00]
    elif label=="RATEc":
        c = [05, 05, 00, 00, 0x30, 00, 00, 00, 00, 00, 00, 00, 00, 0x35, 00]
    elif label=="RATEd":
        c = [05, 05, 00, 00, 0x34, 00, 00, 00, 00, 00, 00, 00, 00, 0x39, 00]
    elif label=="RATEe":
        c = [05, 05, 00, 00, 0x38, 00, 00, 00, 00, 00, 00, 00, 00, 0x3D, 00]
    elif label=="RATEf":
        c = [05, 05, 00, 00, 0x3C, 00, 00, 00, 00, 00, 00, 00, 00, 0x41, 00]

    elif label=="clearID":
        c = [05, 05, 00, 00, 00, 00, 0x04, 00, 00, 00, 00, 00, 00, 0x09, 00]
    elif label=="setID1":
        c = [05, 05, 00, 00, 0xC0, 00, 0x01, 00, 00, 00, 00, 00, 00, 0xC6, 00]
    elif label=="setID2":
        c = [05, 05, 00, 00, 0xC0, 00, 0x02, 00, 00, 00, 00, 00, 00, 0xC7, 00]
    elif label=="setID3":
        c = [05, 05, 00, 00, 0xC0, 00, 0x03, 00, 00, 00, 00, 00, 00, 0xC8, 00]
    elif label=="setID4":
        c = [05, 05, 00, 00, 0xC0, 00, 0x04, 00, 00, 00, 00, 00, 00, 0xC9, 00]
    elif label=="setID0":
        c = [05, 05, 00, 00, 0xC0, 00, 0x05, 00, 00, 00, 00, 00, 00, 0xCA, 00]

    elif label=="setLTAG":
        c = [05, 05, 00, 0x04, 00, 00, 00, 00, 00, 00, 00, 00, 00, 0x09, 00]
    elif label=="setRTAG":
        c = [05, 05, 00, 0x08, 00, 00, 00, 00, 00, 00, 00, 00, 00, 0x0D, 00]
    elif label=="saveID1":
        c = [05, 05, 00, 00, 0x80, 00, 0x01, 00, 00, 00, 00, 00, 00, 0x86, 00]
    elif label=="saveID2":
        c = [05, 05, 00, 00, 0x80, 00, 0x02, 00, 00, 00, 00, 00, 00, 0x87, 00]
    elif label=="saveID3":
        c = [05, 05, 00, 00, 0x80, 00, 0x03, 00, 00, 00, 00, 00, 00, 0x88, 00]
    elif label=="saveID4":
        c = [05, 05, 00, 00, 0x80, 00, 0x04, 00, 00, 00, 00, 00, 00, 0x89, 00]
    elif label=="saveID0":
        c = [05, 05, 00, 00, 0x80, 00, 0x05, 00, 00, 00, 00, 00, 00, 0x8A, 00]

    else:
        c = []
        
    if len(c)!=0:
        cmd_write(c)

def status_parse(input):
    sync = ord(input[0])
    if sync==0x0c:
        data=[ord(x) for x in input][2:]
        s = {}
        
        s['volt'] = '%0.1f' % (float(data[14] + ((data[15]*256)&256))/10)
        if data[4] == 0x08:
            s['stop_mode'] = 'urgent'
        elif data[4] == 0x0A:
            s['stop_mode'] = 'buffer'
        elif data[4] == 0x4C:
            s['stop_mode'] = 'obstacle'
        else:
            s['stop_mode'] = ''

        s['run'] = (data[1] & 0b0100!=0)
        s['wait'] = (data[1] & 0b10000000!=0)
        s['ready'] = (data[2] & 0b0001!=0)
        s['arrival'] = (data[2] & 0b0010!=0)
        
        s['heading'] = 'backward' if (data[2] & 0b100000!=0) else 'forward' 
        
        s['stamp'] = 'ID%d' % data[3]
        s['target'] = 'ID%d' % data[5]
        s['at'] = 'ID%d' % data[6]
  
        if s['stamp']=='ID5':    s['stamp']='IDA'
        if s['target']=='ID5':    s['target']=='IDA'
        if s['at']=='ID5':    s['at']=='IDA'
        if s['stop_mode']!='':    s['at']=''
        #print(s)
        return s

def base_publish(pub, info):
    msg = Bool()
    msg.data = info['run']
    pub['run'].publish(msg)
    msg.data = info['wait']
    pub['wait'].publish(msg)
    msg.data = info['ready']
    pub['ready'].publish(msg)
    msg.data = info['arrival']
    pub['arrival'].publish(msg)
    
    msg = String()
    msg.data = info['stop_mode']
    pub['stop_mode'].publish(msg)
    msg.data = info['volt']
    pub['volt'].publish(msg)
    msg.data = info['heading']
    pub['heading'].publish(msg)
    msg.data = info['stamp']
    pub['stamp'].publish(msg)
    msg.data = info['target']
    pub['target'].publish(msg)
    msg.data = info['at']
    pub['at'].publish(msg)


def comm_listern():
    pub = {}
    pub['run'] = rospy.Publisher('run', Bool, queue_size=2)
    pub['wait'] = rospy.Publisher('wait', Bool, queue_size=2)
    pub['ready'] = rospy.Publisher('ready', Bool, queue_size=2)
    pub['arrival'] = rospy.Publisher('arrival', Bool, queue_size=2)

    pub['stop_mode'] = rospy.Publisher('stop_mode', String, queue_size=2)
    pub['volt'] = rospy.Publisher('volt', String, queue_size=2)
    pub['heading'] = rospy.Publisher('heading', String, queue_size=2)
    pub['stamp'] = rospy.Publisher('stamp', String, queue_size=2)
    pub['target'] = rospy.Publisher('target', String, queue_size=2)
    pub['at'] = rospy.Publisher('at', String, queue_size=2)

    while not (rospy.is_shutdown() or is_shutdown):
        
        sync= ser.read(1)
        rospy.sleep(0.15)
        sync= "0x%02X" % ord(sync) if len(sync) else "0x00"
        if sync=="0x06":
            sync= ser.read(1)
            sync= "0x%02X" % ord(sync) if len(sync) else "0x00"
            if sync=="0x05":
                count= ser.read(2)    #count=12 words
                data= ser.read(26)    #count*2 + 2(checksum word)
                if len(data)==26:
                    #sync = ["0x%02X"%ord(x) for x in data]
                    #print(sync)
                    # parse data and pub status
                    info = status_parse(count+data)
                    base_publish(pub, info)

        #rospy.sleep(0.1)


if __name__ == '__main__':
    try:
        rospy.init_node('mitgo_base', anonymous=True)
        rospy.Subscriber("base_cmd", String, callback_cmd_label)
        rospy.loginfo("publish teleop_base on mitgo")
        cmd_write(PING_ARRAY)
        comm_listern()

    except rospy.ROSInterruptException:
        rospy.loginfo("teleop_base is close")
    
    except serial.serialutil.SerialException:
        rospy.loginfo("teleop_base conn is loss")
    
    finally:
        is_shutdown = True
        rospy.loginfo("mitgo-base is shutdown")
        ser.close()
