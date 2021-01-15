#!/usr/bin/env python 
# -*- coding: utf-8 -*- 

import time, atexit
import numpy as np
import cv2
import modbus_tk
import modbus_tk.modbus_tcp as modbus_tcp
import modbus_tk.defines as tcp_cst

logger = modbus_tk.utils.create_logger(name="console", record_format="%(message)s")
pass_time, accum_error = (0, 0)

def readDI():
    #Read 通道DI0 0x00  
    global accum_error
    try:
        DI = master.execute(1, tcp_cst.READ_DISCRETE_INPUTS, 0, 2)
        return DI
    except:
        accum_error += 1
        logger.error("%s- Code=%d" % ('readDI', -1))
        return (1,)*2

def readAin():
    #Read 通道AIN8-11 0x48 4
    global accum_error
    try:
        Ain = master.execute(1, tcp_cst.READ_INPUT_REGISTERS, 0x48, 4)
        return Ain[:2]
    except:
        accum_error += 1
        logger.error("%s- Code=%d" % ('readAin', -1))
        return (0, 0)

def writeDO(val0, val1):
    #Write and read 通道DO0 0x200
    global accum_error
    try:
        value = (not val0, not val1)
        master.execute(1, tcp_cst.WRITE_MULTIPLE_COILS, 0x200, output_value=value)
        D0, D1 = master.execute(1, tcp_cst.READ_COILS, 0x200, 2)
        return (not D0, not D1) 
    except:
        accum_error += 1
        logger.error("%s- Code=%d" % ('writeDO', -2))
        return (0,)*2

def readCounter32():
    #Read 计数器0-1 0x7c 4
    global accum_error
    try:
        print(master.execute(1, tcp_cst.READ_HOLDING_REGISTERS, 0x7c, 4))
    except modbus_tk.modbus.ModbusError, e:
        accum_error += 1
        logger.error("%s- Code=%d" % ('readCounter32', e.get_exception_code()))
        return (0,)*4

def close():
    global master
    time.sleep(0.4)
    master.close()
    print "--- clean up service on CWIRING! ---"

def timer(seq, cnt, ret=0, limit=60):
    if seq:    tim = 1; cnt = 0
    elif cnt < limit: tim = 1; cnt += 1
    else:    tim = 0
    if seq and ret:    tim = 0; cnt=limit
    return (tim, cnt)

def CWIRE_show(dispbar, DO0, DO1, AI, resp):
    global pass_time
    img = dispbar.copy()
    font, color = (cv2.FONT_HERSHEY_SIMPLEX, [196,196,196])
    bar_msg = 'CWIRE DO %d %d  resp %d  AI %-4d' % (DO0, DO1, resp, AI)
    cv2.putText(img, bar_msg, (20,40), font, 1, color, 2) 
    cv2.putText(img, '%d' % (pass_time), (560,40), font, 1, color, 2) 
    cv2.imshow('CWIRE_Control', img)


if __name__ == "__main__":
    """main"""
    #Create the TCP master
    master = modbus_tcp.TcpMaster(host="192.168.1.30")
    master.set_timeout(0.02)
    atexit.register(close)  
    time.sleep(0.4)
    print("CWIRING connected and running...")
    print("press 'Esc' for closing the server\n")

    cv2.namedWindow('CWIRE_Control')    
    cv2.moveWindow('CWIRE_Control',600,20)
    barpoint = np.array([30,60,90], np.uint8)
    dispbar = np.hstack([np.tile(barpoint, (72,72,1))] * (9))
    _signint = lambda x: (int("0x%04X" % x,16) - (x>>15) * 0x10000)
    DI0, DI1, detect, ret = (0, 0, 0, 0)
    cnt0, cnt1, resp, key = (0, 0, 0, 0)
    im_close = False

    while not im_close:
        #loop time around 10 ms
        if  accum_error<20:
            #Read 通道DI0, 通道AIN8-11
            _DI = readDI()
            AI0, AI1 = readAin()
            AI0 = _signint(AI0)
            AI1 = _signint(AI1)
            
            #control logic
            seq0, seq1 = DI0 ^ _DI[0], DI1 ^ _DI[1]
            seq0, seq1 = seq0 * DI0, seq1 * DI1
            tim0, cnt0 = timer(seq0, cnt0, ret)
            tim1, cnt1 = timer(seq1, cnt1)
            DI0, DI1 = _DI
    
            #Write and read 通道DO        
            start, end = tim0, tim1
            scan = start * (not end)
            ret = (not start) * end
            #start disable when return strike
            if (not scan) and seq1:    cnt0 = 100
            if ret and seq0:    cnt1 = 100
            detect = bool(detect*start) or bool(scan) and (AI0<-400)
	    if detect and seq1:    resp+=1
            _DO = writeDO(start, detect)
            DO0, DO1 = _DO
    
            if (pass_time % 5)==0:
                CWIRE_show(dispbar, DO0, DO1, AI0, resp)
                key = 0xFF & cv2.waitKey(1)
            else:
                time.sleep(0.002)
                  
            pass_time = (pass_time+1) % 1000
	    if key == ord('z'):
                resp = 0
            if key == 27 or im_close:
                break
   
        else:
            #reset TCP when errors occur for read or write
            accum_error = 0
            master.close()
            time.sleep(0.4)
            master = modbus_tcp.TcpMaster(host="192.168.1.30")
            master.set_timeout(0.04)
            time.sleep(0.4)            
            print("CWIRING reconnected!")

    cv2.destroyWindow('CWIRE_Control')
