#!/usr/bin/env python
# -*- coding: utf-8 -*-
''' 
@status: monitor pot temperature and control heater power
@date: 2019-06
@by: trio/clarence
'''
from __future__ import print_function
PKG= 'pot3b'
import roslib; roslib.load_manifest(PKG)
import rospy
from std_msgs.msg import String, Header

import cv2
import numpy as np
import serial
import modbus_tk.defines as cst
import modbus_tk.modbus_rtu as modbus_rtu
 
import atexit, redis, time, sys
import pypyodbc

thread= None

def init_cache():
    ''' conn to POT III '''
    r= redis.Redis('localhost')
    r.set('potSET', 400)   
    r.set('_check_msg_', 0)
    r.set('write_param',' ')
    return r

def isRaspi():
    return (sys.platform=='linux2')     #'win32' or 'linux2'

def datetime():
    return time.strftime('%y-%m-%d %H:%M:%S', time.localtime(time.time())) 


class modbusPOT():
    def __init__(self):
        global thread
        self.r= init_cache()
        self.addr = 0x01
        atexit.register(self.close)  

        rospy.init_node('pot_mb', anonymous=True)            
        interface = rospy.Publisher('/interface', String, queue_size=2)
        rospy.loginfo("publish /interface of POT on Trio socket")
          
        self._signint = lambda x: (int("0x%04X" % x,16) - (x>>15) * 0x10000)
        self._twochr = lambda x: (chr(x/256) + chr(x % 256))
        
        #self.link = serial.Serial('COM9', baudrate=9600, bytesize=8, parity='N', stopbits=1)
        self.link = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.master = modbus_rtu.RtuMaster(self.link)
        self.master.set_timeout(0.04)  #0.025 
        time.sleep(0.2) 
        print("connected service on POT\n")   

        cv2.namedWindow('POT_Control')    
        cv2.moveWindow('POT_Control',800,50)
        barpoint = np.array([90,60,30], np.uint8)
        self.dispbar = np.hstack([np.tile(barpoint, (72,72,1))] * (9))
        self.PID_action(400)
        #self.ID_clone("A7")

        while True:
            try:
                #loop time around 0.1 sec
                key = 0xFF & cv2.waitKey(100)
                #im_visible = cv2.WND_PROP_VISIBLE    ###
                #im_close = cv2.getWindowProperty('POT_Control', im_visible) < 1
                im_close = False
                
                TMPx = self.master.execute(self.addr, cst.READ_HOLDING_REGISTERS, 0, 16)
                self.pot_tmp = self._signint(TMPx[0]) 
                self.pot_set = self._signint(TMPx[2]) 
                self.pot_heatduty = self._signint(TMPx[3]) 
                self.pot_heatrate = self._signint(TMPx[4]) 
                self.pot_heaton = self._signint(TMPx[5]) 
                self.pot_timeset1 = self._signint(TMPx[6]) 
                self.pot_timeset2 = self._signint(TMPx[7]) 
                self.pot_countset1 = self._signint(TMPx[8]) 
                self.pot_countset2 = self._signint(TMPx[9]) 

                pot_basemac = []
                pot_basemac.append(self._twochr(TMPx[10]))
                pot_basemac.append(self._twochr(TMPx[11]))
                pot_basemac.append(self._twochr(TMPx[12]))          
                pot_basemac.append(self._twochr(TMPx[13]))
                pot_basemac.append(self._twochr(TMPx[14]))
                pot_basemac.append(self._twochr(TMPx[15]))                 

                pot_msg = ":".join(pot_basemac)
                pot_msg += ",%03d" % (self.pot_tmp/10)
                pot_msg += ",@%03d" % (self.pot_set/10)
                pot_msg += ",@%d&%d" %  (self.pot_timeset1, self.pot_timeset2)
                pot_count = "$POT,count,%d,%d" % (self.pot_countset1, self.pot_countset2)
                interface.publish(pot_msg + "\r\n")
                self.r.set('interface_msg', pot_msg)
                self.r.set('interface_count', pot_count)
                
                if self.r.get("clear_count") == "True":
                    #need to update counter of addr P,Q
                    self.master.execute(self.addr, cst.WRITE_SINGLE_REGISTER, 8, output_value = 0)
                    time.sleep(0.1)
                    self.master.execute(self.addr, cst.WRITE_SINGLE_REGISTER, 9, output_value = 0)
                    time.sleep(0.1)
                    self.r.set("clear_count",False)
			 
                if "_set_param" in self.r.get("write_param"):
					params = self.r.get("write_param")
					self.pot_set = int(params[12:15])
					self.PID_action(self.pot_set + 1)
					
					time_set1,time_set2 = (30, 30)
					try:
						self.master.execute(03, cst.WRITE_SINGLE_REGISTER, 0X3, output_value = time_set1)
						time.sleep(0.1)
						self.master.execute(04, cst.WRITE_SINGLE_REGISTER, 0X4, output_value = time_set2)
						time.sleep(0.1)
					except:
						pass
					self.master.execute(self.addr, cst.WRITE_SINGLE_REGISTER, 6, output_value = time_set1)
					time.sleep(0.1)
					self.master.execute(self.addr, cst.WRITE_SINGLE_REGISTER, 7, output_value = time_set2)
					time.sleep(0.1)
					self.r.set("write_param"," ")
                
                potset = self.pot_set
                self.PID_show()

                if key == 27 or im_close:
                    break 
                if key == ord('1'):
                    potset = 1000
                if key == ord('2'):
                    potset = 2000               
                if key == ord('3'):
                    potset = 3000                 
                if key == ord('4'):
                    potset = 4000     
                if key == ord('5'):
                    potset = 4888    
                if key == ord('r'):
                    potset = 400
                if self.pot_set!=potset:
                    self.PID_action(potset)
                if key == ord('9'):
                    self.IDLED_action(1)
                if key == ord('0'):
                    self.IDLED_action(0)
                    self.ID_verify()
                if key == ord('t'):
                    self.Timer_action(30)
                if key == ord('f'):
                    self.Timer_action(40)
                if key == ord('s'):
                    self.Timer_action(60)

            except:
                self.PID_action(399)
                print("error in transfer of POT data!")   

        cv2.destroyWindow('POT_Control')
        self.PID_action(399)

    def IDLED_action(self, to_on):
        self.master.execute(0x03,cst.WRITE_SINGLE_COIL,0,output_value=to_on)
        time.sleep(0.01)

    def ID_verify(self):
        self.master.execute(0x03,cst.WRITE_SINGLE_COIL,1,output_value=1)
        time.sleep(0.01)
        
    def ID_clone(self, id_Name='00'):
        self.ID_verify()
        #print("0x%04X" % id_Name16)
        id_Name16 = ord(id_Name[0])*0x100 + ord(id_Name[1])
        self.master.execute(0x03,cst.WRITE_SINGLE_REGISTER,0,output_value=id_Name16)
        time.sleep(0.01)        
        
    def Timer_action(self, timeset):
        self.master.execute(0x03,cst.WRITE_SINGLE_REGISTER,0x3,output_value=timeset)
        time.sleep(0.01)

    def PID_action(self, potset):
        try:
            self.master.execute(self.addr,cst.WRITE_SINGLE_REGISTER,16,output_value=potset)
            time.sleep(0.1)
        except:
            pass

    def PID_show(self):
        img = self.dispbar.copy()
        img = np.vstack([img,img])
        font, color = (cv2.FONT_HERSHEY_SIMPLEX, [196,196,196])
        bar_msg = 'POT PV %.1f        SET SV %.1f' % (float(self.pot_tmp)/10, float(self.pot_set)/10)
        cv2.putText(img, bar_msg, (40,60), font, 1, color, 2) 
        bar_msg = 'Heat Duty %d%%  Heat Rate %d' % (self.pot_heatduty, self.pot_heatrate)
        cv2.putText(img, bar_msg, (40,105), font, 0.62, color, 1) 
        cv2.imshow('POT_Control', img)

    def close(self):
        time.sleep(0.4)
        self.PID_action(399)
        self.master.close()
        self.link.close()
        print("--- clean up service on POT! ---")


class sqlConn():
    """MS SQL interface in cloud V2.0 """  ### draft
    tmc_Dsn= "DSN=sqlDS; UID=Engg; PWD=engg0222; DATABASE=Engg"
    tmc_Str= "DRIVER={SQL Server};SERVER=192.168.33.93;UID=Engg;PWD=engg0222;DATABASE=Engg"
    tmc2_Str= "DRIVER={SQL Server};SERVER=192.168.100.23;UID=engg;PWD=engg;DATABASE=engg"

    def __init__(self):
        self.isRaspi= isRaspi()
        self.connStr= self.tmc_Dsn if self.isRaspi else self.tmc2_Str
        print('\nNC POT odbc data source...')

        try:
            pypyodbc.connection_timeout= 4
            self.connDB= pypyodbc.connect(self.connStr,timeout=4)
            #self.connDB.timeout= 4
            self.cur= self.connDB.cursor()
            atexit.register(self.close)
            print('--- connected on %s ---\n' % datetime())
        except:
            self.cur= None
            self.connDB= None 
            print('not connect\n')

    def isConn(self):
        return  self.connDB is not None and self.cur is not None

    def close(self):
        try:
            if self.connDB is not None:
                self.connDB.close()
            print('--- disconnect Data Source at %s ---' % datetime())
            
        except:
            print('cannot close DS server')

    def sqlDBheading(self):
        fetch= None
        DSindex= (1,2,4,8,9,6,7,16,3,14)
        
        if self.isConn():
            try:            
                SQL= "exec Engg.Proc_Electric_QueryTableHead "
                self.cur.execute(SQL)
                rows= self.cur.fetchall()
                fetch= list( (x[0] for x in rows) )
                fetch= [fetch[i] for i in DSindex]

            except:
                print('cursor fetching error on heading')
        
        return fetch   

    def sqlPOTinfo(self):
        global rows
        fetch= None
        DSindex= range(9)
        
        if self.isConn():
            try:            
                SQL= "exec Engg.Proc_QueryToolTemperatureInfo "
                self.cur.execute(SQL)
                rows= self.cur.fetchall()
                fetch= list((x for x in rows[0]))
                fetch= [fetch[i] for i in DSindex]

            except:
                print('cursor fetching error on heading')
        
        return fetch  


if __name__ == "__main__":
    DS= sqlConn()
    POT = modbusPOT()
    POT.close()
