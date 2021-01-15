'''
III-inone rewrote on Dec 2019
    last history
    # 33 add _tool_lowcut
    # 92 add lowcut_value copy from _tool_lowcut
    # 95/102 fixed limit to lowcut_value 
    #121 save _OFF to ctx object
         scale= _tool_scale[int(r.get('mbBX'))]
    #313 add time.sleep to transfer RQOFF

    last updating
    #  28 change BOX[2] to 2Nm
    # 978 change aftertimeout to neglect first fail
    #1338 change verifybox to have vkey=2

'''    
import serial
import atexit, redis, time, sys
from threading import Thread


class DS_ctype():
    # ds constant
    _signint = lambda x: (int("0x%04X" % x,16) - (x>>15) * 0x10000)  
    _convert = (1.0, 1.0, 8.850746, 10.197162)
    _lowcut  = 0.1
    _round   = 2.0
    _offset  = 0.0
    # ds raw data
    _CTN  = 0
    _VAL  = 0
    _PEAK = 0
    # ds var and sign with {-ve:0, 0:1 +ve:2}
    AVGZ1K = 0
    SIGN = 1
    OFF  = 0.0

class CTX_ctype():
    thread    = None
    AUTOZ_CTN = 0    
    AUTOZ_SEC = 1
    E2_notOK  = 1


ds =  DS_ctype()
ctx = CTX_ctype()

def init_Emit():  

    ### conn to torque II ###
    r= redis.Redis('localhost')
    #r.set('mbBX', 4)
    r.set('mbBXRQ', 4)   
    r.set('mbNew', True)
    r.set('mbOpen', True) 
    r.set('mbQuit', False)
    r.set('setzAD', False) 
    r.set('mbShape_Req', False) 
    r.set('signAD', '1')
    r.set('holdAD', '1')
    r.set('E2BUSY', '1')
    r.set('E2PEAK', None)
    r.set('mbBX', 1) 
    r.set('mbCtn', 0)

    #only used for debug offset transfer
    #r.set('mbRQOFF', 0)  
    r.set('unitAD','N.m')
    
    return r

def loop_Emit(r, mode):
    ### interacting data exchanges on redis DB ###
    ismeasure = (mode=='MEASURE')
    unit_ix= {'None':0,'N.m':1,'lbf.in':2,'Kgf.cm':3}[r.get('unitAD')]
    unit_fig= (2,2,1,1)[unit_ix]
    unit_convert= ds._convert[unit_ix]
    unit_format= ["%1.3f", "%1.3f","%2.2f","%2.2f"][unit_ix]

    #redefine scale from upper limit to constant provided that
    #previous# _val, _peak = 12.0 * ds._VAL / 30000, 12.0 * ds._PEAK / 30000
    #constant# val of 1000 to be 1N.m

    os_val, os_peak = ((ds._VAL+ds.OFF)/1000.0, (ds._PEAK+ds.OFF)/1000.0)
    if os_peak < ds._lowcut:    os_peak = 0
    if os_peak == 0:    ds._PEAK = 0    ##revise
    os_val, sign = (abs(os_val) * unit_convert, ds.SIGN)
    os_val= round(os_val * ds._round, unit_fig)/ds._round
    os_peak = round(os_peak *  unit_convert * 10, unit_fig) / 10

    data_val = os_val if ismeasure else os_peak

    r.set('E2BUSY', ctx.E2_notOK)
    r.set('E2OFF', ds.OFF)
    r.set('AVGZ1K', ds.AVGZ1K/1000.0)
    r.set('mbCtn', int(ds._CTN/100)*100)
    r.set('signAD', sign)
    r.set('E2PEAK', unit_format % os_peak)
    if r.get('setzAD')=='False' and ds._CTN!=0:
        r.set('dataAD', unit_format % data_val)
    else:
        r.set('dataAD','    ')


class Channel():
    def __init__(self):
        self.redis= init_Emit()
        atexit.register(self.close) 
            
        #Connect to the SafePort USB Serial
        # init connection to ESP32 @38400
        self.serial = serial.Serial('/dev/ttyCOM3', 38400, timeout=0.3)
        time.sleep(1.2) 
        
        if ctx.thread is None:
            ctx.thread = Thread(target= self.mainloop_thread)
            ctx.thread.start() 
        print("ESP32 is ready.")
        
    def mainloop_thread(self):
        """ solving loop to have average and peak of measurement value """
        r, last_ctn = (self.redis, 0)

        while not self.is_shutdown():
            ctx.AUTOZ_SEC= int(r.get('holdAD'))
            mode= r.get('modeAD')
            data = self.ds_listern()
            #print(data)
            #if True:
            try:
                if data is not None:
                    last_peak = ds._PEAK
                    ds._PEAK = int(data[16:23])
                    ds._VAL = [int(s) for s in data[23:].split(",")][1]
                    data_ctn = int(data[5:10])

                    #weighted ratio 84 to 16 and scaling factor of 100/16
                    if (data_ctn-last_ctn)>0:
                        ds._CTN = ds._CTN*84 + (data_ctn-last_ctn)*100
                        ds._CTN = int(ds._CTN/100)
                    last_ctn = data_ctn

                    if abs(ds._VAL) < 100:
                        ds.AVGZ1K = 764*ds.AVGZ1K/1000 + 236*ds._VAL

                    val_os = ds._VAL+ds.OFF
                    if abs(val_os) < ds._lowcut*1000:
                        ctx.AUTOZ_CTN += 1
                        ctx.AUTOZ_CTN= min(ctx.AUTOZ_CTN,39)
                    else:
                        #zero counter in all mode during measurment
                        ctx.AUTOZ_CTN = 0
                        #clear zero state when in some mode
                        if mode=='MEASURE':    r.set('setzAD', False) 

                    # signal to clear E2BUSY when E2PEAK value is ready
                    # transferE2 sequence: E2BUSY, !E2e2, E2ACK, !E2BUSY, E2e2, !E2ACK
                    if ds._PEAK > last_peak and last_peak+ds.OFF>ds._lowcut*1000:
                        ctx.E2_notOK= 1
                    if ctx.AUTOZ_CTN > 6 and last_peak!= 0 and mode=='AUTO PEAK':
                        ctx.E2_notOK= 0    #revise

                    #keep stable on auto mode
                    if val_os > 100 or (ds._PEAK > last_peak and mode == 'AUTO PEAK'):
                        ctx.AUTOZ_CTN = 0

                    self.auto_event(mode)

                    #update sign and peak value
                    ds.SIGN= 2 if val_os > 0 else 0
                    ds.SIGN= 1 if round(val_os/2000, 2)==0 else ds.SIGN
 
                    #E2peak = float(r.get("E2PEAK"))
                    #print("peak %.3f  offset %d  BUSY %d" % (E2peak, ds.OFF, ctx.E2_notOK))
                    loop_Emit(self.redis, mode)
                   
                else:
                    mbCtn = int(self.redis.get('mbCtn'))                    
                    if (mbCtn!=0):
                        print("miss data at speed:%d!" % mbCtn)

                    ds._CTN = 0
                    loop_Emit(self.redis, mode)

            except:
                print("ds-stream error at loop:%s!" % self.redis.get('mbCtn'))

    def ds_listern(self):
            data = self.serial.readline()
            if len(data):
                sync, end = (data[0:4], ord(data[-1]))
                if sync == 'VSPI' and end==0x0a:
                    return data[5:-1]
                #else:    print("error with char-code %d!" % end)

    def auto_event(self, mode):
        r = self.redis
        if ctx.AUTOZ_CTN > ctx.AUTOZ_SEC * 5 + 4:    #esp32 6 ticks/sec
            if r.get('setzAD')=='True':
                if mode=='MEASURE':
                    #update new zero offset
                    ds.OFF= round(-ds.AVGZ1K/1000.0, 1) 
                    print "--offset value at %.1f--" % ds.OFF
  
                #send 'z'(zero command) to ESP32 
                ctx.E2_notOK= 0 
                self.ds_zero()
                r.set('setzAD', False)

            #send if in auto mode
            if mode=='AUTO PEAK' and (ds._PEAK+ds.OFF)>50:
                self.ds_zero()   

    def ds_zero(self):
        self.serial.write(b'z')
        time.sleep(0.1)

    def is_shutdown(self):
        return (self.redis.get('mbOpen')=='False')

    def set_shutdown(self):
        self.redis.set('mbOpen',False)
        time.sleep(0.4)

    def close(self):
        self.set_shutdown()
        self.serial.close()
        print "close serial SafePort."


if __name__ == '__main__':
    inone = Channel()
    try:
        r = inone.redis
        while not inone.is_shutdown():
            if r.get('mbShape_Req')=='True':
                #r.set('dataShape', inone.AD_shape())
                r.set('mbShape_Req', False)

    except KeyboardInterrupt:
        print(" user stop the conn...")

    except serial.serialutil.SerialException:
        print("series conn is loss...")

    finally:
        inone.close()
