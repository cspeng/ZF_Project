#!/usr/bin/python
# *AD7606 schematic*
# RANGE L/H selection: +/-5V or +/-10V
# BUSY      pin-11  WDOG    pin-16
# CVA       pin-12  CVB     pin-12
# RST       pin-13  RANGE   pin-15
# DB15      Ground  OS2     3.3V
# DB7(MISO) pin-21
# RD(SCLK)  pin-23
# CS(CE0)   pin-24
# *equation*
# (Vin/10V) = (hex/32768) * (2.5V/Vref)
#
import RPi.GPIO as GPIO
import wiringpi2 as wiringpi
import spidev, atexit, redis, time
_CTN= 0; AUTOZ_CTN= 0; AUTOZ_SEC=1
_OFF= -0.119; _PEAK= -_OFF; _SIGN= 1
_VAL= 0; _RANGE= (0.0, 0.0); _SUM= 0; _ADN= 0
AD= None; E2BUSY=1; E2PEAK= -_OFF
END_OFF= None; AVG_OFF= _OFF; conv_CTN= 1
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

_signint = lambda x: (int("0x%02X%02X" % tuple(x), 16) - (x[0] >> 7) * 65536)
_convert= (1.0, 1.0, 8.850746, 10.197162)


def AD_emit(val,r):
    #store data to database
    global _SIGN, _convert, E2BUSY, E2PEAK
    unit_ix= {'None':0,'N.m':1,'lbf.in':2,'Kgf.cm':3}[r.get('unitAD')]
    unit_fig= (2,2,1,1)[unit_ix]
    val= abs(val * _convert[unit_ix])
    val= round(val*5, unit_fig)/5
    unit_format= ["%1.3f", "%1.3f","%2.2f","%2.2f"][unit_ix]
    E2peak= round(E2PEAK *  _convert[unit_ix] * 5, unit_fig) / 5
    if r.get('setzAD')=='False':
        r.set('dataAD', unit_format % val)
    else:
         r.set('dataAD','    ')
    r.set('signAD', _SIGN)
    r.set('E2BUSY', E2BUSY)
    r.set('E2PEAK', unit_format % E2peak)

def convloop(ADM,ADP,ctn,r):
    #moving average into _VAL
    global AD, _VAL, _SIGN, _PEAK, AUTOZ_CTN, AUTOZ_SEC
    global AVG_OFF, _OFF, E2BUSY, E2PEAK
    AD= round(0.236*AD + 0.764*ADM, 4)
    _VAL= round(0.764*_VAL + 0.236*AD, 4)
    _SIGN= 2 if _VAL+_OFF > 0 else 0
    _SIGN= 1 if round((_VAL+_OFF)*0.5,2) == 0 else _SIGN            
    #set zero, auto, and peak
    mode= r.get('modeAD')
    xabs= abs(ADP+_OFF)
    if  xabs > abs(_PEAK+_OFF) and xabs > 0.1:
        AUTOZ_CTN= 0
        if _SIGN==2:
            E2BUSY= 1
            E2PEAK= round(xabs,3)
        _PEAK= ADP
    else:
        if abs(_VAL+_OFF) <= 0.025:
            AUTOZ_CTN += 1
            AUTOZ_CTN= min(AUTOZ_CTN,500)
        elif ctn % 5 == 4:
            r.set('setzAD', False)      #cancel set zero
    if ctn % 5 == 4:                    #zero counting for auto peak
        AUTOZ_SEC= r.get('holdAD')
    if AUTOZ_CTN > 38 and mode=='AUTO PEAK':
            E2BUSY= 0
    if AUTOZ_CTN > int(AUTOZ_SEC) * 60:
        if r.get('setzAD')=='True':
            if mode=='MEASURE':
                timeout= convoffset(-_VAL,ctn)
                if timeout:
                    r.set('setzAD', False)                     
                    _OFF= round(AVG_OFF, 4)
                    print "offset value at %.4f" % AVG_OFF
            else:
                r.set('setzAD', False)
                E2BUSY= 0
                _PEAK= -_OFF
        if mode=='AUTO PEAK':
            AUTOZ_CTN= 0
            _PEAK= -_OFF
    return (mode=='MEASURE') # check measure mode

def convoffset(AD, ctn):
    global AVG_OFF, END_OFF
    if END_OFF is None:
        END_OFF= (ctn + 80) % 500
    if END_OFF < 80:
        if ctn < 420 and ctn > END_OFF:
            END_OFF= None
    elif ctn > END_OFF:
        END_OFF= None
    #moving average of offset on zero position
    AVG_OFF= round(0.764*AVG_OFF + 0.236*AD, 4)
    return (END_OFF is None)    #check timeout

def convwdog():
    global conv_CTN
    conv_CTN= min(conv_CTN + 1,50001)    #max 40sec
    if conv_CTN % 1680 == 0:
        SPI_reset()
    if conv_CTN % 5000 == 0:
        print "timeup on AD %3d" % (conv_CTN / 1250)

def SPI_reset():
        # set 5V range and reset fn
        GPIO.setmode(GPIO.BOARD)
        GPIO.output(15, False)
        GPIO.output(13, False)
        GPIO.output(13, True)
        GPIO.output(13, True)
        GPIO.output(13, False)
        GPIO.output(13, False)

def init_Emit():
    #conn to torque NOOBS
    r= redis.Redis('localhost')
    r.set('spiOpen',True)     
    r.set('setzAD', False) 
    r.set('signAD', '1')
    r.set('holdAD', '1')
    r.set('E2BUSY', '1')
    r.set('E2PEAK', None)
    r.set('E2e2', 0)
    return r


class dummyCH():
    def __init__(self):
        self.r= init_Emit()
        #setup int callback direct connect on PWM and BUSY tiggering
        GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(11, GPIO.FALLING, callback=self.AD_callback)
        #setup watchDog to monitor AD
        GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
        GPIO.add_event_detect(16, GPIO.FALLING, callback= convwdog)
        #AD7606 setos 800, range, reset pulse
        GPIO.setup(15, GPIO.OUT)    # range fn
        GPIO.setup(13, GPIO.OUT)    # reset fn
        SPI_reset()        

        #setup convst_bar pin_12 PWM
        #theretically 50 (1ms) to 100 (2ms) on some servo and 30-130 may work ok
        wiringpi.wiringPiSetup()
        wiringpi.pinMode(1,2)       # pwm only works on wiringpi port 1
        #wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
        wiringpi.pwmSetClock(192)   # 1536 clock at 50kHz (20us tick)
        wiringpi.pwmSetRange(450)   # range at 450 ticks (1125us)
        wiringpi.pwmWrite(1,447)
        atexit.register(self.AD_close)
        self._CTX= 0

    def AD_close(self):
        self.r.set('spiOpen',False)
        wiringpi.pwmWrite(1, 0)
        GPIO.remove_event_detect(11)
        GPIO.remove_event_detect(16)
        time.sleep(0.4)
        GPIO.cleanup()
        print "clean up GPIO!"

    def AD_get(self):
        global AD
        return AD

    def AD_callback(self,channel):
        #falling edge on pin 12 trigger this callback
        #sys.stdout.write('.')
        global _OFF, _CTN
        try:
            _OFF= 0
            _CTN= (_CTN + 1) % 6000
            convwdog()
            self._CTX= (self._CTX + 1) % 13200
    
            if _CTN % 12 == 0:
                ADx= int(self._CTX / 1200)       #pulse of 0.5Hz
                ADx= ADx/2 if ADx % 2 == 0 else 0 
                self.TransferAD(ADx, _CTN/12)
        except:
            print "cancel callback instance"

    def TransferAD(self,ADx,ctn):
        global AD, _VAL, _OFF, _PEAK
        if AD is None:
            AD= ADx
            _VAL= AD
        ismeasure= convloop(ADx,ADx,ctn,self.r)       #moving average into _VAL        
        # AD messaging
        if ctn % 5 == 0:
            if ismeasure:
                val= round(_VAL + _OFF, 4)
            else:
                val= round(_PEAK + _OFF, 4)
            AD_emit(val,self.r)
        return ("%1.4f " % AD)


class spiChannel():
    def __init__(self):
        self.r= init_Emit()
        #setup int callback
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(11, GPIO.FALLING, callback=self.AD_callback)
        #setup watchDog to monitor AD
        GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
        GPIO.add_event_detect(16, GPIO.FALLING, callback= convwdog)
        #AD7606 setos, range, reset pulse
        GPIO.setup(15, GPIO.OUT)    # range fn
        GPIO.setup(13, GPIO.OUT)    # reset fn
        SPI_reset() 
 
        #setup convst_bar pin_12 PWM
        wiringpi.wiringPiSetup()
        wiringpi.pinMode(1,2)       # pwm only works on wiringpi port 1
        #wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
        wiringpi.pwmSetClock(128)   # 1536 clock at 50kHz (20us tick)
        wiringpi.pwmSetRange(600)   # range at 450 ticks (1125us)
        wiringpi.pwmWrite(1,597)
        #SPI device CS0
        self.spi= spidev.SpiDev()        # create spi object 
        self.spi.open(0, 0)              # open spi port 0, device (CS) 0
        self.spi.max_speed_hz= 11500000  # set clock speed
        atexit.register(self.AD_close)

    def AD_close(self):
        self.r.set('spiOpen',False)
        wiringpi.pwmWrite(1, 0)
        GPIO.remove_event_detect(11)
        GPIO.remove_event_detect(16)
        time.sleep(0.4)
        GPIO.cleanup()
        self.spi.close()
        print "clean up GPIO!"

    def AD_get(self):
        global AD
        return AD

    def AD_callback(self,channel):
        #falling edge on pin 11 trigger to read SPI
        #sys.stdout.write('.')
        global _OFF, _CTN, _RANGE, _SUM, _ADN
        global _signint, conv_CTN
        vals= []
        try:
            _CTN= (_CTN + 1) % 6000     #cycle every 10 sec
            if not GPIO.input(11):         # not busy
                ADx= self.spi.readbytes(8)
                conv_CTN= 1
    
                for k in range(len(ADx)/2):
                    val= 6.0* _signint((ADx[k*2],ADx[k*2+1]))/32768
                    vals.append(round(val,4))
                if len(vals)>1:
                    valzguide= abs(vals[1])<0.01 and abs(vals[2])<0.01
                else:
                    valzguide= False
                if vals[0] > 3.9 and valzguide: 
                    val= vals[3]
                    if val==0.0:
                        print _CTN % 12,vals
                else:
                    SPI_reset()
                    val= None
            else:
                val= None
            if val is not None:
                _RANGE= (max(val, _RANGE[0]),min(val, _RANGE[1]))            
                _SUM += val; _ADN += 1
            if _CTN % 12 == 0:
                #debugging...
                if _ADN<8:
                    print _ADN, vals
                if _ADN != 0:
                    ADM= round(_SUM/_ADN, 4)
                    if _RANGE[0]==0.0:
                        ADP= _RANGE[1]
                    elif _RANGE[1]==0.0:
                        ADP= _RANGE[0]
                    else:
                        ADP= -_OFF
                    self.TransferAD(ADM, ADP, _CTN/12)
                _RANGE= (0,0); _SUM= 0; _ADN= 0
        except:
            print "cancel callback instance"

    def TransferAD(self,ADM, ADP, ctn):
        global AD, _VAL, _OFF, _PEAK

        if AD is None:
            AD= ADM; _VAL= ADM
        ismeasure= convloop(ADM, ADP, ctn, self.r) #moving average into _VAL
        # AD messaging
        if AD is not None and ctn % 5 == 0:
            if ismeasure:
                val= round( _VAL + _OFF, 4)
            else:
                val= round(_PEAK + _OFF, 4)
            AD_emit(val,self.r)
