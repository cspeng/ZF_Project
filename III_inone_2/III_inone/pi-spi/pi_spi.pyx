# distutils: extra_compile_args = -fopenmp
# distutils: extra_link_args = -fopenmp
import RPi.GPIO as GPIO, spidev
import wiringpi2 as wiringpi
from cython cimport boundscheck, wraparound 
from cython.parallel cimport prange
import atexit, redis, time, sys
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
_signint = lambda x: (int("0x%02X%02X" % tuple(x), 16) - (x[0] >> 7) * 65536)
_convert= (1.0, 1.0, 8.850746, 10.197162)
#AD variables
cdef int _CTN= 0
cdef float _OFF= -0.119
cdef float _VAL= 0
cdef float _PEAK= -_OFF
cdef int _SIGN= 1
#result
AD= None
E2BUSY=1; E2PEAK= -_OFF
END_OFF= None; AVG_OFF= _OFF

cdef:
    struct ctx_CS:
        int CTX
        int AUTOZ_CTN        
        int AUTOZ_SEC
        int conv_CTN
        int _ADN
        float _SUM
        float _RANGE0 
        float _RANGE1
    ctx_CS ctx= ctx_CS(0, 0, 1, 1, 0, 0.0, 0.0, 0.0)

cdef void AD_emit(float val,r):
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

cdef int convloop(float ADM, float ADP, int ctn, r):
    #moving average into _VAL
    global AD, _VAL, _SIGN, _PEAK, ctx
    global AVG_OFF, _OFF, E2BUSY, E2PEAK
    AD= round(0.236*AD + 0.764*ADM, 4)
    _VAL= round(0.764*_VAL + 0.236*AD, 4)
    _SIGN= 2 if _VAL+_OFF > 0 else 0
    _SIGN= 1 if round((_VAL+_OFF)*0.5,2) == 0 else _SIGN            
    #set zero, auto, and peak
    mode= r.get('modeAD')
    xabs= abs(ADP+_OFF)
    if  xabs > abs(_PEAK+_OFF) and xabs > 0.1:
        ctx.AUTOZ_CTN= 0
        if _SIGN==2:
            E2BUSY= 1
            E2PEAK= round(xabs,3)
        _PEAK= ADP
    else:
        if abs(_VAL+_OFF) <= 0.025:
            ctx.AUTOZ_CTN += 1
            ctx.AUTOZ_CTN= min(ctx.AUTOZ_CTN,500)
        elif ctn % 5 == 4:
            r.set('setzAD', False)      #cancel set zero
    if ctn % 5 == 4:                    #zero counting for auto peak
        ctx.AUTOZ_SEC= int(r.get('holdAD'))
    if ctx.AUTOZ_CTN > 38 and mode=='AUTO PEAK':
            E2BUSY= 0
    if ctx.AUTOZ_CTN > ctx.AUTOZ_SEC * 60:
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
            ctx.AUTOZ_CTN= 0
            _PEAK= -_OFF
    return (mode=='MEASURE') # check measure mode

cdef int convoffset(float AD, int ctn):
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


class spiPYX_CH():
    def __init__(self):
        import pigpio        
        self.r= init_Emit()
        pi = pigpio.pi()
        #broad= (11, 12, 13), (15, 16)
        (self.busyP, self.pwmP, self.resetP) = (17, 18, 27)
        (self.rangeP, self.wdogP) = (22, 23)
        #setup int callback direct connect on PWM and BUSY tiggering
        pi.set_pull_up_down(self.busyP, pigpio.PUD_UP)
        self.pi_cb1= pi.callback(self.busyP, pigpio.FALLING_EDGE, self.AD_callback)
        #setup watchDog to monitor AD
        pi.set_pull_up_down(self.wdogP, pigpio.PUD_UP)
        self.pi_cb2= pi.callback(self.wdogP, pigpio.FALLING_EDGE, self.convwdog)
        #AD7606 range, reset level
        pi.set_mode(self.rangeP, pigpio.OUTPUT)
        pi.set_mode(self.resetP, pigpio.OUTPUT)	
        self.SPI_reset(pi)
        #setup convst_bar pin_12 PWM
        pi.hardware_PWM(self.pwmP, 909, 995000) # 909Hz 99.5% dutycycle
        self.GPIO= pi
        atexit.register(self.close)

    def convwdog(self, gpio, level, tick):
        global ctx
        ctx.conv_CTN= min(ctx.conv_CTN + 1,50001)    #max 40sec
        if ctx.conv_CTN % 1680 == 0:
            self.SPI_reset(self.GPIO)
        if ctx.conv_CTN % 5000 == 0:
            print "timeup on AD %3d" % (ctx.conv_CTN / 1250)

    def SPI_reset(self,pi):
            # set 5V range and reset fn
            pi.write(self.rangeP, 0)
            pi.write(self.resetP, 0)
            pi.write(self.resetP, 1)
            pi.write(self.resetP, 1)
            pi.write(self.resetP, 0)
            pi.write(self.resetP, 0)

    def close(self):
        import pigpio
        self.r.set('spiOpen',False)
        if self.pi_cb1 is not None:
            self.pi_cb1.cancel()
            self.pi_cb1= None
        if self.pi_cb2 is not None:
            self.pi_cb2.cancel()
            self.pi_cb2= None
        self.GPIO.set_mode(self.pwmP, pigpio.INPUT)
        self.GPIO.set_mode(self.resetP, pigpio.INPUT)	
        self.GPIO.set_mode(self.rangeP, pigpio.INPUT)
        self.GPIO.stop()
        time.sleep(0.4)
        print "clean up GPIO!"

    def AD_get(self):
        global AD
        return AD

    def AD_callback(self, gpio, level, tick):
        #falling edge on pin 12 trigger this callback
        #sys.stdout.write('.')
        global _OFF, _CTN, ctx
        try:
            _OFF= 0
            _CTN= (_CTN + 1) % 6000
            self.convwdog('','','')
            ctx.CTX= (ctx.CTX + 1) % 13200
    
            if _CTN % 12 == 0:
                ADx= int(ctx.CTX / 1200)          #pulse of 0.5Hz
                ADx= ADx/2 if ADx % 2 == 0 else 0 
                self.TransferAD(ADx, _CTN/12)
        except:
            print "cancel callback instance"

    def TransferAD(self, float ADx, int ctn):
        global AD, _VAL, _OFF, _PEAK
        if AD is None:
            AD= ADx
            _VAL= AD
        ismeasure= convloop(ADx,ADx,ctn,self.r)     #moving average into _VAL        
        # AD messaging
        if ctn % 5 == 0:
            if ismeasure:
                val= round(_VAL + _OFF, 4)
            else:
                val= round(_PEAK + _OFF, 4)
            AD_emit(val, self.r)
        return "%1.4f " % AD


class spiXChannel():
    def __init__(self):
        self.r= init_Emit()
        #setup int callback
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(11, GPIO.FALLING, callback=self.AD_callback)
        #setup watchDog to monitor AD
        GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
        GPIO.add_event_detect(16, GPIO.FALLING, callback= self.convwdog)
        #AD7606 setos, range, reset pulse
        GPIO.setup(15, GPIO.OUT)    # range fn
        GPIO.setup(13, GPIO.OUT)    # reset fn
        SPI_reset() 
 
        #setup convst_bar pin_12 PWM
        wiringpi.wiringPiSetup()
        wiringpi.pinMode(1,2)       # pwm only works on wiringpi port 1
        #wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
        wiringpi.pwmSetClock(128)   # 1128 clock at 50kHz (20us tick)
        wiringpi.pwmSetRange(600)   # range at 700 ticks (630Hz)
        wiringpi.pwmWrite(1,597)
        #SPI device CS0
        self.spi= spidev.SpiDev()        # create spi object 
        self.spi.open(0, 0)              # open spi port 0, device (CS) 0
        self.spi.max_speed_hz= 11500000  # set clock speed
        atexit.register(self.close)

    def convwdog(self,channel):
        global ctx
        ctx.conv_CTN= min(ctx.conv_CTN + 1,50001)    #max 40sec
        if ctx.conv_CTN % 1680 == 0:
            SPI_reset()
        if ctx.conv_CTN % 5000 == 0:
            print "timeup on AD %3d" % (ctx.conv_CTN / 1250)


    def close(self):
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
        global _OFF, _CTN
        global _signint, ctx
        vals= []
        try:
            _CTN= (_CTN + 1) % 6000     #cycle every 10 sec
            if not GPIO.input(11):         # not busy
                ADx= self.spi.readbytes(8)
                ctx.conv_CTN= 1
    
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
                        val= None
                else:
                    SPI_reset()
                    val= None
            else:
                val= None
            if val is not None:
                ctx._RANGE0= max(val, ctx._RANGE0)
                ctx._RANGE1= min(val, ctx._RANGE1)
                ctx._SUM += val; ctx._ADN += 1
            if _CTN % 12 == 0:
                #debugging...
                if ctx._ADN<8:
                    print ctx._ADN, vals
                if ctx._ADN != 0:
                    ADM= round(ctx._SUM/ctx._ADN, 4)
                    if ctx._RANGE0==0.0:
                        ADP= ctx._RANGE1
                    elif ctx._RANGE1==0.0:
                        ADP= ctx._RANGE0
                    else:
                        ADP= -_OFF
                    self.TransferAD(ADM, ADP, _CTN/12)
                (ctx._RANGE0,ctx._RANGE1)= (0.0, 0.0)
                (ctx._SUM, ctx._ADN)= (0.0, 0)
        except:
            print "cancel callback instance"

    def TransferAD(self, float ADM, float ADP, int ctn):
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

