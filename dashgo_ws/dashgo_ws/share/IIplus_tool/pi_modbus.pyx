#run in win7 without extra
'''# distutils: extra_compile_args = -fopenmp
# distutils: extra_link_args = -fopenmp'''
import serial, modbus_tk
import modbus_tk.defines as cst
import modbus_tk.modbus_rtu as modbus_rtu
 
from threading import Thread
import atexit, redis, time, sys

_signint = lambda x: (int("0x%04X" % x,16) - (x>>15) * 0x10000)
_convert= (1.0, 1.0, 8.850746, 10.197162)
_tool_scale= (0.0, 6.0, 10.0, 20.0 , 0.0)
_tool_round= (1.0, 5.0, 2.0, 2.0, 1.0)
_tool_offset= [0.0, -0.07, -0.07, -0.03, 0.0]
#_tool_offset= [0.0, -0.07, -0.07, -0.03, 0.0]
thread= None

#AD variables
cdef int _CTN= 0
cdef float _OFF= 0.000
cdef float _VAL= 0
cdef float _PEAK= 0
cdef int _SIGN= 1
#result
AD= None
E2BUSY=1; E2PEAK= -_OFF
END_OFF= None; AVG_OFF= _OFF

cdef:
    struct CTX_t:
        int CTX
        int AUTOZ_CTN        
        int AUTOZ_SEC
        int conv_CTN
        short int BOX
        short int RQOFF 
        short int RQZPEAK
    CTX_t ctx= CTX_t(0, 0, 1, 1, 0, 0, 0)

cdef void AD_emit(float val,r):
    ''' store data to redis DB '''
    global _SIGN, _convert, E2BUSY, E2PEAK, _tool_round
    unit_ix= {'None':0,'N.m':1,'lbf.in':2,'Kgf.cm':3}[r.get('unitAD')]
    unit_fig= (2,2,1,1)[unit_ix]
    unit_round= _tool_round[int(r.get('mbBX'))] 
    val= abs(val * _convert[unit_ix])

    #val= round(val*unit_round, unit_fig)/unit_round
    val= round(val*unit_round, unit_fig)/unit_round
    unit_format= ["%1.3f", "%1.3f","%2.2f","%2.2f"][unit_ix]
    #E2peak= round(E2PEAK *  _convert[unit_ix] * 5, unit_fig) / 5
    E2peak= round(E2PEAK *  _convert[unit_ix] * 10, unit_fig) / 10
    if r.get('setzAD')=='False':
        r.set('dataAD', unit_format % val)
    else:
         r.set('dataAD','    ')
    r.set('signAD', _SIGN)
    r.set('E2BUSY', E2BUSY)
    r.set('E2PEAK', unit_format % E2peak)

cdef int convloop(float ADM, float ADP, int ctn, r):
    ''' moving average value and auto-peak '''
    global _VAL, _SIGN, _PEAK, ctx, _tool_offset
    global AVG_OFF, _OFF, E2BUSY, E2PEAK
    
    #Mavg sign -ve 0, zero 1, +ve 2  
    _VAL= round(0.236 * _VAL + 0.764 * ADM, 4)
    _SIGN= 2 if _VAL+_OFF > 0 else 0
    _SIGN= 1 if round((_VAL+_OFF)*0.5,2) == 0 else _SIGN            

    #update peak value and reset auto counter
    mode= r.get('modeAD')
    xabs= abs(ADP)                     #offseted abs peak value

    if  xabs > abs(_PEAK) and xabs > 0.1:
        ctx.AUTOZ_CTN= 0
        if _SIGN==2:
            E2BUSY= 1
            E2PEAK= round(xabs,3)
        _PEAK= ADP
    else:
        if abs(_VAL+_OFF) <= 0.1:           #historical 0.025, 0.05
            ctx.AUTOZ_CTN += 1
            ctx.AUTOZ_CTN= min(ctx.AUTOZ_CTN,100)
        elif ctn % 5 == 4:
            r.set('setzAD', False)          #cancel zero offset if far away

    if ctn % 5 == 4:                        #update auto-zero hold sec
        ctx.AUTOZ_SEC= int(r.get('holdAD'))

    if ctx.AUTOZ_CTN > 10 and mode=='AUTO PEAK':
            E2BUSY= 0                       #trigger E2PEAK transfer

    if ctx.AUTOZ_CTN > ctx.AUTOZ_SEC * 16:
        if r.get('setzAD')=='True':
            if mode=='MEASURE':
                timeout= convoffset(-_VAL,ctn)
                if timeout:
                    r.set('setzAD', False)
                    _OFF= round(AVG_OFF, 4) #update new zero offset
                    _tool_offset[int(r.get('mbBX'))]= _OFF
                    print "--offset value at %.4f--" % AVG_OFF
            else:
                r.set('setzAD', False)
                E2BUSY= 0
                ctx.RQZPEAK= 1
                _PEAK= 0                #manual reset zero peak

        if mode=='AUTO PEAK':
            ctx.AUTOZ_CTN= 0
            ctx.RQZPEAK= 1
            _PEAK= 0                    #auto reset zero peak

    # check measure mode
    return (mode=='MEASURE') 

cdef int convoffset(float AD, int ctn, int n=16):
    ''' moving average offset '''
    global AVG_OFF, END_OFF
    if END_OFF is None:
        END_OFF= (ctn + n) % 200
    if END_OFF < n:
        if ctn < (200 - n) and ctn > END_OFF:
            END_OFF= None
    elif ctn > END_OFF:
        END_OFF= None
    #moving average of offset on zero position
    AVG_OFF= round(0.764*AVG_OFF + 0.236*AD, 4)
    return (END_OFF is None)    #check timeout

def init_Emit():
    ''' conn to torque II '''
    r= redis.Redis('localhost')
    #r.set('mbBX', 4)
    #r.set('mbBXRQ', 4)   
    r.set('mbNew', True)
    r.set('mbOpen', True) 
    r.set('mbQuit', False)
    r.set('setzAD', False) 
    r.set('signAD', '1')
    r.set('holdAD', '1')
    r.set('E2BUSY', '1')
    r.set('E2PEAK', None)
    r.set('mbCtn', 0)
    return r

class modbusChannel():
    def __init__(self):
        global thread
        self.r= init_Emit()
            
        #Connect to the torque II
        #try:
        if True:
            # initize connected Box for measurment @19200
            self.altmax = 4;
            self.altstat = ["None"] * (self.altmax);
            self.altbox = 3; self.addr = 5;
            self.r.set('mbBX', self.addr-1)            
            #self.link = serial.Serial(port=3, baudrate=19200, bytesize=8, parity='N', stopbits=1)
            self.link = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)
            self.master = modbus_rtu.RtuMaster(self.link)
            self.master.set_timeout(0.04)  #0.025 
            time.sleep(0.2) 
            
            for dev in range(self.altmax):
                self.search_Box(dev)
           
            print("Tools mb Boxes...")
            print self.altstat[0:self.altmax]
            print("connected\n")   
            atexit.register(self.close)  
            
            if thread is None:
                thread = Thread(target= self.background_thread)
                thread.start() 
                
        '''   
        except:
            print("MB or COM on error")
            self.master.close()
            self.link.close()
        '''
            
    def search_Box(self,alt):
        global ctx
        try:
            time.sleep(0.01)		###II+
            ADx= self.master.execute(al t+1, cst.READ_HOLDING_REGISTERS, 0, 1)
            if (ADx[0]&0x08):
                self.addr= alt+1
                ###II+ verified tool
                #print "parked on addr: %s." % (self.addr,)
                time.sleep(0.01)
                ADx= self.master.execute(self.addr, cst.READ_HOLDING_REGISTERS, 0, 8)
                #print ADx
                
                if self.altstat[alt]!="Park":
                    self.r.set('setzAD', True)
                self.altstat[alt]= "Park"
                self.r.set('mbBX', alt) 
                ctx.BOX= alt
            else:
                self.altstat[alt]= "Idle"

        except:
            self.altstat[alt]= "None"

        finally:
            self.r.set('altSTAT',self.altstat[0:self.altmax])
            return (self.altstat[alt]=="Park")

    def next_alt(self):
        self.altbox -= 1
        if self.altbox < 0:
            self.altbox= self.altmax -1  

    def background_thread(self):
        """server generated events for streaming, reporting and watchdog."""
        while self.r.get('mbQuit')=='False':
            self.AD_callback()
            if self.r.get('mbShape_Req')=='True':
                self.r.set('dataShape', self.AD_shape())
                self.r.set('mbShape_Req', False)

    def convwdog(self):
        global ctx
        ctx.conv_CTN= min(ctx.conv_CTN + 1,1001)    #max 40sec
        if ctx.conv_CTN % 100 == 0:
            print "timeup on AD %3d" % (ctx.conv_CTN / 20)

    def close(self):
        self.r.set('mbOpen',False)
        time.sleep(0.4)
        self.master.close()
        self.link.close()
        print "clean up serial service!"

    def AD_get(self):
        global AD
        return AD
       
    def AD_shape(self):
        try:
            shape= (0,)*50
            READ_INPUT= cst.READ_INPUT_REGISTERS
            time.sleep(0.01)
            last_idx = self.master.execute(self.addr, READ_INPUT, 49, 2)[0]
            self.AD_callback()
            time.sleep(0.01)
            for i in range(30):
                shape+=self.master.execute(self.addr, READ_INPUT, (i+2)*25, 25) 
                time.sleep(0.01)
                self.AD_callback()
                time.sleep(0.01)

            scale = _tool_scale[self.addr-1] / 30000.0
            shape = shape[last_idx+1:] + shape[:last_idx+1]
            vals = tuple(_signint(x) for x in shape)
            self.AD_callback()
            time.sleep(0.01)
            vals= tuple(scale * x + _OFF*(x!=0) for x in vals)        
            
            return vals 
        except:
            return []
            
    def AD_callback(self):
        ''' background timer callback '''
        #sys.stdout.write('.')
        global _OFF, _CTN
        global _signint, ctx
        ctnLoop = 0
        vals= []
        val= None

        _CTN= (_CTN + 1) % 200     #cycle every 10 sec
        tooladdr = int(self.r.get('mbBXRQ'))
        tooladdr|= (ctx.RQZPEAK) * 0x10
        tooladdr|= 0xc0

        try:
            #prepare auto peak
            ctx.RQZPEAK= 0            
            if (tooladdr & 0x10):
                self.master.execute(self.addr,cst.WRITE_SINGLE_REGISTER,1,output_value=ctx.RQOFF)
    
            #auto AI mechanism 
            time.sleep(0.01)	###II+
            ADx= self.master.execute(self.addr, cst.READ_HOLDING_REGISTERS, 0, 8)
            ctnLoop= (ADx[5] + 1) % 0x10000
            self.r.set('mbCtn', ctnLoop)
            admean= _signint(ADx[2])
            adpeak= _signint(ADx[3])

            if (tooladdr & 0x10):
                #adpeak= ctx.RQOFF;  #updated prev of raw peak without offset
                adpeak= 0;
              
            #counter-check BOX with addr of MB is online
            if (ADx[0] & 0x08):	
                _OFF= _tool_offset[ADx[0] & 0X03]
                scale= _tool_scale[ADx[0] & 0x03]
                ctx.RQOFF= int(-_OFF * 30000 / scale)  
                ctx.conv_CTN= 1 
                val= scale*admean/30000
                ADM= val
                ADP= scale*adpeak/30000
            else:
                val= None

            if (_CTN % 10==0):		###II+ 10==0
                #print "search alt status..."
                self.search_Box(self.altbox)
                self.next_alt()
   
        except:
            self.convwdog()
            if self.search_Box(self.altbox):
                if ctx.conv_CTN > 5:
                    print "Connecting Boxes..."                
                    print self.altstat[0:self.altmax]
            else:
                self.next_alt()
            time.sleep(0.025)            
            val= None

        try:
            #broadcast tool request 
            ctnLoop = int(self.r.get('mbCtn'))		###
            self.master.execute(0,cst.WRITE_MULTIPLE_REGISTERS,4, output_value=[tooladdr,ctnLoop])
        
        except:
            print "broadcast error!"
            pass

        if val is not None:
            self.TransferAD(ADM, ADP, _CTN)

    def TransferAD(self, float ADM, float ADP, int ctn):
        ''' AD messaging '''
        global AD, _VAL, _OFF, _PEAK

        if AD is None:
            AD= ADM; _VAL= ADM
        AD= round(0.236 * AD + 0.764 * ADM, 4)
        ismeasure= convloop(ADM, ADP, ctn, self.r) 

        if AD is not None and ctn % 5 == 0:
            if ismeasure:
                val= round( _VAL + _OFF, 4)
            else:
                val= round(_PEAK, 4)
            AD_emit(val,self.r)
