#!/usr/bin/python
# *AD7606 schematic*
# RANGE L/H selection: +/-5V or +/-10V
# BUSY      pin-11
# CVA       pin-12  CVB     pin-12
# RST       pin-13  RANGE   pin-15
# DB15      Ground  OS2     3.3V
# DB7(MISO) pin-21
# RD(SCLK)  pin-23
# CS(CE0)   pin-24
# *equation*
# (Vin/10V) = (hex/32768) * (2.5V/Vref)
#
# *logging level*
# DEBUG		Full diagnosing
# INFO		Confirmation working
# WARNING	indication unexpected happened
# ERROR		serious problem to perform function
# CRITICAL	A serious error unable to continue running
# *  *
import logging
logging.basicConfig(level=logging.CRITICAL)
AD= None; _CTN= 0

import RPi.GPIO as GPIO
import wiringpi2 as wiringpi
import spidev, time, re, sys
from blessings import Terminal
from socketIO_client import SocketIO, BaseNamespace

class Namespace(BaseNamespace):
    term= Terminal()
    def htmlparser(self,msg):
        return re.sub("<[bB][rR]>", "\n", msg)

    def on_connect(self):
            self.term.move_y(1)
            print self.term.clear() + "\nTorque SQL connected"

    def on_my_response(self,*args):
        if len(args)!=0 and type(args[0])==dict:
            if args[0]['data']=='Connected':
                msg= ''.join(args[0]['count'])
                print self.htmlparser(msg)

def AD_callback(channel):
    #falling edge on pin 11 trigger to read SPI
    #sys.stdout.write('.')
    global spi,_IP,_CTN
    _CTN= (_CTN + 1) % 1000
    ADx= spi.readbytes(4)
    AD= TransferAD(ADx,_CTN)
    with Namespace.term.location(x=0):
        print "<%3d> AD value: %s  " % (_CTN,AD),

def AD_emit(val):
    global ws_torque, socketIO
    if ws_torque is not None:
        try:
            ws_torque.emit('AD event', {'data': str(val)})
        except:
            socketIO.disconnect()
            print Namespace.term.clear() + '\nservice waiting reconnection...'
            (socketIO, ws_torque)= socketWin()

def AD_close():
    global spi
    wiringpi.pwmWrite(1, 0)
    GPIO.cleanup()
    spi.close()

def BytesToHex(Bytes): 
    return ''.join(["0x%02X " % x for x in Bytes]).strip()     

signint = lambda x: (int("0x%02X%02X" % tuple(x), 16) - (x[0] >> 7) * 65536)

def TransferAD(ADx,ctn):
    global AD 
    vals= []; r= 1
    for k in range(len(ADx)/2):
        val= 100.0* signint((ADx[k*2],ADx[k*2+1]))/32768
        vals.append(round(val,3))
    if len(vals)>1 and vals[0] > 30.0:
        if AD is None:
            AD= vals[r]
        AD= round(0.854*AD + 0.146*vals[r],3)
    if AD is not None:
        vals[r]= AD
        if ctn % 5 == 0:
            AD_emit(AD)
    return ''.join(["%2.2f " % x for x in vals]).strip()

def socketWin(auto=True, NS='/torque', port=80):
	#pending on threading.Timer
    if auto:
        _IP=  "192.168.32.172"  #'192.168.31.111'    
        print 'connecting...'
        socketIO= SocketIO(_IP, port)
        ws= socketIO.define(Namespace, NS)
        socketIO.wait(seconds=1)
        return (socketIO, ws)
    else:
        return (None, None)

#conn to torque server
(socketIO, ws_torque)= socketWin(True)

#setup int callback
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(11, GPIO.FALLING, callback=AD_callback, bouncetime=1)

#AD7606 setos 800, range, reset pulse
GPIO.setup(15, GPIO.OUT)    # range fn
GPIO.output(15, True)
GPIO.setup(13, GPIO.OUT)    # reset fn
GPIO.output(13, False)
GPIO.output(13, True)
GPIO.output(13, True)
GPIO.output(13, False)

#setup convst_bar pin_12 PWM
#theretically 50 (1ms) to 100 (2ms) on some servo and 30-130 may work ok
wiringpi.wiringPiSetup()
wiringpi.pinMode(1,2)       # pwm only works on wiringpi port 1
#wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
wiringpi.pwmSetClock(384)   # clock at 50kHz (20us tick)
wiringpi.pwmSetRange(1000)  # range at 1000 ticks (20ms)
wiringpi.pwmWrite(1,995)

#SPI device CS0
spi= spidev.SpiDev()        # create spi object 
spi.open(0, 0)              # open spi port 0, device (CS) 0
spi.max_speed_hz= 11500000  # set clock speed

try:
    while True:
        pass

except KeyboardInterrupt:   #CTRL+C exit
    AD_close()
    print "\nclean up GPIO!\n"
AD_close()
