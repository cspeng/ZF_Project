# -*- coding: utf-8 -*-
"""
@author: HORAE
"""
from time import sleep
import sys, atexit, signal

def atclose(*args):
    print "see you", args
    sys.exit(0)

if __name__ == '__main__':
    ## $ python signal_SIGINT.py
    signal.signal(signal.SIGINT,atclose)
    atexit.register(atclose,'again')
    try:
        print "Hello"
        i = 0
        while True:
            i += 1
            print "Iteration #%i" % i
            sleep(1)

    except SystemExit, err:
        print "system Exit"
        if err.code != 0:
            print "abnormal exit(%s)" % err        
    
    finally:
        print "Goodbye"
