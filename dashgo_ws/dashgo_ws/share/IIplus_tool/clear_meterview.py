# -*- coding: utf-8 -*-
"""
STM_AD channel clear
"""
import sys, time

def isRaspi():
    return (sys.platform=='linux2')     #'win32' or 'linux2'

def datetime():
    return time.strftime('%y-%m-%d %H:%M:%S', time.localtime(time.time())) 
    
def clearToolOpen():
    try:
        import redis                  
        r= redis.Redis('localhost')     #localDB enquiry
        r.set('mbOpen',False)
        print "mbAD are ready"
        return True  
    except:
        print "mbAD not ready"
        return False


if __name__ == "__main__":
    """ clear Meter Instance """        # XXX ok 
    clearToolOpen()
    time.sleep(0.6)

