# -*- coding: utf-8 -*-
"""
connections to Flask and SQLEXPRESS Data
"""
import time, re, csv, sys, atexit
import random, json
from glob import glob
reload(sys) 
sys.setdefaultencoding('utf8')

import logging
logging.basicConfig(level=logging.CRITICAL)
from socketIO_client import SocketIO, BaseNamespace

class Namespace(BaseNamespace):
    def htmlparser(self,msg):
        return re.sub("<[bB][rR]>", "\n", msg)
		
	def get_DB(self):
		pass

    def on_connect(self):
            print "\nTorque SQL connected"

    def on_my_response(self,*args):
        if len(args)!=0 and type(args[0])==dict:
            if args[0]['data']=='Connected':
                msg= ''.join(args[0]['count'])
                print self.htmlparser(msg)
            if args[0]['data']=='get_DB':
                msg= ''.join(args[0]['count'])
                msg= self.htmlparser(msg)
                msg= msg.strip()
                self.get_DB= list(csv.reader(msg.split('\n')))
            else:
                print args[0]['data']

def socketWin(auto=True, NS='/torque', port=80):
	#pending on threading.Timer
    if auto:
        _IP= "192.168.32.171"    #'192.168.31.111'
        print 'connecting...'
        socketIO= SocketIO(_IP, port)
        ws= socketIO.define(Namespace, NS)
        socketIO.wait(seconds=1)
        return (socketIO, ws)
    else:
        return (None, None)

#data file in json handling
def loadJS(filename='/home/pi/share/areg.dat'):
    db = []
    with open(filename) as fp:
        for line in fp:
            db.append(json.loads(line))
    users_DB= db[0]['id']
    alias_DB= db[0]['alias']
    return (users_DB,alias_DB)
    
def dumpJS(users_DB,alias_DB,filename='/home/pi/share/areg.dat'):
    data= {'id':users_DB,'alias':alias_DB}
    with open(filename, 'w') as fp:
            json.dump(data, fp)
