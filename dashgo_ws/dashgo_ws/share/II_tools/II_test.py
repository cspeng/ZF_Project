#!/usr/bin/python
# -*- coding: utf-8 -*-
'''
Torque App by TRiO
'''
import sys, atexit
from II_tkForm2 import winApp
import pi_modbus
from bspview import sqlConn, ToolClose
import redis

ToolClose()
r= redis.Redis('localhost')
r.set('mbBX', 3)
r.set('mbBXRQ', 3)   
#r.set('altSTAT',None)

r= pi_modbus.modbusChannel().r 
DS= sqlConn(True)
print DS.sqlDBheading()

app= winApp(DS,r)
app.start()
ToolClose()
