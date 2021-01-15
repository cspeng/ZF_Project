# -*- coding: utf-8 -*-
"""
@author: HORAE
"""
import sys

#add logging capability
import logging

import serial, modbus_tk
import modbus_tk.defines as cst
import modbus_tk.modbus_rtu as modbus_rtu

#logger = modbus_tk.utils.create_logger("console")

if __name__ == "__main__":
    try:
        #Connect to the slave
        addr = 3;
        master = modbus_rtu.RtuMaster(serial.Serial(port=3, baudrate=9600, bytesize=8, parity='N', stopbits=1))
        master.set_timeout(0.025)
        #master.set_verbose(True)
        print("connected")

        print(master.execute(addr, cst.READ_COILS, 0, 8))
        print(master.execute(addr, cst.READ_DISCRETE_INPUTS, 0, 8))
        print(master.execute(addr, cst.READ_HOLDING_REGISTERS, 0, 8))
        print(master.execute(addr, cst.READ_INPUT_REGISTERS, 0, 8))
        master.close()
       
    except modbus_tk.modbus.ModbusError, e:
        #logger.error("%s- Code=%d" % (e, e.get_exception_code()))
        master.close()
