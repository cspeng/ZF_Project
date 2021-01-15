#!/usr/bin/env python
# -*- coding: utf-8 -*-
import socket
import time
import datetime
import curses

stdscr = curses.initscr()
clean = " " * 40

def display_info(info, x, y, colorpair=1):
    '''''使用指定的colorpair显示文字'''  
    global stdscr
    stdscr.addstr(y, x, clean, curses.color_pair(colorpair))    
    stdscr.addstr(y, x, info, curses.color_pair(colorpair))
    stdscr.refresh()

def set_win():
    '''''控制台设置'''
    global stdscr

    #文字和背景色设置，设置了两个color pair，分别为1和2
    curses.start_color()
    curses.init_pair(1, curses.COLOR_WHITE, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_CYAN, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.noecho()    #关闭屏幕回显
    curses.cbreak()   #输入时不需要回车确认
    curses.curs_set(0)
    stdscr.nodelay(1)
    stdscr.clear()

def unset_win():
    '''控制台重置'''
    global stdstr
    #恢复控制台默认设置（若不恢复，会导致即使程序结束退出了，控制台仍然是没有回显的）
    curses.nocbreak()
    stdscr.keypad(0)
    curses.echo()
    #结束窗口
    curses.endwin()

#HOST, PORT = "10.0.2.15", 8888
HOST, PORT = "192.168.112.137", 8888

host112_list = ["31.200","112.131","129.140","146.99","129.142","137.40"]
#data = "_check_msg_"
data = "_check_msg_"
is_shutdown = False
hostskip = [0] * len(host112_list)
k = 0

try:
    set_win()
    display_info(" AGV thro' socket,", 0, 1, 1)

    while not is_shutdown:
        # Create a socket (SOCK_STREAM means a TCP socket)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.6)

        t = datetime.datetime.now().strftime('%H:%M')
        display_info("status at {}".format(t), 19, 1, 1)
        
        # Connect to server and send data
        HOST = "192.168."
        HOST += host112_list[k]

        if hostskip[k]==0:
            try:
                sock.connect((HOST, PORT))
                   
                #print((HOST, PORT, data))
                display_info(clean, 0, 4+k*3)
                display_info(clean, 0, 5+k*3)
                display_info('PORT %s:%s  %s' %(HOST, PORT, data), 0, 4+k*3)
                
                sock.sendall(data + "\n")
                
                # Receive data from the server and shut down
                received = sock.recv(1024)
                sock.close()

                #print "Received: {}".format(received[:40])  
                display_info("msg  {}".format(received.strip('\r\n')), 0, 5+k*3)
                hostskip[k] = 0
                
            except socket.timeout:
                #print("connect to %s timeout\n" % HOST)
                display_info('connect to {} timeout'.format(HOST), 0, 4+k*3, 2)
                display_info("$", 0, 5+k*3, 2)
                hostskip[k] = 20
                
            except socket.error:
                #print("connect to %s refused" % HOST)
                display_info('connect to {} refuse'.format(HOST), 0, 4+k*3, 2)
                display_info("$", 0, 5+k*3, 2)
                hostskip[k]= 20
            
        else:
            hostskip[k]= max(hostskip[k] - 1, 0)
            
        k = (k + 1) % len(host112_list)
        time.sleep(0.1)
        
except KeyboardInterrupt:
    print "stop on ctrl-C"
    is_shutdown = True
    #raise
    
finally:
    unset_win()
