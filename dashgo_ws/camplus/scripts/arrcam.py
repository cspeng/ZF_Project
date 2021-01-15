#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Clarence <2018>
"""
from __future__ import print_function
PKG= 'camplus'
import roslib; roslib.load_manifest(PKG)
import rospy
import cv2, trioV
import time, memcache
import io, numpy as np
from threading import Thread

#import RPi.GPIO as GPIO
from trioAI import prepare_data, hog
from std_msgs.msg import String, Header
teleop_remote = None

mtx = [[  2.21272152e+04,   0.00000000e+00,   5.20339792e+02],
       [  0.00000000e+00,   2.18608177e+04,   3.79128212e+02],
       [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]

dist = [[ -1.18408943e+02,   3.88879333e+03,  -1.21960449e-01,
         -2.87685203e-03,   7.66579420e+00]]

mtx = np.array(mtx)
dist = np.array(dist)
acam_status = ""

def callback_teleop_status(msg):
    global acam_status
    acam_status = msg.frame_id

# line parameters
class line_setup(trioV.base_lsetup):
    """A updating signature of current line paraemeters"""
    def update(self):
        # warping matrix
        # widthCam-3B (800,640); widthCam-5A (920,640)
        self.wedgeDrift= np.asarray((0,0))        
        self.pysite= "5A"
        self.assWidth= 960   
        self.ptm= [(37, 0), (975, 0), (212, 767), (790, 767)]
        self.pts= [(0,0),(959,0),(0,767),(959,767)]

# feature container
class hintGuide(trioV.hintEnum):
    """warp interactive view"""
    def update(self):
        self.resCam= (1024,768)
        self.logfname= "/home/eaibot/nginx-flask/static/arr.log"
        self.Ecam_NAME = 'Cam Plus Track [ESC to quit]'
        self.Elsetup= line_setup()
        self.Elsetup.update()
        self.Ewarp= trioV.cvwarp(self.Elsetup)
        self.buoy.assWidth= self.Elsetup.assWidth
        #initial color parameters
        self.inRange= self.Elsetup.inRange()
        self.hQuanta= self.Elsetup.hQuanta(self.inRange)
        self.hRange= None         
        cv2.namedWindow(self.Ecam_NAME)
        #remark mouse event when not in calibration  
        #cv2.setMouseCallback(self.Ecam_NAME, self.hintBox)
        
        self.is_quit = False

    def check(self,warp):
        res= memcache.Client(["127.0.0.1:11211"]) 
        self.altstream= not self.altstream
        resfn= "roi1.jpg" if self.altstream else "roi0.jpg"
        cv2.imwrite("/home/eaibot/nginx-flask/static/"+resfn, warp)
        res.set("pysite",self.Elsetup.pysite)
        res.set("arr_altstream",self.altstream)
        
        if self.Emode is self.warp:
            self.Ewarp.gline(warp)
            
        #res.set("arr_islock", False)                  
        if acam_status=='_NORM_' and res.get("arr_islock"):
            self.BeltPowerOn()            

    def change(self,key):
        if key == ord("t"):
            self.Emode= self.test
        elif key == ord('w'):
            self.Emode= self.warp
    
    def quit(self):
        res= memcache.Client(["127.0.0.1:11211"]) 
        res.set("arr_scanid",False)
        res.set("arr_ishalt",True)
        self.is_quit = True    
    
    # main features
    def BeltPowerOn(self):
        res= memcache.Client(["127.0.0.1:11211"]) 
        teleop_remote.publish('_NORM_')
        res.set("arr_isresume",True) 
        res.set("arr_islock", False)

    def BeltPowerOff(self):
        teleop_remote.publish('_HALT_')

    def datetime(self):
        return time.strftime('%y-%m-%d %H:%M:%S', time.localtime(time.time())) 

    def lockfname(self,resume):
        logpath= '/home/eaibot/log/'
        pysite= self.Elsetup.pysite
        monthcode= time.strftime('-%y%m-', time.localtime(time.time()))
        lockFN= 'f' + pysite + monthcode[0:3] + '*.jpg'
        serialFN= trioV.appendSequence(logpath+lockFN, resume)
        lockFN= 'r' if resume else 'f'
        lockFN+= pysite + monthcode + serialFN + '.jpg'
        return logpath + lockFN

    def ResumeSub(self):
        res= memcache.Client(["127.0.0.1:11211"]) 
        res.set("arr_scanid",True)
        
        (users_DB,alias_DB)= trioV.loadDB('/home/eaibot/share/areg.dat') 
        while not rospy.is_shutdown() and not self.is_quit:
            self.resumeID= trioV.unLockBox(users_DB).resumeID
            if self.resumeID is None:
                break
            else:
                info= "resume ID: " + self.resumeID + ' @ ' + self.datetime()
            self.BeltPowerOn()
            print(info)

        print("quit unlock scan!")    
        
    def lockproc(self,warp):
        res= memcache.Client(["127.0.0.1:11211"])         
        self.BeltPowerOff()
        res.set("arr_islock", True)
        info= "fail@ " + self.datetime()
        lockFN= self.lockfname(resume=False)
        cv2.putText(warp,info,(200,300),self.Efont,0.7,(0,220,220),2)               
        cv2.imwrite(lockFN,warp) 
        self.hog_sample()       

    def hog_sample(self, reset=False):
        blue = cv2.imread("/home/eaibot/share/hog_arr/a2ablue.jpg")
        red = cv2.imread("/home/eaibot/share/hog_arr/a2ared.jpg")
        
        if blue is None or red is None:
            reset = True
            
        if not reset:
            imblue = np.hsplit(blue, blue.shape[1]/38)
            imred = np.hsplit(red, red.shape[1]/38)
        else:
            imblue, imred= ([],[])

        for im in self.track:
            if im.shape[:2]==(38L,38L):
                major=self.stat(im)
                if major=='blue':
                    imblue.append(im)
                if major=='red':
                    imred.append(im)

        blue = np.hstack(imblue) if len(imblue) else None
        red = np.hstack(imred) if len(imred) else None
        
        blue = np.tile(np.array([150,30,30],np.uint8),(38,38,1)) if reset else blue
        red = np.tile(np.array([90,90,200],np.uint8),(38,38,1)) if reset else red
        
        if blue is not None:
            cv2.imwrite("/home/eaibot/share/hog_arr/a2ablue.jpg", blue)
        if red is not None:
            cv2.imwrite("/home/eaibot/share/hog_arr/a2ared.jpg", red)
        if reset:
            nullpoint = np.array([120,120,120],np.uint8)
            null = np.tile(nullpoint, (38,38,1))
            cv2.imwrite("/home/eaibot/share/hog_arr/a2anull.jpg", null)

    #compare color quantizer and hog predictor
    def obsAi(self,warp,rect_cut):
        global majors
        majors= self.color_Screw(warp,rect_cut)
        obs_colors = self.obs_Color()

        m = len(majors[0])        
        for k, obs in enumerate(majors[0]):
            if obs != obs_colors[k]:
                majors[0][k] = 'none'
                
        for k, obs in enumerate(majors[1]):
            if obs != obs_colors[k+m]:
                majors[1][k] = 'none'
        
        buoys= self.buoy.stat(majors,rect_cut)
 
        pairs= self.buoy.pair_Screw(majors,buoys,rect_cut)
        alerts= self.buoy.alert_Match(majors,buoys,pairs)
        majority= {'color':majors,'alert':alerts,'pair':pairs,'buoy':buoys}
        return majority

    def obs_Color(self):
        global result
        obsColor= ['none', 'blue', 'red','none']
        
        null = np.tile(np.array([120,120,120],np.uint8), (38,38,1))
        tracks = [im if im.shape[:2]==(38L,38L) else null for im in self.track]

        hogData = [list(map(hog, tracks))]
        obsData = np.float32(hogData).reshape(-1, 16*16)
        result = svm.predict(obsData)[1]
        result = np.array(result).flatten()

        if result[0] is not None:
            result = result.astype(int).tolist()
            colors = [obsColor[resp] for resp in result]
        else:
            colors = []

        return colors


if __name__ == "__main__":
    ''' acameye '''
    rospy.init_node('acam_eye', anonymous=True)
    rospy.Subscriber('tele_status', Header, callback_teleop_status)
    teleop_remote = rospy.Publisher('/teleop_remote', String, queue_size=2)
    interface = rospy.Publisher('/interface', String, queue_size=2)
    rospy.loginfo("publish /interface of ACAM on Trio socket")
    teleop_remote.publish('_PING_')
    
    hints= []; majority= []
    svm, trainData, responses = prepare_data(0.0)

    hints= hintGuide()
    hints.update()
    MparT= hints.Elsetup.MparT()
    MparB= hints.Elsetup.MparB()    
    h= hints.resCam[1]
    auto= True; mask_enable= False; imthread= None
    warm_time= time.time()
    with open(hints.logfname,'a') as log_file:
        log_file.write('\n### restart cam ' + hints.Elsetup.pysite)     

    capture= cv2.VideoCapture(0)
    capture.set(3,hints.resCam[0])
    capture.set(4,hints.resCam[1])

    cap_wh = hints.resCam
    newcameramtx, cap_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, cap_wh, 1, cap_wh)
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, cap_wh, 5)

    thread =Thread(target=hints.ResumeSub)
    thread.start()

    while auto and not rospy.is_shutdown():
        #try:
        hints.BeltPowerOff()           
        time.sleep(1.6)
        #cam.start_preview()

        start_time= time.time()
        BeltStop= [False]*4
        hints.BeltPowerOn()
        
        with open(hints.logfname,'a') as log_file:
            log_file.write('\n' + hints.datetime() + ' Setup  cam AGC')
            log_file.write('\n' + ' AWB setting at ' + str(hints.Elsetup.AWB))
            log_file.write('\n')

        while not rospy.is_shutdown():
          grabbed, frame = capture.read()
          if grabbed==True: 
            # undistort
            #dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
            #dx, dy, dw, dh = cap_roi
            #frame = dst[dy:dy+dh, dx:dx+dw]
            dst = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
            frame = dst
            
            h, w=frame.shape[:2]  
            frame= [frame[0:h/2],frame[h/2:h]]
            if hints.Emode is not hints.warp:
                Tracks = trioV.pairmap(hints.warpColor_transform,frame,[MparT,MparB])                
                majority= Tracks
            else:
                Tracks = trioV.pairmap(hints.warpColor_transform,frame,[None]*2)                
            track= np.vstack([Tracks[0][0],Tracks[1][0]])
            frame= np.vstack([Tracks[0][1],Tracks[1][1]])
            hints.Eframe= frame
            warp= frame.copy()
            (gray_cut, shiftx)= hints.colorContour(track,hints.inRange)
            rect_cut= hints.buoy.colorRects(gray_cut,shiftx)
            majority= hints.obsAi(warp,rect_cut)
            hints.buoy.rectScrew(warp,rect_cut,majority,mask_enable)

            #stop status
            BeltStop= BeltStop[1:4]
            BeltStop.append(majority['alert'][0].any() or majority['alert'][1].any())
            if acam_status=='_SERV_':   BeltStop = [False]*4
            if acam_status=='_HALT_':   BeltStop = [False]*4

            lag= "lag second: %.2f" % (time.time()-start_time)
            lag+= '    %s' % (acam_status)
            start_time= time.time()
            cv2.putText(warp,lag,(400,16),hints.Efont,0.6,(160,160,160),2)
            hints.check(warp)

            ###checkpoint  warp = hints.colorTrack(frame)
            cv2.imshow(hints.Ecam_NAME,warp)
            #cv2.imshow('cut', np.hstack(gray_cut))
            
            interface.publish("$ACAM," + acam_status + "\r\n")
            
            key= cv2.waitKey(1) & 0xFF
            hints.change(key)            
            if key == 27:
                hints.BeltPowerOff()
                break
                
            if key == ord("c"):
                hints.hog_sample(reset=True)
            if key == ord("h"):
                hints.hog_sample()
            if key == ord('m'):
                hints.screw_mask(hints.Eframe)
                mask_enable= not mask_enable

            if key == ord("l") or np.array(BeltStop).all():
                with open(hints.logfname,'a') as log_file:
                    log_file.write('\n' + hints.datetime() + ' enter lock mode')                
                hints.lockproc(warp)
                BeltStop= [False]*4

            if warm_time > 0 and (start_time-warm_time) > 38:
                warm_time= 0
                
            if key == ord('a') or not warm_time:
                auto= True
                warm_time= -1
                break
            else:
                auto= False
                
        cv2.imwrite("aroi.jpg", hints.Eframe)
        hints.screw_mask(hints.Eframe)
    
        #except:
        #print("cam on resume due to cam error!")
        #auto = True

    hints.BeltPowerOn()
    capture.release()           
    with open(hints.logfname,'a') as log_file:
        log_file.write('\n' + hints.datetime() + ' Close cam Arrow\n')
        
    cv2.destroyAllWindows()
    {cv2.waitKey(i) for i in range(5)}
    hints.BeltPowerOff()
    hints.quit()
