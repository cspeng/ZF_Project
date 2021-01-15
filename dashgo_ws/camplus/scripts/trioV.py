# -*- coding: utf-8 -*-
"""
@author: Clarence
#link on arrcam, arrcolor-test, ecam:) map, cvhist, roiMatch, drawContours, 
#windowBox, imbox.
#link on arrParallel1, arrVmap2:) immap, imRGB, cZip.
#
"""
import cv2, time
import numpy as np
import Tkinter as tk, json
import memcache
from glob import glob
from sklearn.cluster import MiniBatchKMeans
from threading import Thread as xthread

# line parameters
class base_lsetup():
    """A BaseClass setup widget for the line parameters"""
    def __init__(self):
        # color space detection
        self.pink_lower = np.array([171, 90, 90],np.uint8)
        self.pink_upper = np.array([179, 255, 255],np.uint8)    
        self.red_lower = np.array([0,  120, 90],np.uint8)
        self.red_upper = np.array([12, 255, 255],np.uint8)
        self.blue_lower = np.array([95, 60, 90],np.uint8)
        self.blue_upper = np.array([120, 255, 255],np.uint8)
        self.vblue_lower = np.array([121, 60, 90],np.uint8)
        self.vblue_upper = np.array([128, 255, 150],np.uint8)
        self.__update()

     # reserve new signature for updating init parameters
    def update(self):
        # warping matrix
        self.update()
        self.assWidth= 960 
        self.wedgeDrift= np.asarray((0,0))
        self.ptm= [(0,0),(959,0),(0,767),(959,767)]
        self.pts= [(0,0),(959,0),(0,767),(959,767)]
        self.AWB= None

    __update = update      
        
    def inRange(self):
        ir= [(self.pink_lower,self.pink_upper,'red')]
        ir.append((self.red_lower,self.red_upper,'red'))
        ir.append((self.blue_lower,self.blue_upper,'blue'))
        ir.append((self.vblue_lower,self.vblue_upper,'blue')) 
        return ir
        
    def hQuanta(self,ir):
        iq= np.asarray([0 for i in xrange(256)],dtype='uint8')
        cval= {'none':0,'red':1,'blue':2,'gold':3}  
        for hRange in ir:
            for k in xrange(hRange[0][0],hRange[1][0]+1):
                iq[k]= cval[hRange[2]]
        return iq        

    def Mwarp(self):
        ptm= np.float32(np.asarray(self.ptm))
        pts= np.float32(np.asarray(self.pts))
        return cv2.getPerspectiveTransform(ptm,pts)
    
    def Mshape(self):
        pts= np.float32(np.asarray(self.pts))       
        return (int(pts[1][0]-pts[0][0]+1),int(pts[2][1]-pts[0][1]+1))

    def MparT(self):
        Mshape= self.Mshape()
        Mshape= (Mshape[0],Mshape[1]/2)        
        ptm= np.float32(np.asarray(self.ptm))
        pts= np.float32(np.asarray(self.pts))
        midline= ((0,Mshape[1]),Mshape)
        C2= line_intersection((ptm[0],ptm[2]),midline)        
        C3= line_intersection((ptm[1],ptm[3]),midline)        
        ptm[2]= np.asarray(C2) + self.wedgeDrift
        ptm[3]= np.asarray(C3) + self.wedgeDrift
        (pts[2],pts[3])= np.asarray(midline)
        Mwarp= cv2.getPerspectiveTransform(ptm,pts)
        return(Mwarp,Mshape)
        
    def MparB(self):
        Mshape= self.Mshape()
        Mshape= (Mshape[0],Mshape[1]/2)     
        ptm= np.float32(np.asarray(self.ptm))
        pts= np.float32(np.asarray(self.pts))
        midline= ((0,Mshape[1]),Mshape)
        C0= line_intersection((ptm[0],ptm[2]),midline)        
        C1= line_intersection((ptm[1],ptm[3]),midline)
        ptm[0]= np.asarray(C0) 
        ptm[1]= np.asarray(C1)
        (pts[0],pts[1])= np.asarray(midline)
        ptm= ptm - np.float32(np.asarray((0,min(ptm[0,1],ptm[1,1]))))
        pts= pts - pts[0]
        Mwarp= cv2.getPerspectiveTransform(ptm,pts)
        return(Mwarp,Mshape) 

def loadDB(filename='/home/eaibot/share/areg.dat'):
    db = []
    with open(filename) as fp:
        for line in fp:
            db.append(json.loads(line))
    users_DB= db[0]['id']
    alias_DB= db[0]['alias']
    return (users_DB,alias_DB)

def dumpDB(users_DB,alias_DB,filename='/home/eaibot/share/areg.dat'):
    data= {'id':users_DB,'alias':alias_DB}
    with open(filename, 'w') as fp:
         json.dump(data, fp) 

class unLockBox():
    def __init__(self,users_DB):
        self.Box()
        self.resumeID = None
        self.users_DB= users_DB
        self.user = self.makeentry(self.frame, "Resume ID:",12)
        self.status = self.makeentry(self.frame, "Status:",15,textvariable=self.s)
        self.Reader()

    def periodicCall(self):
        res= memcache.Client(["127.0.0.1:11211"])
        if not res.get("arr_scanid"):
            print('scanid is closed')
            self.root.destroy()
            
        elif res.get('<Locksmith>') in self.users_DB:
            self.resumeID = res.get('<Locksmith>')
            res.set('<Locksmith>',False)
            print 'unlock by ID', self.resumeID
            self.root.destroy()
            
        else:
            self.frame.after(500, self.periodicCall)

    def Box(self):
        self.root = tk.Tk()
        self.root.title('waiting resume by ID')
        self.root.attributes("-topmost", True)
        self.frame = tk.Frame(self.root)
        self.frame.grid(row=0, column=0)
        self.frame.pack()
        self.frame.focus_force()
        self.s= tk.StringVar()
        self.s.set('waiting')
        
    def Reader(self):
        self.user.bind('<Return>', self.valid_user)
        self.user.focus_force() 
        self.frame.after(5000, self.periodicCall)
        self.root.mainloop()

    def valid_user(self,event):
        self.resumeID= self.user.get().rstrip()
        if self.resumeID in self.users_DB:
            print 'unlock by ID', self.resumeID
            self.root.destroy()
        else:
            self.s.set('unregister user')
            self.root.update()
            time.sleep(1)
            self.user.delete(0, tk.END)
            self.resumeID= None
            self.s.set('waiting')           

    def makeentry(self,parent, caption, width=None, **options):
        tk.Label(parent, text=caption).pack(side=tk.LEFT)
        entry = tk.Entry(parent, **options)
        if width:
            entry.config(width=width)
        entry.pack(side=tk.LEFT)
        return entry


def imthread(wname,warp,p=None):
    def worker_wrapper(wname,warp):
        cv2.imshow(wname,warp)    
    if p is not None:
        p.join()
    else:
        cv2.imshow(wname,warp)
    p= xthread(target=worker_wrapper,args=(wname,warp))
    p.start()
    return p

def map(func,args):
    n= len(args); result= [None] * n  
    def worker_wrapper(i):
        result[i] = func(args[i])    
    pool= [xthread(target=worker_wrapper,args=(i,)) for i in xrange(n)]
    for p in pool:
        p.start()
    for p in pool:
        p.join()
    return result

def immap(func,im,args):
    n= len(args); result= [None] * n  
    def worker_wrapper(i):
        result[i] = func(im,args[i])    
    pool= [xthread(target=worker_wrapper,args=(i,)) for i in xrange(n)]
    for p in pool:
        p.start()
    for p in pool:
        p.join()
    return result

def pairmap(func,ims,args):
    n= len(args); result= [None] * n  
    def worker_wrapper(i):
        result[i] = func(ims[i],args[i])    
    pool= [xthread(target=worker_wrapper,args=(i,)) for i in xrange(n)]
    for p in pool:
        p.start()
    for p in pool:
        p.join()
    return result

def appendSequence(pathname,resume=False):
    #e.g pathname= "G:\ArrowLog\F5A-15*.jpg"
    seqPOS= len(''.join(x for x in pathname if x=='-'))+1
    seqFN= glob(pathname)
    NextSeq= [x.split('-')[seqPOS].split('.')[0] for x in seqFN]
    NextSeq= [int(x) for x in NextSeq if x.isdigit()]
    NextSeq= max(NextSeq) + (not resume) if len(NextSeq) else 1
    return "{0:03d}".format(NextSeq)

def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1]) #Typo was here

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return (x, y)

def farDist(pt,rect,th):
    mindist= 9999
    for rt in rect:
        norm= np.linalg.norm(np.asarray(rt[0])-np.asarray(pt))
        mindist= np.minimum(mindist,norm)
    return (mindist>th)
	
def roiMatch(tempfile,threshold,roi,flag=' '):
    buoy= []
    mindist= 16
    template= cv2.imread(tempfile,0)    
    w, h = template.shape[::-1]
    res= cv2.matchTemplate(roi, template, cv2.TM_CCOEFF_NORMED)
    loc= np.where( res >= threshold)
    for pt1 in zip(*loc[::-1]):
        if farDist(pt1,buoy,mindist):
            pt2= (pt1[0] + w, pt1[1] + h)
            buoy.append((pt1,pt2,flag))
    return buoy 

def windowBox(points) :
    Box= np.array(points)
    tl= Box.min(axis=0)
    br= Box.max(axis=0)
    return (tl[0], tl[1], br[0], br[1])

def imbox(frame,roi):
    roiPts= roi[0:2] if type(roi[2])==str else cv2.boxPoints(roi)
    roiBox= windowBox(roiPts)
    return frame[roiBox[1]:roiBox[3], roiBox[0]:roiBox[2]]

def imRGB(filename):
    img= cv2.imread(filename)
    img= cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    return img

def imshow(im):
    from matplotlib import pyplot as plt
    img=cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
    plt.imshow(img), plt.show()

def drawContours(im,rects,color=(0,0,255),thickness=2):
    for rect in rects:
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(im,[box], 0, color, thickness)

# find and plot histogram on CV 
class cvhist():
    """A stat widget about HSV histogram"""
    def __init__(self, HEIGHT=100, WIDTH=180,gray=196):
        self.WIDTH= WIDTH
        self.HEIGHT= HEIGHT
        self.h= []
        self.color= [gray,gray,gray]
        self.bgcolor= [176,228,239]
        self.font= cv2.FONT_HERSHEY_SIMPLEX
 
    def colorStack(self,im,cluster=4):
        (img, goldctr, info)= self.stat(im)
       
        imh= []; hcol= (35,35,240)
        for c in goldctr:
            h= img.copy(); h[:,:,0:2]= c
            cv2.putText(h, 'H:'+str(c[0]), (1,19), self.font, 0.3, hcol, 1)
            cv2.putText(h, 'S:'+str(c[1]), (1,28), self.font, 0.3, hcol, 1)            
            imh.append(h)
                    
        cv2.putText(img, 'quantizer', (1,24), self.font, 0.23, hcol, 1) 
        img= np.hstack([img]+imh)
        img= cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        img= np.hstack([im,img])
        img= cv2.convertScaleAbs(img) 
        return img

    def stat(self, im, cluster=4):
        img= cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        (h, w) = img.shape[:2]
        im = img.reshape((h * w, 3))
        imv= im[:,2]; im= im[:,0:2]

        clt = MiniBatchKMeans(cluster)
        labels = clt.fit_predict(im)
        goldctr= clt.cluster_centers_.astype("uint8")
        im = goldctr[labels]        
        im = cv2.merge([im[:,0],im[:,1],imv]) 
        im= im.reshape((h, w, 3))
        
        grayinfo= []
        for clust in xrange(len(goldctr)):
            k=  self.search_index(labels, lambda x: x==clust)
            r=np.asarray(list(k))
            Vlow= str(int(np.percentile(imv[r],21)))
            Vhigh= str(int(np.percentile(imv[r],79)))
            grayinfo.append((Vlow,Vhigh))

        return (im, goldctr,grayinfo)

    def search_index(self,lst, test):
        return ( pair[0] for pair in zip(range(len(lst)), lst) if test(pair[1]) )

    def calc(self,im):
        self.h= self.colorStack(im)
        cv2.rectangle(self.h,(0,0),self.h.shape[0:2][::-1],(160,160,160),3)         
        return self.h.shape

    def hshow(self):
        cv2.imshow('colorhist',self.h)
        cv2.waitKey(0)

# find warping matrix and plot perspective view on CV 
class cvwarp():
    """A stat widget about perspective warping"""
    def __init__(self,lsetup, HEIGHT=320, WIDTH=270,gray=192):
        self.WIDTH= WIDTH
        self.HEIGHT= HEIGHT
        self.h= np.zeros((HEIGHT,WIDTH,3))
        self.color= [gray,gray,gray]
        self.blue= [gray,0,0]
        self.font= cv2.FONT_HERSHEY_SIMPLEX
        self.Elsetup= lsetup

    def normPtm(self,mv):
        norms= []; mv= np.asarray(mv)
        for pt in self.Elsetup.ptm:
            norm= np.linalg.norm(np.asarray(pt) - mv)
            norms.append(norm)
        return norms
 
    def calc(self,im):
        xysize= (self.WIDTH,self.HEIGHT) 
        pts= self.Elsetup.pts
        self.Elsetup.pts= [(0,0),(xysize[0],0),(0,xysize[1]),xysize]
        Mwarp= self.Elsetup.Mwarp()
        Mshape= self.Elsetup.Mshape()
        self.Elsetup.pts= pts 
       
        self.h= cv2.warpPerspective(im,Mwarp,Mshape) 
        self.h=cv2.convertScaleAbs(self.h)
        info= ['new ptm: '] + [str(x) for x in self.Elsetup.ptm]
        xycursor= [(40,i*20) for i in range(3,8)]
        cv2.rectangle(self.h,(0,0),xysize,(40,40,0),3)
        for i ,txt in enumerate(info):
            cv2.putText(self.h, txt, xycursor[i], self.font, 0.5, self.blue, 1)
        return self.h.shape

    def gline(self,frame):
        assWidth= self.Elsetup.assWidth
        (h,w)= frame.shape[0:2]
        gline_top= self.Elsetup.ptm[0:2]
        gline_bot= self.Elsetup.ptm[2:4]
        self.Elsetup.ptm[0:2]= gline_top
        self.Elsetup.ptm[2:4]= gline_bot
        cv2.line(frame,gline_top[0],gline_top[1],self.color,2)
        cv2.line(frame,gline_bot[0],gline_bot[1],self.color,2)

        #solve perspective datum lines by inner lambda
        # e.g. f=2*x^2; f= lambda x: (lambda x2: 2*x2)(x*x)
        linearPath= lambda s,lines: tuple(np.int0((1.0-s)*lines[0]+s*lines[1]))
        dm= {'xp0':25,'xp1':75, 'xlim':105}
        xp0= 1.0*dm['xp0']; xp9= assWidth - xp0
        dtop1= linearPath(xp0/assWidth,np.asarray(gline_top))
        dtop2= linearPath(xp9/assWidth,np.asarray(gline_top))
        dbot1= linearPath(xp0/assWidth,np.asarray(gline_bot))
        dbot2= linearPath(xp9/assWidth,np.asarray(gline_bot))
        cv2.line(frame,dtop1,dbot1,(0,155,255),1)
        cv2.line(frame,dtop2,dbot2,(0,155,255),1)

        for gpt in gline_top+gline_bot:
            cv2.circle(frame, gpt, 4, self.blue, 2)
        return frame

    def hshow(self):
        cv2.imshow('warping guide',self.h)
        cv2.waitKey(0)

#define state of aid
class hintEnum:
    """State of apps flying with different widgets"""
    test,hist,warp= xrange(3)

    def __init__(self):
        self.Emode= self.test
        self.Elsetup= None
        self.Ecam_NAME = 'Cam[ESC to quit]' 
        self.Eframe= None
        self.h= None
        self.Efont= cv2.FONT_HERSHEY_SIMPLEX
        self.Ehist= cvhist()
        self.Epts= None 
        self.Ewarp= None
        self.Emove= None
        self.buoy= cvbuoy()          
        self.track= None
        self.altstream= False
        self.resumeID= None        

    def update(self):
        self.inRange= None
        self.hQuanta= None
        self.hRange= None

    #interactive warp setup and color histogram
    def ishistmode(self):
        return (self.Emode is self.hist and self.Epts is not None)
        
    def iswarpmode(self):
        ismove= self.Emove is not None and type(self.Emove) is not int
        return (self.Emode is self.warp and ismove)

    def hintBox(self,event, x, y, flags, param):
        if self.Emode==self.warp:
            self.warpBox(event, x, y, flags, param)
        elif event == cv2.EVENT_LBUTTONDOWN:
            if self.Emode==self.hist:
                (ylim,xlim,col)= self.Eframe.shape
                ylim-= 20; xlim-=20
                if x>20 and y>20 and x<xlim and  y<ylim:
                    self.Epts= (x,y)
                    self.h= self.Eframe[y-18:y+18,x-18:x+18]
                    self.Ehist.calc(self.h)
            else:
                self.Epts= None
        return (x,y)

    def warpBox(self,event, x, y, flags, param):
        if  event == cv2.EVENT_LBUTTONDOWN:
            if self.Emove is None:
                self.Emove= (x,y)
            else:
                self.Emove= None
        elif self.Emove is not None and event == cv2.EVENT_MOUSEMOVE:
            self.Emove= (x,y)
        #tune the nearby point in ptm by mouse click
        if self.Emove is not None:
            self.movePtm()
        return (x,y)

    def movePtm(self):
        norms= self.Ewarp.normPtm(self.Emove)       
        if len(norms)==4 and min(norms)<50:
            k= np.argmin(norms)
            k2y= [1,0,3,2][k]
            ydelta= self.Elsetup.ptm[k][1]-self.Emove[1]
            ydelta=  min(ydelta,0) if k2y in [0,1] else max(ydelta,0)
            ptm2y_len= self.Eframe.shape[0]
            ptm2xy= self.Elsetup.ptm[k2y]
            ptm2y= ptm2xy[1] + ydelta
            ptm2y= sorted((0, ptm2y, ptm2y_len-1))[1]
            self.Elsetup.ptm[k2y]= (ptm2xy[0],ptm2y)
            self.Elsetup.ptm[k]= self.Emove
        return self.Emove

    def status_bar(self,w=80,h=16,col=(0,0,240)):
        sbar= np.zeros((h,w,3))
        sbar[:,:]= (0,240,240)
        mode= ['test','hist','warp'][self.Emode]
        sbar= cv2.convertScaleAbs(sbar)
        cv2.putText(sbar, mode, (20,10), self.Efont, 0.6, col, 2)
        return sbar

    def grabHist(self,frame):
        shape= frame.shape
        hs= self.Ehist.h.shape
        tl= np.asarray(self.Epts) + [20,20]
        adjx= hs[1]*1.4*(tl[0]>shape[1]*0.7)
        adjy= hs[0]*1.6*(tl[1]>shape[0]*0.75)
        tl= tl - [adjx,adjy]
        br= tl + [hs[1],hs[0]]
        frame[int(tl[1]):int(br[1]),int(tl[0]):int(br[0])]= self.Ehist.h
        if self.Emode is not self.test:
            frame[0:16,shape[1]/2-40:shape[1]/2+40]= self.status_bar()
        return frame

    def grabPersp(self,frame):
        size= np.asarray(frame.shape[0:2])[::-1]
        hs= np.asarray(self.Ewarp.h.shape[0:2])[::-1]
        tl= (size-hs)/2; br= tl + hs
        frame[tl[1]:br[1],tl[0]:br[0]]= self.Ewarp.h
        if self.Emode is not self.test:
            frame[0:16,size[0]/2-40:size[0]/2+40]= self.status_bar()       
        return frame  

    #color quantizer
    def rectAi(self,warp,rect_cut):
        majors= self.color_Screw(warp,rect_cut)
        buoys= self.buoy.stat(majors,rect_cut)
 
        pairs= self.buoy.pair_Screw(majors,buoys,rect_cut)
        alerts= self.buoy.alert_Match(majors,buoys,pairs)
        majority= {'color':majors,'alert':alerts,'pair':pairs,'buoy':buoys}
        return majority 
        
    def color_Screw(self,warp,rect_cut,wh=(38,38)):
        self.track= []; majors= []
        rects= rect_cut[0]+rect_cut[1]        
        if len(rects)!=0:
            for rt in rects:
                rt= (rt[0],wh,0)
                roiPts= cv2.boxPoints(rt)
                roiBox= windowBox(roiPts)
                part= warp[int(roiBox[1]):int(roiBox[3]), int(roiBox[0]):int(roiBox[2])].copy()
                #cv2.resize(part,wh)
                self.track.append(part)

            for part in self.track:
                #pixel statistics
                major=self.stat(part)
                majors.append(major)

        shape= (len(rect_cut[0]), len(rect_cut[1]))  
        majors= [majors[0:shape[0]],majors[shape[0]:sum(shape)]]
        return majors        

    def colorTrack(self,im):
        shape= im.shape
        scale= self.buoy.assPos['scaledown']
        im= cv2.resize(im, (shape[1], shape[0]/scale ))
        quadx= shape[1]/4
        im= np.hstack([im[:,0:quadx],im[:,quadx*3:shape[1]]])
        im= cv2.GaussianBlur(im, (3,3), 0)   
        im= cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        return im 

    def warpColor_transform(self,frame_H2,Mpars):
        if Mpars is not None:
            (Mwarp,Mshape)= Mpars
            frame_H2= cv2.warpPerspective(frame_H2,Mwarp,Mshape)
        track= self.colorTrack(frame_H2)
        return (track,frame_H2)

    def colorContour(self,hsv,inRange):
        gray= []
        for k in xrange(len(inRange)):
            bin_k= cv2.inRange(hsv, inRange[k][0], inRange[k][1])
            gray.append(bin_k)
        gray= gray[0] + gray[1] + gray[2]
        shape= gray.shape; cenx= shape[1]/2
        
        gray= cv2.erode(gray,  np.ones((2, 2), "uint8"))
        gray= cv2.dilate(gray, np.ones((9, 9), "uint8"))    
        
        gray = cv2.morphologyEx(gray, cv2.MORPH_OPEN, np.ones((5,5),"uint8"))
        gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, np.ones((2,2),"uint8"))
        
        gray_cut= [ gray[:,0:cenx], gray[:,cenx:shape[1]] ]
        # shift of leftmost and rightmost quadrant
        shiftx= [0,cenx*3] 
        return (gray_cut, shiftx)

    def screw_mask(self,warp,flash=800):
        track= self.colorTrack(warp)
        cut, shiftx= self.colorContour(track,self.inRange)
        (h,w)= cut[0].shape
        mask= np.hstack([cut[0],np.zeros((h,w*2),dtype='uint8'),cut[1]])
        mask= cv2.resize(mask,(w*4,h*2))
        warp= cv2.bitwise_and(warp,warp,mask = mask)

        cv2.imshow(self.Ecam_NAME,warp)
        cv2.waitKey(flash)

    def stat(self, im, v_lower=90):
        im= cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        if im is not None:
            info= {'none':0,'red':0,'blue':0,'gold':0}
            cval= {'none':0,'red':1,'blue':2,'gold':3} 

            (h,s,v)= cv2.split(im)
            label= self.hQuanta[h.flatten()]
            label[v.flatten()<v_lower] = 0
            for key in info.keys():
                info[key]= (label==cval[key]).sum()          
                
            info['gold']= info['gold']/6
            info.pop("none", None)
            major= max(info, key=info.get)           
            return major

    #Misc add-value features
    def alt_buoy(self,warp,th=0.8):
        roi= cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY)
        buoy= roiMatch('arralt2.jpg', th,roi)
        return buoy
    
    def alt_mark(self,(warp,buoy),w=2):
        for rt in buoy:
            pt1= rt[0]; pt2= rt[1]
            cv2.rectangle(warp,pt1,pt2,224,w)         
        return warp


''' 
@status: design spec on color buoyes
@date: 2018-OCT
@by: trio/clarence
'''
class cvbuoy():
    def __init__(self):
        ### as changing assWidth from 720 to 960
        ### set majEvil/minEvil Low & High to (45, 80)
        self.assWidth= 960
        self.assPos= {'scaledown':2, 'xp1':80, 'xlim':140, 'xsep':700, 'ytol':60}        

    def rectAdd(self, ra, rb, shift_x=8):
        rectx = min(ra[0][0],rb[0][0]) + shift_x
        recty = min(ra[0][1],rb[0][1])
        rectw = ra[1][0]+rb[1][0]
        recth = ra[1][1]+rb[1][1]
        return ((rectx, recty),(rectw, recth), 0.0)

    def colorRects(self,gray_cut,shiftx):
        rect0= self.colorRect(gray_cut[0],shiftx[0])
        rect1= self.colorRect(gray_cut[1],shiftx[1])
        return [rect0,rect1]   

    def colorRect(self,gray,shiftx,max_area=999):
        #update max_area from 700 to 999
        rects, search_rect = ([], [])
        scale= self.assPos['scaledown']
        cntmode=cv2.RETR_LIST; cntmethod= cv2.CHAIN_APPROX_SIMPLE
        im2, contours, hierarchy= cv2.findContours(gray, cntmode, cntmethod)

        cnts_area = map(cv2.contourArea,contours)
        cnts_rect = map(cv2.minAreaRect,contours)

        for ix, rect in enumerate(cnts_rect):
            area = cnts_area[ix]
            if area > 138 and area < max_area:
                for k, k_rect in enumerate(cnts_rect):
                    if cnts_area[k]>0 and k!=ix:
                        delta = np.array(rect[0]) - np.array(k_rect[0])
                        k_dist = np.linalg.norm(delta)
                        if k_dist<32:
                            cnts_area[ix], cnts_area[k] = (0, 0)
                            rect = self.rectAdd(rect, k_rect)
                            area = rect[1][0]*rect[1][1]
                if area > max_area/6 and area < max_area:
                    search_rect.append(rect)

        for rect in search_rect:
              (dx,dy)= rect[1]; dy= dy * scale
              if (1.0*dx/dy + 1.0*dy/dx) < 4:
                  rect = ((rect[0][0]+shiftx, rect[0][1]*scale), (dx,dy), rect[2])
                  rects.append(rect)

        return rects
    
    #visual signal
    def rectScrew(self,im,rect_cut,majority,drawbox=False,thickness=2):
        if drawbox:
            gray= (112,112,112)
            rects= rect_cut[0]+rect_cut[1]
            majors= majority['color'][0]+majority['color'][1]          
            colors= {'red':(160,0,160),'blue':(196,0,0),'gold':(0,127,255),'none':gray}
            for (i,rect) in enumerate(rects):
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                lcol= majors[i]
                if lcol is None:
                    lcol= 'none'
                cv2.drawContours(im,[box], 0, colors[lcol], thickness) 

        #draw match pairs and bad buoy and
        #alert with red thick red-line
        self.front_line(im,rect_cut,majority)

    def alert_Match(self,majors,buoys,pairs):
        shape= pairs.shape[0:2]
        warns= np.empty(shape,dtype='bool'); warns.fill(False)

        #find alert of buoy with pair and not match
        for u in range(shape[0]):
            for v in range(shape[1]):
                isalert= pairs[u,v][1] and not pairs[u,v][2]
                warns[u,v]= isalert
        
        alarts= [warns.sum(axis=1)!=0, warns.sum(axis=0)!=0]
        return alarts

    def front_line(self,im,rect_cut,majority):
        pairs= majority['pair']; alerts= majority['alert']
        if len(pairs)!=0 and len(pairs[0])!=0:
            shape= pairs.shape
            for u in range(shape[0]):
                for v in range(shape[1]):
                    if pairs[u,v][1]:
                        isalert= alerts[0][u] or alerts[1][v]
                        pcol= (0,0,255) if isalert   else (0,255,0)
                        pthick= 2 if isalert else 1
                        pt1= np.asarray(rect_cut[0][u][0],dtype='int')
                        pt2= np.asarray(rect_cut[1][v][0],dtype='int')
                        cv2.line(im,tuple(pt1),tuple(pt2),pcol,pthick)

            #circle the bad buoy
            for (u,isevil) in enumerate(majority['buoy'][2]):
                if isevil and True in pairs[u][:,1]:                
                    pt1= np.asarray(rect_cut[0][u][0],dtype='int')
                    if majority['color'][0][u]=='blue':
                        pt1+= np.asarray((16,16))
                    elif majority['color'][0][u]=='red':
                        pt1+= np.asarray((16,-16))                   
                    cv2.circle(im, tuple(pt1), 38, (112,112,112), 2)   
                    
            for (v,isevil) in enumerate(majority['buoy'][3]):
                if isevil and True in pairs[:,v][:,1]:                
                    pt1= np.asarray(rect_cut[1][v][0],dtype='int')
                    pt1+= np.asarray((-22,0))
                    cv2.ellipse(im,tuple(pt1),(28,48),0,0,360,(112,112,112), 2)                       
      
    #magic buoys
    def stat(self,majors,rect_cut):
        buoy0= []; buoy1= []; evil0= []; evil1= []
        flag= ['na','p1','p2']; width= self.assWidth
        rect0= rect_cut[0]; rect1= rect_cut[1]
 
        #define p1 and p2 poistions       
        for rs in rect0:       
            ispos2= rs[0][0] < self.assPos['xp1']
            ispos1= not ispos2 and rs[0][0] < self.assPos['xlim']
            buoy0.append(flag[ispos2*2+ispos1])

        for rs in rect1:  
            ispos2= rs[0][0]>width - self.assPos['xp1']
            ispos1= not ispos2 and rs[0][0] > width - self.assPos['xlim']
            buoy1.append(flag[ispos2*2+ispos1]) 

        #alert defect buoy of p2 objects in both rims
        cols= majors[0]
        for (u,majcol) in enumerate(cols):
            rl= rect0[u]
            isevil= self.majEvil(majcol,cols,rl,rect0,buoy0) if buoy0[u]=='p2' else False
            evil0.append(isevil)
 
        cols= majors[1]
        for (v,mincol) in enumerate(cols):
            rc= rect1[v]
            isevil= self.minEvil(mincol,cols,rc,rect1,buoy1) if buoy1[v]=='p2' else False
            evil1.append(isevil)
                       
        return (buoy0 , buoy1, evil0, evil1)

    def majEvil(self,majcol,majcols,rl,rect0,buoy0,low=45,high=80):
        isevil= False
        for (k,rc) in enumerate(rect0):
            flag= False                      
            if buoy0[k]=='p1':
                disp= np.asarray(rc[0]) - np.asarray(rl[0])
                norm= np.linalg.norm(disp)
                

                if disp[0] > 0 and disp[1]<0:
                    flag= (majcol=='red' and majcols[k]=='red')
                
                if disp[0] > 0 and disp[1] > 0:
                    flag= (majcol=='blue' and majcols[k]=='blue')
                flag= flag and (norm>low and norm<high)
                isevil= isevil or flag

        return isevil
        
    def minEvil(self,mincol,mincols,rr,rect1,buoy1,low=45,high=80):
        isevil= False
        for (k,rc) in enumerate(rect1):
            flag= False                      
            if buoy1[k]=='p1':
                disp= np.asarray(rc[0]) - np.asarray(rr[0])
                norm= np.linalg.norm(disp)
                if disp[0] < 0:
                    flag=  (mincol=='blue' and mincols[k]=='blue')
                    flag= flag or (mincol=='red' and mincols[k]=='red')
                flag= flag and (norm>low and norm<high)
                isevil= isevil or flag

        return isevil        

    def pair_Screw(self,majors,buoys,rect_cut):
        shape= (len(rect_cut[0]), len(rect_cut[1]),3)
        pairs= np.empty(shape,dtype='bool'); pairs.fill(False)

        #check buoy in level and pair condition
        #inspect color scheme for the matching
        for (u,rl) in enumerate(rect_cut[0]):
            for (v,rr) in enumerate(rect_cut[1]):
                islevel= (abs(rl[0][1]-rr[0][1]) < self.assPos['ytol'])
                islevel= islevel and rl[0][0] < self.assPos['xlim']
                #neglect item in top edge
                islevel= islevel and rl[0][1] > 24
                islevel= islevel and rr[0][1] > 24
                ispair= islevel and (abs(rr[0][0]-rl[0][0]) > self.assPos['xsep'])
                ispair= ispair and (buoys[0][u]== buoys[1][v])
                isevil= buoys[2][u] or buoys[3][v]
                #isgold= (buoys[1][v]== "p1" and majors[1][v] in ["gold","none"])
                ismatch= (majors[0][u]==majors[1][v]) 
                ismatch= ismatch or "none" in (majors[0][u], majors[1][v])
                ismatch= ispair and not isevil and ismatch
                pairs[u,v]= [islevel,ispair,ismatch]

        return pairs     

#End
