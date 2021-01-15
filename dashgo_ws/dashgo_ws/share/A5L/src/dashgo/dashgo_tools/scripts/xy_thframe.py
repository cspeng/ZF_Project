#!/usr/bin/env python
""" 
triogo_corner.py
created by clarence PEMS@2017
"""
import numpy as np
from scipy.misc import comb
from math import copysign, cos, sin, pi

def cut_beyond(x, ll, ul):
    return min(max(x,ll),ul)

def spline_points(lx=2.6, ly=2, nTimes=100):
    midy = ly*0.5
    xpoints = [0, lx*0.1 , lx*0.4 , lx*0.6 ,lx*0.9 ,lx]
    ypoints = [0 ,ly*0.6 ,midy ,midy ,ly*0.4 ,ly]
    return bezier_curve(zip(xpoints,ypoints), nTimes) 

def step_points(PB, PA, mdist, sep=0.4):
    BA = np.subtract(PA, PB)
    ss = np.sqrt(np.square(BA).sum())
    ms, mm, gr= (sep / np.cos(sep/ss), np.arctan2(*BA), mdist/ss)
    MBA = np.multiply(PA, gr) + np.multiply(PB, 1-gr)
    tA, tB = ((mdist-sep*3)/mdist, (ss-mdist-sep*3)/(ss-mdist))  
    MB = (MBA + np.multiply([-cos(mm), sin(mm)], ms*gr))
    MA = (MBA + np.multiply([cos(mm), -sin(mm)], ms*(1-gr)))
    GA = (PA + (MA-PA) * tA).round(3).tolist()
    GB = (PB + (MB-PB) * tB).round(3).tolist()
    return [PB, GB, GA, PA]
    
def bezier_curve(points, nTimes=100):
    ''' return a set of bezier points thro' control points '''
    # points should be a list or tuples of [xn,yn] with time steps 1000
    # polynomial of n, i as a bernstein function of t
    bernstein_poly = lambda i, n, t: comb(n, i) * ( t**(n-i) ) * (1 - t)**i
    nPoints = len(points)
    xPoints = np.array([p[0] for p in points])
    yPoints = np.array([p[1] for p in points])
    t = np.linspace(0.0, 1.0, nTimes)
    polynomial_array = np.array([ bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)   ])
    xvals = np.dot(xPoints, polynomial_array)
    yvals = np.dot(yPoints, polynomial_array)
    return (xvals, yvals)


class xy_thFrame(object):
    ''' photocol for pose using "print" and "with" '''
    def __init__(self, x=0, y=0, zth=0):
        self.x= x; self.y= y; self.zth= zth

    def drive_matrix(self):
        pass
        
    def tolist(self):
        return [self.x,self.y,self.zth]
        
    def tostring(self):
        return str(self)

    def __add__(self, other):
        p= self.pose(other)
        return xy_thFrame(self.x + p.x, self.y + p.y, self.zth + p.zth)

    def __sub__(self, other):
        p= self.pose(other)
        return xy_thFrame(self.x - p.x, self.y - p.y, self.zth - p.zth)
        
    def round(self, ndigits=0):
        self.x= round(self.x, ndigits)
        self.y= round(self.y, ndigits)        
        self.zth= round(self.zth, ndigits)          
        return self

    def __mul__(self, other):
        p= self.pose(other)
        prod = np.inner(self.tolist(), p.tolist()).round(4)
        prodtype = (np.int32, np.float64)
        return prod if type(prod) in prodtype  else xy_thFrame(*prod)
 
    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.__dict__ == other.__dict__
        return False

    def __ne__(self, other):
        return not self.__eq__(other) 

    def pose(self, p):
        if  type(p)==list and len(p)==3:
            p = xy_thFrame(*p)
        elif not isinstance(p, self.__class__):
            raise TypeError        
        return p
        
    def __repr__(self):
        return "<x:%s y:%s zth:%s with xy_thFrame>" % (self.x, self.y, self.zth)

    def __str__(self):
        return "xy_thFrame(%s, %s, %s)" % (self.x, self.y, self.zth)

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        return isinstance(value, TypeError)


class velbot(xy_thFrame):
    ''' velocity frame refer to chassis '''
    def motion(self, v, lag):
        self.x, self.rate, self.lag = (v, 25.0, lag)
        self.steer, self.gain = (-v, self.rate)
        self.curve, self.last_idex = (None, 0)
        
    def get_pose(self, k, cur_pose):
        tk = self.arc_idx(cur_pose.x, cur_pose.y)
        k0, th = ((tk-self.lag) * (tk>self.lag), cur_pose.zth)
        rk, r0 = (self.arc_pose(int(tk)), self.arc_pose(int(k0)))
        dr= np.multiply([sin(th), cos(th), 0], (r0-rk).tolist())
        return  (rk+dr.tolist()).round(3)

    def arc_pose(self, idx):
        return xy_thFrame(self.curve.index[idx], self.curve.iloc[idx])
        
    def arc_idx(self,x, y):
        R= self.curve
        ssq= np.square(R.values-y) + np.square(R.index.values-x)
        return np.argmin(ssq)
        
    def arcSegment(self, seq, cR=1.0):
        v = self.x
        X = cR*(1.0-np.cos(seq*v/cR)).round(3)
        Y = cR*(np.sin(seq*v/cR)).round(3)          
        return (X, Y)

    def splineSegment(self, seq):
        return spline_points(nTimes=len(seq))
    
    def horizStep(self, seq, sep=0.4):
        X, Y = ([], [])
        x0, y0, v = (0.0, 0.0, 0.5)
        xm = x0+seq[-1]*v*0.5
        # collecting locus
        for t in seq:
            x, y = (x0 + t*v, y0)
            if (x > xm - sep):    y= y0 + min((x-xm+sep)*0.5, sep)
            X.append(x), Y.append(y)
        return (np.array(X), np.array(Y))
        
    def vertStep(self, seq, sep=0.25):
        X, Y = ([], [])
        x0, y0, ss, v = (0.0, 0.0, seq[-1]*0.5, 0.5)        
        points = step_points([x0, y0], [x0+sep*1.4, ss], ss*0.5, sep)
        PB, GB, GA, PA = points
        # collecting locus
        for t in seq:
            Gr = np.subtract(GB, PB)
            y, x = (y0 + t*v, x0 + t*v*Gr[0]/Gr[1])
            if y > GB[1]:
                if y< GA[1]:
                   Gr = np.subtract(GA, GB)
                   x = GB[0] + (y-GB[1])*Gr[0]/Gr[1]
                else:
                   Gr = np.subtract(PA, GA)                    
                   x = GA[0] + (y-GA[1])*Gr[0]/Gr[1]
            X.append(x), Y.append(y)
        return (np.array(X).round(3), np.array(Y).round(3))

    def sequence(self, duration):
        seq = [round(x/self.rate,2) for x in np.arange(duration * self.rate)]
        return np.array(seq)
        
    def drive_matrix(self, th):
        # robot differential kinematics
        x = np.array([cos(th), -sin(th), 0]).round(4)
        y = np.array([sin(th),cos(th), 0]).round(4)
        zth = np.array([0, 0, 1])
        return xy_thFrame(x, y, zth)  


class posbot(xy_thFrame):
    ''' position frame refer to inertial '''
    def theta(self, th):
        self.zth = th
        self.loginfo = ["{'track':'[x, y, pid]', 'error':'[ekm1, ekp1]',\
        'theta':'[dz_cumsum, mavg]'}"]
        self.dz_cumsum, self.mavg = (0, 0)
        self.ekm1, self.ekp1 = (0, 0)
        
    def update(self, vel, r, offset):
        ''' update new position and steering angle '''
        rate, gain = (vel.rate, vel.gain)
        dv= vel.drive_matrix(self.zth) * vel
        #steering algorithm with ek = uk
        uk = cut_beyond(self.point_line(r), -0.15, 0.15)
        pid = self.s_pid(uk + self.hold_offset(offset, gain))
        # advance one step into new pose
        self.x += dv.x/rate;  self.y += dv.y/rate
        self.zth += self.motion_ai(pid, vel.steer/rate*gain)
        self.x, self.y, self.zth = self.round(3).tolist()
        #logging info
        data = "{'track':%s" % str(r.tolist() + [pid])
        data += ", 'error':%s" % str([self.ekm1,self.ekp1])
        data += ", 'theta':%s}" % str([self.dz_cumsum, self.mavg])
        self.logdata(data)
        return (uk, self.zth)

    def logdata(self, msg=None):
        if msg is not None:
            self.loginfo.append(msg)
        else:
            return self.loginfo

    def hold_offset(self, offset, gain):
        #log moving averge on previous diff of ekp1 and ekm1
        self.mavg = round(self.mavg*0.4 + (self.ekp1-self.ekm1)*0.6*gain, 3)
        h_offset = copysign(offset,self.mavg) 
        if abs(self.mavg)<0.32:    h_offset *= 0.4
        if abs(self.mavg)<0.16:    h_offset = 0   
        return h_offset

    def motion_ai(self, pid, K, limit=pi/6):
        #get error of zth and keep osc value within control limit
        self.dz_cumsum+= pid * abs(K)
        flipping = self.dz_cumsum*pid<0 and self.ekp1*self.ekm1>0
        if abs(self.dz_cumsum)>limit or flipping:
            self.dz_cumsum, pid= (copysign(limit*0.5, pid), pid*0.2)
        self.dz_cumsum= round(self.dz_cumsum, 3)
        return cut_beyond(round(pid * K, 3), -2, 2)

    def s_pid(self, ek, Kp=0.4, Kd=9):
        ''' default pid: Kp= 0.15, Kd= 0.05 '''
        s_i = self.s_integral(ek)
        pid, self.ekm1 = (Kp * ek + Kd * (ek - self.ekm1) + s_i, ek)
        return round(pid, 3)

    def s_integral(self,ek, Ki=0.2):
        self.ekp1 = round(self.ekp1 * Ki + ek, 3)
        if self.ekp1*ek < 0:    self.ekp1 = ek
        self.ekp1 = cut_beyond(self.ekp1, -0.5/Ki, 0.5*Ki)
        return self.ekp1 * Ki
        
    def point_line(self, r):
        # usage  pdist(r-p,q-p) where p is pos_bot
        s= (r - self).tolist()[:2]
        m= [cos(self.zth), sin(self.zth)]
        pvec= np.cross(np.array(s), np.array(m))
        return round(pvec*200)/200
