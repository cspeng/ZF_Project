#!/usr/bin/env python
""" 
triogo_corner.py
created by clarence PEMS@2017
"""
from xy_thframe import posbot, velbot
import numpy as np, pandas as pd
from pylab import show, plot
from math import pi


class locus():
    pos_bot, vel_bot = (posbot(), velbot())
    
    def __init__(self, duration, shape='vert'):
        # lag_y approx equal to theta for R of corner = 1m
        # wl, wr, ab2 = (0.35/2, 0.125/2, 0.56/2), offset=0.01
        # (A1, B1) = (0, 0)
        lagk, offset = (2, 0.04)
        self.vel_bot.motion(0.7, lagk)
        vert = {'horiz':False, 'vert':True, 'spline':True, 'arc':True}[shape]
        self.pos_bot.theta(pi/2 if vert else 0)    # pi/2 for arc segment
        # initial chaos before start pid
        self.pos_bot.update(self.vel_bot, self.pos_bot, 0)

        # modelling
        X, Y = ([], []); U, TH = ([], [])
        seq = self.vel_bot.sequence(duration)
        R= self.curve(seq, shape); limitx= R.index[-1]
        self.vel_bot.curve = R
        for k, t in enumerate(seq):
            if (self.pos_bot.x < limitx):
                cur_pose = self.pos_bot
                mea_pose= self.vel_bot.get_pose(k, cur_pose)
                su, th= self.pos_bot.update(self.vel_bot, mea_pose, offset)
                U.append(su), TH.append(th)
                X.append(self.pos_bot.x), Y.append(self.pos_bot.y)

        self.info = self.pos_bot.loginfo
        series= {'item':range(len(U)), 'U':U, 'TH':TH, 'Y':Y}
        ds= pd.DataFrame(series, index=X)
        R = zip(*[eval(r)['track'] for r in self.info[1:]])
        R = self.series_fill(R[0], R[1], ds)
        ds['R']= R.loc[ds.index]
        ds['U']= np.round(ds.U * 200) / 2
        self.ds = ds
        self.info = self.pos_bot.logdata()

    def series_fill(self, rX, rY, other=None):
        R= pd.Series(rY, index=rX, name="R")
        R = R[~R.index.duplicated()].sort_index()
        # accept positive region above x-axis
        R = R[R>=0]
        if other is None:
            join = R.index
        else:
            join = R.index.union(other.index)
            join = join[~join.duplicated()]
        # verify series
        R = R.loc[join].interpolate(method='cubic')
        return R.sort_index().round(4)

    def curve(self, seq, shape='spline'):
        if shape=='horiz':
            return self.series_fill(*self.vel_bot.horizStep(seq))
        elif shape=='vert':
            return self.series_fill(*self.vel_bot.vertStep(seq))            
        elif shape=='spline':
            return self.series_fill(*self.vel_bot.splineSegment(seq))
        else:
            #default arc
            return self.series_fill(*self.vel_bot.arcSegment(seq))

    def plot(self, duration=8):
        error = [eval(x)['error'] for x in self.info[1:]]
        theta = [eval(x)['theta'] for x in self.info[1:]]
        plot(error); plot(theta)


if __name__ == '__main__':
    """ TRIOGO corner """
    corner= locus(8,'vert'); ds= corner.ds
    ds[['Y', 'R']].plot()
    show()
    ds[['U', 'TH']].plot()
    show()   
    corner.plot()
    show()
