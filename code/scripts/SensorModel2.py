import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm
import pdb

from MapReader import MapReader
import numpy as np
from scipy.stats import norm, expon

class SensorModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 6.3]
    """

    def __init__(self, occupancy_map):

        self.zHit = 0.6;
        self.zShort = 0.3;
        self.zMax = 0.05;
        self.zRand = 0.05;
        self.sigmaHit = 1;
        self.lamdaShort = float(1);
        self.maxRange = float(10);
        self.distrShort = expon(scale=1/self.lambdaShort); 

    def probHit(self, zRayCast, zK):
        distrHit = norm(zRayCast, self.sigmaHit*self.sigmaHit);
        eta = distrHit.cdf(self.maxRange) - distrHit.cdf(0);
        if (zK>0 && zK<self.maxRange):
            pHit = distrHit.pdf(zK)*eta;
        else:
            pHit = 0;
        return pHit;

    def probShort(self, zRayCast, zK):
        eta = 1/(1-math.exp(-self.lambdaShort*zRayCast))
        if (zK>0 && zK<zRayCast):
            pShort=eta*self.distrShort.pdf(zK);
        else:
            pShort=0;
        return pShort;

    def probMax(self, zK):
        if(zK==self.maxRange):
            pMax=1;
        else:
            pMax=0;

        return pMax;

    def probRandom(self, zK):
        if(zK>0 && zK<self.maxRange):
            pRand = 1/self.maxRange;
        else:
            pRand=0;

        return pRand; 
            

    def beam_range_finder_model(self, z_t1_arr, x_t1):
        """
        param[in] z_t1_arr : laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t
        """
        q = 0;
        for i in xrange(len(z_t1_arr)):
            zK = z_t1_arr[i]; 
            zRayCast = #returned from ray casting
            pHit = self.probHit(zRayCast, zK);
            pShort =self.probShort(zRayCast, zK); 
            pMax = self.probMax(zK);
            pRand = self.probRand(zK);

            p = self.zHit*pHit + self.zShort*pShort + self.zMax*pMax + self.zRand*pRand;

            q = q + math.log(p); 
        

        return q    
 
if __name__=='__main__':
    pass
