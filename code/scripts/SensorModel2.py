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

        self.zHit = 0.7;
        self.zShort = 0.15;
        self.zMax = 0.05;
        self.zRand = 0.1;
        self.sigmaHit = 10;
        self.lambdaShort = float(5);
        self.maxRange = float(8000);
        self.distrShort = expon(scale=1/self.lambdaShort);

        self.occupancyMap = occupancy_map

    def findMeasurement(self,globalAngleForBeam, particleLocation):
        # Given the direction that a distance is desired for and the location of the particle
        # returns the distance to the nearest non-traversable pixel in centimeters
        # globalAngleForBeam is given in radians
        # particleLocation is in centimeters

        # Do I need to worry about overrunning the edges of the map?
        maxDistance = 1000 # centimeters

        particleLocationX = int(round(particleLocation[0]/10)) # convert to decimeters
        particleLocationY = int(round(particleLocation[1]/10))


        
        # This is an integer version of Bresenham's line drawing algorithm
        xSteps = int(maxDistance * math.cos(globalAngleForBeam))
        ySteps = int(maxDistance * math.sin(globalAngleForBeam))

        absXsteps = abs(xSteps)
        absYsteps = abs(ySteps)

        accumulator = 0
        distance = 100 # 10 meters expressed in decimeters



        if abs(xSteps) > abs(ySteps): # There are more steps in X than Y
            Y = particleLocationY
            yStep = 1
            if ySteps < 0:
                yStep = -1
            xStep = 1
            if xSteps < 0:
                xStep = -1
            #print(range(particleLocationX,particleLocationX+xSteps,xStep))
            for X in range(particleLocationX,particleLocationX+xSteps,xStep):   # This fails if the second one is less than the first
                # Check to see if the pixel is out of range
                if X < 0 | X > 799 | Y < 0 | Y > 799:
                    break
                if self.occupancyMap[Y,X] == 0:
                    # Here's our range!
                    # Calculate the distance
                    delta = np.array([particleLocationX,particleLocationY]) - np.array([X,Y])
                    distance = math.sqrt(delta[0]*delta[0]+delta[1]*delta[1])
                    break
                # adjust Y if necessary
                accumulator += absYsteps
                if accumulator >= absXsteps:
                    accumulator -= absXsteps
                    Y += yStep


        else:   # There are more steps in Y than X
            X = particleLocationX
            xStep = 1
            if xSteps < 0:
                xStep = -1
            yStep = 1
            if ySteps < 0:
                yStep = -1
            #print(range(particleLocationY,particleLocationY+ySteps,yStep))
            for Y in range(particleLocationY,particleLocationY+ySteps,yStep):
                # Check the current location 
                # Check to see if the pixel is out of range
                if X < 0 | X > 799 | Y < 0 | Y > 799:
                    break
                if self.occupancyMap[Y,X] == 0:
                    # Here's our range!
                    # Calculate the distance
                    delta = np.array([particleLocationX,particleLocationY])  - np.array([X,Y])
                    distance = math.sqrt(delta[0]*delta[0]+delta[1]*delta[1])
                    break
                # adjust Y if necessary
                accumulator += absXsteps
                if accumulator >= absYsteps:
                    accumulator -= absYsteps
                    X += xStep

        distance *= 10
        if distance >= 1000:
            distance = 999  # To prevent issues with the lookup table

        #print("Angle: %f\t distance: %f\t%d\t%d" % (globalAngleForBeam,distance,xSteps,ySteps))
        return distance  


    def probHit(self, zRayCast, zK):
        distrHit = norm(loc=zRayCast, scale=self.sigmaHit);
        eta = distrHit.cdf(self.maxRange) - distrHit.cdf(0);
        if ((zK>0) & (zK<self.maxRange)):
            pHit = distrHit.pdf(zK)*eta;
        else:
            pHit = 0;
        return pHit;

    def probShort(self, zRayCast, zK):
        #if (int(zRayCast)==0):
         #   zRayCast=0.001; 
        eta = 1#1/(1-math.exp(-self.lambdaShort*zRayCast))
        if ( (zK>0) & (zK<zRayCast)):
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
        if((zK>0) & (zK<self.maxRange)):
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
        angleIncrement = 12; 
        for i in xrange(0,180,angleIncrement):
            zK = z_t1_arr[i]; 
            zRayCast = self.findMeasurement(zK,x_t1);
            #print 'zRayCast=', zRayCast;
            #print 'zK=',zK;
            pHit = self.probHit(zRayCast, zK);
            pShort =self.probShort(zRayCast, zK); 
            pMax = self.probMax(zK);
            pRand = self.probRandom(zK);

            p = self.zHit*pHit + self.zShort*pShort + self.zMax*pMax + self.zRand*pRand;
            
            #print 'p=',p;
            q = q+math.log(p); 
        

        return q    
 
if __name__=='__main__':
    pass
