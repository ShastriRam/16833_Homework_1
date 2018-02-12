import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm
from scipy.stats import expon
from scipy import signal
import pdb

from MapReader import MapReader

# Jack's Notes:
# The adjustments to this algorithm are at the top of __init__  and angleIncrement in beam_range_finder_model
# UNITS: 
# The edgeList is in cm units.


class SensorModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 6.3]
    """

    def __init__(self, occupancy_map):
    #def __init__(self, edgeList):

        # To make things fast, the exponential function should be precomputed as a lookup table
        # It should then be scaled to get it to be the correct size relative to the other distributions.
        # The gaussian should also be precomputed but with twice the width of the ranges.  
        # The gaussian should have things in the form   [pr(x),  sum of this and all previous probabilities]
        # The exponential function should be calculated the same way.
        # I'm not going to use the delta function because I am not seeing it in the data.
        # The uniform distribution should have [pr(x), pr(x) * 1000]
        # Make the exponential distribution

        ############################## KNOBS TO TURN #########################################
        # Adjust these to get an acceptable distribution.  The distribution is adjusted later
        # To get its sum to equal 1.
        self.gaussianScaleFactor = .45
        self.stdev = 50   # Adjusts the width of the gaussian - stdev is this many samples wide
        self.exponentialScaleFactor = .15   # The sum of the exponential curve that I am generating is ~21.6.  
                                        # The values as initially generated range from .99 to .01
        self.uniformValue = .75 # This is the value that is used in each bin for the uniform distribution
        ######################################################################################

        #self.edges = edgeList
        self.occupancyMap = occupancy_map

        self.numSamples = 1000 # This should be 1000 since our max range is 10m and the divisions are in 10cm units.

        x = np.linspace(expon.ppf(0.01), expon.ppf(0.99), self.numSamples) # Makes numSamples samples with a probability ranging from .99 to .1
        self.expPDF = expon.pdf(x)
        self.expPDF *= self.exponentialScaleFactor # Scale it down so that 

        # Make the gaussian distribution
        self.gaussPDF = signal.gaussian(self.numSamples * 2,std=self.stdev) # We want this to be 2000 samples wide
        self.gaussPDF *= self.gaussianScaleFactor

        # Resize the distributions to give them a second row.
        self.expPDF.resize(2,self.numSamples)
        self.gaussPDF.resize(2,self.numSamples*2)

        # Find the sums at each point in the two PDFs
        for I in range(self.numSamples):
            self.expPDF[1][I] = self.expPDF[0][0:I+1].sum()

        for I in range(self.numSamples*2):
            self.gaussPDF[1][I] = self.gaussPDF[0][0:I+1].sum()

        self.uniformSum = self.uniformValue * self.numSamples

        self.rangeLines = np.zeros((180,4))
        self.ranges = np.zeros(180)


    def findMeasurement3(self,globalAngleForBeam, particleLocation):
        # This is a different version of the find measurement function
        # It starts out by moving outward from 0 in 32cm increments until it hits an 
        # obstruction then it does something like a binary search for the rest
        
        # At each step, it will test to see if there is an obstruction.  If so then it
        # will keep going otherwise it will reverse direction and cut the step size in two
        # It will terminate when the step size is 1

        # 

        # For 10000 particles with angle increment of 5, this takes ___ to ___ seconds
        maxDistance = 1000 # centimeters
        currentDistance = 0
        stepSize = 32
        

        particleLocationX = int(round(particleLocation[0]/10)) # convert to decimeters
        particleLocationY = int(round(particleLocation[1]/10))


        stillWorking = True
        hitMaximumDistance = False

        sinOfGlobalAngle = math.sin(globalAngleForBeam)
        cosOfGlobalAngle = math.cos(globalAngleForBeam)

        # moveOutward until it hits an obstruction or goes over the maximum distance
        while stillWorking == True:
            currentDistance += stepSize
            if currentDistance > 1000:
                stillWorking = False

            # Calculate the X and Y coordinates for this point
            X = int(round((particleLocationX + (currentDistance * cosOfGlobalAngle)/10)))
            Y = int(round((particleLocationY + (currentDistance * sinOfGlobalAngle)/10)))
            # limit things to the map region
            if X > 799:
                X = 799
            if Y > 799:
                Y = 799
            elif Y < 0:
                Y = 0

            if self.occupancyMap[Y,X] == 0: # occupied
                stillWorking = False
                

        if hitMaximumDistance == True:
            return 1000

        # Now keep reversing direction until it finds the edge
        searchingForFreeSpace = True
        stepSize *= -.5
        while abs(stepSize) != 1:
            currentDistance += stepSize
            print ("StepSize: %f"%stepSize)

            # Calculate the X and Y coordinates for this point
            X = int(round((particleLocationX + (currentDistance * cosOfGlobalAngle)/10)))
            Y = int(round((particleLocationY + (currentDistance * sinOfGlobalAngle)/10)))
            # limit things to the map region
            if X > 799:
                X = 799
            if Y > 799:
                Y = 799
            elif Y < 0:
                Y = 0

            pixVal = self.occupancyMap[Y,X]

            if searchingForFreeSpace == True:
                if pixVal == 1:
                    stepSize *= -.5
                    searchingForFreeSpace = False
            else:
                if pixVal == 0:
                    stepSize *= -.5
                    searchingForFreeSpace = True

        #print("Finished")

        return currentDistance









    def findMeasurement2(self,globalAngleForBeam, particleLocation):
        # This is a different version of the find measurement function
        # The way that it works is to do a binary search where it repeatedly
        # * finds a point 50% along the range that it is corrently examining
        # * If that point is occupied, it makes that the new maxDistance
        # * if that point is free space, it makes that the new minDistance
        # It stops when the distance between min and max is equal to 1 and 
        # it then returns the maxDistance
        # For 10000 particles with angle increment of 5, this takes 6.5 to 7.2 seconds
        maxDistance = 1000 # centimeters
        minDistance = 0
        

        particleLocationX = int(round(particleLocation[0]/10)) # convert to decimeters
        particleLocationY = int(round(particleLocation[1]/10))


        stillWorking = True

        sinOfGlobalAngle = math.sin(globalAngleForBeam)
        cosOfGlobalAngle = math.cos(globalAngleForBeam)

        while stillWorking == True:
            distanceRange = maxDistance - minDistance
            print ("Distance range: %f\tMax: %d\tMin: %d" % (distanceRange,maxDistance,minDistance))
            if distanceRange == 1:
                stillWorking = False
            else:
                # split the working range into two while keeping it an integer
                midDistance = int(((maxDistance-minDistance)*.5)) + minDistance
                #Find the pixel coordinates for that distance
                midX = round((particleLocationX + (midDistance * cosOfGlobalAngle)/10))
                midY = round((particleLocationY + (midDistance * sinOfGlobalAngle)/10))

                # limit things to the map region
                if midX > 799:
                    midX = 799
                if midY > 799:
                    midY = 799
                elif midY < 0:
                    midY = 0

                
                if self.occupancyMap[midY,midX] == 0: # occupied
                    maxDistance = midDistance
                else:
                    minDistance = midDistance

        return maxDistance







    def findMeasurement(self,globalAngleForBeam, particleLocation):
        # Given the direction that a distance is desired for and the location of the particle
        # returns the distance to the nearest non-traversable pixel in centimeters
        # globalAngleForBeam is given in radians
        # particleLocation is in centimeters
        # For 10000 particles with angle increment of 5, this takes 8 to 9.2 seconds

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





    def calculateProbability(self,actualMeasurement, particleMeasurement):
        # Calculates the probability that the particle is in the right place given the two measurements
  
        probability = self.uniformValue;
        if actualMeasurement <= particleMeasurement:
            probability += self.expPDF[0][int(actualMeasurement)]

        # Figure out the shift of the gaussian.
        # As is, the center is at sample numSamples 
        firstGaussianSample = self.numSamples-1 - particleMeasurement
        lastGaussianSample = firstGaussianSample + self.numSamples-1

        probability += self.gaussPDF[0][int(firstGaussianSample) + int(actualMeasurement)]



        # # find the sum of the whole PDF to divide by
        # normalizer = self.uniformSum
        # normalizer += self.expPDF[1][int(particleMeasurement)]
        # normalizer += self.gaussPDF[1][int(lastGaussianSample)] - self.gaussPDF[1][int(firstGaussianSample) - 1]

        # # Calculate the actual probability
        # probability /= normalizer

        return probability






    def beam_range_finder_model(self, actualMeasurements, particleState):
        """
        Given a state for the particle and the actual LIDAR readings, output a probability 
        for the particle.  
        param[in] actualMeasurements : actual laser range readings [array of 180 values] at time t
        param[in] particleState : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t  (log probability)

        particleState should be in centimeters and radians
        """

        ############################## KNOBS TO TURN #########################################

        angleIncrement = 1; # The number of degrees to move when doing calculations.
                            # 1 results in calculating every angle.
                            # 2 results in calculating every other angle.

        ######################################################################################

        # Adjust the particle's position by 25cm in the particle's direction to account for the 
        # LIDAR unit's offset from the robot's coordinates
        particleX = particleState[0]
        particleY = particleState[1]
        particleAngle = particleState[2] 

        # print('Sensor Model: particleX: ')
        # print(particleX)
        # print('Sensor Model: particleY: ')
        # print(particleY)


        particleX += 25 * math.cos(particleAngle)
        particleY += 25 * math.sin(particleAngle)


        particleLocation = np.array([particleX, particleY])


        cumulativeProbability = 1  # use 0 if doing log probability

        relativeAngle = -math.pi/2
        angleAdder = (angleIncrement/180.0) * math.pi # added to the relative angle each time

        for I in range(0,180,angleIncrement): # calculate a range for all 180 measurements.  To reduce the number 
                                  # of distances calculated, make the last number something other than 1
                                  # 35 gives me beams at [0, 35, 70, 105, 140, 175]
                                  # 25 gives me beams at [0, 25, 50, 75, 100, 125, 150, 175]
                                  # 16 gives me beams at [0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176]

            # Calculate the angle for this reading
            #relativeAngle = (float(I)/180)* math.pi # This is verified to be 0 -> pi
            #absoluteAngle =  particleAngle + relativeAngle - math.pi/2
            absoluteAngle =  particleAngle + relativeAngle



            # calculate the particle's measurement for this angle 
            particleMeasurement = self.findMeasurement(absoluteAngle, particleLocation) # Returns the measurement in cm

            #print (particleMeasurement)
            # # ######################### FOR TESTING ONLY - Laser lines ############################                   ############ REMOVE AFTER TESTING
            # X = particleMeasurement * math.cos(absoluteAngle) + particleX  # This value is in centimeters
            # Y = particleMeasurement * math.sin(absoluteAngle) + particleY
            # self.rangeLines[I][:] = [particleX/10,particleY/10,X/10,Y/10] # all of the /10 are so it displays correctly on the map
            
            # self.ranges[I] = int(particleMeasurement)
            # # ######################### FOR TESTING ONLY ############################                   ############ REMOVE AFTER TESTING



            actualMeasurement = round(float(actualMeasurements[I])/10)  # These are coming in up to about 506 and leaving up to 50
            #actualMeasurement = actualMeasurements[I]
            particleMeasurement = round(particleMeasurement/10)   

            # print ("Actual: %f\tParticle: %f" % (actualMeasurement,particleMeasurement))


            probability = self.calculateProbability(actualMeasurement, particleMeasurement)


            # Now find the log value of the probability
            #cumulativeProbability += math.log(probability)
            cumulativeProbability *= probability
            relativeAngle += angleAdder

        return cumulativeProbability   
 
if __name__=='__main__':
    pass
