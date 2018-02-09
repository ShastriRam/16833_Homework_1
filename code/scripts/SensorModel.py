import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm
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

    #def __init__(self, occupancy_map):
    def __init__(self, edgeList):

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
        self.stdev = 1   # Adjusts the width of the gaussian
        self.exponentialScaleFactor = 1/50   # The sum of the exponential curve that I am generating is ~21.6.  
                                        # The values as initially generated range from .99 to .01
        self.uniformValue = .05 # This is the value that is used in each bin for the uniform distribution
        ######################################################################################

        self.edges = edgeList


        self.numSamples = 1000 # This should be 1000 since our max range is 10m and the divisions are in 10cm units.

        x = np.linspace(expon.ppf(0.01), expon.ppf(0.99), numSamples) # Makes numSamples samples with a probability ranging from .99 to .1
        self.expPDF = expon.pdf(x)
        self.expPDF *= exponentialScaleFactor # Scale it down so that 

        # Make the gaussian distribution
        self.gaussPDF = gauss(self.numSamples * 2,stdev,True) 


        # Resize the distributions to give them a second row.
        self.expPDF.resize(2,self.numSamples)
        self.gaussPDF.resize(2,self.numSamples*2)


        # Find the sums at each point in the two PDFs
        for I in range(numSamples):
            self.expPDF[1][I] = self.expPDF[0][0:I+1].sum()

        for I in range(numSamples*2):
            self.gaussPDF[1][I] = self.gaussPDF[0][0:I+1].sum()

        self.uniformSum = self.uniformValue * self.numSamples;



    def findMeasurement(globalAngleForBeam, remainingEdgesVectorForm)
        # Given the direction that a distance is desired for and the remaining edges in [angle distance]
        # form, returns the distance to the closest edge

        #     - Extract all pixels within +/- 5 degrees of the beam direction.
        #     - Rotate the pixels so that they are centered around 0 degrees
        #     - For each remaining edge, find the Y coordinate
        #     - Use the closest edge that is within 5cm of 0.

        # Find edges within +- 5 degrees of the beam direction
        lowAngle = particleAngle - .0872
        highAngle = particleAngle + .0872

        if lowAngle < -math.pi:
            # Then it wrapped into the high values
            # selection should be -pi -> high and low + 2*pi->pi
            selection = (((remainingEdgesVectorForm[:,0] > -math.pi) & (remainingEdgesVectorForm[:,0] < highAngle)) |
                         ((remainingEdgesVectorForm[:,0] > lowAngle + 2*math.pi) & (remainingEdgesVectorForm[:,0] < math.pi))) 


        elif highAngle > math.pi:
            # It wrapped into low values
            # selection should be low-> pi and -pi-> high - 2*pi
            selection = (((remainingEdgesVectorForm[:,0] > lowAngle) & (remainingEdgesVectorForm[:,0] < math.pi)) |
                         ((remainingEdgesVectorForm[:,0] > -math.pi) & (remainingEdgesVectorForm[:,0] < highAngle - 2*math.pi))) 

        else:
            # It didn't wrap
            selection = (remainingEdgesVectorForm[:,0] > lowAngle) & (remainingEdgesVectorForm[:,0] < highAngle)


        edgesLeft = remainingEdgesVectorForm[selection == True][:]

        # There shouldn't be too many edges left now.  

        # Rotate the edges so that they are centered around 0 degrees
        edgesLeft[:,0] = edgesLeft[:,0] - globalAngleForBeam

        # Find the Y coordinate for all of the 'edgesLeft' - store it where angle is currently 
        edgesLeft[:,0] = edgesLeft[:,1] * math.sin(edgesLeft[:,0])


        # Find the closest one where abs(Y) is less than 5cm if there is one
        distance = 1000   # This is the max distance
        for row in edgesLeft:
            if abs(row[0]) < 5:
                if row[1] < distance:
                    distance = row[1]
        


        return distance


    def beam_range_finder_model(self, z_t1_arr, x_t1):
        """
        Given a state for the particle and the actual LIDAR readings, output a probability 
        for the particle.  
        param[in] z_t1_arr : actual laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t
        """
        actualMeasurement = 206 # cm
        particleMeasurement = 193 

        ############################## KNOBS TO TURN #########################################

        angleIncrement = 1; # The number of degrees to move when doing calculations.
                            # 1 results in calculating every angle.
                            # 2 results in calculating every other angle.

        ######################################################################################

        # Adjust the particle's position by 25cm in the particle's direction to account for the 
        # LIDAR unit's offset from the robot's coordinates
        particleX = x_t1[0]
        particleY = x_t1[1]
        particleAngle = x_t1[2] 

        particleX += 25 * cos(particleAngle)
        particleY += 25 * sin(particleAngle)


        # Eliminate edges that are outside of a bounding box that surrounds the particle
        adjustedEdges = self.edges - np.array([particleX,particleY])
        selection = abs(adjustedEdges) < 1000 # makes a True/False array
        selection = selection.astype(int) # converts True to 1 and False to 0
        selection = np.transpose(selection)
        selection = sum(selection) # Now this contains a row vector with values from 0 to 2
        # I want to keep only rows of edges where the corresponding column in selection is equal to 2
        selection = selection == 2

        remainingEdges = adjustedEdges[selection == True][:]


        # Convert what's left to polar form.
        remainingEdgesVectorForm = zeros(remainingEdges.shape[0],2)
        index = 0
        for row in remainingEdgesVectors:
            angle = math.atan2(row[1],row[0])
            distance = math.sqrt(row[0]*row[0] + row[1]*row[1])
            remainingEdgesVectorForm[index,:] = np.array[angle, distance]       # RemainingEdgesVectorForm is [angle distance]

        # Get rid of all edges that are >95 degrees of the particle's direction to further reduce the 
        # amount of data being worked with. 
        # I'm making the assumption that angles are calculated as -pi to pi.
        # 95 degrees is 1.658 radians
        lowAngle = particleAngle - 1.658
        highAngle = particleAngle + 1.658

        if lowAngle < -math.pi:
            # Then it wrapped into the high values
            # selection should be -pi -> high and low + 2*pi->pi
            selection = (((remainingEdgesVectorForm[:,0] > -math.pi) & (remainingEdgesVectorForm[:,0] < highAngle)) |
                         ((remainingEdgesVectorForm[:,0] > lowAngle + 2*math.pi) & (remainingEdgesVectorForm[:,0] < math.pi))) 


        elif highAngle > math.pi:
            # It wrapped into low values
            # selection should be low-> pi and -pi-> high - 2*pi
            selection = (((remainingEdgesVectorForm[:,0] > lowAngle) & (remainingEdgesVectorForm[:,0] < math.pi)) |
                         ((remainingEdgesVectorForm[:,0] > -math.pi) & (remainingEdgesVectorForm[:,0] < highAngle - 2*math.pi))) 

        else:
            # It didn't wrap
            selection = (remainingEdgesVectorForm[:,0] > lowAngle) & (remainingEdgesVectorForm[:,0] < highAngle)


        remainingEdgesVectorForm = remainingEdgesVectorForm[selection == True][:]

        # It should now have only edges that are +- 95 degrees of the desired angle.  


        actualMeasurements = z_t1_arr;

        cumulativeProbability = 0

        for I in range(0,180,35): # calculate a range for all 180 measurements.  To reduce the number 
                                  # of distances calculated, make the last number something other than 1
                                  # 35 gives me beams at [0, 35, 70, 105, 140, 175]
                                  # 25 gives me beams at [0, 25, 50, 75, 100, 125, 150, 175]
                                  # 16 gives me beams at [0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176]

            # Calculate the angle for this reading
            relativeAngle = (I/180)* math.point
            absoluteAngle = relativeAngle + particleAngle



            # alculate the particle's measurement for this angle 
            particleMeasurement = findMeasurement(absoluteAngle, remainingEdgesVectorForm)


            # Adjust the measurements into 10cm divisions ie: Convert them into their bin locations
            actualMeasurement = round(actualMeasurements[I]/10)
            particleMeasurement = round(particleMeasurement/10)

            # Calculate the probability for this reading.  
            probability = uniformValue;
            if actualMeasurement <= particleMeasurement:
                probability += self.expPDF[0][actualMeasurement]

            # Figure out the shift of the gaussian.
            # As is, the center is at sample numSamples 
            firstGaussianSample = numSamples - particleMeasurement
            lastGaussianSample = firstGaussianSample + numSamples

            probability += self.gaussPDF[0][firstGaussianSample + actualMeasurement]



            # find the sum of the whole PDF to divide by
            normalizer = self.uniformSum
            normalizer += self.expPDF[1][particleMeasurement]
            normalizer += self.gaussPDF[1][lastGaussianSample] - self.gaussPDF[1][firstGaussianSample - 1]

            # Calculate the actual probability
            probability /= normalizer

            # Now find the log value of the probability
            cumulativeProbability += math.log(probability)


        return cumulativeProbability   
 
if __name__=='__main__':
    pass