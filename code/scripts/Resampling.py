import numpy as np
import pdb
import random

class Resampling:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 4.3]
    """

    def __init__(self):
        """
        TODO : Initialize resampling process parameters here
        """


    def multinomial_sampler(self, oldParticles, numberOfParticles):

        """
        param[in] oldParticles : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] newParticles : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """

        p = oldParticles[:,3] # The particle weights
        p = p/np.sum(p)  # 
        uniform_sample = np.random.uniform(size=numberOfParticles)  # create numberOfParticles samples between 0 and 1

        cumulativeSumOfWeights = np.cumsum(p)
        particlesToCopy = np.searchsorted(cumulativeSumOfWeights,uniform_sample,side='right')
        newParticles = oldParticles[particlesToCopy,:]
        return newParticles


    def low_variance_sampler(self, particles, numberOfParticles): # Rahul's version

        """
        particles : [num_particles x 4] sized array containing [x, y, theta, weight] values for all particles
        particles_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        M is the number of particles
        """

        resampledParticles = []
        weightOffset = np.random.uniform(0,(1/float(numberOfParticles))) # gives a number that is between 0 and 1/M, which is the width between the even samples
        cumulativeWeight = particles[0,3] # The weight for the first particle
        #print(c)
        particleIndex = 0   
        for m in range(1,numberOfParticles+1): # m
            sampleLocation = weightOffset + (m - 1) * (1/float(numberOfParticles))  
            # Add the weights of the particles until they are greater than sampleLocation
            while sampleLocation > cumulativeWeight:   
                particleIndex = particleIndex + 1
                cumulativeWeight = cumulativeWeight + particles[particleIndex,3]
            #print (particleIndex)
            resampledParticles.append(particles[particleIndex])
        resampledParticles = np.asarray(resampledParticles)
        return resampledParticles
       


    # def low_variance_sampler(self, oldParticles, numberOfDesiredParticles): # Jack's version
    #     """
    #     oldParticles : [num_particles x 4] sized array containing [x, y, theta, weight] values for all particles
    #     particles_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
    #     M is the number of oldParticles
    #     """

    #     newParticles = oldParticles  # I'm doing this so that resampled particles isn't resized each time a particle is added
    #     totalWeight = sum(oldParticles[:,3])
    #     print("low variance sampler: totalWeight: %f" % totalWeight)

    #     averageWeightOfNewParticles = totalWeight/(numberOfDesiredParticles+1) # The +1 is so that it doesn't try to sample past
    #                                                                     # the end of the old particles
    #     print("low variance sampler: average weight: %f" % averageWeightOfNewParticles)

    #     # Initially set the sample location to be somewhere randomly within the space of one sample width
    #     currentSampleLocation = random.random() * averageWeightOfNewParticles 

    #     currentCumulativeWeight = 0
    #     oldParticleIndex = 0

    #     for I in range(numberOfDesiredParticles):
    #         # Figure out which particle is located at the currentSampleLocation
    #         #print("low variance sampler: currentSampleLocation: %f" % currentSampleLocation)
    #         while currentCumulativeWeight < currentSampleLocation:
    #             currentCumulativeWeight += oldParticles[oldParticleIndex,3] # add the weight of an old particle
    #             #print("low variance sampler: currentCumulativeWeight: %f" % currentCumulativeWeight)
    #             oldParticleIndex += 1
    #         # When it gets here it has passed the particle that it should sample from
    #         newParticles[I,:] = oldParticles[oldParticleIndex - 1,:]
    #         print("low variance sampler: sampling particle #%d"%oldParticleIndex)

    #         currentSampleLocation += averageWeightOfNewParticles

    #     return newParticles

if __name__ == "__main__":
    pass
