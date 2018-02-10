import numpy as np
import pdb

class Resampling:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 4.3]
    """

    def __init__(self):
        """
        TODO : Initialize resampling process parameters here
        """


    def multinomial_sampler(self, X_bar, M):

        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """

        """
        TODO : Add your code here
        """
        p = X_bar[:,3]
        p = p/numpy.sum(p) #p = np.divide(p, numpy.sum(p))
        print (numpy.sum(p))
        uniform_sample = np.random.uniform(size=M)
        p_cum = np.cumsum(p)
        X_bar_resampled = np.searchsorted(p_cum,uniform_sample,side='right')

        return X_bar_resampled


    def low_variance_sampler(self, particles, numberOfParticles):

        """
        particles : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
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
            while ((sampleLocation > cumulativeWeight) & (particleIndex < numberOfParticles-1)):   
                particleIndex = particleIndex + 1
                cumulativeWeight = cumulativeWeight + particles[particleIndex,3]
            print (particleIndex)
            resampledParticles.append(particles[particleIndex])
        resampledParticles = np.asarray(resampledParticles)
       

        return resampledParticles

if __name__ == "__main__":
    pass
