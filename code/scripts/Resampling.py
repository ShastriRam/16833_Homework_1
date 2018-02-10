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
        M = 500
        X_bar_resampled = []

    def multinomial_sampler(self, X_bar):

        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """

        """
        TODO : Add your code here
        """
        p = X_bar[:,4]
        p = p/numpy.sum(p) #p = np.divide(p, numpy.sum(p))
        if sum(p) != 1.: raise ValueError, "p must sum to 1"
        uniform_sample = np.random.uniform(size=M)
        p_cum = np.cumsum(p)
        X_bar_resampled = np.searchsorted(p_cum,uniform_sample,side='right')

        return X_bar_resampled

    def low_variance_sampler(self, X_bar):

        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """

        """
        TODO : Add your code here
        """
        r = np.random.uniform(0,1/M)
        c = X_bar[0,4]
        print(c)
        i = 1
        for m in range(1,M)
            U = r + (m − 1) * (1/M)
            while U > c :
                i = i + 1
                c = c + X_bar[i,4]
            X_bar_resampled = np.append(X_bar[i],axes = 0)
        X_bar_resampled = np.asarray(X_bar_resampled)

        return X_bar_resampled

if __name__ == "__main__":
    pass