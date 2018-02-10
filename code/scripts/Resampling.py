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

    def low_variance_sampler(self, X_bar, M):

        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """

        """
        TODO : Add your code here
        """
        X_bar_resampled = []
        r = np.random.uniform(0,(1/M))
        c = X_bar[0,3]
        print(c)
        i = 1
        for m in range(0, M):
            U = r + (m - 1) * (1/M)
            while U > c :
                i = i + 1
                c = c + X_bar[i,3]
            X_bar_resampled.append(X_bar[i])
        X_bar_resampled = np.asarray(X_bar_resampled)
       

        return X_bar_resampled

if __name__ == "__main__":
    pass