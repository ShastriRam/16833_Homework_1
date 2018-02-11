import numpy as np
import sys

from Resampling import Resampling

def main():
    X_bar = np.array([[-94.234001,-139.953995,-1.342158,0.025863],[-94.234001,-139.953995,-1.342158,0.079745],
    	[-94.234001,-139.953995,-1.342158,0.13982],[-94.234001,-139.953995,-1.342158,0.200218]])
    print (X_bar.shape)
    print(X_bar[0,3])
    sampleObj = Resampling()
    X_bar_resampled = sampleObj.low_variance_sampler(X_bar)
    print (X_bar_resampled)

if __name__=="__main__":
	main()