import numpy as np
import sys

from MapReader import MapReader
from MotionModel import MotionModel
from SensorModel import SensorModel
from Resampling import Resampling

from matplotlib import pyplot as plt
from matplotlib import figure as fig
import time
from time import sleep
from scipy import signal
from scipy.stats import expon




def plotProbabilities(probabilities):
    mng = plt.get_current_fig_manager() 
    #mng.resize(*mng.window.maxsize())
    plt.ion() 
    plt.plot(probabilities)
    plt.show()
    sleep(10)



def main():
    src_path_map = '../data/map/wean.dat'
    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map()
    sensor_model = SensorModel(occupancy_map)


    particleMeasurement = 500
    probabilities = np.zeros(1000)

    index = 0;
    for actualMeasurement in range(1000):
        probabilities[index] = sensor_model.calculateProbability(actualMeasurement,particleMeasurement)
        index += 1
    plotProbabilities(probabilities)


    numSamples  = 1000
    stdev = 100
    gaussPDF = signal.gaussian(numSamples * 2,std=stdev)
    
    #plotProbabilities(gaussPDF)


    x = np.linspace(expon.ppf(0.01), expon.ppf(0.99), numSamples) # Makes numSamples samples with a probability ranging from .99 to .1
    expPDF = expon.pdf(x)
    #plotProbabilities(expPDF)

    # lambdaShort = 1
    # distrShort = expon(scale=1/lambdaShort)
    # expCurv = np.zeros(1000)
    
    # index = 0;
    # for I in range (1000):
    #     expCurv(I) = distrShort.pdf(I)

    # plotProbabilities(expCurve)

    # print (expPDF)

if __name__=="__main__":
    main()