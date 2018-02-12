import numpy as np
import sys
import pdb

from MapReader import MapReader
from MotionModel import MotionModel
#from SensorModel2 import SensorModel   #Shastri's version
from SensorModel import SensorModel     # Jack's version
from Resampling import Resampling

from matplotlib import pyplot as plt
from matplotlib.patches import Circle
from matplotlib import figure as fig
import time
from time import sleep
import math

def visualize_map(occupancy_map,particles):
    x = particles[:,0]/10 
    y = particles[:,1]/10

    plt.imshow(occupancy_map, cmap='Greys');
    plt.scatter(x,y,color='r',marker='.', s = 10); 
    plt.show(block=False)
    #plt.show()
    time.sleep(1)
    plt.close()






def main():


    src_path_map = '../data/map/wean.dat'
    src_path_log = '../data/log/robotdata1.log'

    logfile = open(src_path_log, 'r')

   


    first_time_idx = True
    for time_idx, line in enumerate(logfile):
        # time_idx is just a counter
        # line is the text from a line in the file.


        # Read a single 'line' from the log file (can be either odometry or laser measurement)
        meas_type = line[0] # L : laser scan measurement, O : odometry measurement
        meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ') # convert measurement values from string to double

        state = meas_vals[0:3] # odometry reading [x, y, theta] in odometry frame
        time_stamp = meas_vals[-1]



        if (meas_type == "L"):  # Laser data
             odometry_laser = meas_vals[3:6] # [x, y, theta] coordinates of laser in odometry frame
             ranges = meas_vals[6:-1] # 180 range measurement values from single laser scan
        else: 
            #print("Skipping this record because it is an odometry record.")
            continue    

        # convert the ranges to [X,Y]
        X = np.zeros(180)
        Y = np.zeros(180)
        for I in range(180):
            #lidarPoints[I,:] = ranges[I] * np.array[math.cos(float(I)/180), math.sin(float(I/180)) ]
            angleInRadians = (float(I)/ 180) * math.pi
            X[I] = ranges[I]* math.cos(angleInRadians)
            Y[I] = ranges[I]* math.sin(angleInRadians)

        # plot the points

        print("Time: %f"%time_stamp)
        # plt.scatter(X,Y,color='r',marker='.', s = 10); 
        # plt.axis('equal')
        # plt.show(block=False)

        # #plt.show()
        # time.sleep(1)
        # plt.close()



        fig = plt.figure(1);
        ax = fig.add_subplot(111);

        ax.scatter(X,Y,color='r',marker='.', s = 10); 
        ax.axis('equal')
        fig.canvas.draw()
        plt.show(block=False)
        time.sleep(.1)
        ax.clear()


if __name__=="__main__":
    main()
