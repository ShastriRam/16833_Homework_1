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

def visualize_map(occupancy_map,vectorLaser):

    mng = plt.get_current_fig_manager();  # mng.resize(*mng.window.maxsize())
    plt.ion(); plt.imshow(occupancy_map, cmap='Greys'); plt.axis([0, 800, 0, 800]);
    x1 = vectorLaser[:,0] 
    y1 = vectorLaser[:,1]
    x2 = vectorLaser[:,3] 
    y2 = vectorLaser[:,4]

    for i in rang(0, len(x1)):
        plt.plot([x1[i],x2[i]],[y1[i], y2[i]], color='k', linestyle='-', linewidth=1)
    plt.show()
    #sleep(3)

def main():
    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map()
    vectorLaser = SensorModel.rangeLines
    visualize_map(occupancy_map,vectorLaser)

if __name__=="__main__":
    main()