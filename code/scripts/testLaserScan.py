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
    x2 = vectorLaser[:,2] 
    y2 = vectorLaser[:,3]

    for i in range(0, len(x1)):
        plt.plot([x1[i],x2[i]],[y1[i], y2[i]], color='k', linestyle='-', linewidth=1)
    plt.show()
    #sleep(3)

def main():
    x = np.array([94.234001, -139.953995, -1.342158])
    z = np.array([66, 66, 66, 66, 66, 65, 66, 66, 65, 66, 66, 66, 66, 66, 67, 67, 67, 66, 67,66,67,67,67,68,68,68,69,67,530,514,506,508,494,481,
        470,458,445,420,410,402,393,386,379,371,365,363,363,364,358,353,349,344,339,335,332,328,324,321,304,299,298,294,291,288,287,284,282,281,
        277,277,276,274,273,271,269,268,267,266,265,265,264,263,263,263,262,261,261,261,261,261,193,190,189,189,192,262,262,264,194,191,190,190,
        193,269,271,272,274,275,277,279,279,281,283,285,288,289,292,295,298,300,303,306,309,314,318,321,325,329,335,340,360,366,372,378,384,92,92,
        91,89,88,87,86,85,84,83,82,82,81,81,80,79,78,78,77,76,76,76,75,75,74,74,73,73,72,72,72,71,72,71,71,71,71,71,71,71,71,70,70,70,70])
    src_path_map = '../data/map/wean.dat'
    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map()
    map_obj.makeEdgeList()
    edgelist = map_obj.edgeLocations
    sensorObject = SensorModel(edgelist)
    vectorLaser = sensorObject.beam_range_finder_model(z, x)
    visualize_map(occupancy_map,vectorLaser)

if __name__=="__main__":
    main()