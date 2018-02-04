import numpy as np

from matplotlib import pyplot as plt
from matplotlib import figure as fig

import scipy.ndimage as ndimage


def f(x):
    return 1 if x > .95 else 0 # This gives an image where it is 1 if it is free space and 0 
    #                          # if you can't be there.  .95 seems to be the ideal threshold 

f = np.vectorize(f)  

class MapReader:

    def __init__(self, src_path_map):

        self._occupancy_map = np.genfromtxt(src_path_map, skip_header=7)
        self._occupancy_map[self._occupancy_map < 0] = -1
        #self._occupancy_map[self._occupancy_map > 0] = 1 - self._occupancy_map[self._occupancy_map > 0] # invert the map
        self._occupancy_map = f(self._occupancy_map)  # convert the map into 0 or 1 where 1 indicates a traverable area
        self._occupancy_map = np.flipud(self._occupancy_map)

        # Find the edges   1 indicates an edge.
        kernel = np.array([[0,1,0],[1,1,1],[0,1,0]])
        map2 = ndimage.binary_dilation(self._occupancy_map, structure=kernel).astype(self._occupancy_map.dtype)
        self.edges = map2 - self._occupancy_map
 

        

        self._resolution = 10 # each cell has a 10cm resolution in x,y axes
        self._size_x = self._occupancy_map.shape[0] * self._resolution
        self._size_y = self._occupancy_map.shape[1] * self._resolution

        print 'Finished reading 2D map of size : ' + '(' + str(self._size_x) + ',' + str(self._size_y) + ')'

    def visualize_map(self):
        # fig = plt.figure()
        # # plt.switch_backend('TkAgg')
        # mng = plt.get_current_fig_manager(); mng.resize(*mng.window.maxsize())
        # # This is the way that it was that draws it really small in the bottom left corner
        # #plt.ion(); plt.imshow(self._occupancy_map, cmap='Greys'); plt.axis([0, self._size_x, 0, self._size_y]); plt.draw() 
        # plt.ion(); plt.imshow(self._occupancy_map, cmap='Greys'); plt.axis([0, self._occupancy_map.shape[0], 0, self._occupancy_map.shape[1]]); plt.draw()
        
        # plt.pause(100)

        fig1 = plt.figure(1)
        mng = plt.get_current_fig_manager(); 
        mng.resize(*mng.window.maxsize())
        #plt.ion() # turns on interactive mode
        plt.subplot(121)
        plt.imshow(self._occupancy_map, cmap='Greys')
        plt.axis([0, self._occupancy_map.shape[0], 0, self._occupancy_map.shape[1]]) # set the limits of where it is drawn
        plt.draw()
        




    def visualize_edges(self):
        fig1 = plt.figure(1)
        plt.subplot(122)
        plt.imshow(self.edges,'gray') # gray results in white edges and grey background
                                        # binary results in black edges and grey background
        plt.axis([0, self.edges.shape[0], 0, self.edges.shape[1]])
        plt.draw()
        
        plt.pause(100) # pauses for 100 seconds then program exits


    def get_map(self):
        return self._occupancy_map

    def get_map_size_x(self): # in cm
        return self._size_x

    def get_map_size_y(self): # in cm
        return self._size_y

    def makeEdgeList(self):
        # takes the edges array and makes a list of all of the coordinates for all pixels that equal 1
        # edgeMapWidth = self.edges.shape[1]
        # edgeMapHeight = self.edges.shape[0]

        # Sum all of the pixels in the map.  This gives me a count of all of the edge pixels.
        # values, counts = np.unique(self.edges, return_counts=True)
        # numberOfEdgePixels = counts[1]
        # print('Number of edge pixels: ', numberOfEdgePixels)
        # edgeLocations = np.zeros(numberOfEdgePixels,2)


        x,y = np.nonzero(self.edges)

        temp = np.concatenate((x,y))
        length = temp.shape[0]
        edgeLocations = np.fliplr(np.reshape(temp,(2,length/2)).T)  # This will be a matrix that has 
                                                                    # rows with [X Y] that correspond
                                                                    # to the locations of the edge pixels 
                                                                    # in our case, this is (5643, 2)




if __name__=="__main__":
    
    src_path_map = '../data/map/wean.dat'
    map1 = MapReader(src_path_map)
    map1.makeEdgeList()
    map1.visualize_map()
    map1.visualize_edges()

