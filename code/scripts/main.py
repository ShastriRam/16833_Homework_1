import numpy as np
import sys
import pdb

from MapReader import MapReader
from MotionModel import MotionModel
from SensorModel import SensorModel
from Resampling import Resampling

from matplotlib import pyplot as plt
from matplotlib.patches import Circle
from matplotlib import figure as fig
import time
from time import sleep

<<<<<<< HEAD
def visualize_map(occupancy_map,particles):
    plt.figure(1)
    #ax.set_aspect('equal')
    # plt.switch_backend('TkAgg')
    #mng = plt.get_current_fig_manager();  # mng.resize(*mng.window.maxsize())
    #plt.ion(); ax.imshow(occupancy_map, cmap='Greys'); ax.axis([0, 800, 0, 800]);
    x = particles[:,0]/10 
    y = particles[:,1]/10
    #x = np.mean(x);
    #y = np.mean(y);
  
=======
def visualize_map(occupancy_map,particles, i):
    
    figure,ax = plt.subplots(1)
    ax.set_aspect('equal')
    mng = plt.get_current_fig_manager();
    mng.resize(*mng.window.maxsize())
    plt.ion(); 
    x = particles[:,0]/10 
    y = particles[:,1]/10
>>>>>>> 5ab1adf8be08e59cb81875dea8caacfdf1c20957
    # Now, loop through coord arrays, and create a circle at each x,y pair
<<<<<<< HEAD
    for xx,yy in zip(x,y):
    	circ = Circle((xx,yy),5)
    	ax.add_patch(circ)
    if i == 3:
        ax.imshow(occupancy_map, cmap='Greys'); 
        ax.axis([0, 800, 0, 800]);
=======
    #for xx,yy in zip(x,y):
    #	circ = Circle((xx,yy),0.5)
    #	ax.add_patch(circ)


    #circ = Circle((x,y),5)
    #ax.add_patch(circ)
    #plt.show()
    #sleep(3)
<<<<<<< HEAD
    plt.imshow(occupancy_map, cmap='Greys'); 
    plt.scatter(x,y); 
    plt.show(block=False)
    time.sleep(3)
    plt.close()

=======
>>>>>>> e899b2893c82250af4b210c2cf3d709782588738
>>>>>>> 5ab1adf8be08e59cb81875dea8caacfdf1c20957
    

def visualize_timestep(X_bar, tstep):
    x_locs = X_bar[:,0]/10.0
    y_locs = X_bar[:,1]/10.0
    scat = plt.scatter(x_locs, y_locs, c='r', marker='o')
    plt.pause(0.00001)
    scat.remove()


def init_particles_random(num_particles, occupancy_map):

    # initialize [x, y, theta] positions in world_frame for all particles
    # (randomly across the map) 
    y0_vals = np.random.uniform( 0, 7000, (num_particles, 1) ) # Generate the 
    x0_vals = np.random.uniform( 3000, 7000, (num_particles, 1) )
    theta0_vals = np.random.uniform( -3.14, 3.14, (num_particles, 1) )

    # initialize weights for all particles
    w0_vals = np.ones( (num_particles,1), dtype=np.float64)
    w0_vals = w0_vals / num_particles

    X_bar_init = np.hstack((x0_vals,y0_vals,theta0_vals,w0_vals))
    
    return X_bar_init




def init_particles_freespace(num_particles, occupancy_map):

    # initialize [x, y, theta] positions in world_frame for all particles
    # (in free space areas of the map)

    # Initialize the arrays so that they are the proper size
    print("Starting init_particles_freespace")
    startTime = time.time()
    y0_vals = np.random.uniform( 0, 7000, (num_particles, 1) ) # Generate the 
    x0_vals = np.random.uniform( 3000, 7000, (num_particles, 1) )

    for I in range(num_particles):

        stillWorking = True
        while stillWorking == True:
            # Generate a particle location
            Y = np.random.uniform( 0, 7000)
            X = np.random.uniform( 3000, 7000)

            # Check to see if this is in free space or not
            Xx = int(X/10) # Convert from cm to dm
            Yy = int(Y/10)
            if occupancy_map[Yy,Xx] == 1:
                stillWorking = False
        #print("X: %d\tY: %d" % (X,Y))
        x0_vals[I] = X
        y0_vals[I] = Y


    theta0_vals = np.random.uniform( -3.14, 3.14, (num_particles, 1) )

    # initialize weights for all particles
    w0_vals = np.ones( (num_particles,1), dtype=np.float64)
    w0_vals = w0_vals / num_particles
    X_bar_init = np.hstack((x0_vals,y0_vals,theta0_vals,w0_vals))
    print("finished init_particles_freespace")
    print("Completed in  %s seconds" % (time.time() - startTime))
    return X_bar_init



def main():

    """
    Description of variables used
    u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]   
    u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
    x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
    x_t1 : particle state belief [x, y, theta] at time t [world_frame]
    X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
    z_t : array of 180 range measurements for each laser scan
    """

    """
    Initialize Parameters
    """
    src_path_map = '../data/map/wean.dat'
    src_path_log = '../data/log/robotdata1.log'

    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map()
    map_size_x = map_obj.get_map_size_x()
    map_size_y = map_obj.get_map_size_y()

    logfile = open(src_path_log, 'r')

    motion_model = MotionModel()
    sensor_model = SensorModel(occupancy_map)
    resampler = Resampling()

    num_particles = 500
    #X_bar = init_particles_random(num_particles, occupancy_map)
    X_bar = init_particles_freespace(num_particles, occupancy_map)

       
    vis_flag = 1

    """
    Monte Carlo Localization Algorithm : Main Loop
    """
    i = 1
    first_time_idx = True
    for time_idx, line in enumerate(logfile):
        # time_idx is just a counter
        # line is the text from a line in the file.


        # Read a single 'line' from the log file (can be either odometry or laser measurement)
        meas_type = line[0] # L : laser scan measurement, O : odometry measurement
        meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ') # convert measurement values from string to double

        odometry_robot = meas_vals[0:3] # odometry reading [x, y, theta] in odometry frame
        time_stamp = meas_vals[-1]

        # if ((time_stamp <= 0.0) | (meas_type == "O")): # ignore pure odometry measurements for now (faster debugging) 
            # continue

        if (meas_type == "L"):  # Laser data
             odometry_laser = meas_vals[3:6] # [x, y, theta] coordinates of laser in odometry frame
             ranges = meas_vals[6:-1] # 180 range measurement values from single laser scan
        
        print "Processing time step " + str(time_idx) + " at time " + str(time_stamp) + "s"

        if (first_time_idx):
            u_t0 = odometry_robot
            first_time_idx = False
            continue

        X_bar_new = np.zeros( (num_particles,4), dtype=np.float64)
        u_t1 = odometry_robot
        for m in range(0, num_particles):

            """
            MOTION MODEL
            """
            x_t0 = X_bar[m, 0:3]
            x_t1 = motion_model.update(u_t0, u_t1, x_t0)

            """
            SENSOR MODEL
            """
            if (meas_type == "L"):
                z_t = ranges
                w_t = sensor_model.beam_range_finder_model(z_t, x_t1)
                # w_t = 1/num_particles
                X_bar_new[m,:] = np.hstack((x_t1, w_t))
            else:
                X_bar_new[m,:] = np.hstack((x_t1, X_bar[m,3]))
        
        X_bar = X_bar_new
        u_t0 = u_t1

        if vis_flag:
            visualize_map(occupancy_map,X_bar,i)
            i = i + 1
        """
        RESAMPLING
        """
<<<<<<< HEAD
        X_bar = resampler.low_variance_sampler(X_bar,num_particles)
=======
        X_bar = resampler.low_variance_sampler(X_bar, num_particles)
>>>>>>> 5ab1adf8be08e59cb81875dea8caacfdf1c20957

        if vis_flag:
            visualize_timestep(X_bar, time_idx)

if __name__=="__main__":
    main()
