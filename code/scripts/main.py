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

def visualize_map(occupancy_map,particles,imageNumber):
    x = particles[:,0]/10 
    y = particles[:,1]/10

    fig = plt.figure(1);
    # mng = plt.get_current_fig_manager(); 
    # mng.resize(*mng.window.maxsize())
    ax = fig.add_subplot(111);
    
    ax.scatter(x,y,color='r',marker='.', s = 10);
    ax.imshow(occupancy_map, cmap='Greys');
    fig.canvas.draw()
    plt.show(block=False)
    
    fileName = 'Renders/working%04d.png' % imageNumber
    plt.savefig(fileName)
    ax.clear()

    #time.sleep(2)
    """
    plt.figure(1)
    plt.ion()
    plt.imshow(occupancy_map, cmap='Greys');
    plt.scatter(x,y,color='r',marker='.', s = 10); 
    plt.show(block=False)
    #plt.show()
    time.sleep(1)
    res.remove()
    plt.close()
    """

def init_particles_freespace(num_particles, occupancy_map):

    # initialize [x, y, theta] positions in world_frame for all particles
    # (in free space areas of the map)

    # Initialize the arrays so that they are the proper size
    print("Starting init_particles_freespace")
    startTime = time.time()
    x0_vals = np.random.uniform( 3300, 7000, (num_particles, 1)) # Create a initial array so that
    y0_vals = np.random.uniform( 0, 7500, (num_particles, 1))  # it doesn't get resized each time

    for I in range(num_particles):

        stillWorking = True
        while stillWorking == True:
            # Generate a particle location
            # Initial position:  4130, 4000.0, 3.05

            initializeOverWholeMap = 1

            if initializeOverWholeMap == 0:         # Tight rectangle around start point
                X = np.random.uniform( 4030, 4230)
                Y = np.random.uniform( 3900, 4100)
            elif initializeOverWholeMap == 1:       # The whole map
                X = np.random.uniform( 3300, 7000)
                Y = np.random.uniform( 0, 7500)
            elif initializeOverWholeMap == 2:       # Wider area around start point
                X = np.random.uniform( 3700, 4500)
                Y = np.random.uniform( 3700, 4300)
            elif initializeOverWholeMap == 3:       # hybrid where there are a lot of particles at the start point
                choose = np.random.uniform(0,20)
                if choose > 18:
                    X = np.random.uniform( 4000, 4270) # right around the start point
                    Y = np.random.uniform( 3850, 4150)
                else:
                    X = np.random.uniform( 3300, 7000) # whole map
                    Y = np.random.uniform( 0, 7500)

            # Check to see if this is in free space or not
            Xx = int(X/10) # Convert from cm to dm
            Yy = int(Y/10)
            if occupancy_map[Yy,Xx] == 1:
                stillWorking = False
        x0_vals[I] = X
        y0_vals[I] = Y


    theta0_vals = np.random.uniform( -3.14, 3.14, (num_particles, 1) )

    # initialize weights for all particles
    w0_vals = np.ones( (num_particles,1), dtype=np.float64)
    w0_vals = w0_vals / num_particles
    particles_init = np.hstack((x0_vals,y0_vals,theta0_vals,w0_vals))
    print("finished init_particles_freespace")
    #startTime = time.time()
    print("Completed in  %s seconds" % (time.time() - startTime))
    return particles_init



def main():

    """
    Description of variables used
    u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]   
    u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
    x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
    x_t1 : particle state belief [x, y, theta] at time t [world_frame]
    particles : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
    z_t : array of 180 range measurements for each laser scan
    """

    """
    Initialize Parameters
    """
    ###########################################  SET THE NUMBER OF PARTICLES #####################################
    num_particles = 10000
    ###########################################  SET THE NUMBER OF PARTICLES #####################################

    src_path_map = '../data/map/wean.dat'
    src_path_log = '../data/log/robotdata1.log'

    #src_path_log = '../data/log/robotdata5.log'

    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map()
    map_size_x = map_obj.get_map_size_x()
    map_size_y = map_obj.get_map_size_y()

    logfile = open(src_path_log, 'r')

    motion_model = MotionModel()
    sensor_model = SensorModel(occupancy_map)
    resampler = Resampling()



    particles = init_particles_freespace(num_particles, occupancy_map)
    #particles = np.array([[4100,3990,3,1],[4060,3990,3,1],[4000,3990,3,1],[4000,3990,2,1],[6150,1270,1.7,1]]) 
    # The particles above are
    # In the correct location with approximately the correct angle
    # Correct angle but a little farther out into the hallway
    # Correct angle but squarely in the hallway
    # In the center of the hallway and at wrong angle
    # Completely wrong in the big room at the bottom


       
    vis_flag = 1

    """
    Monte Carlo Localization Algorithm : Main Loop
    """
    first_time_idx = True
    lastUsedOdometry = np.array([0,0,0])
    imageNumber = 0

    for time_idx, line in enumerate(logfile):
        # time_idx is just a counter
        # line is the text from a line in the file.


        # Read a single 'line' from the log file (can be either odometry or laser measurement)
        meas_type = line[0] # L : laser scan measurement, O : odometry measurement
        meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ') # convert measurement values from string to double

        state = meas_vals[0:3] # odometry reading [x, y, theta] in odometry frame
        time_stamp = meas_vals[-1]

        # if ((time_stamp <= 0.0) | (meas_type == "O")): # ignore pure odometry measurements for now (faster debugging) 
            # continue

        if (meas_type == "L"):  # Laser data
             odometry_laser = meas_vals[3:6] # [x, y, theta] coordinates of laser in odometry frame
             #print(odometry_laser)
             delta = odometry_laser - lastUsedOdometry
             distance = math.sqrt(delta[0]*delta[0] + delta[1] * delta[1])
             

             # Don't update if it didn't move
             if ((distance < 35) and (delta[2] < .05)): # Don't update or do anything if the robot is stationary
                print("Time: %f\tDistance: %f\tangle: %f\tDidn't move enough..."%(time_stamp,distance,delta[2]))
                continue

             print("Time: %f\tDistance: %f\tangle: %f"%(time_stamp,distance,delta[2]))

             lastUsedOdometry =  odometry_laser
             ranges = meas_vals[6:-1] # 180 range measurement values from single laser scan

        else: 
            #print("Skipping this record because it is an odometry record.")
            continue    


        #print "Processing time step " + str(time_idx) + " at time " + str(time_stamp) + "s"

        if (first_time_idx):
            lastState = state
            first_time_idx = False
            continue

        #particles_new = np.zeros( (num_particles,4), dtype=np.float64)
        currentState = state



        # MOTION MODEL - move each particle
        startTime = time.time()
        for m in range(num_particles): 
            oldParticle = particles[m, 0:3]
            particles[m, 0:3] = motion_model.update(lastState, currentState, oldParticle) # This is [X,Y,theta]
            # # Use this line for testing probabilities
            # newParticle = particles[m,0:3] ######### Don't update the position of the particle.####################
        print("Motion model completed in  %s seconds" % (time.time() - startTime))  # Typically takes .125 seconds for 
                                                                                    # 10000 particles




        # SENSOR MODEL - find the liklihood of each particle
        startTime = time.time()
        for m in range(num_particles): 
            particles[m,3] = sensor_model.beam_range_finder_model(ranges, particles[m,0:3])
        print("Sensor model completed in  %s seconds" % (time.time() - startTime)) # Typically takes 7.85 seconds for 
                                                                                    # 10000 particles
                



        lastState = currentState
                

        # print '***********Particles before normalizing***************'
        # print(particles)

        
        # #normalize the weights
        #minWeight = min(particles[:,3]);
        #maxWeight = max(particles[:,3]);
        #weightRng = (maxWeight - minWeight);
        #if (abs(weightRng)<0.0000001):
        #    particles[:,3] = (1/float(num_particles))*np.ones(num_particles);
        #else:
        #    particles[:,3] = (particles[:,3] - minWeight)/weightRng;
        
        
        #print '***********Particles after normalizing***************'
        #
        #print(particles)
        #particles = resampler.low_variance_sampler(particles,num_particles)
        particles = resampler.multinomial_sampler(particles, num_particles)

        #print("Completed in  %s seconds" % (time.time() - startTime))  # this is currently taking about .4 seconds per particle
        # Resampling typically takes 8 ms for 5000 particles.


        if vis_flag:
            imageNumber += 1
            visualize_map(occupancy_map,particles,imageNumber)

if __name__=="__main__":
    main()
