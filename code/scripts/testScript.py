from MotionModel import MotionModel
from matplotlib import pyplot as plt
import numpy as np

control = np.array([ [10, 0, 0],
                     [0, 10, 0],
                     [10, 0, 0],
                     [0, 10, 0] ]);
numParticles = 50;

def initParticles():
    particlesX = np.random.uniform(-0.1,0.1, (numParticles,1));
    particlesY = np.random.uniform(-0.1,0.1, (numParticles,1));
    particlesTheta = np.zeros((numParticles,1));
    particlesPos = np.hstack((particlesX, particlesY, particlesTheta));
    return particlesPos;

def main():
    plt.figure(1); 
    motionModel = MotionModel();
    pose = initParticles();
    
    plt.scatter(pose[:,0], pose[:,1]);
    
    init = True; 
    for i in xrange(len(control)):
        if (init):
            u_t0 = np.array([0,0,0]);
            
        u_t1 = control[i,:];

        for p in xrange(numParticles):
            pose[p,:] = motionModel.update(u_t0, u_t1, pose[p,:]);

        plt.scatter(pose[:,0], pose[:,1]);
        u_t0 = u_t1;

    plt.show();

main(); 
                                   

