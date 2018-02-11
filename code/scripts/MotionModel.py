import sys
import numpy as np
import math

class MotionModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 5]
    """

    def __init__(self):

        #Initialize Motion Model parameters here
        self.alpha_1 = 0.0001;
        self.alpha_2 = 0.000;
        self.alpha_3 = 0.000;
        self.alpha_4 = 0.0001; 

    def sample_normal(self, b_sq):
        b = math.sqrt(b_sq);
        total = 0;
        for i in xrange(12):
            total = total + np.random.uniform(-b,b);
        return (0.5*total);

    def sample_triangular(self, b_sq):
        b = math.sqrt(b_sq);
        val = np.random.uniform(-b,b) + np.random.uniform(-b,b);
        val = val*math.sqrt(6)*0.5;
        return val; 

    def update(self, u_t0, u_t1, x_t0):
        """
        param[in] u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]   
        param[in] u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
        param[in] x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
        param[out] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        """

        x_odom_0 = u_t0[0];
        y_odom_0 = u_t0[1];
        theta_odom_0 = u_t0[2];

        x_odom_1 = u_t1[0];
        y_odom_1 = u_t1[1];
        theta_odom_1 = u_t1[2];
                
        pose_x = x_t0[0];
        pose_y = x_t0[1];
        pose_theta = x_t0[2]; 

        delta_rot_1 = math.atan2(y_odom_1-y_odom_0, x_odom_1-x_odom_0 - theta_odom_0);
        delta_trans = math.sqrt( (x_odom_0-x_odom_1)*(x_odom_0-x_odom_1) +
                                 (y_odom_0-y_odom_1)*(y_odom_0-y_odom_1));
        delta_rot_2 = theta_odom_1 - theta_odom_0 - delta_rot_1;

        delta_rot_1_prime = delta_rot_1 - self.sample_normal(self.alpha_1*pow(delta_rot_1,2) +
                                                             self.alpha_2*pow(delta_trans,2));
        delta_trans_prime = delta_trans - self.sample_normal(self.alpha_3*pow(delta_trans,2) +
                                                             self.alpha_4*pow(delta_rot_1,2) +
                                                             self.alpha_4*pow(delta_rot_2,2));
        delta_rot_2_prime = delta_rot_2 - self.sample_normal(self.alpha_1*pow(delta_rot_2,2) +
                                                             self.alpha_2*pow(delta_trans,2));

        x_prime = pose_x + delta_trans_prime*math.cos(pose_theta+delta_rot_1_prime);
        y_prime = pose_y + delta_trans_prime*math.sin(pose_theta+delta_rot_1_prime);
        theta_prime = pose_theta + delta_rot_1_prime + delta_rot_2_prime;

        x_t1 = np.array([x_prime, y_prime, theta_prime]);  
                                                 
        return x_t1

if __name__=="__main__":
    pass
