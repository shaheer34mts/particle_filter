#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <iostream>
#include <string>
#include <math.h>
#include <fstream>
#include <random>
#include <algorithm>
#include <numeric>
#include <vector>
#include <unistd.h>

#include "ros/ros.h"
#include "eigen3/Eigen/Eigen"
#include <binders.h>

#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>


namespace particle_filter {
typedef struct  {
	int id_;
	geometry_msgs::Quaternion orientation_;							
	geometry_msgs::Point pose_;
    double weight_;
}Particle;



class ParticleFilter {

public:
	
	ParticleFilter(int n_p, ros::NodeHandle node);

	// Destructor
	~ParticleFilter();
	


	int num_particles_; 								// Number of particles to draw
	bool is_initialized_;							// Flag, if filter is initialized
	std::vector<double> weights_;    				// Vector of weights of all particles
	ros::Time stamp_;                                // timestamp
	std::vector<Particle> particles_;				// Set of current particles

	ros::Publisher publish_particlecloud_;
    ros::Publisher publish_pose_;
    ros::NodeHandle node_;
	geometry_msgs::PoseArray particles_ros_;
    geometry_msgs::PoseStamped robot_ros_;

	// Constructor
	// @param M Number of particles


	/**
	 * init Initializes particle filter by initializing particles to Gaussian
	 *   distribution around first position and all the weights to 1.
	 * @param x Initial x position [m] 
	 * @param y Initial y position [m]
     * @param z Initial z position [m]
	 * @param orientation_ Initial orientation [quaternion]
	 * @param std_dev[] Array of dimension 7 [standard deviation of x [m], standard deviation of y [m], standard deviation of z [m]
	 *   standard deviation of yaw [rad]]
	 */
	void init( double std_dev[]);
	void prediction();
	void Visualize();


};

}

#endif /* PARTICLE_FILTER_H_ */