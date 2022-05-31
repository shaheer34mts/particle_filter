#include "particle_filter.h"

namespace particle_filter {

ParticleFilter::ParticleFilter(int n_p, ros::NodeHandle node){
    
    this->num_particles_ = n_p;
    this->is_initialized_ = false;
    this->node_ = node;
    //publish_particlecloud_ = node_.advertise<geometry_msgs::PoseArray>("particle_cloud", 1, true);
	this->particles_ros_.header.stamp = ros::Time::now();
	this->particles_ros_.header.frame_id = "map";
	this->particles_ros_.poses.resize(num_particles_);
    

}

ParticleFilter::~ParticleFilter() {

}

void ParticleFilter::init(double std_dev[]){

    
    // Standard deviation for x , y , z coordinates
    double std_x, std_y, std_z, std_ox, std_oy, std_oz, std_ow;
    std_x = std_dev[0]; std_y= std_dev[1];  std_z= std_dev[2];  std_ox= std_dev[3];  std_oy= std_dev[4];  std_oz= std_dev[5];  std_ow= std_dev[6];
    std::normal_distribution<double> dist_x(0, std_x);
    std::normal_distribution<double> dist_y(0, std_y);
    std::normal_distribution<double> dist_z(0, std_z);
    std::normal_distribution<double> dist_ox(0, std_ox);
    std::normal_distribution<double> dist_oy(0, std_oy);
    std::normal_distribution<double> dist_oz(0, std_oz);
    std::normal_distribution<double> dist_ow(0, std_ow);

    // Normally distributed random numbers generator
    std::default_random_engine rand_gen;
    
    Particle sample_particle;
       
    for(int i=0; i<num_particles_; ++i){

        // Assign ID to particle
        sample_particle.id_ = i;

        // Fill pose of sample particle
        sample_particle.pose_.x = dist_x(rand_gen);
        sample_particle.pose_.y = dist_y(rand_gen);
        sample_particle.pose_.z = dist_z(rand_gen);

        

        //Fill orientation of sample particle
        sample_particle.orientation_.x = dist_ox(rand_gen);
        sample_particle.orientation_.y = dist_oy(rand_gen);
        sample_particle.orientation_.z = dist_oz(rand_gen);
        sample_particle.orientation_.w = dist_ow(rand_gen);

        // Assign weight 1 to particle
        sample_particle.weight_ = 1;
        particles_.push_back(sample_particle);


    }

    geometry_msgs::Pose pose_ros;
	geometry_msgs::Quaternion q; 
    
    for(int i=0; i<num_particles_; ++i){

        q = particles_[i].orientation_;

        pose_ros.position.x = particles_[i].pose_.x;
        pose_ros.position.y = particles_[i].pose_.y;
        pose_ros.position.z = particles_[i].pose_.z;
        pose_ros.orientation = q;

    
        particles_ros_.poses[i] = pose_ros;

    }
    publish_particlecloud_.publish(particles_ros_);
    
    is_initialized_ = true;
}

void ParticleFilter::prediction(){


}

void ParticleFilter::Visualize(){

    publish_particlecloud_.publish(particles_ros_);
}


}