#include<iostream>
#include "particle_filter.h"
#include <ros/ros.h>
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseArray.h"
#include "tf/tf.h"
#include "stdlib.h"


int main(int argc, char** argv){
    
    ros::init(argc, argv,"localization_node");
    ros::NodeHandle n;
    int n_p = 100;
    double std[7] ={6, 12, 1, 0.9, 0.9, 0.9, 0.9}; 
    particle_filter::ParticleFilter pf(n_p, n);
    pf.init(std);
    while(ros::ok()){
        pf.Visualize();
    }
    








    return 0;
}