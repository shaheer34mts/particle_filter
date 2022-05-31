#include <nodelet/nodelet.h>
#include <string>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "particle_filter.h"

namespace particle_filter {

class ParticleFilterLocalizationNodelet : public nodelet::Nodelet {


public:

  ParticleFilterLocalizationNodelet() {}
  virtual ~ParticleFilterLocalizationNodelet() {}

  virtual void onInit() {
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();

    map_frame_id = private_nh.param<std::string>("map_frame_id", "map");

    publish_particlecloud_ = nh.advertise<geometry_msgs::PoseArray>("particle_cloud", 1, true);
    particle_filter::ParticleFilter pf (100, nh);
    double std[7] ={6, 12, 1, 0.9, 0.9, 0.9, 0.9};
    pf.init(std);
    //pf.Visualize();
  }

  private:






private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  //Subsribers
  ros::Subscriber odom_sub;
  

  std::string map_frame_id;

 


  ros::Publisher publish_particlecloud_;
  // geometry_msgs::PoseArray particles_ros_;
  // geometry_msgs::PoseStamped robot_ros_;

};

}


PLUGINLIB_EXPORT_CLASS(particle_filter::ParticleFilterLocalizationNodelet, nodelet::Nodelet)