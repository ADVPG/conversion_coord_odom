#include "ros/ros.h"
#include "conversion_coord_odom/OdomToCoord.h"
#include <cstdlib>

#include <nav_msgs/Odometry.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_to_coord_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<conversion_coord_odom::OdomToCoord>("odom_to_coord");
  conversion_coord_odom::OdomToCoord srv;

  boost::shared_ptr<nav_msgs::Odometry const> odom_ptr;
  nav_msgs::Odometry odom_var;

  odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");

  if(odom_ptr != NULL){
    odom_var = *odom_ptr;
  }

  ROS_INFO("odom_x: %f", odom_var.pose.pose.position.x);
  ROS_INFO("odom_y: %f", odom_var.pose.pose.position.y);

  srv.request.x = odom_var.pose.pose.position.x;
  srv.request.y = odom_var.pose.pose.position.y;
  srv.request.gnss_lat = 38.3847222222;
  srv.request.gnss_lon = -0.5148083;

  if (client.call(srv))
  {
    ROS_INFO("Lat: %f", srv.response.lat);
    ROS_INFO("Lon: %f", srv.response.lon);
  }
  else
  {
    ROS_ERROR("Failed to call service odom_to_coord");
    return 1;
  }

  return 0;
}
