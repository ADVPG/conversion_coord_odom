#include "ros/ros.h"
#include "conversion_coord_odom/CoordToOdom.h"
#include <cstdlib>

#include <nav_msgs/Odometry.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "coord_to_odom_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<conversion_coord_odom::CoordToOdom>("coord_to_odom");
  conversion_coord_odom::CoordToOdom srv;

  srv.request.lat = 38.3849872;
  srv.request.lon = -0.5148027;
  srv.request.gnss_lat = 38.3847222222;
  srv.request.gnss_lon = -0.5148083;

  if (client.call(srv))
  {
    ROS_INFO("odom_x: %f", srv.response.x);
    ROS_INFO("odom_y: %f", srv.response.y);
  }
  else
  {
    ROS_ERROR("Failed to call service coord_to_odom");
    return 1;
  }

  return 0;
}
