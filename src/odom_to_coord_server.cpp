#include "ros/ros.h"
#include "conversion_coord_odom/OdomToCoord.h"

#include <nav_msgs/Odometry.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "robot_localization/navsat_conversions.h"

bool odom_to_utm(conversion_coord_odom::OdomToCoord::Request  &req,
                 conversion_coord_odom::OdomToCoord::Response &res)
{
  
  // Get odometry as pose 

  geometry_msgs::Pose robot_pose;
  robot_pose.position.x = req.x;
  robot_pose.position.y = req.y;

  // Transform pose from odom to utm

  //ROS_INFO("odom_x: %f", robot_pose.position.x);
  //ROS_INFO("odom_y: %f", robot_pose.position.y);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener(tf_buffer);
  geometry_msgs::TransformStamped odom_to_utm;

  try{
    odom_to_utm = tf_buffer.lookupTransform("utm", "odom", ros::Time(0), ros::Duration(1.0) );
  }
  catch (tf2::TransformException &ex) {
    ROS_ERROR("Could NOT transform odom to utm");
    return false;
  }

  tf2::doTransform(robot_pose, robot_pose, odom_to_utm);

  //ROS_INFO("utm_x: %f", robot_pose.position.x);
  //ROS_INFO("utm_y: %f", robot_pose.position.y);

  // Get the utm zone of the robot which is not provided by the tf

  std::string robot_utm_zone;
  double northing, easting; // this variables are not actually needed, but the function requires them to be set
  RobotLocalization::NavsatConversions::LLtoUTM(req.gnss_lat, req.gnss_lon, northing, easting, robot_utm_zone);
  //ROS_INFO("robot_utm_zone: %s", robot_utm_zone.c_str());

  // Transform utm pose to lat lon coordinates

  RobotLocalization::NavsatConversions::UTMtoLL (robot_pose.position.y, robot_pose.position.x, robot_utm_zone, res.lat, res.lon);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_to_coord_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("odom_to_coord", odom_to_utm);
  ROS_INFO("Ready to convert from odom to coord.");
  ros::spin();

  return 0;
}
