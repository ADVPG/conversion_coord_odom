#include "ros/ros.h"
#include "conversion_coord_odom/CoordToOdom.h"

#include <nav_msgs/Odometry.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "robot_localization/navsat_conversions.h"

bool utm_to_odom(conversion_coord_odom::CoordToOdom::Request  &req,
                 conversion_coord_odom::CoordToOdom::Response &res)
{
  // Transform lat lon coordinates to utm pose

  geometry_msgs::Pose robot_pose;
  
  std::string marker_utm_zone;
  RobotLocalization::NavsatConversions::LLtoUTM(req.lat, req.lon, robot_pose.position.y, robot_pose.position.x, marker_utm_zone);
  //ROS_INFO("marker_utm_zone: %s", marker_utm_zone.c_str());


  // Check that the utm zone of the robot and the marker coincide

  std::string robot_utm_zone;
  double northing, easting; // this variables are not actually needed, but the function requires them to be set
  RobotLocalization::NavsatConversions::LLtoUTM(req.gnss_lat, req.gnss_lon, northing, easting, robot_utm_zone);
  //ROS_INFO("robot_utm_zone: %s", robot_utm_zone.c_str());
  
  if(marker_utm_zone != robot_utm_zone){
    ROS_ERROR("Goal point is outside robot's UTM zone");
    return false;
  }


  // Transform pose from utm to odom

  //ROS_INFO("utm_x: %f", robot_pose.position.x);
  //ROS_INFO("utm_y: %f", robot_pose.position.y);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener(tf_buffer);
  geometry_msgs::TransformStamped utm_to_odom;

  try{
    utm_to_odom = tf_buffer.lookupTransform("odom", "utm", ros::Time(0), ros::Duration(1.0) );
  }
  catch (tf2::TransformException &ex) {
    ROS_ERROR("Could NOT transform utm to odom");
    return false;
  }

  tf2::doTransform(robot_pose, robot_pose, utm_to_odom);

  //ROS_INFO("odom_x: %f", robot_pose.position.x);
  //ROS_INFO("odom_y: %f", robot_pose.position.y);


  // Set odometry pose

  res.x = robot_pose.position.x;
  res.y = robot_pose.position.y;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "coord_to_odom_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("coord_to_odom", utm_to_odom);
  ROS_INFO("Ready to convert from coord to odom.");
  ros::spin();

  return 0;
}
