#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

ros::Publisher map_pub;
//CONSTANTS
#define MAX_DISTANCE 10 // Max distance of 10 meters
#define RESOLUTION 0.25 // Resolution of map (meters)


/**
 * @brief
 *
 * @param msg
 */
void onLidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //Instantiate message to publish to
  nav_msgs::OccupancyGrid map_msg;
  //Instantiate variables for obstacle detection
  float angle;
  float distance;
  std::vector<float> ranges = msg->ranges;

  for (int i=0; i < ranges.size(); i++)
  {
    // If something is detected within range
    if (ranges[i] > 0 && ranges[i] <= MAX_DISTANCE) 
    {
      // TODO: Convert polar coordinates provided from LiDAR to cartesian and occupy map
      angle = i/2.0;
      distance = ranges[i];
    }
  }

  // Publish message data to the topic
  map_pub.publish(map_msh);
}

/**
 * @brief
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char** argv)
{
  //Initialize the node
  ros::init(argc, argv, "lidar_map_node");
  //Set up node
  ros::NodeHandle lidar_map_node;
  //Create the publisher for the map
  map_pub = lidar_map_node.advertise<nav_msgs::OccupancyGrid>(obstacle_node.resolveName("/igvc_vision/map"), 10);
  //Create the subscriber for the LiDAR
  ros::Subscriber lidar = obstacle_node.subscribe(obstacle_node.resolveName("/scan"), 10, onLidarCallback);
  //Automatically handles callbacks
  ros::spin();
}
