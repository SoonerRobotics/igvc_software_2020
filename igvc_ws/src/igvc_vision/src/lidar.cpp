#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

ros::Publisher map_pub;
//CONSTANTS
#define MAX_DISTANCE 10 // Max distance of 10 meters
#define RESOLUTION 0.25 // Resolution of map (meters)

struct Obstacle
{
  // Polar (Native from LiDAR)
  float angle;
  float distance;
  // Cartesian (For map grid)
  float x;
  float y;
};

/**
 * @brief
 *
 * @param msg
 */
void onLidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  //Instantiate message to publish to
  nav_msgs::OccupancyGrid map_msg;
  //Instantiate variables for obstacle detection
  std::vector<Obstacle> obstacles;
  std::vector<float> ranges = msg->ranges;
  bool obstacle_detected = false;
  Obstacle ob;

  /************************************************************************************************\
   * Currently this for loop goes through the vector of ranges and checks for obstacles. When an
   * obstacle is detected the closest distance of the object is updated and the angle is updated to
   * the corresponding indice of this closest distance divided by two, which corresponds to the
   * correct angle. The angles increment upwards counterclockwise. If the object varies by more than
   * +/- 0.25 meters, then we treat this as a new obstacle.
   \************************************************************************************************/
  for (int i=0; i < ranges.size(); i++)
  {
    //If an object is detected and hasn't already been initialized
    if (ranges[i] > 0 && ranges[i] <= MAX_DISTANCE && !obstacle_detected) {
      obstacle_detected = true;
      ob.angle = i/2.0;
      ob.distance = ranges[i];
    }
    //If an object is detected and has been iniitalized
    else if(ranges[i] > 0 && ranges[i] <= MAX_DISTANCE && obstacle_detected)
    {
      //Check that the distance is still within +/- 0.25 meters, otherwise create a new object
      if(abs(ranges[i] - ob.distance) < OBS_DST_DELTA)
      {
        //Update distance to closest distance
        ob.distance = (ob.distance < ranges[i]) ? ob.distance : ranges[i];
        //Update angle to closest distance angle
        ob.angle = (ob.distance < ranges[i]) ? ob.angle : i/2.0;
      }
      else
      {
        //Push the previous object
        obs.push_back(ob);
        //Begin the new object
        ob.angle = i/2.0;
        ob.distance = ranges[i];
      }
    }
    //If an object that was initialized is no longer detected
    else if((ranges[i] == 0 || ranges[i] > MAX_DISTANCE) && obstacle_detected)
    {
      //Update boolean and push the object to the vector
      obstacle_detected = false;
      obs.push_back(ob);
    }
  }

  //Update the obstacle message with all obstacles found
  for (int i=0; i < obs.size(); i++) {
    ob_msg.angles.push_back(obs.at(i).angle);
    ob_msg.distances.push_back(obs.at(i).distance);
  }

  //Publish message data to the topic
  map_pub.publish(ob_msg);
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
