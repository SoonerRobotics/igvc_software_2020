#include <ros/ros.h>
#include "igvc_msgs/lane_error.h"
#include "igvc_msgs/vel.h"
#include "igvc_nav/robotlib/PIDController.h"

// Constants
#define DEFAULT_SPEED 2
#define HDG_KP (float)(0.4)
#define HDG_KI (float)(0.001)
#define HDG_KD (float)(0.002)
#define MAX_QUEUE 10


// Publishers
ros::Publisher vel_output;  // velocity command sent to the motion node (Arduino/Nucleo)


//PID Control
PIDController hdg_control;  // heading PID (for now, this is just turn speed control)


/**
 * @brief Callback for handling the lane error message from upstream
 * 
 * @param lane_error the deviation from the center of the lane
 */
void lane_error_cb(const igvc_msgs::lane_error::ConstPtr& lane_error)
{
    // Declare local variables
    float err;
    igvc_msgs::vel motion_cmd;
    float turn_output;

    // Get the error from the lane detection node 
    err = lane_error->error;

    // Apply the PID to reduce the error
    turn_output = hdg_control.getOutput(err, 0.0);

    // Make a vel msg with the correct speed and heading control
    motion_cmd.speed = DEFAULT_SPEED;
    motion_cmd.hdg = turn_output;

    // Publish the motion command
    vel_output.publish(motion_cmd);
}


/**
 * @brief Main function for the navigation node
 */
int main(int argc, char **argv)
{
    // Initialize this node
    ros::init(argc, argv, "nav_node");

    // Create a node handle for the nav node
    ros::NodeHandle nav_node;

    // Publishers
    vel_output = nav_node.advertise<igvc_msgs::vel>(nav_node.resolveName("/nav/motion_cmd"), MAX_QUEUE);

    // Subscribers
    ros::Subscriber lane_error_sub = nav_node.subscribe(nav_node.resolveName("/igvc/lane_deviation"), MAX_QUEUE, &lane_error_cb);

    // Initialize PID
    hdg_control.initialize(0.0, HDG_KP, HDG_KI, HDG_KD);

    // Pump ROS callbacks as they come in
    ros::spin();

    //Exit with error code 0
    return 0;
}