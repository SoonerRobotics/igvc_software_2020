#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>

#include "igvc_ekf/ekf.h"
#include "igvc_msgs/ekf_state.h"

// Init the EKF
EKF ekf;

// State and control vectors
Eigen::VectorXd x, u;

// Measurement vector
Eigen::VectorXd z;
double last_heading = -1010;

// Hash table for measurements
std::vector<bool> sensor_updates(6);

ros::Publisher output_pub;




void updateEKF(const ros::TimerEvent& time_event)
{
    // EKF state message
    igvc_msgs::ekf_state state;

    // Update the extended kalman filter
    x = ekf.run_filter(z, u);

    // Construct the state vector to publish to other modules
    for (int i = 0; i < 11; ++i)
    {
        state.x_k[i] = x(i);
    }

    // Publish predicted state
    output_pub.publish(state);
}


/******
 * Measurement callbacks
 ******/


void updateGPS(const geometry_msgs::Vector3::ConstPtr& gps_msg)
{
    // Update GPS coords
    z(0) = degreesToRadians(gps_msg->x);  // Latitude
    z(1) = degreesToRadians(gps_msg->y);  // Longitude
}

void updateVelocity(const std_msgs::Float64::ConstPtr& vel_msg)
{
    z(2) = vel_msg->data;
}

void updateAccel(const std_msgs::Float64::ConstPtr& accel_msg)
{
    z(3) = accel_msg->data;
}

void updateHeading(const std_msgs::Float64::ConstPtr& heading_msg)
{
    if(last_heading < -1000)
    {
        last_heading = heading_msg->data;
    }

    z(5) += heading_msg->data - last_heading; // Update local heading from change to global heading
    z(4) = heading_msg->data;                 // Update global heading
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "igvc_ekf_node");

    ros::NodeHandle ekf_node;

    // Initialize the measurement vector
    z.resize(6);
    z.setZero(6);

    // Initialize the subscribers
    ros::Subscriber accel_sub = ekf_node.subscribe(ekf_node.resolveName("/igvc/acceleration"), 10, &updateAccel);
    ros::Subscriber heading_sub = ekf_node.subscribe(ekf_node.resolveName("/igvc/heading"), 10, &updateHeading);
    ros::Subscriber vel_sub = ekf_node.subscribe(ekf_node.resolveName("/igvc/velocity"), 10, &updateVelocity);
    ros::Subscriber gps_sub = ekf_node.subscribe(ekf_node.resolveName("/igvc/gps"), 10, &updateGPS);

    // Initialize the state and control vectors
    x.resize(11);
    u.resize(2);
    x << degreesToRadians(35.194881), degreesToRadians(-97.438621), degreesToRadians(0), 0, 0, degreesToRadians(0), 0, 0, 0, 0, 0;
    u << 0, 0; //6.3, 3.14;

    // Initialize the EKF
    ekf.init(x);

    // Publishers
    output_pub = ekf_node.advertise<igvc_msgs::ekf_state>(ekf_node.resolveName("/igvc_ekf/filter_output"), 10);

    // Timers
    ros::Timer ekf_update = ekf_node.createTimer(ros::Duration(0.02), &updateEKF, false);

    ros::spin();

    return 0;
}