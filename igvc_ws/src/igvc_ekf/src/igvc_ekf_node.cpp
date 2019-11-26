#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <fstream>

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <ros/package.h>

#include "igvc_ekf/ekf.h"
#include "igvc_msgs/ekf_state.h"
#include "igvc_msgs/gps.h"
#include "igvc_msgs/imuodom.h"
#include "igvc_msgs/velocity.h"

// Init the EKF
EKF ekf;

// State and control vectors
Eigen::VectorXd x(11), u(2);

// Measurement vector
Eigen::VectorXd z(9);
double last_heading = -1010;

ros::Publisher output_pub;

uint16_t data_init = 0;

// Logging
std::ofstream gps_file;
std::ofstream vel_file;
std::ofstream accel_file;
std::ofstream hdg_file;


void updateEKF(const ros::TimerEvent& time_event)
{
    // If all sensor data has been received once
    if(data_init & 0xF)
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
}



/******
 * Helper functions
 ******/


float constrainAngle(float x){
    x = std::fmod(x + 180,360);
    if (x < 0)
        x += 360;
    return x - 180;
}

float angleDiff(float a,float b){
    float dif = std::fmod(b - a + 180,360);
    if (dif < 0)
        dif += 360;
    return dif - 180;
}



/******
 * Measurement callbacks
 ******/


void updateGPS(const igvc_msgs::gps::ConstPtr& gps_msg)
{
    std::string data = std::to_string(gps_msg->latitude) + ", " + std::to_string(gps_msg->longitude) + "\n";
    gps_file << data;

    // Update GPS coords
    z(0) = degreesToRadians(gps_msg->latitude);   // Latitude
    z(1) = degreesToRadians(gps_msg->longitude);  // Longitude

    // Show the GPS has been initialized
    data_init |= 1;
}

void updateVelocity(const igvc_msgs::velocity::ConstPtr& vel_msg)
{
    // Update measurement
    z(2) = (vel_msg->leftVel + vel_msg->rightVel) / 2.0;
    z(7) = vel_msg->leftVel;
    z(8) = vel_msg->rightVel;

    // Update control through hacks
    u(0) = vel_msg->leftVel;
    u(1) = vel_msg->rightVel;

    // Show that the velocity has been updated
    data_init |= (1 << 1);

    std::string data = std::to_string(z(2)) + "\n";
    vel_file << data;
}

void updateIMU(const igvc_msgs::imuodom::ConstPtr& imu_msg)
{
    z(3) = imu_msg->acceleration;

    // Show that the acceleration and heading have been updated
    data_init |= (1 << 2) | (1 << 3);

    if(last_heading < -1000)
    {
        last_heading = imu_msg->heading;
    }

    z(6) = degreesToRadians(angleDiff(imu_msg->heading, last_heading)) / 0.02;
    z(5) += degreesToRadians(angleDiff(imu_msg->heading, last_heading)); // Update local heading from change to global heading
    z(4) = degreesToRadians(imu_msg->heading);                 // Update global heading
    last_heading = imu_msg->heading;

    // Update the heading file
    std::string data = std::to_string(z(4)) + ", " + std::to_string(z(5)) + "\n";
    hdg_file << data;

    // Update the accel file
    data = std::to_string(z(3)) + "\n";
    accel_file << data;
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "igvc_ekf_node");

    ros::NodeHandle ekf_node;

    // Initialize the measurement vector
    z.resize(9);
    z.setZero(9);

    // Initialize the subscribers
    ros::Subscriber imu_sub = ekf_node.subscribe(ekf_node.resolveName("/igvc/imu"), 10, &updateIMU);
    ros::Subscriber vel_sub = ekf_node.subscribe(ekf_node.resolveName("/igvc/velocity"), 10, &updateVelocity);
    ros::Subscriber gps_sub = ekf_node.subscribe(ekf_node.resolveName("/igvc/gps"), 10, &updateGPS);

    // Initialize logging
    std::string path = ros::package::getPath("igvc_ekf");

    gps_file.open(path + "/data/gps_log.csv");
    vel_file.open(path + "/data/vel_log.csv");
    accel_file.open(path + "/data/accel_log.csv");
    hdg_file.open(path + "/data/hdg_log.csv");

    gps_file << "lat, lon\n";

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