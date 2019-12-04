#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include "igvc_ekf/ekf.h"
#include "igvc_msgs/ekf_state.h"

// Init the EKF
EKF ekf;

// State and control vectors
Eigen::VectorXd x, u;

ros::Publisher output_pub;

void updateEKF(const ros::TimerEvent& time_event)
{
    // HACK: currently no sensor data is here, so using "u" as a placeholder
    x = ekf.run_filter(u, u);

    igvc_msgs::ekf_state state;

    for (int i = 0; i < 11; ++i)
    {
        state.x_k[i] = x(i);
    }

    output_pub.publish(state);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "igvc_ekf_node");

    ros::NodeHandle ekf_node;

    // Initialize the state and control vectors
    x.resize(11);
    u.resize(2);
    x << degreesToRadians(35.210467), degreesToRadians(-97.441811), degreesToRadians(90), 0, 0, degreesToRadians(0), 0, 0, 0, 0, 0;
    u << 6.3, 3.14;

    // Initialize the EKF
    ekf.init(x);

    output_pub = ekf_node.advertise<igvc_msgs::ekf_state>(ekf_node.resolveName("/igvc_ekf/filter_output"), 10);

    ros::Timer ekf_update = ekf_node.createTimer(ros::Duration(0.02), &updateEKF, false);

    ros::spin();

    return 0;
}