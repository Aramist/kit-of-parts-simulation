#include <functional>
#include <vector>

#include "ros/ros.h"
#include "ros/console.h"

#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"


using geometry_msgs::Vector3;


void commandCallback(const Vector3::ConstPtr &command, const double &d, const double &r,
                     const std::vector<ros::Publisher> &leftSide,
                     const std::vector<ros::Publisher> &rightSide) {
    double velocity = command->x;
    double angular = command->y;

    double left = -(2 * velocity - d * angular) / (2 * r);  // The negative is there to invert the motors
    double right = -(2 * velocity + d * angular) / (2 * r);

    for (const ros::Publisher &wheel : leftSide) {
        std_msgs::Float64 msg;
        msg.data = left;
        wheel.publish(msg);
    }


    for (const ros::Publisher &wheel : rightSide) {
        std_msgs::Float64 msg;
        msg.data = right;
        wheel.publish(msg);
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "kop_interface");
    ros::NodeHandle handle;

    double baseWidth = 0.0;
    double wheelRadius = 0.0;
    if (!(handle.getParam("robot_params/wheelbase_width", baseWidth)
          && handle.getParam("robot_params/wheel_radius", wheelRadius))) {
        ROS_ERROR("Failed to read radius and wheelbase width parameters from parameter server");
        return -1;
    }


    std::vector<ros::Publisher> rightPubs;
    rightPubs.push_back(handle.advertise<std_msgs::Float64>(
            "/kop/pid/rf/setpoint", 5));
    rightPubs.push_back(handle.advertise<std_msgs::Float64>(
            "/kop/pid/rm/setpoint", 5));
    rightPubs.push_back(handle.advertise<std_msgs::Float64>(
            "/kop/pid/rr/setpoint", 5));


    std::vector<ros::Publisher> leftPubs;
    leftPubs.push_back(handle.advertise<std_msgs::Float64>(
            "/kop/pid/lf/setpoint", 5));
    leftPubs.push_back(handle.advertise<std_msgs::Float64>(
            "/kop/pid/lm/setpoint", 5));
    leftPubs.push_back(handle.advertise<std_msgs::Float64>(
            "/kop/pid/lr/setpoint", 5));

    ros::Subscriber command = handle.subscribe<Vector3>(
            "/kop/interface/drive_command", 1,
            std::bind(&commandCallback, std::placeholders::_1,
                      baseWidth, wheelRadius, leftPubs, rightPubs));

    ros::spin();
}
