#include <functional>
#include <vector>

#include "ros/ros.h"

#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"


const std::string joints[]{"lf", "lm", "lr", "rf", "rm", "rr"};


void publishVelocities(const sensor_msgs::JointState::ConstPtr &state, const std::vector<ros::Publisher> &floatPubs) {
    for(size_t i = 0; i < 6; i++){
        std_msgs::Float64 velState;
        velState.data = state->velocity[i];
        floatPubs[i].publish(velState);
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "velocity_publisher");
    ros::NodeHandle handle;


    std::vector<ros::Publisher> floatPubs;
    for (const std::string &joint : joints) {
        const std::string fullName = "/kop/pid/" + joint + "/state";
        floatPubs.push_back(handle.advertise<std_msgs::Float64>(fullName, 1));
    }

    ros::Subscriber stateListener = handle.subscribe<sensor_msgs::JointState>("/kop/joint_states", 1,
                                                                        std::bind(&publishVelocities,
                                                                                  std::placeholders::_1, floatPubs));
    ros::spin();
}
