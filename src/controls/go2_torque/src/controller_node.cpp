#include "controller.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "go2_torque_node");

    ros::NodeHandle nh;
    
    // Parameters
    double frequency=500;

    std::string topic_leg_state;
    std::string topic_leg_command;
    std::string topic_body_state;
    std::string topic_imu;

    // LOADING PARAMETERS FROM THE ROS SERVER 

    // Topic names
    if (!nh.getParam("topic_leg_state", topic_leg_state)) { ROS_ERROR("Couldn't retrieve the topic name for the state of the leg."); return -1; }
    if (!nh.getParam("topic_leg_command", topic_leg_command)) { ROS_ERROR("Couldn't retrieve the topic name for commanding the leg."); return -1; }
    if (!nh.getParam("topic_body_state", topic_body_state)) { ROS_ERROR("Couldn't retrieve the topic name for the state of the body."); return -1; }
    if (!nh.getParam("topic_imu", topic_imu)) { ROS_ERROR("Couldn't retrieve the topic name for the imu sensor data."); return -1; }

    controller controller;

    controller.init(
        nh,
        topic_leg_state,
        topic_leg_command,
        topic_body_state,
        topic_imu,
        frequency);

    controller.run();

    return 0;
}