#ifndef CONTROLLER_H_
#define CONTROLLER_H_
#include "pinocchio_interface.hpp"
#include "RobotLeg.h"

#include <ros/node_handle.h>
#include <std_srvs/Empty.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/Wrench.h"
#include "std_msgs/Float64MultiArray.h"
#include <sensor_msgs/JointState.h>
#include <iostream>
#include "TrajectoryGenerator.h"
#include "ros_plot.h"

NumericalTool::LowPassFilter rms_LPF[Leg_N];
NumericalTool::LowPassFilter torque_LPF[Leg_N][DoF];

enum ControlMode
{
    INIT,
    HOMING,
    STANDING,
    MOTION,
    FINISH = 6
};

enum QUATERNION
{
    Q1,
    Q2,
    Q3,
    Q0
};

class controller
{
public:
    controller() {}
    virtual ~controller() {}

    void init(ros::NodeHandle &nh,
              std::string topic_leg_state,
              std::string topic_leg_command,
              std::string topic_body_state,
              std::string topic_imu,
              const double frequency);
    void run();

private:
    // Subscriber callback functions
    void state_leg_callback(const sensor_msgs::JointState &state);
    void state_body_callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void imu_sensor_callback(const sensor_msgs::Imu::ConstPtr& msg);

    void command(bool flag);

    void send_commands_to_robot();

    void print();

    void rqt_plot();
    void init_rqt();

    Eigen::Vector3d GetActBodyPos();

protected:
    double control_period;
    double control_time;
    ros::NodeHandle nh_;

    //  Subscribers:
    ros::Subscriber sub_leg_state_;
    ros::Subscriber sub_body_state_, sub_imu_;
    Eigen::VectorXd total_leg_q_, total_leg_dq_;

    //  Publisher:
    ros::Publisher pub_leg_cmd_;

    // Robot Parameter:
    int init_size;
    double force_sum = 0;
    Eigen::Vector3d body_pos_ ,body_vel_, orientation_, angular_velocity_, linear_acceleration_;
    Eigen::VectorXd q_, dq_, torque_;
    Eigen::Quaterniond quaternion_;
    Eigen::Matrix3d rotation_matrix_;
    double tmp_Kp, tmp_Kd, tmp_rKp, tmp_rKd, homing_period;
    Eigen::VectorXd homing_ref_q, homing_Kp, homing_Kd;
    double pi, rad2deg, deg2rad;

    Eigen::VectorXd walking_init_x_[Leg_N];

    ControlMode controlmode;

    bool Recieved_Joint_State;
    bool Recieved_Body_State;
    bool Recieved_Mode[10];
    
    // plot variables
private:
    ros::NodeHandle nh_plot;
};

#endif