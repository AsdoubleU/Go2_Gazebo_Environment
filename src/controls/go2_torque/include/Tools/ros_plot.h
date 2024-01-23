#ifndef ROS_PLOT_H_
#define ROS_PLOT_H_

#include <ros/node_handle.h>
#include <std_msgs/Float64.h>
#include <iostream>

// Body Position Publisher
ros::Publisher p_body_pos_x;
ros::Publisher p_body_pos_y;
ros::Publisher p_body_pos_z;

// Body Velocity Publisher
ros::Publisher p_body_vel_x;
ros::Publisher p_body_vel_y;
ros::Publisher p_body_vel_z;

// IMU RPY Publisher
ros::Publisher p_imu_roll;
ros::Publisher p_imu_pitch;
ros::Publisher p_imu_yaw;

// Joint Angle Publisher
ros::Publisher p_fl_hr_q;
ros::Publisher p_fl_hp_q;
ros::Publisher p_fl_kp_q;

ros::Publisher p_fr_hr_q;
ros::Publisher p_fr_hp_q;
ros::Publisher p_fr_kp_q;

ros::Publisher p_rl_hr_q;
ros::Publisher p_rl_hp_q;
ros::Publisher p_rl_kp_q;

ros::Publisher p_rr_hr_q;
ros::Publisher p_rr_hp_q;
ros::Publisher p_rr_kp_q;

// Joint Angular Velocity Publisher
ros::Publisher p_fl_hr_dq;
ros::Publisher p_fl_hp_dq;
ros::Publisher p_fl_kp_dq;

ros::Publisher p_fr_hr_dq;
ros::Publisher p_fr_hp_dq;
ros::Publisher p_fr_kp_dq;

ros::Publisher p_rl_hr_dq;
ros::Publisher p_rl_hp_dq;
ros::Publisher p_rl_kp_dq;

ros::Publisher p_rr_hr_dq;
ros::Publisher p_rr_hp_dq;
ros::Publisher p_rr_kp_dq;

// Ref & Act Foot Position Publisher
ros::Publisher p_fl_ref_pos_x;
ros::Publisher p_fl_ref_pos_y;
ros::Publisher p_fl_ref_pos_z;

ros::Publisher p_fl_act_pos_x;
ros::Publisher p_fl_act_pos_y;
ros::Publisher p_fl_act_pos_z;

ros::Publisher p_fr_ref_pos_x;
ros::Publisher p_fr_ref_pos_y;
ros::Publisher p_fr_ref_pos_z;

ros::Publisher p_fr_act_pos_x;
ros::Publisher p_fr_act_pos_y;
ros::Publisher p_fr_act_pos_z;

ros::Publisher p_rl_ref_pos_x;
ros::Publisher p_rl_ref_pos_y;
ros::Publisher p_rl_ref_pos_z;

ros::Publisher p_rl_act_pos_x;
ros::Publisher p_rl_act_pos_y;
ros::Publisher p_rl_act_pos_z;

ros::Publisher p_rr_ref_pos_x;
ros::Publisher p_rr_ref_pos_y;
ros::Publisher p_rr_ref_pos_z;

ros::Publisher p_rr_act_pos_x;
ros::Publisher p_rr_act_pos_y;
ros::Publisher p_rr_act_pos_z;

// Joint Torque Publisher
ros::Publisher p_fl_hr_torque;
ros::Publisher p_fl_hp_torque;
ros::Publisher p_fl_kp_torque;

ros::Publisher p_fr_hr_torque;
ros::Publisher p_fr_hp_torque;
ros::Publisher p_fr_kp_torque;

ros::Publisher p_rl_hr_torque;
ros::Publisher p_rl_hp_torque;
ros::Publisher p_rl_kp_torque;

ros::Publisher p_rr_hr_torque;
ros::Publisher p_rr_hp_torque;
ros::Publisher p_rr_kp_torque;

// Joint Power Publisher
ros::Publisher p_fl_hr_power;
ros::Publisher p_fl_hp_power;
ros::Publisher p_fl_kp_power;

ros::Publisher p_fr_hr_power;
ros::Publisher p_fr_hp_power;
ros::Publisher p_fr_kp_power;

ros::Publisher p_rl_hr_power;
ros::Publisher p_rl_hp_power;
ros::Publisher p_rl_kp_power;

ros::Publisher p_rr_hr_power;
ros::Publisher p_rr_hp_power;
ros::Publisher p_rr_kp_power;

// Body Position Float64 Message 
std_msgs::Float64 m_body_pos_x;
std_msgs::Float64 m_body_pos_y;
std_msgs::Float64 m_body_pos_z;

// Body Velocity Float64 Message
std_msgs::Float64 m_body_vel_x;
std_msgs::Float64 m_body_vel_y;
std_msgs::Float64 m_body_vel_z;

// IMU Roll, Pitch, Yaw Float64 Message
std_msgs::Float64 m_imu_roll;
std_msgs::Float64 m_imu_pitch;
std_msgs::Float64 m_imu_yaw;

// Joint Angle Float64 Message
std_msgs::Float64 m_fl_hr_q;
std_msgs::Float64 m_fl_hp_q;
std_msgs::Float64 m_fl_kp_q;

std_msgs::Float64 m_fr_hr_q;
std_msgs::Float64 m_fr_hp_q;
std_msgs::Float64 m_fr_kp_q;

std_msgs::Float64 m_rl_hr_q;
std_msgs::Float64 m_rl_hp_q;
std_msgs::Float64 m_rl_kp_q;

std_msgs::Float64 m_rr_hr_q;
std_msgs::Float64 m_rr_hp_q;
std_msgs::Float64 m_rr_kp_q;

// Joint Angular Velocity Float64 Message
std_msgs::Float64 m_fl_hr_dq;
std_msgs::Float64 m_fl_hp_dq;
std_msgs::Float64 m_fl_kp_dq;

std_msgs::Float64 m_fr_hr_dq;
std_msgs::Float64 m_fr_hp_dq;
std_msgs::Float64 m_fr_kp_dq;

std_msgs::Float64 m_rl_hr_dq;
std_msgs::Float64 m_rl_hp_dq;
std_msgs::Float64 m_rl_kp_dq;

std_msgs::Float64 m_rr_hr_dq;
std_msgs::Float64 m_rr_hp_dq;
std_msgs::Float64 m_rr_kp_dq;

// Ref & Act Position Float64 Message
std_msgs::Float64 m_fl_ref_pos_x;
std_msgs::Float64 m_fl_ref_pos_y;
std_msgs::Float64 m_fl_ref_pos_z;

std_msgs::Float64 m_fl_act_pos_x;
std_msgs::Float64 m_fl_act_pos_y;
std_msgs::Float64 m_fl_act_pos_z;

std_msgs::Float64 m_fr_ref_pos_x;
std_msgs::Float64 m_fr_ref_pos_y;
std_msgs::Float64 m_fr_ref_pos_z;

std_msgs::Float64 m_fr_act_pos_x;
std_msgs::Float64 m_fr_act_pos_y;
std_msgs::Float64 m_fr_act_pos_z;

std_msgs::Float64 m_rl_ref_pos_x;
std_msgs::Float64 m_rl_ref_pos_y;
std_msgs::Float64 m_rl_ref_pos_z;

std_msgs::Float64 m_rl_act_pos_x;
std_msgs::Float64 m_rl_act_pos_y;
std_msgs::Float64 m_rl_act_pos_z;

std_msgs::Float64 m_rr_ref_pos_x;
std_msgs::Float64 m_rr_ref_pos_y;
std_msgs::Float64 m_rr_ref_pos_z;

std_msgs::Float64 m_rr_act_pos_x;
std_msgs::Float64 m_rr_act_pos_y;
std_msgs::Float64 m_rr_act_pos_z;

// Joint Torque Float64 Message
std_msgs::Float64 m_fl_hr_torque;
std_msgs::Float64 m_fl_hp_torque;
std_msgs::Float64 m_fl_kp_torque;

std_msgs::Float64 m_fr_hr_torque;
std_msgs::Float64 m_fr_hp_torque;
std_msgs::Float64 m_fr_kp_torque;

std_msgs::Float64 m_rl_hr_torque;
std_msgs::Float64 m_rl_hp_torque;
std_msgs::Float64 m_rl_kp_torque;

std_msgs::Float64 m_rr_hr_torque;
std_msgs::Float64 m_rr_hp_torque;
std_msgs::Float64 m_rr_kp_torque;

// Joint Power Float64 Message
std_msgs::Float64 m_fl_hr_power;
std_msgs::Float64 m_fl_hp_power;
std_msgs::Float64 m_fl_kp_power;

std_msgs::Float64 m_fr_hr_power;
std_msgs::Float64 m_fr_hp_power;
std_msgs::Float64 m_fr_kp_power;

std_msgs::Float64 m_rl_hr_power;
std_msgs::Float64 m_rl_hp_power;
std_msgs::Float64 m_rl_kp_power;

std_msgs::Float64 m_rr_hr_power;
std_msgs::Float64 m_rr_hp_power;
std_msgs::Float64 m_rr_kp_power;

#endif