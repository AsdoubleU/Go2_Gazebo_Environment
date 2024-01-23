#include "controller.h"
#include <math.h>
using namespace std;

std::string urdf_filename = "/home/simon/go2_ws/src/robots/go2_description/urdf/go2_description.urdf";

std::vector<std::string> foot_name = {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};

PinocchioInterface pin(urdf_filename, foot_name);
RobotLeg robotleg[Leg_N];
TrajectoryGenerator traj[Leg_N];

void controller::init(ros::NodeHandle &nh,
                      std::string topic_leg_state,
                      std::string topic_leg_command,
                      std::string topic_body_state,
                      std::string topic_imu,
                      const double frequency)
{
    // Reset Gazebo Simualtion
    system("clear");
    std_srvs::Empty reset;
    ros::service::call("/gazebo/reset_simulation", reset);
    std::cout << "Reset Gazebo Simulation" << std::endl;
    nh_.ok();

    uint32_t queue_size = 1;

    // Subscribers
    sub_leg_state_ = nh_.subscribe(topic_leg_state, queue_size, &controller::state_leg_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_body_state_ = nh_.subscribe(topic_body_state, queue_size, &controller::state_body_callback,this, ros::TransportHints().reliable().tcpNoDelay());
    sub_imu_ = nh_.subscribe(topic_imu, queue_size, &controller::imu_sensor_callback,this, ros::TransportHints().reliable().tcpNoDelay());

    // Publisher
    pub_leg_cmd_ = nh_.advertise<std_msgs::Float64MultiArray>(topic_leg_command, queue_size);

    control_period = (1.0 / frequency);
    control_time = 0;

    // init size
    init_size = DoF * Leg_N;
    q_.setZero(init_size);
    dq_.setZero(init_size);
    torque_.setZero(init_size);

    pi = M_PI;
    deg2rad = pi / 180.0;
    rad2deg = 180.0 / pi;

    controlmode = INIT;
    Recieved_Joint_State = false;

    homing_ref_q.setZero(DoF);
    homing_Kp.setZero(DoF);
    homing_Kd.setZero(DoF);
    for (size_t i = 0; i < Leg_N; i++)
    {
        walking_init_x_[i].setZero(DoF);
    }

    init_rqt();

    for (size_t i = 0; i < Leg_N; i++)
    {
        traj[i].init(control_period);
        robotleg[i].init(control_period);
    }
}

void controller::state_leg_callback(const sensor_msgs::JointState &state)
{
    // HR, HP, KP
    for (size_t i = 0; i < Leg_N; i++)
    {
        q_(DoF*i) = state.position[DoF*i+1];
        q_(DoF*i+1) = state.position[DoF*i+2];
        q_(DoF*i+2) = state.position[DoF*i];

        dq_(DoF*i) = state.velocity[DoF*i+1];
        dq_(DoF*i+1) = state.velocity[DoF*i+2];
        dq_(DoF*i+2) = state.velocity[DoF*i];

        Recieved_Joint_State = true;
    }
}

void controller::state_body_callback(const gazebo_msgs::ModelStates::ConstPtr& msg) 
{
    std::vector<std::string> modelNames = msg->name;
    std::vector<geometry_msgs::Pose> modelPoses = msg->pose;
    std::vector<geometry_msgs::Twist> modelTwists = msg->twist;

    std::string modelName = "go2";
    int modelIndex = -1;

    for (size_t i = 0; i < modelNames.size(); ++i) {
        if (modelNames[i] == modelName) {
            modelIndex = i;
            break;
        }
    }

    if (modelIndex >= 0 && modelIndex < modelPoses.size() && modelIndex < modelTwists.size()) {
        geometry_msgs::Pose modelPose = modelPoses[modelIndex];
        geometry_msgs::Twist modelTwist = modelTwists[modelIndex];

        body_pos_(X) = modelPose.position.x;
        body_pos_(Y) = modelPose.position.y;
        body_pos_(Z) = modelPose.position.z;

        body_vel_(X) = modelTwist.linear.x;
        body_vel_(Y) = modelTwist.linear.y;
        body_vel_(Z) = modelTwist.linear.z;
    }
    Recieved_Body_State = true;
}

void controller::imu_sensor_callback(const sensor_msgs::Imu::ConstPtr& msg)
{

    // Quaternion
    Eigen::Quaterniond quat(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    quaternion_ = quat;

    // Angular Velocity [rad/s]
    angular_velocity_(X) = msg->angular_velocity.x; 
    angular_velocity_(Y) = msg->angular_velocity.y;
    angular_velocity_(Z) = msg->angular_velocity.z;

    // Linear Accceleration [m/s^2]
    linear_acceleration_(X) = msg->linear_acceleration.x;
    linear_acceleration_(Y) = msg->linear_acceleration.y;
    linear_acceleration_(Z) = msg->linear_acceleration.z;

    // Rotation Matrix
    rotation_matrix_ = quat.toRotationMatrix();

    // Euler Angle [rad]
    orientation_(Y) = std::asin(-rotation_matrix_(2, 0));
    if (std::cos(orientation_(1)) != 0) { // Check for edge cases
        orientation_(X) = std::atan2(rotation_matrix_(2, 1), rotation_matrix_(2, 2));
        orientation_(Z) = std::atan2(rotation_matrix_(1, 0), rotation_matrix_(0, 0));
    } 
    else { // Gimbal lock case (cos(pitch) == 0),  In this case, we can set roll to 0 and calculate yaw differently
        orientation_(X) = 0.0;
        orientation_(Z) = std::atan2(-rotation_matrix_(0, 1), rotation_matrix_(1, 1));
    }
    
}

void controller::command(bool flag)
{
    if (flag)
    {
        // Pinocchio
        pin.SetRobotParameter(q_, dq_);

        for (size_t i = 0; i < Leg_N; i++)
        {
            pin.SetKinematics(i);
        }

        switch (controlmode)
        {
        case INIT:
            if (!Recieved_Mode[INIT])
            {
                ROS_INFO("INIT MODE");
                Recieved_Mode[INIT] = true;
                Recieved_Mode[HOMING] = false;
                Recieved_Mode[STANDING] = false;
                Recieved_Mode[MOTION] = false;
                Recieved_Mode[FINISH] = false;
                controlmode = HOMING;
            }
            break;

        case HOMING:
        {
            if (!Recieved_Mode[HOMING])
            {
                ROS_INFO("HOMING MODE");
                for (size_t i = 0; i < Leg_N; i++)
                {
                    robotleg[i].isLegInit = false;
                    traj[i].isEnd = false;
                    traj[i].t_ = 0.0;
                }

                homing_ref_q << 0 * deg2rad, 90 * deg2rad, -180 * deg2rad;

                homing_Kp << 100,60,10;
                homing_Kd << 1,0.1,0.1;

                homing_period = 3.0;

                Recieved_Mode[INIT] = false;
                Recieved_Mode[HOMING] = true;
                Recieved_Mode[STANDING] = false;
                Recieved_Mode[MOTION] = false;
                Recieved_Mode[FINISH] = false;
            }

            for (size_t i = 0; i < Leg_N; i++)
            {
                robotleg[i].SetLegInit(q_.block<DoF, 1>(i * DoF, 0));

                traj[i].SetSinusoidalTrajectory(homing_ref_q, robotleg[i].GetInitQ(), homing_period);

                robotleg[i].SetPDControl_q(homing_Kp, homing_Kd, traj[i].GetRefVar(), q_.block<DoF, 1>(i * DoF, 0), traj[i].GetRefVarDDot(), dq_.block<DoF, 1>(i * DoF, 0));

                for (size_t j = 0; j < DoF; j++)
                {
                    torque_(i * DoF + j) = robotleg[i].GetJointTorque()(j);
                }

                if (traj[Leg_N - 1].isEnd)
                {
                    controlmode = STANDING;
                }
            }
            break;
        }

        case STANDING:
        {
            if (!Recieved_Mode[STANDING])
            {
                ROS_INFO("STANDING MODE");
                for (size_t i = 0; i < Leg_N; i++)
                {
                    robotleg[i].isLegInit = false;
                    traj[i].isEnd = false;
                    traj[i].t_ = 0.0;
                }

                homing_ref_q << 0 * deg2rad, 80 * deg2rad, -100 * deg2rad;

                homing_Kp << 100,60,10;
                homing_Kd << 1,0.1,0.1;

                homing_period = 5.0;

                Recieved_Mode[INIT] = false;
                Recieved_Mode[HOMING] = false;
                Recieved_Mode[STANDING] = true;
                Recieved_Mode[MOTION] = false;
                Recieved_Mode[FINISH] = false;
            }

            for (size_t i = 0; i < Leg_N; i++)
            {
                robotleg[i].SetLegInit(q_.block<DoF, 1>(i * DoF, 0));

                traj[i].SetSinusoidalTrajectory(homing_ref_q, robotleg[i].GetInitQ(), homing_period);

                robotleg[i].SetPDControl_q(homing_Kp, homing_Kd, traj[i].GetRefVar(), q_.block<DoF, 1>(i * DoF, 0), traj[i].GetRefVarDDot(), dq_.block<DoF, 1>(i * DoF, 0));

                for (size_t j = 0; j < DoF; j++)
                {
                    torque_(i * DoF + j) = robotleg[i].GetJointTorque()(j);
                }

                // if (traj[Leg_N - 1].isEnd)
                // {
                //     controlmode = MOTION;
                // }
            }
            break;
        }

        // case MOTION:
        // {
        //     if (!Recieved_Mode[MOTION])
        //     {
        //         ROS_INFO("MOTION START");
        //         for (size_t i = 0; i < Leg_N; i++)
        //         {
        //             robotleg[i].isLegInit = false;
        //             traj[i].isEnd = false;
        //             traj[i].t_ = 0.0;
        //             robotleg[i].SetLegInit(pin.GetPos(i));
        //             walking_init_x_[i] = traj[i].GetRefVar();
        //         }
        //         Recieved_Mode[INIT] = false;
        //         Recieved_Mode[HOMING] = false;
        //         Recieved_Mode[MOTION] = true;
        //         Recieved_Mode[FINISH] = false;

        //         tmp_Kp = 2000;
        //         tmp_Kd = tmp_Kp * 0.01;

        //         tmp_rKp = tmp_Kp * 1.0;
        //         tmp_rKd = tmp_Kd * 1.0;
        //     }

        //     break;
        //     }

        // case FINISH:
        //     if (!Recieved_Mode[FINISH])
        //     {
        //         ROS_INFO("FINISH...");
        //         for (size_t i = 0; i < Leg_N; i++)
        //         {
        //             robotleg[i].isLegInit = false;
        //             traj[i].isEnd = false;
        //             traj[i].t_ = 0.0;
        //         }
        //         Recieved_Mode[INIT] = false;
        //         Recieved_Mode[HOMING] = false;
        //         Recieved_Mode[MOTION] = false;
        //         Recieved_Mode[FINISH] = true;
        //     }
        //     break;
        }

        // Recieved_Joint_State = false;
        send_commands_to_robot();
    }

    else
    {
        ROS_INFO("Not Connected Please Wait...");
    }
}


void controller::send_commands_to_robot()
{
    std_msgs::Float64MultiArray msg;

    for (size_t i = 0; i < init_size; i++)
    {
        // if (controlmode == INIT || controlmode == FINISH || controlmode == MOTION)
        if (controlmode == INIT || controlmode == FINISH)
        {
            msg.data.push_back(0);
        }
        else
        {
            msg.data.push_back(torque_(i));
        }
    }

    pub_leg_cmd_.publish(msg);

    msg.data.clear();
}

void controller::run()
{
    ROS_INFO("Running the torque control loop .................");

    const ros::Duration control_period_(control_period);

    ros::AsyncSpinner spinner(12);
    spinner.start();

    ros::Time start_time = ros::Time::now();
    ros::Time last_control_time = start_time;

    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();

        ros::Duration elapsed_time = current_time - last_control_time;

        if (elapsed_time >= control_period_)
        {
            // Update the last control time
            last_control_time = current_time;
            control_time += control_period;

            // Perform control actions here

            // ROS_INFO("Control loop running");

            command(Recieved_Joint_State);

            // rqt
            rqt_plot();

            ros::spinOnce();
            
            // Sleep to enforce the desired control loop frequency
            ros::Duration sleep_time = control_period_ - elapsed_time;
            if (sleep_time > ros::Duration(0))
            {
                sleep_time.sleep();
            }
        }
    }
}

void controller::print()
{
    // std::cout<<"l1_J_ : "<<l1_J_<<"\n"<<std::endl;
    // std::cout<<"l2_J_ : "<<l2_J_<<"\n"<<std::endl;
    // std::cout<<"l3_J_ : "<<l3_J_<<"\n"<<std::endl;
    // std::cout<<"r1_J_ : "<<r1_J_<<"\n"<<std::endl;
    // std::cout<<"r2_J_ : "<<r2_J_<<"\n"<<std::endl;
    // std::cout<<"r3_J_ : "<<r3_J_<<"\n"<<std::endl;

    // std::cout<<"Total J : "<<pin.GetJacobian(0)<<"\n"<<std::endl;

    // std::cout<<"L1 q : "<<l1_q_<<std::endl;
    // std::cout<<"L2 q : "<<l2_q_<<std::endl;
    // std::cout<<"L3 q : "<<l3_q_<<std::endl;
    // std::cout<<"R1 q : "<<r1_q_<<std::endl;
    // std::cout<<"R2 q : "<<r2_q_<<std::endl;
    // std::cout<<"R3 q : "<<r3_q_<<std::endl;

    // std::cout<<"total q : "<<q_<<std::endl;
}

void controller::init_rqt()
{
    // Body Position Publisher
    p_body_pos_x = nh_plot.advertise<std_msgs::Float64>("/body_pos_x/",1);
    p_body_pos_y = nh_plot.advertise<std_msgs::Float64>("/body_pos_y/",1);
    p_body_pos_z = nh_plot.advertise<std_msgs::Float64>("/body_pos_z/",1);

    // Body Velocity Publisher
    p_body_vel_x = nh_plot.advertise<std_msgs::Float64>("/body_vel_x/",1);
    p_body_vel_y = nh_plot.advertise<std_msgs::Float64>("/body_vel_y/",1);
    p_body_vel_z = nh_plot.advertise<std_msgs::Float64>("/body_vel_z/",1);

    p_imu_roll = nh_plot.advertise<std_msgs::Float64>("/imu_roll/",1);
    p_imu_pitch = nh_plot.advertise<std_msgs::Float64>("/imu_pitch/",1);
    p_imu_yaw = nh_plot.advertise<std_msgs::Float64>("/imu_yaw/",1);

    // Joint Angle Publisher
    p_fl_hr_q = nh_plot.advertise<std_msgs::Float64>("/fl_hr_q/",1); p_fl_hp_q = nh_plot.advertise<std_msgs::Float64>("/fl_hp_q/",1);
    p_fl_kp_q = nh_plot.advertise<std_msgs::Float64>("/fl_kp_q/",1); p_fr_hr_q = nh_plot.advertise<std_msgs::Float64>("/fr_hr_q/",1); 
    p_fr_hp_q = nh_plot.advertise<std_msgs::Float64>("/fr_hp_q/",1); p_fr_kp_q = nh_plot.advertise<std_msgs::Float64>("/fr_kp_q/",1);
    p_rl_hr_q = nh_plot.advertise<std_msgs::Float64>("/rl_hr_q/",1); p_rl_hp_q = nh_plot.advertise<std_msgs::Float64>("/rl_hp_q/",1);
    p_rl_kp_q = nh_plot.advertise<std_msgs::Float64>("/rl_kp_q/",1); p_rr_hr_q = nh_plot.advertise<std_msgs::Float64>("/rr_hr_q/",1); 
    p_rr_hp_q = nh_plot.advertise<std_msgs::Float64>("/rr_hp_q/",1); p_rr_kp_q = nh_plot.advertise<std_msgs::Float64>("/rr_kp_q/",1);

    // Joint Angular Velocidy Publisher
    p_fl_hr_dq = nh_plot.advertise<std_msgs::Float64>("/fl_hr_dq/",1); p_fl_hp_dq = nh_plot.advertise<std_msgs::Float64>("/fl_hp_dq/",1);
    p_fl_kp_dq = nh_plot.advertise<std_msgs::Float64>("/fl_kp_dq/",1); p_fr_hr_dq = nh_plot.advertise<std_msgs::Float64>("/fr_hr_dq/",1); 
    p_fr_hp_dq = nh_plot.advertise<std_msgs::Float64>("/fr_hp_dq/",1); p_fr_kp_dq = nh_plot.advertise<std_msgs::Float64>("/fr_kp_dq/",1); 
    p_rl_hr_dq = nh_plot.advertise<std_msgs::Float64>("/rl_hr_dq/",1); p_rl_hp_dq = nh_plot.advertise<std_msgs::Float64>("/rl_hp_dq/",1);
    p_rl_kp_dq = nh_plot.advertise<std_msgs::Float64>("/rl_kp_dq/",1); p_rr_hr_dq = nh_plot.advertise<std_msgs::Float64>("/rr_hr_dq/",1); 
    p_rr_hp_dq = nh_plot.advertise<std_msgs::Float64>("/rr_hp_dq/",1); p_rr_kp_dq = nh_plot.advertise<std_msgs::Float64>("/rr_kp_dq/",1); 

    // Ref & Act Foot Position Publisher
    p_fl_ref_pos_x = nh_plot.advertise<std_msgs::Float64>("/fl_ref_pos_x/", 1);
    p_fl_ref_pos_y = nh_plot.advertise<std_msgs::Float64>("/fl_ref_pos_y/", 1);
    p_fl_ref_pos_z = nh_plot.advertise<std_msgs::Float64>("/fl_ref_pos_z/", 1);

    p_fl_act_pos_x = nh_plot.advertise<std_msgs::Float64>("/fl_act_pos_x/", 1);
    p_fl_act_pos_y = nh_plot.advertise<std_msgs::Float64>("/fl_act_pos_y/", 1);
    p_fl_act_pos_z = nh_plot.advertise<std_msgs::Float64>("/fl_act_pos_z/", 1);

    p_fr_ref_pos_x = nh_plot.advertise<std_msgs::Float64>("/fr_ref_pos_x/", 1);
    p_fr_ref_pos_y = nh_plot.advertise<std_msgs::Float64>("/fr_ref_pos_y/", 1);
    p_fr_ref_pos_z = nh_plot.advertise<std_msgs::Float64>("/fr_ref_pos_z/", 1);

    p_fr_act_pos_x = nh_plot.advertise<std_msgs::Float64>("/fr_act_pos_x/", 1);
    p_fr_act_pos_y = nh_plot.advertise<std_msgs::Float64>("/fr_act_pos_y/", 1);
    p_fr_act_pos_z = nh_plot.advertise<std_msgs::Float64>("/fr_act_pos_z/", 1);

    p_rl_ref_pos_x = nh_plot.advertise<std_msgs::Float64>("/rl_ref_pos_x/", 1);
    p_rl_ref_pos_y = nh_plot.advertise<std_msgs::Float64>("/rl_ref_pos_y/", 1);
    p_rl_ref_pos_z = nh_plot.advertise<std_msgs::Float64>("/rl_ref_pos_z/", 1);

    p_rl_act_pos_x = nh_plot.advertise<std_msgs::Float64>("/rl_act_pos_x/", 1);
    p_rl_act_pos_y = nh_plot.advertise<std_msgs::Float64>("/rl_act_pos_y/", 1);
    p_rl_act_pos_z = nh_plot.advertise<std_msgs::Float64>("/rl_act_pos_z/", 1);

    p_rr_ref_pos_x = nh_plot.advertise<std_msgs::Float64>("/rr_ref_pos_x/", 1);
    p_rr_ref_pos_y = nh_plot.advertise<std_msgs::Float64>("/rr_ref_pos_y/", 1);
    p_rr_ref_pos_z = nh_plot.advertise<std_msgs::Float64>("/rr_ref_pos_z/", 1);

    p_rr_act_pos_x = nh_plot.advertise<std_msgs::Float64>("/rr_act_pos_x/", 1);
    p_rr_act_pos_y = nh_plot.advertise<std_msgs::Float64>("/rr_act_pos_y/", 1);
    p_rr_act_pos_z = nh_plot.advertise<std_msgs::Float64>("/rr_act_pos_z/", 1);

    // // Joint Torque Publisher
    p_fl_hr_torque = nh_plot.advertise<std_msgs::Float64>("/fl_hr_torque/", 1);
    p_fl_hp_torque = nh_plot.advertise<std_msgs::Float64>("/fl_hp_torque/", 1);
    p_fl_kp_torque = nh_plot.advertise<std_msgs::Float64>("/fl_kp_torque/", 1);

    p_fr_hr_torque = nh_plot.advertise<std_msgs::Float64>("/fr_hr_torque/", 1);
    p_fr_hp_torque = nh_plot.advertise<std_msgs::Float64>("/fr_hp_torque/", 1);
    p_fr_kp_torque = nh_plot.advertise<std_msgs::Float64>("/fr_kp_torque/", 1);

    p_rl_hr_torque = nh_plot.advertise<std_msgs::Float64>("/rl_hr_torque/", 1);
    p_rl_hp_torque = nh_plot.advertise<std_msgs::Float64>("/rl_hp_torque/", 1);
    p_rl_kp_torque = nh_plot.advertise<std_msgs::Float64>("/rl_kp_torque/", 1);

    p_rr_hr_torque = nh_plot.advertise<std_msgs::Float64>("/rr_hr_torque/", 1);
    p_rr_hp_torque = nh_plot.advertise<std_msgs::Float64>("/rr_hp_torque/", 1);
    p_rr_kp_torque = nh_plot.advertise<std_msgs::Float64>("/rr_kp_torque/", 1);

    // Joint Power Publisher
    p_fl_hr_power = nh_plot.advertise<std_msgs::Float64>("/fl_hr_power/",1);
    p_fl_hp_power = nh_plot.advertise<std_msgs::Float64>("/fl_hp_power/",1);
    p_fl_kp_power = nh_plot.advertise<std_msgs::Float64>("/fl_kp_power/",1);

    p_fr_hr_power = nh_plot.advertise<std_msgs::Float64>("/fr_hr_power/",1);
    p_fr_hp_power = nh_plot.advertise<std_msgs::Float64>("/fr_hp_power/",1);
    p_fr_kp_power = nh_plot.advertise<std_msgs::Float64>("/fr_kp_power/",1);

    p_rl_hr_power = nh_plot.advertise<std_msgs::Float64>("/rl_hr_power/",1);
    p_rl_hp_power = nh_plot.advertise<std_msgs::Float64>("/rl_hp_power/",1);
    p_rl_kp_power = nh_plot.advertise<std_msgs::Float64>("/rl_kp_power/",1);

    p_rr_hr_power = nh_plot.advertise<std_msgs::Float64>("/rr_hr_power/",1);
    p_rr_hp_power = nh_plot.advertise<std_msgs::Float64>("/rr_hp_power/",1);
    p_rr_kp_power = nh_plot.advertise<std_msgs::Float64>("/rr_kp_power/",1);

}

void controller::rqt_plot()
{
    // Body Position
    m_body_pos_x.data = body_pos_(X); m_body_pos_y.data = body_pos_(Y); m_body_pos_z.data = body_pos_(Z); 
    p_body_pos_x.publish(m_body_pos_x); p_body_pos_y.publish(m_body_pos_y); p_body_pos_z.publish(m_body_pos_z); 

     // Body Velocity
    m_body_vel_x.data = body_vel_(X); m_body_vel_y.data = body_vel_(Y); m_body_vel_z.data = body_vel_(Z); 
    p_body_vel_x.publish(m_body_vel_x); p_body_vel_y.publish(m_body_vel_y); p_body_vel_z.publish(m_body_vel_z); 

    // Roll, Pitch, Yaw
    m_imu_roll.data = orientation_(X); m_imu_pitch.data = orientation_(Y); m_imu_yaw.data = orientation_(Z);
    p_imu_roll.publish(m_imu_roll); p_imu_pitch.publish(m_imu_pitch); p_imu_yaw.publish(m_imu_yaw);

    // Joint Angle 
    m_fl_hr_q.data = q_(FL*DoF+HR); m_fl_hp_q.data = q_(FL*DoF+HP); m_fl_kp_q.data = q_(FL*DoF+KP);
    m_fr_hr_q.data = q_(FR*DoF+HR); m_fr_hp_q.data = q_(FR*DoF+HP); m_fr_kp_q.data = q_(FR*DoF+KP);
    m_rl_hr_q.data = q_(RL*DoF+HR); m_rl_hp_q.data = q_(RL*DoF+HP); m_rl_kp_q.data = q_(RL*DoF+KP);
    m_rr_hr_q.data = q_(RR*DoF+HR); m_rr_hp_q.data = q_(RR*DoF+HP); m_rr_kp_q.data = q_(RR*DoF+KP);

    p_fl_hr_q.publish(m_fl_hr_q); p_fl_hp_q.publish(m_fl_hp_q); p_fl_kp_q.publish(m_fl_kp_q);
    p_fr_hr_q.publish(m_fr_hr_q); p_fr_hp_q.publish(m_fr_hp_q); p_fr_kp_q.publish(m_fr_kp_q);
    p_rl_hr_q.publish(m_rl_hr_q); p_rl_hp_q.publish(m_rl_hp_q); p_rl_kp_q.publish(m_rl_kp_q);
    p_rr_hr_q.publish(m_rr_hr_q); p_rr_hp_q.publish(m_rr_hp_q); p_rr_kp_q.publish(m_rr_kp_q);

    // Joint Angular Velocidy 
    m_fl_hr_dq.data = dq_(FL*DoF+HR); m_fl_hp_dq.data = dq_(FL*DoF+HP); m_fl_kp_dq.data = dq_(FL*DoF+KP);
    m_fr_hr_dq.data = dq_(FR*DoF+HR); m_fr_hp_dq.data = dq_(FR*DoF+HP); m_fr_kp_dq.data = dq_(FR*DoF+KP);
    m_rl_hr_dq.data = dq_(RL*DoF+HR); m_rl_hp_dq.data = dq_(RL*DoF+HP); m_rl_kp_dq.data = dq_(RL*DoF+KP);
    m_rr_hr_dq.data = dq_(RR*DoF+HR); m_rr_hp_dq.data = dq_(RR*DoF+HP); m_rr_kp_dq.data = dq_(RR*DoF+KP);

    p_fl_hr_dq.publish(m_fl_hr_dq); p_fl_hp_dq.publish(m_fl_hp_dq); p_fl_kp_dq.publish(m_fl_kp_dq);
    p_fr_hr_dq.publish(m_fr_hr_dq); p_fr_hp_dq.publish(m_fr_hp_dq); p_fr_kp_dq.publish(m_fr_kp_dq);
    p_rl_hr_dq.publish(m_rl_hr_dq); p_rl_hp_dq.publish(m_rl_hp_dq); p_rl_kp_dq.publish(m_rl_kp_dq);
    p_rr_hr_dq.publish(m_rr_hr_dq); p_rr_hp_dq.publish(m_rr_hp_dq); p_rr_kp_dq.publish(m_rr_kp_dq);

    // Ref & Act Position
    m_fl_ref_pos_x.data = traj[FL].GetRefVar()(X); m_fl_ref_pos_y.data = traj[FL].GetRefVar()(Y); m_fl_ref_pos_z.data = traj[FL].GetRefVar()(Z);
    m_fl_act_pos_x.data = pin.GetPos(FL)(X); m_fl_act_pos_y.data = pin.GetPos(FL)(Y); m_fl_act_pos_z.data = pin.GetPos(FL)(Z);

    p_fl_ref_pos_x.publish(m_fl_ref_pos_x); p_fl_ref_pos_y.publish(m_fl_ref_pos_y); p_fl_ref_pos_z.publish(m_fl_ref_pos_z);
    p_fl_act_pos_x.publish(m_fl_act_pos_x); p_fl_act_pos_y.publish(m_fl_act_pos_y); p_fl_act_pos_z.publish(m_fl_act_pos_z);

    m_fr_ref_pos_x.data = traj[FR].GetRefVar()(X); m_fr_ref_pos_y.data = traj[FR].GetRefVar()(Y); m_fr_ref_pos_z.data = traj[FR].GetRefVar()(Z);
    m_fr_act_pos_x.data = pin.GetPos(FR)(X); m_fr_act_pos_y.data = pin.GetPos(FR)(Y); m_fr_act_pos_z.data = pin.GetPos(FR)(Z);

    p_fr_ref_pos_x.publish(m_fr_ref_pos_x); p_fr_ref_pos_y.publish(m_fr_ref_pos_y); p_fr_ref_pos_z.publish(m_fr_ref_pos_z);
    p_fr_act_pos_x.publish(m_fr_act_pos_x); p_fr_act_pos_y.publish(m_fr_act_pos_y); p_fr_act_pos_z.publish(m_fr_act_pos_z);

    m_rl_ref_pos_x.data = traj[RL].GetRefVar()(X); m_rl_ref_pos_y.data = traj[RL].GetRefVar()(Y); m_rl_ref_pos_z.data = traj[RL].GetRefVar()(Z);
    m_rl_act_pos_x.data = pin.GetPos(RL)(X); m_rl_act_pos_y.data = pin.GetPos(RL)(Y); m_rl_act_pos_z.data = pin.GetPos(RL)(Z);

    p_rl_ref_pos_x.publish(m_rl_ref_pos_x); p_rl_ref_pos_y.publish(m_rl_ref_pos_y); p_rl_ref_pos_z.publish(m_rl_ref_pos_z);
    p_rl_act_pos_x.publish(m_rl_act_pos_x); p_rl_act_pos_y.publish(m_rl_act_pos_y); p_rl_act_pos_z.publish(m_rl_act_pos_z);

    m_rr_ref_pos_x.data = traj[RR].GetRefVar()(X); m_rr_ref_pos_y.data = traj[RR].GetRefVar()(Y); m_rr_ref_pos_z.data = traj[RR].GetRefVar()(Z);
    m_rr_act_pos_x.data = pin.GetPos(RR)(X); m_rr_act_pos_y.data = pin.GetPos(RR)(Y); m_rr_act_pos_z.data = pin.GetPos(RR)(Z);

    p_rr_ref_pos_x.publish(m_rr_ref_pos_x); p_rr_ref_pos_y.publish(m_rr_ref_pos_y); p_rr_ref_pos_z.publish(m_rr_ref_pos_z);
    p_rr_act_pos_x.publish(m_rr_act_pos_x); p_rr_act_pos_y.publish(m_rr_act_pos_y); p_rr_act_pos_z.publish(m_rr_act_pos_z);

    // Joint Toruqe
    m_fl_hr_torque.data = torque_LPF[FL][HR].Filter(torque_(FL * DoF + HR),15); m_fl_hp_torque.data = torque_LPF[FL][HP].Filter(torque_(FL * DoF + HP),15);
    m_fl_kp_torque.data = torque_LPF[FL][KP].Filter(torque_(FL * DoF + KP),15); 

    p_fl_hr_torque.publish(m_fl_hr_torque); p_fl_hp_torque.publish(m_fl_hp_torque); p_fl_kp_torque.publish(m_fl_kp_torque); 

    m_fr_hr_torque.data = torque_LPF[FR][HR].Filter(torque_(FR * DoF + HR),15); m_fr_hp_torque.data = torque_LPF[FR][HP].Filter(torque_(FR * DoF + HP),15);
    m_fr_kp_torque.data = torque_LPF[FR][KP].Filter(torque_(FR * DoF + KP),15); 

    p_fr_hr_torque.publish(m_fr_hr_torque); p_fr_hp_torque.publish(m_fr_hp_torque); p_fr_kp_torque.publish(m_fr_kp_torque); 

    m_rl_hr_torque.data = torque_LPF[RL][HR].Filter(torque_(RL * DoF + HR),15); m_rl_hp_torque.data = torque_LPF[RL][HP].Filter(torque_(RL * DoF + HP),15);
    m_rl_kp_torque.data = torque_LPF[RL][KP].Filter(torque_(RL * DoF + KP),15); 

    p_rl_hr_torque.publish(m_rl_hr_torque); p_rl_hp_torque.publish(m_rl_hp_torque); p_rl_kp_torque.publish(m_rl_kp_torque); 

    m_rr_hr_torque.data = torque_LPF[RR][HR].Filter(torque_(RR * DoF + HR),15); m_rr_hp_torque.data = torque_LPF[RR][HP].Filter(torque_(RR * DoF + HP),15);
    m_rr_kp_torque.data = torque_LPF[RR][KP].Filter(torque_(RR * DoF + KP),15); 

    p_rr_hr_torque.publish(m_rr_hr_torque); p_rr_hp_torque.publish(m_rr_hp_torque); p_rr_kp_torque.publish(m_rr_kp_torque); 

    // // Joint Power
    m_fl_hr_power.data = abs(torque_(FL*DoF + HR)*dq_(FL*DoF+HR)); m_fl_hp_power.data = abs(torque_(FL*DoF + HP)*dq_(FL*DoF+HP));
    m_fl_kp_power.data = abs(torque_(FL*DoF + KP)*dq_(FL*DoF+KP)); 

    m_fr_hr_power.data = abs(torque_(FR*DoF + HR)*dq_(FR*DoF+HR)); m_fr_hp_power.data = abs(torque_(FR*DoF + HP)*dq_(FR*DoF+HP));
    m_fr_kp_power.data = abs(torque_(FR*DoF + KP)*dq_(FR*DoF+KP));

    m_rl_hr_power.data = abs(torque_(RL*DoF + HR)*dq_(RL*DoF+HR)); m_rl_hp_power.data = abs(torque_(RL*DoF + HP)*dq_(RL*DoF+HP));
    m_rl_kp_power.data = abs(torque_(RL*DoF + KP)*dq_(RL*DoF+KP));

    m_rr_hr_power.data = abs(torque_(RR*DoF + HR)*dq_(RR*DoF+HR)); m_rr_hp_power.data = abs(torque_(RR*DoF + HP)*dq_(RR*DoF+HP));
    m_rr_kp_power.data = abs(torque_(RR*DoF + KP)*dq_(RR*DoF+KP));

    p_fl_hr_power.publish(m_fl_hr_power); p_fl_hp_power.publish(m_fl_hp_power); p_fl_kp_power.publish(m_fl_kp_power);
    p_fr_hr_power.publish(m_fr_hr_power); p_fr_hp_power.publish(m_fr_hp_power); p_fr_kp_power.publish(m_fr_kp_power);
    p_rl_hr_power.publish(m_rl_hr_power); p_rl_hp_power.publish(m_rl_hp_power); p_rl_kp_power.publish(m_rl_kp_power);
    p_rr_hr_power.publish(m_rr_hr_power); p_rr_hp_power.publish(m_rr_hp_power); p_rr_kp_power.publish(m_rr_kp_power);

}