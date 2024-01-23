#include "RobotLeg.h"


void RobotLeg::init(double dt)
{
    dt_ = dt;
    kp_.setZero(DoF);
    kd_.setZero(DoF);
    init_q_.setZero(DoF);
    ref_q_.setZero(DoF);
    act_q_.setZero(DoF);
    ref_qdot_.setZero(DoF);
    act_qdot_.setZero(DoF);
    act_qdot_.setZero(DoF);
    torque_.setZero(DoF);
    error_x_.setZero(DoF);
    error_xdot_.setZero(DoF);
    ref_xdot_.setZero(DoF);
    act_xdot_.setZero(DoF);
    torque_x_.setZero(DoF);
    J_.setZero(DoF,DoF);
}

void RobotLeg::SetLegInit(Eigen::VectorXd init_q)
{
    if (!isLegInit)
    {
        init_q_ = init_q;
        isLegInit = true;
    }
}

void RobotLeg::SetPDControl_q(Eigen::VectorXd kp, Eigen::VectorXd kd, Eigen::VectorXd ref_q, Eigen::VectorXd act_q, Eigen::VectorXd ref_qdot, Eigen::VectorXd act_qdot)
{
    for (size_t i = 0; i < DoF; i++)
    {
        kp_(i) = kp(i);
        kd_(i) = kd(i);
        ref_q_(i) = ref_q(i);
        act_q_(i) = act_q(i);
        ref_qdot_(i) = ref_qdot(i);
        act_qdot_(i) = act_qdot(i);
    }

    for (size_t i = 0; i < DoF; i++)
    {
        torque_(i) = kp_(i) * (ref_q_(i) - act_q_(i)) + kd_(i) * (ref_qdot_(i) - act_qdot_(i));
    }
}

void RobotLeg::SetPDControl_X(Eigen::VectorXd kp, Eigen::VectorXd kd, Eigen::VectorXd ref_x, Eigen::VectorXd act_x, Eigen::MatrixXd J)
{
    int cnt_=0;
    error_x_ = ref_x-act_x;
    J_ = J;
    kp_ = kp;
    kd_ = kd;

    for (size_t i = 0; i < DoF; i++)
    {
        error_xdot_(i) = pd_error_xdot[i].Diff(error_x_(i));
    }

    for (size_t i = 0; i < DoF; i++)
    {
        torque_x_(i) = kp(i)*error_x_(i) + kd(i)*error_xdot_(i);
    }
    // std::cout<<"error X : "<<error_x_<<std::endl;
    // std::cout<<"error XDot : "<<error_xdot_<<"\n"<<std::endl;

    torque_ = J_.transpose()*torque_x_;
}