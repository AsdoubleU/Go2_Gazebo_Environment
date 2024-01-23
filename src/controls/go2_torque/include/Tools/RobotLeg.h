#ifndef ROBOTLEG_H_
#define ROBOTLEG_H_

#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <cmath>
#include "Eigen/Dense"
#include <functional>

#include "NumericalTool.h"

enum Joint
{
    HR,
    HP,
    KP,
    DoF = 3
};

enum Leg
{
    FL,
    FR,
    RL,
    RR,
    Leg_N = 4
};

enum Pos
{
    X,
    Y,
    Z,
    ROLL = 3,
    PITCH = 4,
    YAW = 5,
    Pos = 6,
    XYZ = 3
};

class RobotLeg
{
NumericalTool::Calculus pd_error_xdot[DoF];

public:
    RobotLeg()
    {
        // init();
    };
    virtual ~RobotLeg(){};

    void init(double dt);

    void SetLegInit(Eigen::VectorXd init_q);
    Eigen::VectorXd GetInitQ() { return init_q_; }

    void SetPDControl_q(Eigen::VectorXd kp, Eigen::VectorXd kd, Eigen::VectorXd ref_q, Eigen::VectorXd act_q, Eigen::VectorXd ref_qdot, Eigen::VectorXd act_qdot);

    void SetPDControl_X(Eigen::VectorXd kp, Eigen::VectorXd kd, Eigen::VectorXd ref_x, Eigen::VectorXd act_x, Eigen::MatrixXd J);

    Eigen::VectorXd GetJointTorque() { return torque_; }

    bool isLegInit;

private:
    // void init();

    int init_size_;
    int legtype_;
    double dt_;

    Eigen::VectorXd init_q_;
    Eigen::VectorXd kp_, kd_, ref_q_, act_q_, ref_qdot_, act_qdot_, torque_;
    Eigen::VectorXd error_x_, error_xdot_;
    Eigen::VectorXd ref_xdot_, act_xdot_;
    Eigen::VectorXd torque_x_;
    Eigen::MatrixXd J_;

};

#endif