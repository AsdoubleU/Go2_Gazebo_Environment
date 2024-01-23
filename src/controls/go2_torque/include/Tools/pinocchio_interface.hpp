#ifndef PINOCCHIOINTERFACE_H_
#define PINOCCHIOINTERFACE_H_

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/centroidal-derivatives.hpp>

#include "Eigen/Dense"
#include "Eigen/Core"
#include <iostream>
#include "NumericalTool.h"
#include "RobotLeg.h"

class PinocchioInterface
{
private:
    pinocchio::Model _model;
    pinocchio::Data _data;
    pinocchio::FrameIndex frame_id;
    int n;
    NumericalTool::Calculus vel_diff[Leg_N][DoF];
    NumericalTool::Calculus M_diff[DoF][DoF];

public:
    PinocchioInterface(const std::string urdf_file, const std::vector<std::string> foot_name) : urdf_file_(urdf_file), foot_name_(foot_name)
    {
        pinocchio::urdf::buildModel(urdf_file_, _model);

        pinocchio::Data data(_model);
        _data = data;

        n = Leg_N*DoF;

        Initialize();
    };
    virtual ~PinocchioInterface(){};

public:
    void Initialize();
    void SetRobotParameter(Eigen::VectorXd q, Eigen::VectorXd dq);
    void SetKinematics(int legtype);
    void SetJacobian(int legtype);
    Eigen::VectorXd NuIK(Eigen::VectorXd q, Eigen::Vector3d walking_ref_x_);
    Eigen::Vector3d FK(int legtype, Eigen::Vector3d q);


    void SetDynamics(Eigen::MatrixXd pos_d, Eigen::MatrixXd vel_d, Eigen::MatrixXd acc_d, Eigen::Matrix3d Kp, Eigen::Matrix3d Kd);
    void SetMatrix_MH();

    Eigen::Vector3d GetPos(int legtype) { return pos_[legtype]; }
    Eigen::Vector3d* GetPos() { return pos_; }
    Eigen::Vector3d GetVel(int legtype) { return vel_[legtype]; }
    Eigen::Vector3d GetAcc(int legtype) { return acc_[legtype]; }
    Eigen::Matrix3d GetJacobian(int legtype) { return J_[legtype]; }
    Eigen::Matrix3d GetJacobianDot(int legtype) { return dJ_[legtype]; }

    Eigen::VectorXd GetDynamics();
    Eigen::MatrixXd GetInertiaMatrix() { return r_M_; }
    Eigen::MatrixXd GetCoriCentriMatrix() { return r_C_; }
    Eigen::MatrixXd GetInertiaMatrixDot() { return r_dM_; }
    Eigen::VectorXd GetNLEffects() { return B_[0]; }
    Eigen::VectorXd GetGravityCompensation() { return G_[0]; }
    Eigen::VectorXd GetTau() {return T_[0];}

    void pinocchio_print();

private:
    // Variables for Pinocchio
    std::string urdf_file_;
    std::vector<std::string> foot_name_;
    Eigen::VectorXd _q, _dq, _ddq;
    Eigen::Matrix<double, DoF, 1>  _pos;
    Eigen::MatrixXd _J, _dJ;
    Eigen::Vector3d old_q_;
    Eigen::Vector3d joint_var_err_;

    // Actual Angle, Angular Velocity from Model Joint States
    Eigen::Matrix<double, DoF, 1> q_[Leg_N], dq_[Leg_N], ddq_[Leg_N];

    // Actual Foot Position, Veloicy
    Eigen::Matrix<double, DoF, 1> pos_[Leg_N], vel_[Leg_N], acc_[Leg_N];

    // Jacobian
    Eigen::Matrix<double, DoF, DoF> J_[Leg_N], dJ_[Leg_N];

    // Recursive-Newon Euler
    Eigen::Matrix<double, DoF, DoF> M_[Leg_N],dM_[Leg_N], C_[Leg_N];
    Eigen::Matrix<double, DoF, 1> G_[Leg_N], B_[Leg_N], T_[Leg_N];

    // Desired Pos, Vel, Acc
    Eigen::Matrix<double, DoF, 1> pos_d_[Leg_N], vel_d_[Leg_N], acc_d_[Leg_N];

    // Error between Des & Act [Position, Velocity]
    Eigen::Matrix<double, DoF, 1> err_[Leg_N], err_dot_[Leg_N];

    // Control Gain
    Eigen::Matrix<double, DoF, DoF> Kp_, Kd_;

    // Caculate Torque
    Eigen::VectorXd target_torque_;

    // return
    Eigen::Matrix<double, DoF, 1> r_pos_, r_vel_, r_acc_;
    Eigen::Matrix<double, DoF, DoF> r_J_, r_dJ_;
    Eigen::MatrixXd r_M_, r_dM_, r_C_;
    Eigen::VectorXd r_B_, r_G_, r_T_;
};

#endif