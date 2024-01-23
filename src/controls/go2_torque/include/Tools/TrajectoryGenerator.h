#ifndef TRAJECTORYGENERATOR_H_
#define TRAJECTORYGENERATOR_H_

#include <iostream>
#include <cmath>
#include <string>
#include "Eigen/Dense"

class TrajectoryGenerator
{
public:
    TrajectoryGenerator() {};
    virtual ~TrajectoryGenerator() {};

    void init(double dt);

    void SetSinusoidalTrajectory(Eigen::VectorXd qf, Eigen::VectorXd qi, double period);

    void SetWalkingSinusoidalTrajectory(Eigen::VectorXd qf, Eigen::VectorXd qi, double period, int legtype);

    void SetSwingSinusoidalTrajectory(Eigen::VectorXd qf, Eigen::VectorXd qi, double period, int legtype);

    Eigen::VectorXd GetRefVar() { return ref_var_; }
    Eigen::VectorXd GetRefVarDot() { return ref_vardot_; }
    Eigen::VectorXd GetRefVarDDot() { return ref_varddot_; }

    double t_;
    Eigen::VectorXd pre_ref_var_, pre_ref_vardot_, pre_ref_varddot_;
    Eigen::VectorXd qf_, qi_;
    int walking_flag;
    bool isEnd;

private:
    // void init(double dt);
    Eigen::VectorXd ref_var_, ref_vardot_, ref_varddot_;
    
    // Eigen::VectorXd qf_, qi_;
    double period_, dt_, pi_;
    int legtype_;

    // int walking_flag;
    int legtype;

    int init_stride_;

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
};

#endif