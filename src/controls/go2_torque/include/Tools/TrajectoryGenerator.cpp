#include "TrajectoryGenerator.h"

void TrajectoryGenerator::init(double dt)
{
    pi_ = M_PI;
    t_ = 0.0;
    dt_ = dt;

    period_ = 0.0;
    legtype_ = 0;

    qf_.setZero(DoF);
    qi_.setZero(DoF);
    ref_var_.setZero(DoF);
    ref_vardot_.setZero(DoF);
    ref_varddot_.setZero(DoF);
    pre_ref_var_.setZero(DoF);
    pre_ref_vardot_.setZero(DoF);
    pre_ref_varddot_.setZero(DoF);

    walking_flag = 0;
    init_stride_ = false;
    isEnd = false;
}

void TrajectoryGenerator::SetSinusoidalTrajectory(Eigen::VectorXd qf, Eigen::VectorXd qi, double period)
{
    // qf : final angle, gi : initial angle
    qf_ = qf;
    qi_ = qi;
    period_ = period;
    t_ += dt_;

    if (t_ >= period_)
    {
        t_ = period_;
        isEnd = true;
    }

    for (size_t i = 0; i < DoF; i++)
    {
        ref_var_(i) = (qf_(i) - qi_(i)) * 0.5 * (1 - cos(1.0 * pi_ * t_ / period_)) + qi_(i);

        ref_vardot_(i) = (qf_(i) - qi_(i)) * 0.5 * (pi_ / period_) * (sin(1.0 * pi_ * t_ / period_));

        ref_varddot_(i) = (qf_(i) - qi_(i)) * 0.5 * pow((pi_ / period_), 2) * (cos(1.0 * pi_ * t_ / period_));
    }
}

void TrajectoryGenerator::SetWalkingSinusoidalTrajectory(Eigen::VectorXd qf, Eigen::VectorXd qi, double period, int legtype)
{
    qf_ = qf;
    qi_ = qi;
    period_ = period;
    t_ += dt_;

    // if(init_stride_==false)
    // {
    //     qf_(0) = 0.5*qf(0);
    //     qf_(1) = 0.5*qf(1);
    //     init_stride_ = true; 
    // }

    if (legtype % 2 == 0)
    {
        if (walking_flag % 2 == 0)
        {
            // std::cout << "L1 L3 R2 swing" << legtype << std::endl;
            for (size_t i = 0; i < DoF; i++)
            {
                pre_ref_var_(i) = ref_var_(i);
                pre_ref_vardot_(i) = ref_vardot_(i);
                pre_ref_varddot_(i) = ref_varddot_(i);

                ref_var_(i) = (qf_(i) - qi_(i)) * 0.5 * (1 - cos(1.0 * pi_ * t_ / period_)) + qi_(i);

                ref_vardot_(i) = (qf_(i) - qi_(i)) * 0.5 * (1 * pi_ / period_) * (sin(1.0 * pi_ * t_ / period_));

                ref_varddot_(i) = (qf_(i) - qi_(i)) * 0.5 * pow((1 * pi_ / period_), 2) * (cos(1.0 * pi_ * t_ / period_));
                

                if (i == 2)
                {
                    ref_var_(i) = (qf_(i) - qi_(i)) * 0.5 * (1 - cos(2.0 * pi_ * t_ / period_)) + qi_(i);

                    ref_vardot_(i) = (qf_(i) - qi_(i)) * 0.5 * (2 * pi_ / period_) * (sin(2.0 * pi_ * t_ / period_));

                    ref_varddot_(i) = (qf_(i) - qi_(i)) * 0.5 * pow((2 * pi_ / period_), 2) * (cos(2.0 * pi_ * t_ / period_));
                }
            }
        }
        else
        {
            // std::cout << "L1 L3 R2 swing" << legtype << std::endl;
            for (size_t i = 0; i < DoF; i++)
            {
                ref_var_(i) = (qf_(i) - qi_(i)) * 0.5 * (1 - cos(1.0 * pi_ * t_ / period_)) + qi_(i);

                ref_vardot_(i) = (qf_(i) - qi_(i)) * 0.5 * (1 * pi_ / period_) * (sin(1.0 * pi_ * t_ / period_));

                ref_varddot_(i) = (qf_(i) - qi_(i)) * 0.5 * pow((1 * pi_ / period_), 2) * (cos(1.0 * pi_ * t_ / period_));

                if (i == 2)
                {
                    ref_var_(i) = qi_(i);

                    ref_vardot_(i) = 0;

                    ref_varddot_(i) = 0;
                }
            }
        }
    }
    else
    {
        if (walking_flag % 2 == 0)
        {
            // std::cout << "L1 L3 R2 swing" << legtype << std::endl;
            for (size_t i = 0; i < DoF; i++)
            {
                if (walking_flag < 2)
                {
                    ref_var_(i) = -(qf_(i) - qi_(i)) * 0.5 * (1 - cos(1.0 * pi_ * t_ / period_)) + qi_(i);

                    ref_vardot_(i) = -(qf_(i) - qi_(i)) * 0.5 * (1 * pi_ / period_) * (sin(1.0 * pi_ * t_ / period_));

                    ref_varddot_(i) = -(qf_(i) - qi_(i)) * 0.5 * pow((1 * pi_ / period_), 2) * (cos(1.0 * pi_ * t_ / period_));
                }
                else
                {
                    ref_var_(i) = -1.0 * (qf_(i) - qi_(i)) * 0.5 * (1 - cos(1.0 * pi_ * t_ / period_)) + qf_(i);

                    ref_vardot_(i) = -1.0*(qf_(i) - qi_(i)) * 0.5 * (1 * pi_ / period_) * (sin(1.0 * pi_ * t_ / period_));

                    ref_varddot_(i) = -1.0*(qf_(i) - qi_(i)) * 0.5 * pow((1 * pi_ / period_), 2) * (cos(1.0 * pi_ * t_ / period_));
                }


                if (i == 2)
                {
                    ref_var_(i) = qi_(i);

                    ref_vardot_(i) = 0;

                    ref_varddot_(i) = 0;
                }
            }
        }
        else
        {
            // std::cout << "L1 L3 R2 swing" << legtype << std::endl;
            for (size_t i = 0; i < DoF; i++)
            {
                ref_var_(i) = -1.0 * (qf_(i) - qi_(i)) * 0.5 * (1 - cos(1.0 * pi_ * t_ / period_)) + qf_(i);

                ref_vardot_(i) = -1.0*(qf_(i) - qi_(i)) * 0.5 * (1 * pi_ / period_) * (sin(1.0 * pi_ * t_ / period_));

                ref_varddot_(i) = -1.0*(qf_(i) - qi_(i)) * 0.5 * pow((1 * pi_ / period_), 2) * (cos(1.0 * pi_ * t_ / period_));

                if (i == 2)
                {
                    ref_var_(i) = (qf_(i) - qi_(i)) * 0.5 * (1 - cos(2.0 * pi_ * t_ / period_)) + qi_(i);

                    ref_vardot_(i) = (qf_(i) - qi_(i)) * 0.5 * (2 * pi_ / period_) * (sin(2.0 * pi_ * t_ / period_));

                    ref_varddot_(i) = (qf_(i) - qi_(i)) * 0.5 * pow((2 * pi_ / period_), 2) * (cos(2.0 * pi_ * t_ / period_));
                }
            }
        }
    }
    if (fmod(t_, period_) < 0.005)
    {
        walking_flag++;
    }
}

void TrajectoryGenerator::SetSwingSinusoidalTrajectory(Eigen::VectorXd qf, Eigen::VectorXd qi, double period, int legtype)
{
    qf_ = qf;
    qi_ = qi;
    period_ = period;
    t_ += dt_;

    if (walking_flag % 2 == 0){
        // std::cout << "L1 L3 R2 swing" << legtype << std::endl;
        for (size_t i = 0; i < DoF; i++){
            pre_ref_var_(i) = ref_var_(i);
            pre_ref_vardot_(i) = ref_vardot_(i);
            pre_ref_varddot_(i) = ref_varddot_(i);

            ref_var_(i) = (qf_(i) - qi_(i)) * 0.5 * (1 - cos(1.0 * pi_ * t_ / period_)) + qi_(i);

            ref_vardot_(i) = (qf_(i) - qi_(i)) * 0.5 * (1 * pi_ / period_) * (sin(1.0 * pi_ * t_ / period_));

            ref_varddot_(i) = (qf_(i) - qi_(i)) * 0.5 * pow((1 * pi_ / period_), 2) * (cos(1.0 * pi_ * t_ / period_));
                

            if (i == 2){
                ref_var_(i) = (qf_(i) - qi_(i)) * 0.5 * (1 - cos(2.0 * pi_ * t_ / period_)) + qi_(i);

                ref_vardot_(i) = (qf_(i) - qi_(i)) * 0.5 * (2 * pi_ / period_) * (sin(2.0 * pi_ * t_ / period_));

                ref_varddot_(i) = (qf_(i) - qi_(i)) * 0.5 * pow((2 * pi_ / period_), 2) * (cos(2.0 * pi_ * t_ / period_));
            }
        }
    }
    else{
        // std::cout << "L1 L3 R2 swing" << legtype << std::endl;
        for (size_t i = 0; i < DoF; i++){
            ref_var_(i) = (qf_(i) - qi_(i)) * 0.5 * (1 - cos(1.0 * pi_ * t_ / period_)) + qi_(i);

            ref_vardot_(i) = (qf_(i) - qi_(i)) * 0.5 * (1 * pi_ / period_) * (sin(1.0 * pi_ * t_ / period_));

            ref_varddot_(i) = (qf_(i) - qi_(i)) * 0.5 * pow((1 * pi_ / period_), 2) * (cos(1.0 * pi_ * t_ / period_));

            if (i == 2){
                ref_var_(i) = qi_(i);

                ref_vardot_(i) = 0;

                ref_varddot_(i) = 0;
            }
        }
    }
}