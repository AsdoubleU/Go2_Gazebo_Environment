#include "pinocchio_interface.hpp"

void PinocchioInterface::Initialize()
{
    std::cout<<"Pinocchio Init"<<std::endl;

    _q.setZero(n);
    _dq.setZero(n);
    _ddq.setZero(n);

    _J.setZero(6, n);
    _dJ.setZero(6, n);

    target_torque_.setZero(n);
}



Eigen::VectorXd PinocchioInterface::NuIK(Eigen::VectorXd q, Eigen::Vector3d walking_ref_x_){
    _q = q;

    const double eps  = 1e-4; // allowable error
    const int IT_MAX  = 1000; // maximum iteration
    const double DT   = 1e-1;
    const double damp = 1e-6;

    // Eigen::Matrix3d rot;
    // rot<<_data.oMf[frame_id].rotation();
    pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Eigen::Vector3d(walking_ref_x_(X), walking_ref_x_(Y), walking_ref_x_(Z)));   
    //Eigen::VectorXd q = pinocchio::neutral(model);



    bool success = false;
    Eigen::Matrix<double, 6, 1> err;
    Eigen::VectorXd v(_model.nv);
    frame_id=_model.getFrameId(foot_name_[0]);
    for (int i=0;;i++)
    {
        // for (size_t i = 0; i < DoF; i++)
        // {
        //     old_q_(i) = _q(i);
        // }

        // std::cout<<"walking_ref_x_"<<walking_ref_x_<<'\n'<<std::endl;
        // std::cout<<"pin.GetPos"<<GetPos(0)<<'\n'<<std::endl;
        //SetKinematics(0)
        pinocchio::framesForwardKinematics(_model,_data,_q);         
        pinocchio::updateFramePlacement(_model, _data, frame_id);
        //after running forwardKinematics(model,data,q), run updateFramePlacements(model,data). This will fill data.oMf
        
        const pinocchio::SE3 dMf = oMdes.actInv(_data.oMf[frame_id]); //transfer matrix
        err = pinocchio::log6(dMf).toVector();

        double err_norm=sqrt(err(0)*err(0)+err(1)*err(1)+err(2)*err(2));

        for (size_t i = 0; i < 3; i++)
        {
            joint_var_err_(i) = err(i);
        }

        // std::cout<<"_data.soMf[DoF]"<<_data.oMf[frame_id]<<std::endl;
        // std::cout<<'\n'<<std::endl;
        // std::cout<<"oMdes"<<oMdes<<std::endl;
        // std::cout<<'\n'<<std::endl;
        // std::cout<<"dMf"<<dMf<<std::endl;
        // std::cout<<'\n'<<std::endl;
        // std::cout<<"err"<<err<<std::endl;
        // std::cout<<'\n'<<std::endl;
        // std::cout<<"err.norm()"<<err_norm<<std::endl;
        // std::cout<<'\n'<<std::endl;


        if(err_norm < eps)
        {
            success = true;
            break;
        }
        if (i >= IT_MAX)
        {
            success = false;
            std::cout<<"fail"<<std::endl;
            break;
        }

        SetJacobian(0);
        _q= _q-GetJacobian(0).inverse()*joint_var_err_;
        // std::cout<<GetJacobian(0)<<'\n'<<std::endl;


        // Eigen::MatrixXd JJt(6,6);
        // JJt.noalias() = _J * _J.transpose();
        // JJt.diagonal().array() += damp;
        // v.noalias() = - _J.transpose() * JJt.ldlt().solve(err); // v(4x1)
        // // std::cout<<"v="<<v<<std::endl;
        // // std::cout<<'\n'<<std::endl;
        // // std::cout<<"before update_q="<<_q<<std::endl;
        // // std::cout<<'\n'<<std::endl;
        // _q = pinocchio::integrate(_model,_q,v*DT); //_q(4x1)
        // // std::cout<<"update_q="<<_q<<std::endl;
        // // std::cout<<'\n'<<std::endl;
        // // if(!(i%10))
        // // std::cout << i << ": error = " << err << std::endl;
    }
    //std::cout <<"_q2="<<'\n'<<_q<< std::endl;
    return _q;
}






void PinocchioInterface::SetRobotParameter(Eigen::VectorXd q, Eigen::VectorXd dq)
{
    _q = q;
    _dq = dq;

    for (size_t i = 0; i < Leg_N; i++)
    {
        for (size_t j = 0; j < DoF; j++)
        {
            _ddq(i + j * DoF) = vel_diff[i][j].Diff(_dq(i + j * DoF));
        }
    }

    for (size_t i = 0; i < Leg_N; i++)
    {
        q_[i] = _q.block<DoF, 1>(i * DoF, 0);
        dq_[i] = _dq.block<DoF, 1>(i * DoF, 0);
        ddq_[i] = _ddq.block<DoF, 1>(i * DoF, 0);
        //each row is each leg
    }

    pinocchio::forwardKinematics(_model, _data, _q);
    pinocchio::computeJointJacobians(_model, _data, _q);
    pinocchio::computeJointJacobiansTimeVariation(_model, _data, _q, _dq);
    // pinocchio::computeAllTerms(_model, _data, _q, _dq);
}

Eigen::Vector3d PinocchioInterface::FK(int legtype, Eigen::Vector3d q){
    pinocchio::framesForwardKinematics(_model,_data,q); // framesForwardKinematics = forwardKinematics + updateFramePlacements
    // after running forwardKinematics(model,data,q), run updateFramePlacements(model,data). This will fill data.oMf
    auto val =_data.oMf[frame_id];
    Eigen::Vector3d tmpvar;
    tmpvar<<val.translation();
    return tmpvar;
}

void PinocchioInterface::SetKinematics(int legtype)
{
    int legtype_ = legtype;

    frame_id = _model.getFrameId(foot_name_[legtype_]);

    // pinocchio::updateFramePlacement(_model, _data, frame_id);

    auto pos = _data.oMf[frame_id];

    // if (DoF == 4)
    //     _pos << pos.translation(), rpy(1);
    if (DoF == 3)
        _pos << pos.translation();

    // Foot Position
    pos_[legtype_] = _pos;

    SetJacobian(legtype_);

    // Foot Velocity
    vel_[legtype_] = GetJacobian(legtype_) * dq_[legtype_];

    // Foot Acceleration
    acc_[legtype_] = GetJacobianDot(legtype_) * dq_[legtype_] + GetJacobian(legtype_) * ddq_[legtype_];
}

void PinocchioInterface::SetJacobian(int legtype)
{
    int legtype_ = legtype;

    pinocchio::getFrameJacobian(_model, _data, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, _J);

    pinocchio::getFrameJacobianTimeVariation(_model, _data, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, _dJ);

    Eigen::Matrix<double, 6,DoF > J[Leg_N];
    Eigen::Matrix<double, 6,DoF> dJ[Leg_N];


        // Jacobian
    J[legtype_] = _J.block<6,DoF>(0, legtype_ * DoF);
    J_[legtype_] = J[legtype_].block<3, 3>(0, 0);
        //jacobian is J_ : end efector linear variable(x,y,z) / joint variable(theta1, theta2, theta3)\
            + end effector angular variable(phi) / joint variable(theta1, theta2, theta3)
        // '<<' operator is used to sequentially write multiple values into a single stream. 
            // This operation is known as the "output operator" or "stream insertion operator."

    // Jacobian Dot
    dJ[legtype_] = _dJ.block<6,DoF>(0, legtype_ * DoF);
    dJ_[legtype_] = dJ[legtype_].block<3, 3>(0, 0);


}

void PinocchioInterface::SetDynamics(Eigen::MatrixXd pos_d, Eigen::MatrixXd vel_d, Eigen::MatrixXd acc_d, Eigen::Matrix3d Kp, Eigen::Matrix3d Kd)
{

    Kp_ = Kp;
    Kd_ = Kd;

    Eigen::VectorXd _a;

    _a.setZero(n);

    // RNE [M, C, G, b]
    pinocchio::crba(_model, _data, _q);
    pinocchio::rnea(_model, _data, _q, _dq, _a);

    pinocchio::computeCoriolisMatrix(_model, _data, _q, _dq);
    pinocchio::computeGeneralizedGravity(_model, _data, _q); 
    pinocchio::nonLinearEffects(_model,_data,_q,_dq);

    r_M_ = _data.M;
    r_T_ = _data.tau;
    r_B_ = _data.nle;

    r_G_ = _data.g;
    r_C_ = _data.C;

    for (size_t i = 0; i < Leg_N; i++)
    {
        SetJacobian(i);
        // Inertia Matrix

        M_[i] = r_M_.block<DoF, DoF>(i * DoF, i * DoF);
        M_[i].triangularView<Eigen::Lower>() = M_[i].transpose();

        // Nonlinear Effects Matrix
        B_[i] = r_B_.block<DoF, 1>(i * DoF, 0);

        // tau Matrix
        T_[i] = r_T_.block<DoF, 1>(i * DoF, 0);

        // Coriolis Matrix
        C_[i] = r_C_.block<DoF, DoF>(i * DoF, i * DoF);

        // // Gravity Matrix
        G_[i] = r_G_.block<DoF, 1>(i * DoF, 0);

        // Desired Pos, Vel, Acc
        pos_d_[i] = pos_d.row(i);
        vel_d_[i] = vel_d.row(i);
        acc_d_[i] = acc_d.row(i);

        // Error beween Des & Act
        err_[i] = pos_d_[i] - pos_[i];
        err_dot_[i] = vel_d_[i] - vel_[i];
    }
    for (size_t i = 0; i < DoF; i++)
    {
        for (size_t j = 0; j < DoF; j++)
        {
            dM_[0](i,j) = M_diff[i][j].Diff(M_[0](i,j));
        }
        
    }
 
    r_dM_ = dM_[0];
    

    // pinocchio::computeCentroidalMomentum(_model,_data,_q,_dq);

    // std::cout<<"CM : "<<_data.hg<<"\n"<<std::endl;
}

void PinocchioInterface::SetMatrix_MH()
{
    Eigen::VectorXd _a;
    _a.setZero(n);

    pinocchio::crba(_model, _data, _q);
    pinocchio::rnea(_model, _data, _q, _dq, _a);

    r_M_ = _data.M;
    r_B_ = _data.nle;

    for (size_t i = 0; i < Leg_N; i++)
    {
        M_[i] = r_M_.block<DoF, DoF>(i * Leg_N, i * Leg_N);
        B_[i] = r_B_.block<DoF, 1>(i * Leg_N, 0);
    }
    
}


Eigen::VectorXd PinocchioInterface::GetDynamics()
{
    Eigen::Matrix<double, DoF, 1> tmp_torque[Leg_N];
    const double torque_limit_=300;

    for (size_t i = 0; i < Leg_N; i++)
    {
        // CTC
        tmp_torque[i] = M_[i] * J_[i].inverse() * (acc_d_[i] + Kp_ * err_[i] + Kd_ * err_dot_[i] - dJ_[i] * dq_[i]) + B_[i];

        for (size_t j = 0; j < DoF; j++)
        {
            if(tmp_torque[i](j,0)>=torque_limit_)
            {
                tmp_torque[i](j,0)=300;
                std::cout<<"Leg ["<<i<<"] "<<" joint ["<<j<<"] torque Max"<<std::endl;
            }
            if(tmp_torque[i](j,0)<=-torque_limit_)
            {
                tmp_torque[i](j,0)=-300;
                std::cout<<"Leg ["<<i<<"] "<<" joint ["<<j<<"] torque Min"<<std::endl;
            }
        }
        target_torque_.block<DoF, 1>(i * DoF, 0) = tmp_torque[i];
    }

    return target_torque_;
}

void PinocchioInterface::pinocchio_print()
{
    for (size_t i = 0; i < Leg_N; i++)
    {
        std::cout << "M [" << i << "]"
                  << "\n"
                  << M_[i] << std::endl;
        // std::cout<<"B ["<<i<<"]"<<"\n"<<B_[i]<<std::endl;
        // std::cout<<"J_ ["<<i<<"]"<<"\n"<<J_[i]<<std::endl;
        // std::cout<<"dJ_ ["<<i<<"]"<<"\n"<<dJ_[i]<<std::endl;
        // std::cout<<"pos_d_ ["<<i<<"]"<<"\n"<<pos_d_[i]<<std::endl;
        // std::cout<<"vel_d_ ["<<i<<"]"<<"\n"<<vel_d_[i]<<std::endl;
        // std::cout<<"acc_d_ ["<<i<<"]"<<"\n"<<acc_d_[i]<<std::endl;
        // std::cout<<"pos_ ["<<i<<"]"<<"\n"<<pos_[i]<<std::endl;
        // std::cout<<"vel_ ["<<i<<"]"<<"\n"<<vel_[i]<<std::endl;
        // std::cout<<"torque ["<<i<<"]"<<"\n"<<target_torque_.block<DoF, 1>(i * DoF, 0)<<std::endl;
        // std::cout<<"Position Error ["<<i<<"]"<<"\n"<<err_[i]<<std::endl;
        // std::cout<<"Velocidy Error ["<<i<<"]"<<"\n"<<err_dot_[i]<<std::endl;
        // std::cout<<"\n"<<std::endl;
        // std::cout << "Kp : " << Kp_ << std::endl;
    }
    // Eigen::Matrix4d M1_t, M1_s;
    // M1_t = M_[L1].triangularView<Eigen::Upper>();
    // M1_s = M_[L1].selfadjointView<Eigen::Upper>();
    // std::cout<<"M ["<<"L1"<<"]"<<"\n"<<M_[L1]<<std::endl;
    // std::cout<<"M ["<<"L1"<<"]_triangularView"<<"\n"<<M1_t<<std::endl;
    // std::cout<<"M ["<<"L1"<<"]selfadjointView"<<"\n"<<M1_s<<std::endl;
    // M_[L1].triangularView<Eigen::Lower>() = M_[L1].transpose();
    // std::cout<<"M_[L1]_total : "<<M_[L1]<<"\n"<<std::endl;
    std::cout << "\n"
              << std::endl;
}

