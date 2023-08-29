// Header file for C++
#include "main.h"

#define GazeboVersion 11

using namespace std;
using namespace gazebo;
RB1_500E _rb;

// using Eigen::MatrixXd;
// using Eigen::VectorXd;

void gazebo::RB1_500E::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    /*
     * Loading model data and initializing the system before simulation 
     */

    //* model.sdf file based model data input to [physics::ModelPtr model] for gazebo simulation

    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "RB1_500E");
    ROS_INFO("PLUGIN_LOADED");

    // printf("\n Loading Complete \n");
    
    this->model = _model;

    #if GazeboVersion < 8
        this->BASE_LINK = this->model->GetLink("BASE_LINK");
        this->LINK1 = this->model->GetLink("LINK1");
        this->LINK2 = this->model->GetLink("LINK2");
        this->LINK3 = this->model->GetLink("LINK3");
        this->LINK4 = this->model->GetLink("LINK4");
        this->LINK5 = this->model->GetLink("LINK5");
        this->LINK6 = this->model->GetLink("LINK6");
        
        this->JT0 = this->model->GetJoint("JT0");
        this->JT1 = this->model->GetJoint("JT1");
        this->JT2 = this->model->GetJoint("JT2");
        this->JT3 = this->model->GetJoint("JT3");
        this->JT4 = this->model->GetJoint("JT4");
        this->JT5 = this->model->GetJoint("JT5");
        this->last_update_time = this->model->GetWorld()->GetSimTime();
    #else
        BASE_LINK = this->model->GetLink("BASE_LINK");
        LINK1 = this->model->GetLink("LINK1");
        LINK2 = this->model->GetLink("LINK2");
        LINK3 = this->model->GetLink("LINK3");
        LINK4 = this->model->GetLink("LINK4");
        LINK5 = this->model->GetLink("LINK5");
        LINK6 = this->model->GetLink("LINK6");
        
        JT0 = this->model->GetJoint("JT0");
        JT1 = this->model->GetJoint("JT1");
        JT2 = this->model->GetJoint("JT2");
        JT3 = this->model->GetJoint("JT3");
        JT4 = this->model->GetJoint("JT4");
        JT5 = this->model->GetJoint("JT5");

        last_update_time = model->GetWorld()->SimTime();
    #endif
    
    //* RBDL setting
    // printf("\n RBDL load start \n");
    // Addons::URDFReadFromFile(RB1_500e_MODEL_DIR, _rb.rb1_500e_model, false, false);
    // _rb.rb1_500e_model->gravity = Vector3d(0., 0., -9.81);

    // printf("\n RBDL load Complete \n");
    
    //* setting for getting dt
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&RB1_500E::UpdateAlgorithm, this));

    printf("\n System load complete \n");

}

void gazebo::RB1_500E::UpdateAlgorithm()
{   
    #if GazeboVersion < 8
        common::Time current_time = this->model->GetWorld()->GetSimTime();
        
        _rb_info.rb.Inc_q(0) = this->JT0->GetAngle(2).Radian();
        _rb_info.rb.Inc_q(1) = this->JT1->GetAngle(1).Radian();
        _rb_info.rb.Inc_q(2) = this->JT2->GetAngle(1).Radian();
        _rb_info.rb.Inc_q(3) = this->JT3->GetAngle(2).Radian();
        _rb_info.rb.Inc_q(4) = this->JT4->GetAngle(1).Radian();
        _rb_info.rb.Inc_q(5) = this->JT5->GetAngle(2).Radian();

    #else
        common::Time current_time = model->GetWorld()->SimTime();

        _rb.enc_q[0] = JT0->Position(2);
        _rb.enc_q[1] = JT1->Position(1);
        _rb.enc_q[2] = JT2->Position(1);
        _rb.enc_q[3] = JT3->Position(2);
        _rb.enc_q[4] = JT4->Position(1);
        _rb.enc_q[5] = JT5->Position(2);

        _rb.enc_q_vec << _rb.enc_q[0], _rb.enc_q[1], _rb.enc_q[2], 
                         _rb.enc_q[3], _rb.enc_q[4], _rb.enc_q[5];

    #endif
    gazebo::RB1_500E::GET_RB_INFO();

    dt = current_time.Double() - this->last_update_time.Double();
    time = time + dt;

    // _rb.des_q[0] = 0.0; _rb.des_q[1] = 0.0; _rb.des_q[2] = 0.0;
    // _rb.des_q[3] = -0.0; _rb.des_q[4] = -0.0; _rb.des_q[5] = -0.0;

    _rb.des_q[0] = 0.0; _rb.des_q[1] = 0.0; _rb.des_q[2] = 0.0;
    _rb.des_q[3] = -0.0; _rb.des_q[4] = -0.0; _rb.des_q[5] = -0.0;

    // step input //
    _rb.kp[0] = 20; _rb.kp[1] = 50; _rb.kp[2] = 50;
    _rb.kp[3] = 30; _rb.kp[4] = 30; _rb.kp[5] = 10;

    _rb.kd[0] = 0.5; _rb.kd[1] = 0.5; _rb.kd[2] = 0.1;
    _rb.kd[3] = 0.1; _rb.kd[4] = 0.1; _rb.kd[5] = 0.1;
    
    for(int i = 0; i < NUM_OF_MC; i++)
    {
        _rb.tau[i] = _rb.kp[i]*(_rb.des_q[i]*D2R - _rb.enc_q[i]) + _rb.kd[i]*(0.0 - _rb.enc_Dq[i]);
    }

    this->JT0 -> SetForce(2, _rb.tau[0]); //setForce(axis,Force value)
    this->JT1 -> SetForce(1, _rb.tau[1]);
    this->JT2 -> SetForce(1, _rb.tau[2]);
    this->JT3 -> SetForce(2, _rb.tau[3]);
    this->JT4 -> SetForce(1, _rb.tau[4]);
    this->JT5 -> SetForce(2, _rb.tau[5]);

    gazebo::RB1_500E::print_task();
   
    this->last_update_time = current_time;
    
}
void gazebo::RB1_500E::GET_RB_INFO(void)
{
    for(int i = 0; i < NUM_OF_MC; i++)
    {
        _rb.enc_Dq[i] = (_rb.enc_q[i] - _rb.p_enc_q[i])/0.001;

        _rb.p_enc_q[i] = _rb.enc_q[i];

        rb.q(i) = _rb.enc_q[i];
        rb.Dq(i) = _rb.enc_Dq[i];
    }

    // Calculate inverse dynamics
    // VectorNd nlt = VectorNd::Zero(_rb.rb1_500e_model->dof_count);
    // NonlinearEffects(*_rb.rb1_500e_model, rb.q, rb.Dq, nlt);


    // for(int i = 0; i < NUM_OF_MC; i++)
    // {
    //     _rb.tau_nlt[i] = nlt(i);
    // }
    


}
void gazebo::RB1_500E::print_task(void)
{
    PRINT_CNT = PRINT_CNT + 1;

    if(PRINT_CNT%150 == 0)
    {
        std::cout << PF_CR_bGR "JT.0: " << _rb.enc_q[0]*R2D << " , JT.1: " << _rb.enc_q[1]*R2D << " , JT.2: " << _rb.enc_q[2]*R2D << PF_NC << std::endl;
        std::cout << PF_CR_bGR "JT.3: " << _rb.enc_q[3]*R2D << " , JT.4: " << _rb.enc_q[4]*R2D << " , JT.5: " << _rb.enc_q[5]*R2D << PF_NC << std::endl << std::endl;

        std::cout << PF_CR_bGR "dJT.0: " << _rb.enc_Dq[0] << " , dJT.1: " << _rb.enc_Dq[1] << " , dJT.2: " << _rb.enc_Dq[2] << PF_NC << std::endl;
        std::cout << PF_CR_bGR "dJT.3: " << _rb.enc_Dq[3] << " , dJT.4: " << _rb.enc_Dq[4] << " , dJT.5: " << _rb.enc_Dq[5] << PF_NC << std::endl << std::endl;


        std::cout << PF_CR_bGR "Tau.0: " << _rb.tau[0] << " , Tau.1: " << _rb.tau[1] << " , Tau.2: " << _rb.tau[2] << PF_NC << std::endl;
        std::cout << PF_CR_bGR "Tau.3: " << _rb.tau[3] << " , Tau.4: " << _rb.tau[4] << " , Tau.5: " << _rb.tau[5] << PF_NC << std::endl << std::endl;

        // std::cout << PF_CR_BL "T01(0): " << std::endl << _rb.jointToTransform01(_rb.enc_q_vec)<< PF_NC << std::endl;
    }
    
}
