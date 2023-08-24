/*
  main header
  made by JS
*/

#ifndef MAIN_H_

#define MAIN_H_


// **********Basic libraries*********//
#include <sys/mman.h>
#include <signal.h>
#include <unistd.h>
// for time check
#include <time.h>
#include <iostream>
#include <stdio.h>
#include <sys/time.h>
#include <inttypes.h>

// **********ROS libraries*********//
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <std_msgs/Float64.h>
#include <functional>
#include <ignition/math/Vector3.hh>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include "gazebo_msgs/LinkStates.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <functional>

#include <math.h>

#define NUM_OF_MC   6
#define PI          3.141592653589793238462
#define R2D         180/PI
#define D2R         PI/180


#define PF_CR_R        "\x1b[31;1m" //red
#define PF_CR_GR       "\x1b[32;1m"
#define PF_CR_Y        "\x1b[33;1m"
#define PF_CR_BL       "\x1b[34;1m"
#define PF_CR_PRPL     "\x1b[35;1m"

#define PF_CR_bR       "\x1b[91;1m"
#define PF_CR_bGR      "\x1b[92;1m"
#define PF_CR_bY       "\x1b[93;1m"
#define PF_CR_bBL      "\x1b[94;1m"
#define PF_CR_bPRPL    "\x1b[95;1m"

#define PF_NC          "\x1b[0m"

#define PF_CR_RESET    "\x1b[0m"

// ===== rbdl ===== //
#define RB1_500e_MODEL_DIR "/home/js/catkin_ws/src/RobotControl2023/urdf/RB1_500e.urdf"
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "Eigen/Dense"
#ifndef RBDL_BUILD_ADDON_URDFREADER
   #error "Error: RBDL addon URDFReader not enabled."
#endif

#include "Eigen/Dense"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using Eigen::MatrixXd;
using Eigen::VectorXd;

typedef Eigen::Matrix<double, NUM_OF_MC, 1> VectorM;

namespace gazebo
{
    class RB1_500E : public ModelPlugin
    {
        float dt;
        double time = 0;
        int CNT = 0;
        int PRINT_CNT = 0; //[ms]

        physics::LinkPtr BASE_LINK;
        physics::LinkPtr LINK1;
        physics::LinkPtr LINK2;
        physics::LinkPtr LINK3;
        physics::LinkPtr LINK4;
        physics::LinkPtr LINK5;
        physics::LinkPtr LINK6;

        physics::JointPtr fixed_to_ground;
        physics::JointPtr JT0;
        physics::JointPtr JT1;
        physics::JointPtr JT2;
        physics::JointPtr JT3;
        physics::JointPtr JT4;
        physics::JointPtr JT5;
        
        physics::ModelPtr model;

        common::Time last_update_time;
        event::ConnectionPtr update_connection_;

        ros::NodeHandle nh;   

        public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
        void UpdateAlgorithm();
        void print_task();
        void GET_RB_INFO();

        public:
        MatrixXd jointToTransform01(VectorXd q);
        MatrixXd jointToTransform12(VectorXd q);
        MatrixXd jointToTransform23(VectorXd q);
        MatrixXd jointToTransform34(VectorXd q);
        MatrixXd jointToTransform45(VectorXd q);
        MatrixXd jointToTransform56(VectorXd q);
        MatrixXd jointToTransform6E();

        VectorXd jointToPosVec(VectorXd q);
        MatrixXd jointToRotMat(VectorXd q);
        VectorXd rotMatToEuler(MatrixXd rot);
        
        VectorXd enc_q_vec = VectorXd::Zero(6);


        public:
        float enc_q[NUM_OF_MC];
        float p_enc_q[NUM_OF_MC] = {0.0, };
        float enc_Dq[NUM_OF_MC] = {0.0, };
        float des_q[NUM_OF_MC];
        float tau[NUM_OF_MC];
        float tau_nlt[NUM_OF_MC];

        float kp[NUM_OF_MC];
        float kd[NUM_OF_MC];

        // RBDL
        RigidBodyDynamics::Model* rb1_500e_model = new Model();
        typedef struct{
            VectorM q;
            VectorM Dq;
            VectorM D2q;

        } states;
        states rb;

    };
    GZ_REGISTER_MODEL_PLUGIN(RB1_500E);
}


#endif
