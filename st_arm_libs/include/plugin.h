#ifndef JINU_MANIPULATOR_CONTROL_PLUGIN_H
#define JINU_MANIPULATOR_CONTROL_PLUGIN_H

#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

#include <sensor_msgs/JointState.h>

#include <boost/bind.hpp>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <functional>
#include <eigen3/Eigen/Dense>
#include <armadillo>
#include <fstream>

#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <ignition/math.hh>

#include <rbdl/rbdl.h>
#include <arm_6dof_pkgs/CTCMsg.h>

using namespace std;
using namespace Eigen;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

typedef struct EndPoint
{
  Eigen::MatrixXd T_matrix;
} ENDPOINT;

typedef struct Joint_torque
{
  double torque;
} JOINT;

#define NUM_OF_JOINT_AND_TOOL 7

// ************* 단위변환 ************************//
#define PI 3.14159265358979
#define M2R 2*PI/4096
#define deg2rad		0.017453292519943
#define rad2deg		57.295779513082323

#define L1 0.10
#define L2 0.25
#define L3 0.25
#define L4 0.00
#define L5 0.10
#define L6 0.06
#define g 9.81;
#define m_Link1 0.573
#define m_Link2 0.729
#define m_Link3 0.490
#define m_Link4 0.136
#define m_Link5 0.148
#define m_Link6 0.239
#define M1 2.315 // m_Arm
#define M2 1.742 //m_Link2+m_Link3+m_Link4+m_Link5+m_Link6;
#define M3 1.013 //m_Link3+m_Link4+m_Link5+m_Link6;
#define M4 0.523 //m_Link4+m_Link5+m_Link6;
#define M5 0.387 //m_Link5+m_Link6;
#define M6 0.239 //m_Link6;
#define inner_dt 0.001

// ***************행렬 선언*********************//
double ref_Arm_th[6] = {0.,};

//*************** Trajectroy Variables**************//    
double step_time = 0;
double cnt_time = 0;
unsigned int cnt = 0;
unsigned int start_flag = 0;

float qw = 1;
float qx = 0;
float qy = 0;
float qz = 0;



////////////CoM IK Variables /////////////////
Eigen::Vector3d Act_CoM; //_X = 0, Act_CoM_Y = 0, Act_CoM_Z = 0;
Eigen::Vector3d Act_CoM1; Eigen::Vector3d Act_CoM2; Eigen::Vector3d Act_CoM3; Eigen::Vector3d Act_CoM4; Eigen::Vector3d Act_CoM5; Eigen::Vector3d Act_CoM6;
Eigen::Vector3d Act_CoM_Vel;
Eigen::Vector3d Act_Pos_Vel;
Eigen::Vector3d Pre_Act_CoM;
Eigen::Vector3d Act_Ori;
Eigen::Vector3d Act_Pos;
Eigen::Vector3d Act_Ori_Vel;
Eigen::Vector3d Pre_Act_Ori;
Eigen::Vector3d Pre_Act_Pos;
Eigen::Vector3d ref_Pos;
Eigen::Vector3d ref_CoM;
double fi, theta, csi;

//*************** RBDL variable ***************//
typedef struct Arm_rbdl_model_
{
  Model* rbdl_model = new Model();
  VectorNd Q, QDot, QDDot, prevQ, prevQDot, Tau;

  unsigned int body_LINK1_id, body_LINK2_id, body_LINK3_id, body_LINK4_id, body_LINK5_id, body_LINK6_id;//id have information of the body
  Body body_LINK1, body_LINK2, body_LINK3, body_LINK4, body_LINK5, body_LINK6;//make body.
  Joint joint_LINK1_YAW, joint_LINK2_PITCH, joint_LINK3_PITCH, joint_LINK4_PITCH, joint_LINK5_ROLL, joint_LINK6_YAW;//make joint
  Math::Matrix3d bodyI_LINK1, bodyI_LINK2, bodyI_LINK3, bodyI_LINK4, bodyI_LINK5, bodyI_LINK6;//Inertia of Body
} A_RBDL;

A_RBDL Arm;


namespace gazebo
{
  class arm_6dof_plugin : public ModelPlugin
  {
    // ************* Model variables ****************//
    physics::ModelPtr model;

    physics::LinkPtr Base;
    physics::LinkPtr Link1;
    physics::LinkPtr Link2;
    physics::LinkPtr Link3;
    physics::LinkPtr Link4;
    physics::LinkPtr Link5;
    physics::LinkPtr Link6;

    physics::JointPtr Joint1;
    physics::JointPtr Joint2;
    physics::JointPtr Joint3;
    physics::JointPtr Joint4;
    physics::JointPtr Joint5;
    physics::JointPtr Joint6;

    // ************* Joint space variables ****************// 기본이 열벡터임에 주의할것!

    VectorXd actual_joint_pos = VectorXd::Zero(6);
    VectorXd pre_actual_joint_pos = VectorXd::Zero(6);
    VectorXd actual_joint_vel = VectorXd::Zero(6);
    VectorXd pre_actual_joint_vel = VectorXd::Zero(6);
    VectorXd actual_joint_acc = VectorXd::Zero(6);
    VectorXd Kp_v = VectorXd::Zero(6);
    VectorXd Kd_v = VectorXd::Zero(6);
    VectorXd Kp_s = VectorXd::Zero(3);
    VectorXd Kd_s = VectorXd::Zero(3);
    

    // *************Time variables ****************//
    common::Time last_update_time;
    event::ConnectionPtr update_connection;
    double dt;
    double time = 0;
    common::Time current_time;

    // *************Kinematic variables ****************//
    VectorXd Arm_CoM = VectorXd::Zero(6);

    MatrixXd A0 = MatrixXd::Zero(4,4); MatrixXd A1 = MatrixXd::Zero(4,4); MatrixXd A2 = MatrixXd::Zero(4,4); MatrixXd A3 = MatrixXd::Zero(4,4);
    MatrixXd A4 = MatrixXd::Zero(4,4); MatrixXd A5 = MatrixXd::Zero(4,4); MatrixXd A6 = MatrixXd::Zero(4,4);
    MatrixXd T00 = MatrixXd::Zero(4,4); MatrixXd T01 = MatrixXd::Zero(4,4); MatrixXd T02 = MatrixXd::Zero(4,4); MatrixXd T03 = MatrixXd::Zero(4,4);
    MatrixXd T04 = MatrixXd::Zero(4,4); MatrixXd T05 = MatrixXd::Zero(4,4); MatrixXd T06 = MatrixXd::Zero(4,4);
    MatrixXd T12 = MatrixXd::Zero(4,4); MatrixXd T23 = MatrixXd::Zero(4,4); MatrixXd T45 = MatrixXd::Zero(4,4);
    MatrixXd T56 = MatrixXd::Zero(4,4);

    Eigen::Matrix3d R_act, R_ref;
    Eigen::Vector3d R1_act, R2_act, R3_act, R1_ref, R2_ref, R3_ref;

    Eigen::VectorXd Init_Pose = VectorXd::Zero(6); Eigen::VectorXd ref_Pose = VectorXd::Zero(6);
    // ************* ROS Communication ****************//
    ros::NodeHandle n;
	  // ************* publisher ************************//
    ros::Publisher P_Times;
    ros::Publisher P_ros_msg;
    
    ros::Publisher P_actual_joint_pos_1;
    ros::Publisher P_actual_joint_pos_2;
    ros::Publisher P_actual_joint_pos_3;
    ros::Publisher P_actual_joint_pos_4;
    ros::Publisher P_actual_joint_pos_5;
    ros::Publisher P_actual_joint_pos_6;
    ros::Publisher P_actual_R_Ankle_R_J;

    ros::Publisher P_actual_joint_torque_1;
    ros::Publisher P_actual_joint_torque_2;
    ros::Publisher P_actual_joint_torque_3;
    ros::Publisher P_actual_joint_torque_4;
    ros::Publisher P_actual_joint_torque_5;
    ros::Publisher P_actual_joint_torque_6;

    ros::Publisher P_actual_Pos_X;
    ros::Publisher P_actual_Pos_Y;
    ros::Publisher P_actual_Pos_Z;
    ros::Publisher P_actual_Pos_Roll;
    ros::Publisher P_actual_Pos_Pitch;
    ros::Publisher P_actual_Pos_Yaw;
    ros::Publisher P_ref_Pos_X;
    ros::Publisher P_ref_Pos_Y;
    ros::Publisher P_ref_Pos_Z;
    ros::Publisher P_ref_Pos_Roll;
    ros::Publisher P_ref_Pos_Pitch;
    ros::Publisher P_ref_Pos_Yaw;    

    ros::Publisher P_Arm_CoM_X;
    ros::Publisher P_Arm_CoM_Y;
    ros::Publisher P_Arm_CoM_Z;
    ros::Publisher P_Arm_CoM_Roll;
    ros::Publisher P_Arm_CoM_Pitch;
    ros::Publisher P_Arm_CoM_Yaw;

    ros::Publisher P_ref_CoM_X;
    ros::Publisher P_ref_CoM_Y;
    ros::Publisher P_ref_CoM_Z;
    ros::Publisher P_Act_Ori_Vel_X;
    ros::Publisher P_Act_Ori_Vel_Y;
    ros::Publisher P_Act_Ori_Vel_Z;
    ros::Publisher P_Act_X_CoM;
    ros::Publisher P_Act_Y_CoM;
    ros::Publisher P_Act_Z_CoM;
  

    // ************ msg ***************** //
    std_msgs::Float64 m_Times;
    
    std_msgs::Float64MultiArray m_ros_msg;
    
    std_msgs::Float64 m_actual_joint_pos_1;
    std_msgs::Float64 m_actual_joint_pos_2;
    std_msgs::Float64 m_actual_joint_pos_3;
    std_msgs::Float64 m_actual_joint_pos_4;
    std_msgs::Float64 m_actual_joint_pos_5;
    std_msgs::Float64 m_actual_joint_pos_6;

    std_msgs::Float64 m_actual_joint_torque_1;
    std_msgs::Float64 m_actual_joint_torque_2;
    std_msgs::Float64 m_actual_joint_torque_3;
    std_msgs::Float64 m_actual_joint_torque_4;
    std_msgs::Float64 m_actual_joint_torque_5;
    std_msgs::Float64 m_actual_joint_torque_6;

    std_msgs::Float64 m_actual_Pos_X;
    std_msgs::Float64 m_actual_Pos_Y;
    std_msgs::Float64 m_actual_Pos_Z;
    std_msgs::Float64 m_actual_Roll;
    std_msgs::Float64 m_actual_Pitch;
    std_msgs::Float64 m_actual_Yaw;
    std_msgs::Float64 m_ref_Pos_X;
    std_msgs::Float64 m_ref_Pos_Y;
    std_msgs::Float64 m_ref_Pos_Z;
    std_msgs::Float64 m_ref_Roll;
    std_msgs::Float64 m_ref_Pitch;
    std_msgs::Float64 m_ref_Yaw;

    std_msgs::Float64 m_Arm_CoM_X;
    std_msgs::Float64 m_Arm_CoM_Y;
    std_msgs::Float64 m_Arm_CoM_Z;
    std_msgs::Float64 m_Arm_CoM_Roll;
    std_msgs::Float64 m_Arm_CoM_Pitch;
    std_msgs::Float64 m_Arm_CoM_Yaw;

    std_msgs::Float64 m_Act_X_CoM;
    std_msgs::Float64 m_Act_Y_CoM;
    std_msgs::Float64 m_Act_Z_CoM;
    std_msgs::Float64 m_ref_CoM_X;
    std_msgs::Float64 m_ref_CoM_Y;
    std_msgs::Float64 m_ref_CoM_Z;
    std_msgs::Float64 m_Act_Ori_Vel_X;
    std_msgs::Float64 m_Act_Ori_Vel_Y;
    std_msgs::Float64 m_Act_Ori_Vel_Z;

    VectorXd TmpData = VectorXd::Zero(50);
    ros::Subscriber server_sub1;
    ros::Subscriber server_sub2;
    ros::Subscriber server_sub3;
    ros::Subscriber server_sub4;

    ros::Subscriber open_manipulator_joint_states_sub_;

    std::vector<double> present_joint_angle_;

    // ************* Structure variables ****************//
    JOINT* joint;
    JOINT* old_joint;
    JOINT* prev_out_joint;
    JOINT* out_joint;

    enum ControlMode
    {
      IDLE = 0,
      CoM_EEOri,
      JOINT_Angle,
      EE_Pos_Ori,
      Jinu_GC,
      Jinu_EE,
      Jinu_EE_Rot,
      Jinu_EE_Rot_Quat
    };
    enum ControlMode CONTROL_MODE;

    // ************* Functions ****************//
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
    void UpdateAlgorithm();

    void RBDL_INIT();
    void rbdl_variable_init(A_RBDL &rbdl);
    void Arm_Model(A_RBDL &rbdl);
    void GetLinks();
    void GetJoints();
    void LinksCoMRead();
    void InitROSPubSetting();

    void EncoderRead();
    void torque_interpolation();
    void jointController();
    void ROSMsgPublish();
    void Callback1(const std_msgs::Int32Ptr &msg);
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &mgs);

    void PostureGeneration();

    void RBDL_variable_update();
    void Ref_Ori();
    void Calc_CTC_Torque(A_RBDL &rbdl);
    void Calc_CTC_Torque_JointSpace(A_RBDL &rbdl , double q[6], double qdot[6], double qddot[6]);
    void Calc_ZMP();

    void Init_Pos_Traj();
    void Input_CoM_EEOri();
    void Input_Joint_Angle();
    void Input_EE_Pos_Ori();
    void Jinu_Gravity_Compensation();
    void Jinu_Input_EE_Pos_Ori();
    void Jinu_Input_EE_Pos_Ori_Rot();
    void Jinu_Input_EE_Pos_Ori_Rot_Quat();

    void Print(void); //Print function
  };
  GZ_REGISTER_MODEL_PLUGIN(arm_6dof_plugin); //model plugin 등록함수
}

/*
void gazebo::arm_6dof_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr) {} //처음키면 한번 실행되는 함수
void gazebo::arm_6dof_plugin::RBDL_INIT() {}
void gazebo::arm_6dof_plugin::rbdl_variable_init(A_RBDL &rbdl) {}
void gazebo::arm_6dof_plugin::Arm_Model(A_RBDL &rbdl) {}
void gazebo::arm_6dof_plugin::UpdateAlgorithm() {}      // 여러번 실행되는 함수
void gazebo::arm_6dof_plugin::GetLinks() {}
void gazebo::arm_6dof_plugin::GetJoints(){}
void gazebo::arm_6dof_plugin::InitROSPubSetting(){}
void gazebo::arm_6dof_plugin::LinksCoMRead() {}
void gazebo::arm_6dof_plugin::EncoderRead() {}
void gazebo::arm_6dof_plugin::jointController() {}      //limit torque 설정
void gazebo::arm_6dof_plugin::Callback1(const std_msgs::Int32Ptr & msg) {}
void gazebo::arm_6dof_plugin::PostureGeneration() {}
void gazebo::arm_6dof_plugin::RBDL_variable_update() {}
void gazebo::arm_6dof_plugin::Init_Pos_Traj() {}        // 0
void gazebo::arm_6dof_plugin::Input_CoM_EEOri() {}      // 1
void gazebo::arm_6dof_plugin::Input_Joint_Angle() {}    // 2
void gazebo::arm_6dof_plugin::Input_EE_Pos_Ori() {}     // 3
void gazebo::arm_6dof_plugin::Print() {}
void gazebo::arm_6dof_plugin::ROSMsgPublish() {}
*/

#endif  // end of the JINU_MANIPULATOR_CONTROL_PLUGIN_H