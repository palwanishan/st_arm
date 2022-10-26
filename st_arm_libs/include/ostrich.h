#ifndef OSTRICH_H
#define OSTRICH_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <eigen3/Eigen/Dense>

#include <ignition/math.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

#include <string.h>
#include <iostream>
#include <boost/bind.hpp>
#include <vector>

#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <rbdl/rbdl.h>


#define PI 3.14159265358979
#define M2R 2*PI/4096
#define deg2rad		0.017453292519943
#define rad2deg		57.295779513082323
#define inner_dt 0.001

#define M_Robot  19.320
#define M_base  4.9133
#define M_L_HIP_Y  0.9518
#define M_L_HIP_R  0.8580
#define M_L_HIP_P  2.4902
#define M_L_KNEE_P  1.5349
#define M_L_ANKLE_P  0.3672
#define M_L_ANKLE_R  1.226
#define M_R_HIP_Y  0.9518
#define M_R_HIP_R  0.8580
#define M_R_HIP_P  2.4902
#define M_R_KNEE_P  1.5349
#define M_R_ANKLE_P  0.3672
#define M_R_ANKLE_R  1.226


using gazebo::physics::ModelPtr;
using gazebo::physics::LinkPtr;
using gazebo::sensors::ImuSensorPtr;
using gazebo::sensors::SensorPtr;
using gazebo::physics::JointPtr;
using gazebo::event::ConnectionPtr;
using gazebo::common::Time;

using namespace std;
using namespace Eigen;

using Eigen::Vector3d;
using Eigen::Quaterniond;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


FILE *Torque_data1;
FILE *Torque_data2;
FILE *Torque_data3;
FILE *Torque_data4;

typedef struct Air_rbdl_model_
{
  Model* rbdl_model = new Model();
  VectorNd Q, QDot, QDDot, prevQ, prevQDot, Tau, Foot_Pos, Foot_Pos_dot, Des_X, Des_XDot, Des_XDDot, torque_CTC, Old_Des_X, Old_Des_XDot, Old_Des_XDDot, New_Des_X, New_Des_XDot, New_Des_XDDot, Kp, Kv, Friction_Torque;
  MatrixNd A_Jacobian, prev_A_Jacobian, A_Jacobian_dot, Inv_A_Jacobian;

  unsigned int body_Base_id, body_hip_y_id, body_hip_r_id, body_hip_p_id, body_knee_p_id, body_ankle_p_id, body_ankle_r_id, body_foot_id;//id have information of the body
  Body /*body_Base,*/ body_hip_y, body_hip_r, body_knee_p, body_hip_p, body_ankle_p, body_ankle_r, body_foot;//make body.
  Joint /*joint_Base,*/ joint_HIP_YAW, joint_HIP_ROLL, joint_HIP_PITCH, joint_KNEE_PITCH, joint_ANKLE_PITCH, joint_ANKLE_ROLL , joint_FOOT; //make joint
  Math::Matrix3d /*bodyI_Base,*/ bodyI_hip_y, bodyI_hip_r, bodyI_hip_p, bodyI_knee_p, bodyI_ankle_p, bodyI_ankle_r, bodyI_foot;//Inertia of Body
} A_RBDL;

A_RBDL A_L, A_R; //Swing phase상태(발이 공중에서 땅을 디디기 직전까지) 왼발, 오른발 좌표계가 골반에서 풀기시작함.


namespace gazebo
{
    class Ostrich_simple : public ModelPlugin
    {
        float qw = 1;
        float qx = 0;
        float qy = 0;
        float qz = 0;

        double step_time = 0;
        double cnt_time = 0;
        unsigned int cnt = 0;
        double P_RL_th[8]={0.,0.,0.,0.,0.,0.,0.,0.} , P_LL_th[8]={0.,0.,0.,0.,0.,0.,0.,0.};
        double P_RL_th2[8]={0.,0.,0.,0.,0.,0.,0.,0.} , P_LL_th2[8]={0.,0.,0.,0.,0.,0.,0.,0.};
        double Theo_RL_th[8] = {0.,0.,0.,0.,0.,0.,0.,0.}, Theo_LL_th[8] = {0.,0.,0.,0.,0.,0.,0.,0.};
        double Init_th[12] = {0.,};


        typedef struct Joint_states {
            double torque;
            double th;
            double th_dot;
        } JOINT;


        JOINT* joint;
        JOINT* old_joint;
        JOINT* prev_out_joint;
        JOINT* out_joint;

        enum ControlMode
        {
            IDLE = 0,
            walk_ready = 1 ,
            motion_TS_PD = 2 ,
            gravitiy_compensation_air = 3 
        };
        enum ControlMode CONTROL_MODE;

        // float L0 = 0.1;
        // float L1 = 0.081;
        // float L2 = 0.103;
        // float L3 = 0.27;
        // float L4 = 0.042;
        // float L5 = 0.27;
        // float L6 = 0.045;
        // float L7 = 0.081;

        float L0 = 0.1;
        float L1 = 0.081;
        float L2 = 0.111;
        float L3 = 0.27;
        float L4 = 0.047;
        float L5 = 0.272;
        float L6 = 0.039;
        float L7 = 0.081;

        float L8 = 0.1;
        float L9 = 0.081;
        float L10 = 0.111;
        float L11 = 0.27;
        float L12 = 0.047;
        float L13 = 0.272;
        float L14 = 0.039;
        float L15 = 0.081;

        public:
        VectorXd actual_joint_pos = VectorXd::Zero(12);
        VectorXd actual_joint_torque = VectorXd::Zero(12);
        VectorXd actual_joint_vel = VectorXd::Zero(12);
        VectorXd Kp_s = VectorXd::Zero(12);
        VectorXd Kd_s = VectorXd::Zero(6);
        VectorXd pre_actual_joint_vel = VectorXd::Zero(12);
        VectorXd pre_actual_joint_pos = VectorXd::Zero(12);

        Quaterniond body_quat;
        
        double body_roll;
        double body_pitch;            

        ModelPtr model;

        SensorPtr Sensor;
        ImuSensorPtr BODY_IMU;

        LinkPtr BASE_LINK;
        LinkPtr L_ANKLE_P_LINK;
        LinkPtr L_ANKLE_R_LINK;
        LinkPtr L_HIP_P_LINK;
        LinkPtr L_HIP_R_LINK;
        LinkPtr L_HIP_Y_LINK;
        LinkPtr L_KNEE_P_LINK;
        LinkPtr L_MAINFOOT_P_LINK;
        LinkPtr L_SUBFOOT_P_LINK;

        LinkPtr R_ANKLE_P_LINK;
        LinkPtr R_ANKLE_R_LINK;
        LinkPtr R_HIP_P_LINK;
        LinkPtr R_HIP_R_LINK;
        LinkPtr R_HIP_Y_LINK;
        LinkPtr R_KNEE_P_LINK;
        LinkPtr R_MAINFOOT_P_LINK;
        LinkPtr R_SUBFOOT_P_LINK;

        
        JointPtr L_ANKLE_P_JOINT;
        JointPtr L_ANKLE_R_JOINT;
        JointPtr L_HIP_P_JOINT;
        JointPtr L_HIP_R_JOINT;
        JointPtr L_HIP_Y_JOINT;
        JointPtr L_KNEE_P_JOINT;
        JointPtr L_MAINFOOT_P_JOINT;
        JointPtr L_SUBFOOT_P_JOINT;
        
        JointPtr R_ANKLE_P_JOINT;
        JointPtr R_ANKLE_R_JOINT;
        JointPtr R_HIP_P_JOINT;
        JointPtr R_HIP_R_JOINT;
        JointPtr R_HIP_Y_JOINT;
        JointPtr R_KNEE_P_JOINT;
        JointPtr R_MAINFOOT_P_JOINT;
        JointPtr R_SUBFOOT_P_JOINT;
        

        const std::vector<std::string> joint_names = {
        "L_HIP_Y", "L_HIP_R", "L_HIP_P", "L_KNEE_P", "L_ANKLE_P", "L_ANKLE_R",
        "L_MAINFOOT_P", "L_SUBFOOT_P", "R_HIP_Y", "R_HIP_R", "R_HIP_P", "R_KNEE_P",
        "R_ANKLE_P", "R_ANKLE_R", "R_MAINFOOT_P", "R_SUBFOOT_P"
        };
        
        ros::NodeHandle node_handle;
        ros::Publisher p_joint_state;
        ros::Publisher p_ee_pose;
        ros::Publisher p_ee_pose_ref;
        ros::Publisher p_imu;
        ros::Publisher p_com; 

        ros::Subscriber s_gain_p;
        ros::Subscriber s_gain_w;
        ros::Subscriber s_gain_d;

        ros::Subscriber server_sub1;
        ros::Publisher p_ros_msg;
        std_msgs::Float64MultiArray m_ros_msg;
        ros::Publisher p_torque_L_Knee_P_J;
        std_msgs::Float64 m_torque_L_Knee_P_J;

        double dt;
        Time last_update_time;
        Time current_time;
        ConnectionPtr update_connection;

        Eigen::Vector3d a0, a1, a2, a3;
        Eigen::Vector3d a11, a12, a13;
        Eigen::Vector3d P3_P0, P3_P1, P3_P2;
        Eigen::Vector3d P13_P0, P13_P11, P13_P12;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
      

        VectorXd R_Pos_HIP_Y_CoM = VectorXd::Zero(3); 
        VectorXd R_Pos_HIP_R_CoM = VectorXd::Zero(3);
        VectorXd R_Pos_HIP_P_CoM = VectorXd::Zero(3); 
        VectorXd R_Pos_KNEE_P_CoM = VectorXd::Zero(3); 
        VectorXd R_Pos_ANKLE_P_CoM = VectorXd::Zero(3); 
        VectorXd R_Pos_ANKLE_R_CoM = VectorXd::Zero(3);

        VectorXd L_Pos_HIP_Y_CoM = VectorXd::Zero(3); 
        VectorXd L_Pos_HIP_R_CoM = VectorXd::Zero(3);
        VectorXd L_Pos_HIP_P_CoM = VectorXd::Zero(3); 
        VectorXd L_Pos_KNEE_P_CoM = VectorXd::Zero(3); 
        VectorXd L_Pos_ANKLE_P_CoM = VectorXd::Zero(3); 
        VectorXd L_Pos_ANKLE_R_CoM = VectorXd::Zero(3); 

        VectorXd Base_Pos_CoM = VectorXd::Zero(3); 

        VectorXd Robot_Pos_CoM = VectorXd::Zero(3); 

        MatrixXd L_T00 = MatrixXd::Zero(4,4);
        MatrixXd L_T01 = MatrixXd::Zero(4,4);
        MatrixXd L_T02 = MatrixXd::Zero(4,4);
        MatrixXd L_T03 = MatrixXd::Zero(4,4);
        MatrixXd L_T04 = MatrixXd::Zero(4,4);
        MatrixXd L_T05 = MatrixXd::Zero(4,4);
        MatrixXd L_T06 = MatrixXd::Zero(4,4);

        MatrixXd R_T00 = MatrixXd::Zero(4,4);
        MatrixXd R_T01 = MatrixXd::Zero(4,4);
        MatrixXd R_T02 = MatrixXd::Zero(4,4);
        MatrixXd R_T03 = MatrixXd::Zero(4,4);
        MatrixXd R_T04 = MatrixXd::Zero(4,4);
        MatrixXd R_T05 = MatrixXd::Zero(4,4);
        MatrixXd R_T06 = MatrixXd::Zero(4,4);

        Eigen::Matrix3d L_T00_rot;
        Eigen::Matrix3d L_T01_rot;
        Eigen::Matrix3d L_T02_rot;
        Eigen::Matrix3d L_T03_rot;
        Eigen::Matrix3d L_T04_rot;
        Eigen::Matrix3d L_T05_rot;
        Eigen::Matrix3d L_T06_rot;

        Eigen::Matrix3d R_T00_rot;
        Eigen::Matrix3d R_T01_rot;
        Eigen::Matrix3d R_T02_rot;
        Eigen::Matrix3d R_T03_rot;
        Eigen::Matrix3d R_T04_rot;
        Eigen::Matrix3d R_T05_rot;
        Eigen::Matrix3d R_T06_rot;

        VectorXd L_T00_pos = VectorXd::Zero(3); 
        VectorXd L_T01_pos = VectorXd::Zero(3); 
        VectorXd L_T02_pos = VectorXd::Zero(3); 
        VectorXd L_T03_pos = VectorXd::Zero(3); 
        VectorXd L_T04_pos = VectorXd::Zero(3);
        VectorXd L_T05_pos = VectorXd::Zero(3); 

        VectorXd R_T00_pos = VectorXd::Zero(3);
        VectorXd R_T01_pos = VectorXd::Zero(3);
        VectorXd R_T02_pos = VectorXd::Zero(3);
        VectorXd R_T03_pos = VectorXd::Zero(3);
        VectorXd R_T04_pos = VectorXd::Zero(3);
        VectorXd R_T05_pos = VectorXd::Zero(3);        

       

        MatrixXd R_HIP_Y_CoM = MatrixXd::Zero(4,4); 
        MatrixXd R_HIP_R_CoM = MatrixXd::Zero(4,4);
        MatrixXd R_HIP_P_CoM = MatrixXd::Zero(4,4); 
        MatrixXd R_KNEE_P_CoM = MatrixXd::Zero(4,4); 
        MatrixXd R_ANKLE_P_CoM = MatrixXd::Zero(4,4); 
        MatrixXd R_ANKLE_R_CoM = MatrixXd::Zero(4,4); 
        MatrixXd BASE_CoM  = MatrixXd::Zero(4,4);
        MatrixXd L_HIP_Y_CoM = MatrixXd::Zero(4,4); 
        MatrixXd L_HIP_R_CoM = MatrixXd::Zero(4,4);
        MatrixXd L_HIP_P_CoM = MatrixXd::Zero(4,4); 
        MatrixXd L_KNEE_P_CoM = MatrixXd::Zero(4,4); 
        MatrixXd L_ANKLE_P_CoM = MatrixXd::Zero(4,4); 
        MatrixXd L_ANKLE_R_CoM = MatrixXd::Zero(4,4);

        MatrixXd L_A1 = MatrixXd::Zero(4,4); 
        MatrixXd L_A2 = MatrixXd::Zero(4,4); 
        MatrixXd L_A3 = MatrixXd::Zero(4,4);
        MatrixXd L_A4 = MatrixXd::Zero(4,4);
        MatrixXd L_A5 = MatrixXd::Zero(4,4);
        MatrixXd L_A6 = MatrixXd::Zero(4,4);
        MatrixXd L_A7 = MatrixXd::Zero(4,4);
        MatrixXd L_A8 = MatrixXd::Zero(4,4);
        MatrixXd L_A9 = MatrixXd::Zero(4,4);
        MatrixXd L_A10 = MatrixXd::Zero(4,4);  

        MatrixXd R_A1 = MatrixXd::Zero(4,4); 
        MatrixXd R_A2 = MatrixXd::Zero(4,4); 
        MatrixXd R_A3 = MatrixXd::Zero(4,4);
        MatrixXd R_A4 = MatrixXd::Zero(4,4);
        MatrixXd R_A5 = MatrixXd::Zero(4,4);
        MatrixXd R_A6 = MatrixXd::Zero(4,4);
        MatrixXd R_A7 = MatrixXd::Zero(4,4);
        MatrixXd R_A8 = MatrixXd::Zero(4,4);
        MatrixXd R_A9 = MatrixXd::Zero(4,4);
        MatrixXd R_A10 = MatrixXd::Zero(4,4);
 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
        VectorXd J1 = VectorXd::Zero(6); 
        VectorXd J2 = VectorXd::Zero(6); 
        VectorXd J3 = VectorXd::Zero(6); 
        VectorXd J11 = VectorXd::Zero(6); 
        VectorXd J12 = VectorXd::Zero(6); 
        VectorXd J13 = VectorXd::Zero(6); 

        MatrixXd Jacobian = MatrixXd::Zero(6,3); 
        MatrixXd Jacobian_R = MatrixXd::Zero(6,3); 

        Eigen::Vector3d  ee_position, 
                ref_ee_position, 
                initial_ee_position, 
                gain_p, gain_w;

        VectorXd gain_d = VectorXd::Zero(6); 

        Eigen::Vector3d  r_ee_position, 
                r_ref_ee_position, 
                r_initial_ee_position;

        Eigen::Vector3d ee_rotation_x, ee_rotation_y, ee_rotation_z, 
                ref_ee_rotation_x, ref_ee_rotation_y, ref_ee_rotation_z,
                ee_orientation_error, ee_force, ee_momentum;

        Eigen::Vector3d r_ee_rotation_x, r_ee_rotation_y, r_ee_rotation_z, 
                r_ref_ee_rotation_x, r_ref_ee_rotation_y, r_ref_ee_rotation_z,
                r_ee_orientation_error, r_ee_force, r_ee_momentum;

        Eigen::Matrix3d ee_rotation, ref_ee_rotation;
        Eigen::Matrix3d r_ee_rotation, r_ref_ee_rotation;

        Quaterniond ref_ee_quaternion;  
        Quaterniond ee_quaternion;  
        Quaterniond r_ref_ee_quaternion;  
        Quaterniond r_ee_quaternion; 

        VectorXd th = VectorXd::Zero(12);
        
        MatrixXd A0 = MatrixXd::Zero(4,4);  
        MatrixXd A1 = MatrixXd::Zero(4,4); 
        MatrixXd A2 = MatrixXd::Zero(4,4); 
        MatrixXd A3 = MatrixXd::Zero(4,4);
        MatrixXd A4 = MatrixXd::Zero(4,4);
        MatrixXd A5 = MatrixXd::Zero(4,4);
        MatrixXd A6 = MatrixXd::Zero(4,4);
        MatrixXd A7 = MatrixXd::Zero(4,4);
        MatrixXd A8 = MatrixXd::Zero(4,4);
        MatrixXd A9 = MatrixXd::Zero(4,4);
        
        MatrixXd A10 = MatrixXd::Zero(4,4);
        MatrixXd A11 = MatrixXd::Zero(4,4); 
        MatrixXd A12 = MatrixXd::Zero(4,4); 
        MatrixXd A13 = MatrixXd::Zero(4,4);
        MatrixXd A14 = MatrixXd::Zero(4,4);
        MatrixXd A15 = MatrixXd::Zero(4,4);
        MatrixXd A16 = MatrixXd::Zero(4,4);
        MatrixXd A17 = MatrixXd::Zero(4,4);
        MatrixXd A18 = MatrixXd::Zero(4,4);
        MatrixXd A19 = MatrixXd::Zero(4,4);
        MatrixXd A20 = MatrixXd::Zero(4,4);

        MatrixXd T00 = MatrixXd::Zero(4,4); 
        MatrixXd T01 = MatrixXd::Zero(4,4); 
        MatrixXd T02 = MatrixXd::Zero(4,4); 
        MatrixXd T03 = MatrixXd::Zero(4,4);
        MatrixXd T04 = MatrixXd::Zero(4,4);
        MatrixXd T05 = MatrixXd::Zero(4,4);
        MatrixXd T06 = MatrixXd::Zero(4,4);
        MatrixXd T07 = MatrixXd::Zero(4,4);
        MatrixXd T08 = MatrixXd::Zero(4,4);
        MatrixXd T09 = MatrixXd::Zero(4,4);
        MatrixXd T10 = MatrixXd::Zero(4,4);

        MatrixXd T11 = MatrixXd::Zero(4,4); 
        MatrixXd T12 = MatrixXd::Zero(4,4); 
        MatrixXd T13 = MatrixXd::Zero(4,4);
        MatrixXd T14 = MatrixXd::Zero(4,4);
        MatrixXd T15 = MatrixXd::Zero(4,4);
        MatrixXd T16 = MatrixXd::Zero(4,4);
        MatrixXd T17 = MatrixXd::Zero(4,4);
        MatrixXd T18 = MatrixXd::Zero(4,4);
        MatrixXd T19 = MatrixXd::Zero(4,4);
        MatrixXd T20 = MatrixXd::Zero(4,4);

        

        VectorXd joint_torque = VectorXd::Zero(6);
        VectorXd virtual_spring = VectorXd::Zero(6);
        VectorXd r_virtual_spring = VectorXd::Zero(6);
        VectorXd tau_viscous_damping = VectorXd::Zero(6);
        VectorXd r_tau_viscous_damping = VectorXd::Zero(6);
        VectorXd tau = VectorXd::Zero(6);
        VectorXd r_tau = VectorXd::Zero(6);

        void Load(physics::ModelPtr _model, sdf::ElementPtr);
        void UpdateAlgorithm();

        void RBDL_INIT();
        void rbdl_variable_init(A_RBDL &rbdl);
        void L_Air_Model(A_RBDL &rbdl);
        void R_Air_Model(A_RBDL &rbdl);
        
        void EncoderRead();
        void VelocityRead();
        void IMUSensorRead();
        void GetJoints();
        void GetLinks();
        void SensorSetting();
        void InitROSPubSetting();
        void ROSMsgPublish();
        void PostureGeneration();
        void RBDL_variable_update();
        void Callback1(const std_msgs::Int32Ptr & msg);
        void JointController();
        void Init_Pos_Traj();
        void Walk_ready();
        void Cal_CoM();
        void Task_Space_PD_both_legs();
        void SwitchMode(const std_msgs::Int32Ptr &msg);
        void SwitchGainP(const std_msgs::Float32MultiArrayPtr &msg);
        void SwitchGainW(const std_msgs::Float32MultiArrayPtr &msg);
        void SwitchGainD(const std_msgs::Float32MultiArrayPtr &msg);
        void Calculate_CoM();
        void Gravity_Compensation_air();

        private:                   
    };  
    GZ_REGISTER_MODEL_PLUGIN(Ostrich_simple); 
}
#endif