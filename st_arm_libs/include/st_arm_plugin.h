#ifndef ST_ARM_PLUGIN_H
#define ST_ARM_PLUGIN_H


#include <eigen3/Eigen/Dense>
#include <ignition/math.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
//#include <gazebo/plugins/CameraPlugin.hh>           //++

#include <string.h>
#include <iostream>
#include <boost/bind.hpp>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
//#include <gazebo_plugins/gazebo_ros_camera_utils.h>   //++
#include <std_msgs/Bool.h>     //++
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <rbdl/rbdl.h>


using gazebo::physics::ModelPtr;
using gazebo::physics::LinkPtr;
using gazebo::sensors::ImuSensorPtr;
using gazebo::sensors::SensorPtr;
using gazebo::physics::JointPtr;
using gazebo::event::ConnectionPtr;
using gazebo::common::Time;

using Eigen::Vector3d;
using Eigen::Quaterniond;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Matrix4d;

namespace RBDL = RigidBodyDynamics;
namespace RBDLMath = RigidBodyDynamics::Math;

using RBDLModel = RBDL::Model;
using RBDLBody = RBDL::Body;
using RBDLVector3d = RBDL::Math::Vector3d;
using RBDLVectorNd = RBDL::Math::VectorNd;
using RBDLMatrixNd = RBDL::Math::MatrixNd;
using RBDLMatrix3d = RBDL::Math::Matrix3d;
using RBDLJoint = RBDL::Joint;

#define NUM_OF_JOINTS_WITH_TOOL 8
#define NUM_OF_JOINTS_WITHOUT_TOOL 6

#define PI 3.14159265358979
#define M2R 2*PI/4096
#define DEG2RAD		0.017453292519943
#define RAD2DEG		57.295779513082323
#define G 9.81;

#define L1 0.16
#define L2 0.25
#define L3 0.25
#define L4 0.00
#define L5 0.10
#define L6 0.06

#define m_Link1 0.573
#define m_Link2 0.729
#define m_Link3 0.490
#define m_Link4 0.136
#define m_Link5 0.148
#define m_Link6 0.239

#define m_Arm 2.315 // (m_Link1~6 합친거)
#define M1 2.315 // m_Arm
#define M2 1.742 //m_Link2+m_Link3+m_Link4+m_Link5+m_Link6;
#define M3 1.013 //m_Link3+m_Link4+m_Link5+m_Link6;
#define M4 0.523 //m_Link4+m_Link5+m_Link6;
#define M5 0.387 //m_Link5+m_Link6;
#define M6 0.239 //m_Link6;
#define inner_dt 0.001


typedef struct
{
  RBDLModel* rbdl_model;
  RBDLVectorNd q, q_dot, q_d_dot, tau;
  RBDLMatrixNd jacobian, jacobian_prev, jacobian_dot, jacobian_inverse;

  unsigned int base_id, shoulder_yaw_id, shoulder_pitch_id, elbow_pitch_id, wrist_pitch_id, wrist_roll_id, wrist_yaw_id;                               //id have information of the body
  RBDLBody base_link, shoulder_yaw_link, shoulder_pitch_link, elbow_pitch_link, wrist_pitch_link, wrist_roll_link, wrist_yaw_link;
  RBDLJoint base_joint, shoulder_yaw_joint, shoulder_pitch_joint, elbow_pitch_joint, wrist_pitch_joint, wrist_roll_joint, wrist_yaw_joint;
  RBDLMatrix3d base_inertia, shoulder_yaw_inertia, shoulder_pitch_inertia, elbow_pitch_inertia, wrist_pitch_inertia, wrist_roll_inertia, wrist_yaw_inertia; //Inertia of links
} Arm_RBDL;

Arm_RBDL arm_rbdl;


namespace gazebo
{
  class STArmPlugin : public ModelPlugin//, CameraPlugin, GazeboRosCameraUtils     //++ CameraPlugin, GazeboRosCameraUtils
  {
    ModelPtr model;
    LinkPtr Base, Link1, Link2, Link3, Link4, Link5, Link6, LinkGripperL, LinkGripperR;
    JointPtr Joint1, Joint2, Joint3, Joint4, Joint5, Joint6, JointGripperL, JointGripperR;


    //*************** RBQ3 Variables**************//

    // JointPtr FL_hip_joint, FL_thigh_joint, FL_calf_joint,
    //           FR_hip_joint, FR_thigh_joint, FR_calf_joint,
    //           RL_hip_joint, RL_thigh_joint, RL_calf_joint,
    //           RR_hip_joint, RR_thigh_joint, RR_calf_joint;

    JointPtr HRR, HRP, HRK, HLR, HLP, HLK, FRR, FRP, FRK, FLR, FLP, FLK;

    LinkPtr rbq3_base_link;
    JointPtr rbq3_base_joint;
    SensorPtr Sensor;
    ImuSensorPtr RBQ3BaseImu;

    Vector3d rbq3_ref_trajectory, amplitude, frequency, horizontal_translation, vertical_translation, rbq3_base_range_of_motion;
    float traj_time;
    float rbq_base_gain_p;
    float rbq_base_gain_d;
    bool is_move_rbq3{true};
    
    Vector3d rbq3_base_imu_rpy, rbq3_base_rpy, rbq3_base_rpy_ref, rbq3_base_rpy_dot, rbq3_base_torque;

    const std::vector<std::string> rbq3_joint_names = {"HRR", "HRP", "HRK", "HLR", "HLP", "HLK", "FRR", "FRP", "FRK", "FLR", "FLP", "FLK"};

    VectorXd quad_th = VectorXd::Zero(12);
    VectorXd quad_th_dot = VectorXd::Zero(12);
    VectorXd quad_joint_torque = VectorXd::Zero(12);
    VectorXd quad_th_ref = VectorXd::Zero(12);

    ros::Publisher pub_rbq3_joint_state;



    const std::vector<std::string> joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint_g_l", "joint_g_r"};
    
    common::Time last_update_time;
    common::Time current_time;
    event::ConnectionPtr update_connection;
    double dt;
    double time{0};
    double trajectory;


    //*************** Trajectory Variables**************//
    double step_time{0};
    double cnt_time{0};
    unsigned int cnt{0};
    unsigned int start_flag{0};

    // float qw{0};
    // float qx{0};
    // float qy{0};
    // float qz{0};

    // float pre_data_x{0};
    // float pre_data_y{0};
    // float pre_data_z{0};

    // float input_P = 200;
    // float input_D{0};

    Vector3d manipulator_com, 
            ref_com_position, 
            shoulder_link_com, 
            arm_link_com, 
            elbow_link_com, 
            forearm_link_com, 
            wrist_link_com, 
            endeffector_link_com;

    Vector3d C1, C2, C3, C4, C5, C6;
    Vector3d a0, a1, a2, a3, a4, a5;
    Vector3d C1_P0, C2_P1, C3_P2, C4_P3, C5_P4, C6_P5;
    Vector3d P6_P0, P6_P1, P6_P2, P6_P3, P6_P4, P6_P5;
    Vector3d J1_CoM, J2_CoM, J3_CoM, J4_CoM, J5_CoM, J6_CoM;
    Vector3d initial_com_position, desired_com_position, virtual_spring_com, com_force; 
    VectorXd virtual_spring_rotational = VectorXd::Zero(6); 
    VectorXd tau_com = VectorXd::Zero(6);
    VectorXd tau_rotational = VectorXd::Zero(6);
    
    VectorXd J1 = VectorXd::Zero(6); 
    VectorXd J2 = VectorXd::Zero(6); 
    VectorXd J3 = VectorXd::Zero(6); 
    VectorXd J4 = VectorXd::Zero(6); 
    VectorXd J5 = VectorXd::Zero(6); 
    VectorXd J6 = VectorXd::Zero(6);

    MatrixXd Jacobian = MatrixXd::Zero(6,6); 
    MatrixXd J_CoM = MatrixXd::Zero(3,6);

    Vector3d  ee_position, 
              ee_velocity, 
              pre_ee_position, 
              ref_ee_position, 
              initial_ee_position, 
              hmd_position;
    Vector3d gain_p, gain_d, gain_w;
    VectorXd gain_p_joint_space = VectorXd(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd gain_d_joint_space = VectorXd(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd gain_p_joint_space_idle = VectorXd(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd gain_d_joint_space_idle = VectorXd(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd gain_r = VectorXd(6);
    VectorXd threshold = VectorXd(6);
    Vector3d gain_p_task_space, gain_w_task_space;


    Vector3d ee_rotation_x, ee_rotation_y, ee_rotation_z, 
            ref_ee_rotation_x, ref_ee_rotation_y, ref_ee_rotation_z,
            ee_orientation_error, ee_force, ee_momentum;

    Matrix3d ee_rotation, ref_ee_rotation;

    Quaterniond ref_ee_quaternion;
    Quaterniond ee_quaternion;
    Quaterniond hmd_quaternion;

    VectorXd th = VectorXd::Zero(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd ref_th = VectorXd::Zero(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd last_th = VectorXd::Zero(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd th_dot = VectorXd::Zero(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd last_th_dot = VectorXd::Zero(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd th_d_dot = VectorXd::Zero(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd init_position = VectorXd::Zero(NUM_OF_JOINTS_WITH_TOOL);

    VectorXd ik_th = VectorXd::Zero(6);
    
    MatrixXd A0 = MatrixXd::Zero(4,4);  MatrixXd A1 = MatrixXd::Zero(4,4); MatrixXd A2 = MatrixXd::Zero(4,4); 
    MatrixXd A3 = MatrixXd::Zero(4,4);
    MatrixXd A4 = MatrixXd::Zero(4,4);  MatrixXd A5 = MatrixXd::Zero(4,4); MatrixXd A6 = MatrixXd::Zero(4,4);
    MatrixXd T00 = MatrixXd::Zero(4,4); MatrixXd T01 = MatrixXd::Zero(4,4); MatrixXd T02 = MatrixXd::Zero(4,4); 
    MatrixXd T03 = MatrixXd::Zero(4,4);
    MatrixXd T04 = MatrixXd::Zero(4,4); MatrixXd T05 = MatrixXd::Zero(4,4); MatrixXd T06 = MatrixXd::Zero(4,4);
    MatrixXd T12 = MatrixXd::Zero(4,4); MatrixXd T23 = MatrixXd::Zero(4,4); MatrixXd T34 = MatrixXd::Zero(4,4); 
    MatrixXd T45 = MatrixXd::Zero(4,4); MatrixXd T56 = MatrixXd::Zero(4,4);

    MatrixXd joint_limit = MatrixXd::Zero(2,6);

    VectorXd joint_torque = VectorXd::Zero(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd tau_gravity_compensation = VectorXd::Zero(NUM_OF_JOINTS_WITH_TOOL);
    // VectorXd gripper_torque = VectorXd::Zero(2);
    VectorXd virtual_spring = VectorXd::Zero(6);
    VectorXd tau_viscous_damping = VectorXd::Zero(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd tau_rbdl = VectorXd::Zero(6);
    VectorXd tau = VectorXd::Zero(6);

    // Temporary variables

    // End of Temporary variables

    ros::NodeHandle node_handle;
    ros::Publisher pub_joint_state;
    ros::Publisher pub_joint_state_deg;
    ros::Publisher pub_joint_state_ik;
    ros::Publisher pub_ee_pose;
    ros::Publisher pub_ref_ee_pose;
    ros::Publisher pub_base_imu_pose;

    ros::Subscriber sub_gripper_state;
    ros::Subscriber sub_hmd_pose;
    ros::Subscriber sub_mode_selector;
    ros::Subscriber sub_gain_p_task_space;
    ros::Subscriber sub_gain_w_task_space;
    ros::Subscriber sub_gain_p_joint_space;
    ros::Subscriber sub_gain_d_joint_space;
    ros::Subscriber sub_gain_r;
    ros::Subscriber sub_rbq3_motion_switch;

    //ros::Publisher pub_gazebo_camera;   //++

    enum ControlMode
    {
      IDLE = 0,
      Motion_1,
      Motion_2,
      Motion_3,
      Motion_4,
      Motion_5    
    };
    enum ControlMode control_mode;

    void Load(ModelPtr _model, sdf::ElementPtr/*, sensors::SensorPtr _parent*/);    //++
    void Loop();
    void GetLinks();
    void GetJoints();
    void GetSensors();
    void InitROSPubSetting();

    void SetRBDLVariables();
    void InitializeRBDLVariables();

    void GetJointPosition();
    void GetJointVelocity();
    void GetJointAcceleration();
    void GetSensorValues();
    void SetJointTorque();
    void ROSMsgPublish();

    void PostureGeneration();
    void Idle();
    void Motion1();
    void Motion2();
    void Motion3();
    void Motion4();
    void Motion5();
    void SwitchMode(const std_msgs::Int32Ptr & msg);
    void SwitchGain(const std_msgs::Int32Ptr & msg);
    void SwitchGainJointSpaceP(const std_msgs::Float32MultiArrayConstPtr &msg);
    void SwitchGainJointSpaceD(const std_msgs::Float32MultiArrayConstPtr &msg);
    void SwitchGainTaskSpaceP(const std_msgs::Float32MultiArrayConstPtr &msg);
    void SwitchGainTaskSpaceW(const std_msgs::Float32MultiArrayConstPtr &msg);
    void SwitchGainR(const std_msgs::Float32MultiArrayConstPtr &msg);
    void SwitchModeRBQ3(const std_msgs::Bool &msg);
    void GripperStateCallback(const std_msgs::Float32ConstPtr &msg);
    void HMDPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void RBQ3Motion1();
    void GetRBQ3Joints();
    void GetRBQ3JointPosition();
    void GetRBQ3JointVelocity();
    void SetRBQ3JointTorque();
    
    void GripperControl();

    bool InverseSolverUsingSRJacobian(Vector3d target_position, Matrix3d target_orientation);

    void GetJacobians(VectorXd a_th, MatrixXd Jacobian, Matrix4d Current_Pose);

    Vector3d PositionDifference(Vector3d desired_position, Vector3d present_position);
    Vector3d OrientationDifference(Matrix3d desired_orientation, Matrix3d present_orientation);
    Vector3d MatrixLogarithm(Matrix3d rotation_matrix);
    Matrix3d skewSymmetricMatrix(Vector3d v);
    MatrixXd getDampedPseudoInverse(MatrixXd Jacobian, VectorXd lamda);
    VectorXd PoseDifference(Vector3d desired_position, Matrix3d desired_orientation, Matrix4d present_pose);


    // void SolveInverseKinematics();
    // VectorXd SolveForwardKinematics(VectorXd a_th);
    // VectorXd poseDifference(Vector3d desired_position, Vector3d present_position,
    //                             Matrix3d desired_orientation, Matrix3d present_orientation);
    // MatrixXd jacobian();
  };
  GZ_REGISTER_MODEL_PLUGIN(STArmPlugin);
}

#endif  // end of the ST_ARM_PLUGIN_H