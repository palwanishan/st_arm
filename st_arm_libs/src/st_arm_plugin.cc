#include "st_arm_plugin.h"

namespace gazebo
{
  void STArmPlugin::Load(ModelPtr _model, sdf::ElementPtr/*, sensors::SensorPtr _parent*/)
  {
    this->model = _model;
    GetLinks();
    GetJoints();
    GetSensors();
    InitROSPubSetting();
    std::cout << "Before Calling RBDL Initialize function" << std::endl;
    InitializeRBDLVariables();
    this->last_update_time = this->model->GetWorld()->SimTime();
    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&STArmPlugin::Loop, this));
    
    is_move_rbq3 = true;
    if(is_move_rbq3)
    {
      GetRBQ3Joints();
    }

    std::cout << "Load..." << std::endl;
  }


  void STArmPlugin::GetLinks()
  {
    this->Base  = this->model->GetLink("base_link");
    this->Link1 = this->model->GetLink("shoulder_yaw_link");
    this->Link2 = this->model->GetLink("shoulder_pitch_link");
    this->Link3 = this->model->GetLink("elbow_pitch_link");
    this->Link4 = this->model->GetLink("wrist_pitch_link");
    this->Link5 = this->model->GetLink("wrist_roll_link");
    this->Link6 = this->model->GetLink("wrist_yaw_link");
    this->LinkGripperL = this->model->GetLink("gripper_left_link");
    this->LinkGripperR = this->model->GetLink("gripper_right_link");
  }


  void STArmPlugin::GetJoints()
  {
    this->Joint1 = this->model->GetJoint("shoulder_yaw_joint");
    this->Joint2 = this->model->GetJoint("shoulder_pitch_joint");
    this->Joint3 = this->model->GetJoint("elbow_pitch_joint");
    this->Joint4 = this->model->GetJoint("wrist_pitch_joint");
    this->Joint5 = this->model->GetJoint("wrist_roll_joint");
    this->Joint6 = this->model->GetJoint("wrist_yaw_joint");
    this->JointGripperL = this->model->GetJoint("gripper_left_joint");
    this->JointGripperR = this->model->GetJoint("gripper_right_joint");
  }


  void STArmPlugin::GetSensors()
  {
    this->Sensor = sensors::get_sensor("rbq3_base_imu");
    this->RBQ3BaseImu = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor);
  }


  void STArmPlugin::InitROSPubSetting()
  {
    pub_joint_state = node_handle.advertise<sensor_msgs::JointState>("st_arm/joint_states", 100);
    pub_joint_state_deg = node_handle.advertise<sensor_msgs::JointState>("st_arm/joint_states_deg", 100);
    pub_joint_state_ik = node_handle.advertise<sensor_msgs::JointState>("st_arm/joint_states_ik", 100);
    sub_mode_selector = node_handle.subscribe("st_arm/mode_selector", 10, &gazebo::STArmPlugin::SwitchMode, this); 
    sub_gain_p_task_space = node_handle.subscribe("st_arm/TS_Kp", 10, &gazebo::STArmPlugin::SwitchGainTaskSpaceP, this); 
    sub_gain_w_task_space = node_handle.subscribe("st_arm/TS_Kw", 10, &gazebo::STArmPlugin::SwitchGainTaskSpaceW, this); 
    sub_gain_p_joint_space = node_handle.subscribe("st_arm/JS_Kp", 10, &gazebo::STArmPlugin::SwitchGainJointSpaceP, this); 
    sub_gain_d_joint_space = node_handle.subscribe("st_arm/JS_Kd", 10, &gazebo::STArmPlugin::SwitchGainJointSpaceD, this); 
    sub_gain_r = node_handle.subscribe("st_arm/JS_Kr", 10, &gazebo::STArmPlugin::SwitchGainR, this); 
    
    sub_gripper_state = node_handle.subscribe("unity/gripper_state", 10, &gazebo::STArmPlugin::GripperStateCallback, this);
    sub_hmd_pose = node_handle.subscribe("unity/hmd_pose", 1, &gazebo::STArmPlugin::HMDPoseCallback, this);

    sub_rbq3_motion_switch = node_handle.subscribe("rbq3/motion_switch", 10, &gazebo::STArmPlugin::SwitchModeRBQ3, this);

    pub_ee_pose = node_handle.advertise<geometry_msgs::TransformStamped>("st_arm/ee_pose", 10);
    pub_ref_ee_pose = node_handle.advertise<geometry_msgs::TransformStamped>("st_arm/ref_ee_pose", 10);
    //pub_gazebo_camera = node_handle.advertise<sensor_msgs::Image>("st_arm/gazebocamera", 60);  //++
//    sub_gripper_activation = node_handle.subscribe("unity/gripper_activation", 1, &gazebo::STArmPlugin::GripperActivationCallback, this);  //++
  
    pub_rbq3_joint_state = node_handle.advertise<sensor_msgs::JointState>("rbq3/joint_states", 200);
  }


  void STArmPlugin::InitializeRBDLVariables()
  {
    std::cout << "Before Check RBDL API VERSION" << std::endl;
    rbdl_check_api_version(RBDL_API_VERSION);
    std::cout << "Checked RBDL API VERSION" << std::endl;

    arm_rbdl.rbdl_model = new RBDLModel();
    arm_rbdl.rbdl_model->gravity = RBDL::Math::Vector3d(0.0, 0.0, -9.81);

    arm_rbdl.base_inertia = RBDLMatrix3d(0.00033, 0,        0,
                                         0,       0.00034,  0,
                                         0,       0,        0.00056);

    arm_rbdl.shoulder_yaw_inertia = RBDLMatrix3d( 0.00024,  0,        0,
                                                  0,        0.00040,  0,
                                                  0,        0,        0.00026);

    arm_rbdl.shoulder_pitch_inertia = RBDLMatrix3d(0.00028, 0,        0,
                                                   0,       0.00064,  0,
                                                   0,       0,        0.00048);

    arm_rbdl.elbow_pitch_inertia = RBDLMatrix3d(0.00003,  0,        0,
                                                0,        0.00019,  0,
                                                0,        0,        0.00020);

    arm_rbdl.wrist_pitch_inertia = RBDLMatrix3d(0.00002,  0,        0,
                                                0,        0.00002,  0,
                                                0,        0,        0.00001);

    arm_rbdl.wrist_roll_inertia = RBDLMatrix3d(0.00001,  0,        0,
                                                0,        0.00002,  0,
                                                0,        0,        0.00001);

    arm_rbdl.wrist_yaw_inertia = RBDLMatrix3d(0.00006,  0,        0,
                                                0,        0.00003,  0,
                                                0,        0,        0.00005);

    // arm_rbdl.base_link = RBDLBody(0.59468, RBDLVector3d(0, 0.00033, 0.03107), arm_rbdl.base_inertia);
    // arm_rbdl.base_joint = RBDLJoint(RBDL::JointType::JointTypeFixed, RBDLVector3d(0,0,0));
    // arm_rbdl.base_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(0, RBDL::Math::Xtrans(RBDLVector3d(0,0,0)), arm_rbdl.base_joint, arm_rbdl.base_link);

    arm_rbdl.shoulder_yaw_link = RBDLBody(0.55230, RBDLVector3d(0.00007, -0.00199, 0.09998), arm_rbdl.shoulder_yaw_inertia);
    arm_rbdl.shoulder_yaw_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0,0,1));
    arm_rbdl.shoulder_yaw_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(0, RBDL::Math::Xtrans(RBDLVector3d(0,0,0)), arm_rbdl.shoulder_yaw_joint, arm_rbdl.shoulder_yaw_link);

    arm_rbdl.shoulder_pitch_link = RBDLBody(0.65326, RBDLVector3d(0.22204, 0.04573, 0), arm_rbdl.shoulder_pitch_inertia);
    arm_rbdl.shoulder_pitch_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0,1,0));
    arm_rbdl.shoulder_pitch_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(1, RBDL::Math::Xtrans(RBDLVector3d(0, 0, 0.1019)), arm_rbdl.shoulder_pitch_joint, arm_rbdl.shoulder_pitch_link);

    arm_rbdl.elbow_pitch_link = RBDLBody(0.17029, RBDLVector3d(0.17044, 0.00120, 0.00004), arm_rbdl.elbow_pitch_inertia);
    arm_rbdl.elbow_pitch_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0,1,0));
    arm_rbdl.elbow_pitch_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(2, RBDL::Math::Xtrans(RBDLVector3d(0.25,0,0)), arm_rbdl.elbow_pitch_joint, arm_rbdl.elbow_pitch_link);

    arm_rbdl.wrist_pitch_link = RBDLBody(0.09234, RBDLVector3d(0.04278, 0, 0.01132), arm_rbdl.wrist_pitch_inertia);
    arm_rbdl.wrist_pitch_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0,1,0));
    arm_rbdl.wrist_pitch_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(3, RBDL::Math::Xtrans(RBDLVector3d(0.25,0,0)), arm_rbdl.wrist_pitch_joint, arm_rbdl.wrist_pitch_link);

    arm_rbdl.wrist_roll_link = RBDLBody(0.08696, RBDLVector3d(0.09137, 0, 0.00036), arm_rbdl.wrist_roll_inertia);
    arm_rbdl.wrist_roll_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(1,0,0));
    arm_rbdl.wrist_roll_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(4, RBDL::Math::Xtrans(RBDLVector3d(0,0,0)), arm_rbdl.wrist_roll_joint, arm_rbdl.wrist_roll_link);

    arm_rbdl.wrist_yaw_link = RBDLBody(0.14876, RBDLVector3d(0.05210, 0.00034, 0.02218), arm_rbdl.wrist_yaw_inertia);
    arm_rbdl.wrist_yaw_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0,0,1));
    arm_rbdl.wrist_yaw_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(5, RBDL::Math::Xtrans(RBDLVector3d(0.1045,0,0)), arm_rbdl.wrist_yaw_joint, arm_rbdl.wrist_yaw_link);

    arm_rbdl.q = RBDLVectorNd::Zero(6);
    arm_rbdl.q_dot = RBDLVectorNd::Zero(6);
    arm_rbdl.q_d_dot = RBDLVectorNd::Zero(6);
    arm_rbdl.tau = RBDLVectorNd::Zero(6);

    arm_rbdl.jacobian = RBDLMatrixNd::Zero(6,6);
    arm_rbdl.jacobian_prev = RBDLMatrixNd::Zero(6,6);
    arm_rbdl.jacobian_dot = RBDLMatrixNd::Zero(6,6);
    arm_rbdl.jacobian_inverse = RBDLMatrixNd::Zero(6,6);

    std::cout << "RBDL Initialize function success" << std::endl;
  }


  void STArmPlugin::Loop()
  {
    current_time = this->model->GetWorld()->SimTime();
    dt = current_time.Double() - last_update_time.Double();
    last_update_time = current_time;
    
    GetJointPosition();
    GetJointVelocity();
    GetJointAcceleration();
    GetSensorValues();
    SetRBDLVariables();
    ROSMsgPublish();
    PostureGeneration();  
    SetJointTorque();
    if(is_move_rbq3)
    {
      GetRBQ3JointPosition();
      GetRBQ3JointVelocity();
      RBQ3Motion2();
      SetRBQ3JointTorque();
    } 
  }


  void STArmPlugin::GetJointPosition()
  {
    th[0] = this->Joint1->Position(2);
    th[1] = this->Joint2->Position(1);
    th[2] = this->Joint3->Position(1);
    th[3] = this->Joint4->Position(1);
    th[4] = this->Joint5->Position(0);
    th[5] = this->Joint6->Position(2);
    th[6] = this->JointGripperL->Position(1);
    th[7] = this->JointGripperR->Position(1);
  }


  void STArmPlugin::GetJointVelocity()
  {
    th_dot[0] = this->Joint1->GetVelocity(2);
    th_dot[1] = this->Joint2->GetVelocity(1);
    th_dot[2] = this->Joint3->GetVelocity(1);
    th_dot[3] = this->Joint4->GetVelocity(1);
    th_dot[4] = this->Joint5->GetVelocity(0);
    th_dot[5] = this->Joint6->GetVelocity(2);
    th_dot[6] = this->JointGripperL->GetVelocity(1);
    th_dot[7] = this->JointGripperR->GetVelocity(1);
  }


  void STArmPlugin::GetJointAcceleration()
  {
    th_d_dot = (th_dot - last_th_dot) / dt;
    last_th_dot = th_dot;
  }


  void STArmPlugin::GetSensorValues()
  {
    auto pose = this->model->WorldPose();
    auto rbq3_base_imu_quat = this->RBQ3BaseImu->Orientation();

    auto xyz = rbq3_base_imu_quat.Euler();
    rbq3_base_imu_rpy = {xyz[0], xyz[1], xyz[2]};
  }


  void STArmPlugin::ROSMsgPublish()
  {
    sensor_msgs::JointState joint_state_msg;
    
    joint_state_msg.header.stamp = ros::Time::now();
    for (uint8_t i=0; i<NUM_OF_JOINTS_WITH_TOOL; i++)
    {
      joint_state_msg.name.push_back((std::string)joint_names.at(i));
      joint_state_msg.position.push_back((float)(th[i]));
      joint_state_msg.velocity.push_back((float)(th_dot[i]));
      joint_state_msg.effort.push_back((float)joint_torque[i]);
    }
    pub_joint_state.publish(joint_state_msg); 

    // joint_state_msg.header.stamp = ros::Time::now();
    // for (uint8_t i=0; i<NUM_OF_JOINTS_WITH_TOOL; i++)
    // {
    //   joint_state_msg.name.push_back((std::string)joint_names.at(i));
    //   joint_state_msg.position.push_back((float)(th[i]*RAD2DEG));
    //   joint_state_msg.velocity.push_back((float)(th_dot[i]));
    //   joint_state_msg.effort.push_back((float)joint_torque[i]);
    // }
    // pub_joint_state_deg.publish(joint_state_msg); 

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    for (uint8_t i=0; i<6; i++)
    {
      msg.name.push_back((std::string)joint_names.at(i));
      msg.position.push_back((float)(ik_th[i]));
      msg.velocity.push_back((float)(ik_current_pose[i]));
    }
    pub_joint_state_ik.publish(msg); 

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();
    ee_quaternion = ee_rotation;
    tf_msg.transform.translation.x = ee_position(0);
    tf_msg.transform.translation.y = ee_position(1);
    tf_msg.transform.translation.z = ee_position(2);
    tf_msg.transform.rotation.x = ee_quaternion.x();
    tf_msg.transform.rotation.y = ee_quaternion.y();
    tf_msg.transform.rotation.z = ee_quaternion.z();
    tf_msg.transform.rotation.w = ee_quaternion.w();

    pub_ee_pose.publish(tf_msg); 

    tf_msg.header.stamp = ros::Time::now();
    ref_ee_quaternion = ref_ee_rotation;
    tf_msg.transform.translation.x = ref_ee_position(0);
    tf_msg.transform.translation.y = ref_ee_position(1);
    tf_msg.transform.translation.z = ref_ee_position(2);
    tf_msg.transform.rotation.x = ref_ee_quaternion.x();
    tf_msg.transform.rotation.y = ref_ee_quaternion.y();
    tf_msg.transform.rotation.z = ref_ee_quaternion.z();
    tf_msg.transform.rotation.w = ref_ee_quaternion.w();

    pub_ref_ee_pose.publish(tf_msg); 

    sensor_msgs::JointState rbq3_joint_state_msg;
    
    joint_state_msg.header.stamp = ros::Time::now();
    for (uint8_t i=0; i<12; i++)
    {
      rbq3_joint_state_msg.name.push_back((std::string)rbq3_joint_names.at(i));
      rbq3_joint_state_msg.position.push_back((float)(quad_th[i]));
      rbq3_joint_state_msg.velocity.push_back((float)(quad_th_dot[i]));
      rbq3_joint_state_msg.effort.push_back((float)quad_joint_torque[i]);
    }
    pub_rbq3_joint_state.publish(rbq3_joint_state_msg); 
  }


  void STArmPlugin::PostureGeneration()
  {
    if (control_mode == IDLE) Idle();
    else if (control_mode == Motion_1) Motion1(); 
    else if (control_mode == Motion_2) Motion2();
    else if (control_mode == Motion_3) Motion3();
    else if (control_mode == Motion_4) Motion4();
    else if (control_mode == Motion_5) Motion5();
    else Idle();
  }


  void STArmPlugin::SetJointTorque()
  {
    this->Joint1->SetForce(2, joint_torque(0)); 
    this->Joint2->SetForce(1, joint_torque(1)); 
    this->Joint3->SetForce(1, joint_torque(2));
    this->Joint4->SetForce(1, joint_torque(3));
    this->Joint5->SetForce(0, joint_torque(4)); 
    this->Joint6->SetForce(2, joint_torque(5));
    this->JointGripperL->SetForce(1, joint_torque(6));
    this->JointGripperR->SetForce(1, joint_torque(7));
  }


  void STArmPlugin::SwitchMode(const std_msgs::Int32Ptr & msg)
  {
    cnt = 0;
    if      (msg -> data == 0) control_mode = IDLE;
    else if (msg -> data == 1) control_mode = Motion_1;
    else if (msg -> data == 2) control_mode = Motion_2;  
    else if (msg -> data == 3) control_mode = Motion_3;  
    else if (msg -> data == 4) control_mode = Motion_4; 
    else if (msg -> data == 5) control_mode = Motion_5;   
    else                       control_mode = IDLE;    
  }


  void STArmPlugin::Idle()
  {
    gain_p_joint_space_idle << 200, 200, 200, 100, 1000, 100, 35, 35;
    // gain_d_joint_space_idle << 20, 20, 20, 10, 10, 10, 3, 3;
    gain_d_joint_space_idle << 1, 1, 1, 1, 1, 1, 1, 1;
    // gain_d_joint_space_idle = gain_p_joint_space_idle * 0.1;
    step_time = 4; 
    
    cnt_time = cnt * inner_dt;
    trajectory = 0.5 * (1 - cos(PI * (cnt_time / step_time)));
    
    if(cnt_time <= step_time)
    {
      ref_th[0] =   0 * trajectory * DEG2RAD;
      ref_th[1] = -60 * trajectory * DEG2RAD;
      ref_th[2] =  90 * trajectory * DEG2RAD;
      ref_th[3] = -30 * trajectory * DEG2RAD;
      ref_th[4] =   0 * trajectory * DEG2RAD;
      ref_th[5] =   0 * trajectory * DEG2RAD;
      ref_th[6] =  -0.03 * trajectory;
      ref_th[7] =  -0.03 * trajectory;
    }

    // joint_torque = gain_p_joint_space_idle * (ref_th - th) - gain_d_joint_space_idle * th_dot;

    for (uint8_t i=0; i<NUM_OF_JOINTS_WITH_TOOL; i++)
    {
      joint_torque[i] = gain_p_joint_space_idle[i] * (ref_th[i] - th[i]) - gain_d_joint_space_idle[i] * th_dot[i];
    }

    cnt++;
  }

  //	Infinity Drawer || Gravity Compensation
  void STArmPlugin::Motion1()
  { 
    // gain_p << 2000, 200, 200;
    // gain_w << 10, 10, 10;
    // gain_r << 1, 1, 1, 1, 1, 1; //adjust GC intensity

    step_time = 3;
    
    cnt_time = cnt*inner_dt;   

    gain_p = gain_p_task_space;
    gain_w = gain_w_task_space; 
    gain_r << 1, 1, 1, 1, 1, 1; //adjust GC intensity

    threshold << 0.2, 0.1, 0.1, 0.1, 0.1, 0.1; 
    joint_limit << 3.14,     0,  2.8,  1.87,  1.57,  1.57,
                    -3.14, -3.14, -0.3, -1.27, -1.57, -1.57;

    A0 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;
    A1 << cos(th[0]), 0, -sin(th[0]), 0,
          sin(th[0]), 0, cos(th[0]), 0,
          0, -1, 0, L1,
          0, 0, 0, 1;
    A2 << cos(th[1]), -sin(th[1]), 0, L2*cos(th[1]),
          sin(th[1]), cos(th[1]), 0, L2*sin(th[1]),
          0, 0, 1, 0, 
          0, 0, 0, 1;
    A3 << cos(th[2]), -sin(th[2]), 0, L3*cos(th[2]), 
          sin(th[2]), cos(th[2]), 0, L3*sin(th[2]), 
          0, 0, 1, 0,
          0, 0, 0, 1;
    A4 << sin(th[3]), 0, cos(th[3]), 0,
          -cos(th[3]), 0, sin(th[3]), 0,
          0, -1, 0, 0,
          0, 0, 0, 1;
    A5 << -sin(th[4]), 0, cos(th[4]), 0,
          cos(th[4]), 0, sin(th[4]), 0,
          0, 1, 0, L5,
          0, 0, 0, 1;
    A6 << -sin(th[5]), -cos(th[5]), 0, -L6*sin(th[5]),
          cos(th[5]), -sin(th[5]), 0, L6*cos(th[5]),
          0, 0, 1, 0, 
          0, 0, 0, 1;          
          
    T00 = A0;
    T01 = T00*A1;
    T02 = T01*A2;
    T03 = T02*A3;
    T04 = T03*A4;
    T05 = T04*A5;
    T06 = T05*A6;
  
    a0 << T00(0,2), T00(1,2), T00(2,2);
    a1 << T01(0,2), T01(1,2), T01(2,2);
    a2 << T02(0,2), T02(1,2), T02(2,2);
    a3 << T03(0,2), T03(1,2), T03(2,2);
    a4 << T04(0,2), T04(1,2), T04(2,2);
    a5 << T05(0,2), T05(1,2), T05(2,2);

    P6_P0 << T06(0,3)-T00(0,3), T06(1,3)-T00(1,3), T06(2,3)-T00(2,3);
    P6_P1 << T06(0,3)-T01(0,3), T06(1,3)-T01(1,3), T06(2,3)-T01(2,3);
    P6_P2 << T06(0,3)-T02(0,3), T06(1,3)-T02(1,3), T06(2,3)-T02(2,3);
    P6_P3 << T06(0,3)-T03(0,3), T06(1,3)-T03(1,3), T06(2,3)-T03(2,3);
    P6_P4 << T06(0,3)-T04(0,3), T06(1,3)-T04(1,3), T06(2,3)-T04(2,3);
    P6_P5 << T06(0,3)-T05(0,3), T06(1,3)-T05(1,3), T06(2,3)-T05(2,3);

    J1 << a0.cross(P6_P0), a0;
    J2 << a1.cross(P6_P1), a1;
    J3 << a2.cross(P6_P2), a2;
    J4 << a3.cross(P6_P3), a3;
    J5 << a4.cross(P6_P4), a4;
    J6 << a5.cross(P6_P5), a5;

    Jacobian << J1, J2, J3, J4, J5, J6;

    ee_position << T06(0,3), T06(1,3), T06(2,3);
    
    // if (cnt<1) initial_ee_position << ee_position(0), ee_position(1), ee_position(2);
    if (cnt<1) initial_ee_position << 0.4, 0, 0.3;

    if(cnt_time <= step_time*100)
    { 
      ref_ee_position(0) = initial_ee_position(0) - 0.2*abs(sin(PI/2*(cnt_time/step_time)));
      ref_ee_position(1) = initial_ee_position(1) - 0.3*sin(PI/2*(cnt_time/step_time));
      ref_ee_position(2) = initial_ee_position(2) + 0.2*sin(PI*(cnt_time/step_time));
      ref_ee_quaternion.w() = 1;
      ref_ee_quaternion.x() = 0;
      ref_ee_quaternion.y() = 0;
      ref_ee_quaternion.z() = 0;
    }

    ee_force(0) = gain_p(0) * (ref_ee_position(0) - ee_position(0));
    ee_force(1) = gain_p(1) * (ref_ee_position(1) - ee_position(1));
    ee_force(2) = gain_p(2) * (ref_ee_position(2) - ee_position(2));

    ee_rotation = T06.block<3,3>(0,0);
    ee_rotation_x = ee_rotation.block<3,1>(0,0); 
    ee_rotation_y = ee_rotation.block<3,1>(0,1); 
    ee_rotation_z = ee_rotation.block<3,1>(0,2);

    ref_ee_rotation = ref_ee_quaternion.normalized().toRotationMatrix();    

    ref_ee_rotation_x = ref_ee_rotation.block<3,1>(0,0); 
    ref_ee_rotation_y = ref_ee_rotation.block<3,1>(0,1); 
    ref_ee_rotation_z = ref_ee_rotation.block<3,1>(0,2);
    ee_orientation_error = ee_rotation_x.cross(ref_ee_rotation_x) + ee_rotation_y.cross(ref_ee_rotation_y) + ee_rotation_z.cross(ref_ee_rotation_z);
    ee_momentum << gain_w(0) * ee_orientation_error(0), gain_w(1) * ee_orientation_error(1), gain_w(2) * ee_orientation_error(2);

    virtual_spring << ee_force(0), ee_force(1), ee_force(2), ee_momentum(0), ee_momentum(1), ee_momentum(2);
  
    RBDL::NonlinearEffects(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.q_dot, arm_rbdl.tau, NULL);
    for(uint8_t i = 0; i < 6; i++)
    {
      tau_rbdl(i) = arm_rbdl.tau(i);
    }

    tau = Jacobian.transpose() * virtual_spring;

    for(uint8_t i=0; i<6; i++) 
    {
      tau_viscous_damping[i] = gain_d_joint_space[i] * th_dot[i]; 
      tau_gravity_compensation[i] = tau_rbdl[i] * gain_r[i];
    }

    for(uint8_t i=0; i<6; i++){
      if(th(i) > joint_limit(0,i) - threshold(i) && tau(i) > 0 || th(i) < joint_limit(1,i) + threshold(i) && tau(i) < 0)
      {
        joint_torque[i] = tau_gravity_compensation[i] - tau_viscous_damping[i];
      }
      else
      {
        joint_torque[i] = tau_gravity_compensation[i] - tau_viscous_damping[i] + tau[i]; 
      } 
    }

    cnt++;

    // InverseSolverUsingSRJacobian(ref_ee_position, ref_ee_rotation);
    InverseSolverUsingJacobian(ref_ee_position, ref_ee_rotation);
  }

  //	RBDL
  void STArmPlugin::Motion2()
  {
    RBDL::NonlinearEffects(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.q_dot, arm_rbdl.tau, NULL);
    for(uint8_t i = 0; i < 6; i++)
    {
      tau_rbdl(i) = arm_rbdl.tau(i);
    }
    for(uint8_t i = 0; i < 6; i++)
    {
      joint_torque(i) = tau_rbdl(i);
    }
  }

  //	HMD Virtual Box follower
  void STArmPlugin::Motion3()
  {
    gain_p << 100, 100, 100;
    gain_d << 5, 5, 5;
    gain_w << 10, 10, 10;
    threshold << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2; // limit threshold angle
    joint_limit << 3.14,     0,  2.8,  1.87,  1.57,  1.57,
                  -3.14, -3.14, -0.3, -1.27, -1.57, -1.57;
    gain_r << 1, 0.3, 0.5, 0.5, 0.5, 0.5; // affects joint limit control
    
    cnt_time = cnt * dt;   

    A0 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;
    A1 << cos(th[0]), 0, -sin(th[0]), 0,
          sin(th[0]), 0, cos(th[0]), 0,
          0, -1, 0, L1,
          0, 0, 0, 1;
    A2 << cos(th[1]), -sin(th[1]), 0, L2*cos(th[1]),
          sin(th[1]), cos(th[1]), 0, L2*sin(th[1]),
          0, 0, 1, 0, 
          0, 0, 0, 1;
    A3 << cos(th[2]), -sin(th[2]), 0, L3*cos(th[2]), 
          sin(th[2]), cos(th[2]), 0, L3*sin(th[2]), 
          0, 0, 1, 0,
          0, 0, 0, 1;
    A4 << sin(th[3]), 0, cos(th[3]), 0,
          -cos(th[3]), 0, sin(th[3]), 0,
          0, -1, 0, 0,
          0, 0, 0, 1;
    A5 << -sin(th[4]), 0, cos(th[4]), 0,
          cos(th[4]), 0, sin(th[4]), 0,
          0, 1, 0, L5,
          0, 0, 0, 1;
    A6 << -sin(th[5]), -cos(th[5]), 0, -L6*sin(th[5]),
          cos(th[5]), -sin(th[5]), 0, L6*cos(th[5]),
          0, 0, 1, 0, 
          0, 0, 0, 1;          
          
    T00 = A0;
    T01 = T00*A1;
    T02 = T01*A2;
    T03 = T02*A3;
    T04 = T03*A4;
    T05 = T04*A5;
    T06 = T05*A6;
  
    a0 << T00(0,2), T00(1,2), T00(2,2);
    a1 << T01(0,2), T01(1,2), T01(2,2);
    a2 << T02(0,2), T02(1,2), T02(2,2);
    a3 << T03(0,2), T03(1,2), T03(2,2);
    a4 << T04(0,2), T04(1,2), T04(2,2);
    a5 << T05(0,2), T05(1,2), T05(2,2);

    P6_P0 << T06(0,3)-T00(0,3), T06(1,3)-T00(1,3), T06(2,3)-T00(2,3);
    P6_P1 << T06(0,3)-T01(0,3), T06(1,3)-T01(1,3), T06(2,3)-T01(2,3);
    P6_P2 << T06(0,3)-T02(0,3), T06(1,3)-T02(1,3), T06(2,3)-T02(2,3);
    P6_P3 << T06(0,3)-T03(0,3), T06(1,3)-T03(1,3), T06(2,3)-T03(2,3);
    P6_P4 << T06(0,3)-T04(0,3), T06(1,3)-T04(1,3), T06(2,3)-T04(2,3);
    P6_P5 << T06(0,3)-T05(0,3), T06(1,3)-T05(1,3), T06(2,3)-T05(2,3);

    J1 << a0.cross(P6_P0), a0;
    J2 << a1.cross(P6_P1), a1;
    J3 << a2.cross(P6_P2), a2;
    J4 << a3.cross(P6_P3), a3;
    J5 << a4.cross(P6_P4), a4;
    J6 << a5.cross(P6_P5), a5;

    Jacobian << J1, J2, J3, J4, J5, J6;

    ee_position << T06(0,3), T06(1,3), T06(2,3);
    if (cnt<1) pre_ee_position = ee_position; 
    ee_velocity = (ee_position - pre_ee_position) / dt;
    pre_ee_position = ee_position;

    ref_ee_position = hmd_position;

    ee_force(0) = gain_p(0) * (ref_ee_position(0) - ee_position(0)) - gain_d(0) * ee_velocity(0);
    ee_force(1) = gain_p(1) * (ref_ee_position(1) - ee_position(1)) - gain_d(1) * ee_velocity(1);
    ee_force(2) = gain_p(2) * (ref_ee_position(2) - ee_position(2)) - gain_d(2) * ee_velocity(2);

    ee_rotation = T06.block<3,3>(0,0);
    ee_rotation_x = ee_rotation.block<3,1>(0,0); 
    ee_rotation_y = ee_rotation.block<3,1>(0,1); 
    ee_rotation_z = ee_rotation.block<3,1>(0,2);

    ref_ee_rotation = hmd_quaternion.normalized().toRotationMatrix();    

    ref_ee_rotation_x = ref_ee_rotation.block<3,1>(0,0); 
    ref_ee_rotation_y = ref_ee_rotation.block<3,1>(0,1); 
    ref_ee_rotation_z = ref_ee_rotation.block<3,1>(0,2);

    ee_orientation_error = ee_rotation_x.cross(ref_ee_rotation_x) 
                        + ee_rotation_y.cross(ref_ee_rotation_y) 
                        + ee_rotation_z.cross(ref_ee_rotation_z);
    
    ee_momentum << gain_w(0) * ee_orientation_error(0), 
                  gain_w(1) * ee_orientation_error(1), 
                  gain_w(2) * ee_orientation_error(2);

    virtual_spring << ee_force(0), ee_force(1), ee_force(2), ee_momentum(0), ee_momentum(1), ee_momentum(2);

    tau =  Jacobian.transpose() * virtual_spring;

    SetRBDLVariables();

    RBDL::NonlinearEffects(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.q_dot, arm_rbdl.tau, NULL);
    for(uint8_t i = 0; i < 6; i++)
    {
      tau_gravity_compensation(i) = arm_rbdl.tau(i);
    }

    for(uint8_t i=0; i<6; i++)
    {
      if(th(i)>joint_limit(0,i)-threshold(i) && tau(i)>0 || th(i)<joint_limit(1,i)+threshold(i) && tau(i)<0)
        joint_torque(i) = gain_r(i)*tau_gravity_compensation(i);
      else
        joint_torque(i) = tau(i) + gain_r(i)*tau_gravity_compensation(i); 
    }

    cnt++;
  }

  //	IK based Infinity Drawer || Gravity Compensation
  void STArmPlugin::Motion4()
  { 
    step_time = 3;
    
    cnt_time = cnt*inner_dt;   

    gain_r << 1, 1, 1, 1, 1, 1; //adjust GC intensity

    threshold << 0.2, 0.1, 0.1, 0.1, 0.1, 0.1; 
    joint_limit << 3.14,     0,  2.8,  1.87,  1.57,  1.57,
                    -3.14, -3.14, -0.3, -1.27, -1.57, -1.57;

    if (cnt<1) initial_ee_position << 0.4, 0, 0.3;

    if(cnt_time <= step_time*100)
    { 
      ref_ee_position(0) = initial_ee_position(0) - 0.2*abs(sin(PI/2*(cnt_time/step_time)));
      ref_ee_position(1) = initial_ee_position(1) - 0.3*sin(PI/2*(cnt_time/step_time));
      ref_ee_position(2) = initial_ee_position(2) + 0.2*sin(PI*(cnt_time/step_time));
      ref_ee_quaternion.w() = 1;
      ref_ee_quaternion.x() = 0;
      ref_ee_quaternion.y() = 0;
      ref_ee_quaternion.z() = 0;
    }

    InverseSolverUsingJacobian(ref_ee_position, ref_ee_rotation);

    RBDL::NonlinearEffects(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.q_dot, arm_rbdl.tau, NULL);
    for(uint8_t i = 0; i < 6; i++)
    {
      tau_rbdl(i) = arm_rbdl.tau(i);
    }

    for(uint8_t i=0; i<6; i++) 
    {
      tau_viscous_damping[i] = gain_d_joint_space[i] * th_dot[i]; 
      tau_gravity_compensation[i] = tau_rbdl[i] * gain_r[i];
      tau[i] = gain_p_joint_space[i] * (ik_th[i] - th[i]);
    }

    for(uint8_t i=0; i<6; i++){
      if(th(i) > joint_limit(0,i) - threshold(i) && tau(i) > 0 || th(i) < joint_limit(1,i) + threshold(i) && tau(i) < 0)
      {
        joint_torque[i] = tau_gravity_compensation[i] - tau_viscous_damping[i];
      }
      else
      {
        joint_torque[i] = tau_gravity_compensation[i] - tau_viscous_damping[i] + tau[i]; 
      } 
    }

    cnt++;
  }

  //	COM based HMD orientation follower
  void STArmPlugin::Motion5()
  {
    gain_p << 700, 700, 700; 
    gain_w << 5, 5, 5;
    threshold << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2; 
    joint_limit << 3.14,     0,  2.8,  1.87,  1.57,  1.57,
                  -3.14, -3.14, -0.3, -1.27, -1.57, -1.57;
    gain_r << 0.5, 0.3, 0.3, 0.3, 0.3, 0.3;

    step_time = 6;
    
    cnt_time = cnt*inner_dt;   

    A0 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;
    A1 << cos(th[0]), 0, -sin(th[0]), 0,
          sin(th[0]), 0, cos(th[0]), 0,
          0, -1, 0, L1,
          0, 0, 0, 1;
    A2 << cos(th[1]), -sin(th[1]), 0, L2*cos(th[1]),
          sin(th[1]), cos(th[1]), 0, L2*sin(th[1]),
          0, 0, 1, 0, 
          0, 0, 0, 1;
    A3 << cos(th[2]), -sin(th[2]), 0, L3*cos(th[2]), 
          sin(th[2]), cos(th[2]), 0, L3*sin(th[2]), 
          0, 0, 1, 0,
          0, 0, 0, 1;
    A4 << sin(th[3]), 0, cos(th[3]), 0,
          -cos(th[3]), 0, sin(th[3]), 0,
          0, -1, 0, 0,
          0, 0, 0, 1;
    A5 << -sin(th[4]), 0, cos(th[4]), 0,
          cos(th[4]), 0, sin(th[4]), 0,
          0, 1, 0, L5,
          0, 0, 0, 1;
    A6 << -sin(th[5]), -cos(th[5]), 0, -L6*sin(th[5]),
          cos(th[5]), -sin(th[5]), 0, L6*cos(th[5]),
          0, 0, 1, 0, 
          0, 0, 0, 1;          
          
    T00 = A0;
    T01 = T00*A1;
    T02 = T01*A2;
    T03 = T02*A3;
    T04 = T03*A4;
    T05 = T04*A5;
    T06 = T05*A6;

    T12 = T02-T01;
    T23 = T03-T02;
    T45 = T05-T04;
    T56 = T06-T05;
  
    a0 << T00(0,2), T00(1,2), T00(2,2);
    a1 << T01(0,2), T01(1,2), T01(2,2);
    a2 << T02(0,2), T02(1,2), T02(2,2);
    a3 << T03(0,2), T03(1,2), T03(2,2);
    a4 << T04(0,2), T04(1,2), T04(2,2);
    a5 << T05(0,2), T05(1,2), T05(2,2);

    P6_P0 << T06(0,3)-T00(0,3), T06(1,3)-T00(1,3), T06(2,3)-T00(2,3);
    P6_P1 << T06(0,3)-T01(0,3), T06(1,3)-T01(1,3), T06(2,3)-T01(2,3);
    P6_P2 << T06(0,3)-T02(0,3), T06(1,3)-T02(1,3), T06(2,3)-T02(2,3);
    P6_P3 << T06(0,3)-T03(0,3), T06(1,3)-T03(1,3), T06(2,3)-T03(2,3);
    P6_P4 << T06(0,3)-T04(0,3), T06(1,3)-T04(1,3), T06(2,3)-T04(2,3);
    P6_P5 << T06(0,3)-T05(0,3), T06(1,3)-T05(1,3), T06(2,3)-T05(2,3);
    
    J1 << a0.cross(P6_P0), a0;
    J2 << a1.cross(P6_P1), a1;
    J3 << a2.cross(P6_P2), a2;
    J4 << a3.cross(P6_P3), a3;
    J5 << a4.cross(P6_P4), a4;
    J6 << a5.cross(P6_P5), a5;

    Jacobian << J1, J2, J3, J4, J5, J6;

    ee_position << T06(0,3), T06(1,3), T06(2,3);

    //---updated GC---
    tau_gravity_compensation[0] = 0.0;
    tau_gravity_compensation[1] = 1.9076*sin(th[1])*sin(th[2]) - 1.9076*cos(th[1])*cos(th[2]) - 2.7704*cos(th[1]) + 0.43376*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43376*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14068*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14068*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    tau_gravity_compensation[2] = 1.9076*sin(th[1])*sin(th[2]) - 1.9076*cos(th[1])*cos(th[2]) + 0.43376*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43376*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14068*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14068*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    tau_gravity_compensation[3] = 0.43376*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43376*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14068*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14068*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    tau_gravity_compensation[4] = 0.14068*cos(th[5] + 1.5708)*sin(th[4] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    tau_gravity_compensation[5] = 0.14068*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14068*cos(th[4] + 1.5708)*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));

    //---updated accurate com---
    shoulder_link_com << 0.092/L1*T01(0,3), 0.092/L1*T01(1,3), 0.092/L1*T01(2,3);
    arm_link_com << T01(0,3)+0.04/L2*T12(0,3), T01(1,3)+0.04/L2*T12(1,3), T01(2,3)+0.04/L2*T12(2,3);
    elbow_link_com << T02(0,3)+0.13/L3*T23(0,3), T02(1,3)+0.13/L3*T23(1,3), T02(2,3)+0.13/L3*T23(2,3);
    forearm_link_com << T04(0,3)+0.046/L5*T45(0,3), T04(1,3)+0.046/L5*T45(1,3), T04(2,3)+0.046/L5*T45(2,3);
    wrist_link_com << T04(0,3)+0.095/L5*T45(0,3), T04(1,3)+0.095/L5*T45(1,3), T04(2,3)+0.095/L5*T45(2,3);
    endeffector_link_com << T06(0,3), T06(1,3), T06(2,3);

    manipulator_com <<  (m_Link1*shoulder_link_com(0) + m_Link2*arm_link_com(0) + m_Link3*elbow_link_com(0) + m_Link4*forearm_link_com(0) + m_Link5*wrist_link_com(0) + m_Link6*endeffector_link_com(0)) / m_Arm,
                        (m_Link1*shoulder_link_com(1) + m_Link2*arm_link_com(1) + m_Link3*elbow_link_com(1) + m_Link4*forearm_link_com(1) + m_Link5*wrist_link_com(1) + m_Link6*endeffector_link_com(1)) / m_Arm,
                        (m_Link1*shoulder_link_com(2) + m_Link2*arm_link_com(2) + m_Link3*elbow_link_com(2) + m_Link4*forearm_link_com(2) + m_Link5*wrist_link_com(2) + m_Link6*endeffector_link_com(2)) / m_Arm;

    C1 << manipulator_com(0), manipulator_com(1), manipulator_com(2);
    C2 << (m_Link2*arm_link_com(0) + m_Link3*elbow_link_com(0) + m_Link4*forearm_link_com(0) + m_Link5*wrist_link_com(0) + m_Link6*endeffector_link_com(0))/M2, (m_Link2*arm_link_com(1) + m_Link3*elbow_link_com(1) + m_Link4*forearm_link_com(1) + m_Link5*wrist_link_com(1) + m_Link6*endeffector_link_com(1))/M2, (m_Link2*arm_link_com(2) + m_Link3*elbow_link_com(2) + m_Link4*forearm_link_com(2) + m_Link5*wrist_link_com(2) + m_Link6*endeffector_link_com(2))/M2;
    C3 << (m_Link3*elbow_link_com(0) + m_Link4*forearm_link_com(0) + m_Link5*wrist_link_com(0) + m_Link6*endeffector_link_com(0))/M3, (m_Link3*elbow_link_com(1) + m_Link4*forearm_link_com(1) + m_Link5*wrist_link_com(1) + m_Link6*endeffector_link_com(1))/M3, (m_Link3*elbow_link_com(2) + m_Link4*forearm_link_com(2) + m_Link5*wrist_link_com(2) + m_Link6*endeffector_link_com(2))/M3;
    C4 << (m_Link4*forearm_link_com(0) + m_Link5*wrist_link_com(0) + m_Link6*endeffector_link_com(0))/M4, (m_Link4*forearm_link_com(1) + m_Link5*wrist_link_com(1) + m_Link6*endeffector_link_com(1))/M4, (m_Link4*forearm_link_com(2) + m_Link5*wrist_link_com(2) + m_Link6*endeffector_link_com(2))/M4;
    C5 << (m_Link5*wrist_link_com(0) + m_Link6*endeffector_link_com(0))/M5, (m_Link5*wrist_link_com(1) + m_Link6*endeffector_link_com(1))/M5, (m_Link5*wrist_link_com(2) + m_Link6*endeffector_link_com(2))/M5;
    C6 << endeffector_link_com(0), endeffector_link_com(1), endeffector_link_com(2);

    C1_P0 << C1(0)-T00(0,3), C1(1)-T00(1,3), C1(2)-T00(2,3);
    C2_P1 << C2(0)-T01(0,3), C2(1)-T01(1,3), C2(2)-T01(2,3);
    C3_P2 << C3(0)-T02(0,3), C3(1)-T02(1,3), C3(2)-T02(2,3);
    C4_P3 << C4(0)-T03(0,3), C4(1)-T03(1,3), C4(2)-T03(2,3);
    C5_P4 << C5(0)-T04(0,3), C5(1)-T04(1,3), C5(2)-T04(2,3);
    C6_P5 << C6(0)-T05(0,3), C6(1)-T05(1,3), C6(2)-T05(2,3);
    
    J1_CoM = M1 / m_Arm * a0.cross(C1_P0);
    J2_CoM = M2 / m_Arm * a1.cross(C2_P1);
    J3_CoM = M3 / m_Arm * a2.cross(C3_P2);
    J4_CoM = M4 / m_Arm * a3.cross(C4_P3);
    J5_CoM = M5 / m_Arm * a4.cross(C5_P4);
    J6_CoM = M6 / m_Arm * a5.cross(C6_P5);

    J_CoM << J1_CoM, J2_CoM, J3_CoM, J4_CoM, J5_CoM, J6_CoM;

    if (cnt<1) initial_com_position << 0.05,0,0.25;
		desired_com_position = initial_com_position;

		for(uint i=0; i<3; i++) virtual_spring_com(i) = gain_p(i) * (desired_com_position(i) - manipulator_com(i));
		
    tau_com = J_CoM.transpose() * virtual_spring_com;
    
    ee_rotation = T06.block<3,3>(0,0);
    ee_rotation_x = ee_rotation.block<3,1>(0,0);
    ee_rotation_y = ee_rotation.block<3,1>(0,1);
    ee_rotation_z = ee_rotation.block<3,1>(0,2);

    ref_ee_rotation = hmd_quaternion.normalized().toRotationMatrix();  
    
    ref_ee_rotation_x = ref_ee_rotation.block<3,1>(0,0);
    ref_ee_rotation_y = ref_ee_rotation.block<3,1>(0,1);
    ref_ee_rotation_z = ref_ee_rotation.block<3,1>(0,2);

    ee_orientation_error = ee_rotation_x.cross(ref_ee_rotation_x) + ee_rotation_y.cross(ref_ee_rotation_y) + ee_rotation_z.cross(ref_ee_rotation_z);
    
		ee_momentum << gain_w(0) * ee_orientation_error(0), gain_w(1) * ee_orientation_error(1), gain_w(2) * ee_orientation_error(2);

    virtual_spring_rotational << 0, 0, 0, ee_momentum(0), ee_momentum(1), ee_momentum(2);
    tau_rotational = Jacobian.transpose() * virtual_spring_rotational;

    tau_gravity_compensation[0] = 0.0;
    tau_gravity_compensation[1] = 1.9318*sin(th[1])*sin(th[2]) - 1.9318*cos(th[1])*cos(th[2]) - 2.9498*cos(th[1]) + 0.43025*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43025*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14096*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14096*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    tau_gravity_compensation[2] = 1.9318*sin(th[1])*sin(th[2]) - 1.9318*cos(th[1])*cos(th[2]) + 0.43025*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43025*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14096*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14096*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    tau_gravity_compensation[3] = 0.43025*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43025*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14096*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14096*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    tau_gravity_compensation[4] = 0.14096*cos(th[5] + 1.5708)*sin(th[4] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    tau_gravity_compensation[5] = 0.14096*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14096*cos(th[4] + 1.5708)*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
   
		tau =  tau_com + tau_rotational;// - tau_viscous_damping;
		
    for(int i=0; i<6; i++)
    {               
      if(th(i)>joint_limit(0,i)-threshold(i) && tau(i)>0 || th(i)<joint_limit(1,i)+threshold(i) && tau(i)<0)
        joint_torque(i) = gain_r(i)*tau_gravity_compensation(i);//we can add more damping here
      else
        joint_torque(i) = tau(i) + gain_r(i)*tau_gravity_compensation(i); 
    }
  
    cnt++;
  }


  void STArmPlugin::HMDPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    hmd_position.x() = msg->pose.position.x;
    hmd_position.y() = msg->pose.position.y;
    hmd_position.z() = msg->pose.position.z;

    hmd_quaternion.x() = msg->pose.orientation.x;
    hmd_quaternion.y() = msg->pose.orientation.y;
    hmd_quaternion.z() = msg->pose.orientation.z;
    hmd_quaternion.w() = msg->pose.orientation.w;
  }


  void STArmPlugin::SetRBDLVariables()
  {
    for(uint8_t i = 0; i < 6; i++)
    {
      arm_rbdl.q(i) = th(i);
      arm_rbdl.q_dot(i) = th_dot(i);
      arm_rbdl.q_d_dot(i) = th_d_dot(i);
    }
  }


  void STArmPlugin::SwitchGainJointSpaceP(const std_msgs::Float32MultiArrayConstPtr &msg)
  {
    for(uint8_t i=0; i<NUM_OF_JOINTS_WITH_TOOL; i++)
    {
      gain_p_joint_space(i) = msg->data.at(i);
    }
  }


  void STArmPlugin::SwitchGainJointSpaceD(const std_msgs::Float32MultiArrayConstPtr &msg)
  {
    for(uint8_t i=0; i<NUM_OF_JOINTS_WITH_TOOL; i++)
    {
      gain_d_joint_space(i) = msg->data.at(i);
    }
  }


  void STArmPlugin::SwitchGainTaskSpaceP(const std_msgs::Float32MultiArrayConstPtr &msg)
  {
    for(uint8_t i=0; i<3; i++)
    {
      gain_p_task_space(i) = msg->data.at(i);
    }
  }


  void STArmPlugin::SwitchGainTaskSpaceW(const std_msgs::Float32MultiArrayConstPtr &msg)
  {
    for(uint8_t i=0; i<3; i++)
    {
      gain_w_task_space(i) = msg->data.at(i);
    }
  }


  void STArmPlugin::SwitchGainR(const std_msgs::Float32MultiArrayConstPtr &msg)
  {
    for(uint8_t i=0; i<6; i++)
    {
      gain_r(i) = msg->data.at(i);
    }
  }


  void STArmPlugin::GripperStateCallback(const std_msgs::Float32ConstPtr &msg)
  {
    float ref_gripper_state = msg->data;
  }


  void STArmPlugin::RBQ3Motion1()
  {
    traj_time = cnt * inner_dt;
    frequency << 0.2, 0.2, 0.2;
    amplitude << 1, 1, 1;
    horizontal_translation << 0, 1, 0;
    vertical_translation << 0, 0, 0;
    rbq3_base_range_of_motion << 5, 5, 5;
    // rbq3_base_range_of_motion << 0, 0, 0;

    for(uint8_t i=0; i<3; i++)
    {
      rbq3_ref_trajectory[i] = amplitude[i] * sin(2 * PI * frequency[i] * (traj_time - horizontal_translation[i])) + vertical_translation[i];
      rbq3_base_rpy_ref[i] = DEG2RAD * rbq3_base_range_of_motion[i] * rbq3_ref_trajectory[i];
    }

    rbq_base_gain_p = 1000;
    rbq_base_gain_d = 50;

    rbq3_base_torque = rbq_base_gain_p * (rbq3_base_rpy_ref - rbq3_base_rpy) - rbq_base_gain_d * rbq3_base_rpy_dot;

    float quad_js_p = 100;
    float quad_js_d = 0;

    quad_th_ref << 0, 60, -90,
                   0, 60, -90,
                   0, 60, -90,
                   0, 60, -90;
    for(uint8_t i=0; i<12; i++)
    {
      quad_joint_torque[i] = quad_js_p * (quad_th_ref[i] * DEG2RAD - quad_th[i]) - quad_js_d * quad_th_dot[i];
    }
  }


  void STArmPlugin::RBQ3Motion2()
  {
    traj_time = cnt * inner_dt;
    frequency << 0.2, 0.2, 0.2;
    amplitude << 1, 1, 1;
    horizontal_translation << 0, 1, 0;
    vertical_translation << 0, 0, 0;
    rbq3_base_range_of_motion << 5, 5, 5;
    // rbq3_base_range_of_motion << 0, 0, 0;

    for(uint8_t i=0; i<3; i++)
    {
      rbq3_ref_trajectory[i] = amplitude[i] * sin(2 * PI * frequency[i] * (traj_time - horizontal_translation[i])) + vertical_translation[i];
      rbq3_base_rpy_ref[i] = DEG2RAD * rbq3_base_range_of_motion[i] * rbq3_ref_trajectory[i];
    }

    rbq_base_gain_p = 1000;
    rbq_base_gain_d = 50;

    rbq3_base_torque = rbq_base_gain_p * (rbq3_base_rpy_ref - rbq3_base_rpy) - rbq_base_gain_d * rbq3_base_rpy_dot;

    float quad_js_p = 100;
    float quad_js_d = 0;

    Vector3d rr_q, rl_q, fr_q, fl_q;

    Vector3d reference_position_right, reference_position_left;

    reference_position_right << 0, -0.05, -0.3;
    reference_position_left << 0, 0.05, -0.3;

    rr_q = GetRBQ3RightIK(reference_position_right);
    fr_q = GetRBQ3RightIK(reference_position_right);

    rl_q = GetRBQ3LeftIK(reference_position_left);
    fl_q = GetRBQ3LeftIK(reference_position_left);

    quad_th_ref << rr_q(0), rr_q(1), rr_q(2),
                   rl_q(0), rl_q(1), rl_q(2),
                   fr_q(0), fr_q(1), fr_q(2),
                   fl_q(0), fl_q(1), fl_q(2);

    for(uint8_t i=0; i<12; i++)
    {
      quad_joint_torque(i) = quad_js_p * (quad_th_ref(i) - quad_th(i)) - quad_js_d * quad_th_dot(i);
    }



    // l_HT0 << 1, 0, 0, 0,
    //           0, 1, 0, 0,
    //           0, 0, 1, 0,
    //           0, 0, 0, 1;
    // l_HT1 << cosf(a_th[0]), 0, -sinf(a_th[0]), 0,
    //           sinf(a_th[0]), 0, cosf(a_th[0]), 0,
    //           0, -1, 0, 0,
    //           0, 0, 0, 1;
    // l_HT4 << sinf(a_th[3]), 0, cosf(a_th[3]), 0,
    //           -cosf(a_th[3]), 0, sinf(a_th[3]), 0,
    //           0, -1, 0, 0,
    //           0, 0, 0, 1;
    // l_HT5 << -sinf(a_th[4]), 0, cosf(a_th[4]), 0,
    //           cosf(a_th[4]), 0, sinf(a_th[4]), 0,
    //           0, 1, 0, 0,
    //           0, 0, 0, 1;





  }

  
  Vector3d STArmPlugin::GetRBQ3RightIK(Vector3d position)
  {
    // const float d1 = 0.29785, d2 = 0.055, d3 = 0.110945, d4 = 0.3205, d5 = 0.025, d6 = 0.3395, d3r = -0.110945;
    const float d1 = 0.22445, d2 = 0.07946, d3 = 0.06995, d4 = 0.225, d5 = 0.0, d6 = 0.225, d3r = -0.06995;

    Vector3d q;

    float alpha = atan2(-1 * position(2), position(1));
    float beta = atan2(sqrt(position(1) * position(1) + position(2) * position(2) - d3 * d3), d3r);
    q(0) = beta - alpha;

    float dESquare = position(0) * position(0) + position(1) * position(1) + position(2) * position(2);
    float d43 = sqrt(d4 * d4 + d3 * d3);
    float cosSigma = (dESquare - (d43 * d43 + d6 * d6 + d3 * d3)) / (2 * d43 * d6);
    float sinSigma = -sqrt(1 - (cosSigma * cosSigma));
    float sigma = atan2(d6 * sinSigma, d6 * cosSigma);
    q(2) = sigma;

    alpha = atan2(-1 * position(0), sqrt(position(1) * position(1) + position(2) * position(2) - d3 * d3));
    beta = atan2(d6 * sinSigma, d4 + d6 * cosSigma);
    q(1) = alpha - beta;

    return q;
  }


  Vector3d STArmPlugin::GetRBQ3LeftIK(Vector3d position)
  {
    // const float d1 = 0.29785, d2 = 0.055, d3 = 0.110945, d4 = 0.3205, d5 = 0.025, d6 = 0.3395, d3r = -0.110945;
    const float d1 = 0.22445, d2 = 0.07946, d3 = 0.06995, d4 = 0.225, d5 = 0.0, d6 = 0.225, d3r = -0.06995;

    Vector3d q;

    float alpha = atan2(-1 * position(2), position(1));
    float beta = atan2(sqrt(position(1) * position(1) + position(2) * position(2) - d3 * d3), d3);
    q(0) = beta - alpha;

    float dESquare = position(0) * position(0) + position(1) * position(1) + position(2) * position(2);
    float d43 = sqrt(d4 * d4 + d3 * d3);
    float cosSigma = (dESquare - (d43 * d43 + d6 * d6 + d3 * d3)) / (2 * d43 * d6);
    float sinSigma = -sqrt(1 - (cosSigma * cosSigma));
    float sigma = atan2(d6 * sinSigma, d6 * cosSigma);
    q(2) = sigma;

    alpha = atan2(-1 * position(0), sqrt(position(1) * position(1) + position(2) * position(2) - d3 * d3));
    beta = atan2(d6 * sinSigma, d4 + d6 * cosSigma);
    q(1) = alpha - beta;

    return q;
  }


  void STArmPlugin::SwitchModeRBQ3(const std_msgs::Bool &msg)
  {
    is_move_rbq3 = &msg;
  }


  void STArmPlugin::GetRBQ3Joints()
  {
    this->HRR = this->model->GetJoint("RR_hip_joint");
    this->HRP = this->model->GetJoint("RR_thigh_joint");
    this->HRK = this->model->GetJoint("RR_calf_joint");
    this->HLR = this->model->GetJoint("RL_hip_joint");
    this->HLP = this->model->GetJoint("RL_thigh_joint");
    this->HLK = this->model->GetJoint("RL_calf_joint");
    this->FRR = this->model->GetJoint("FR_hip_joint");
    this->FRP = this->model->GetJoint("FR_thigh_joint");
    this->FRK = this->model->GetJoint("FR_calf_joint");
    this->FLR = this->model->GetJoint("FL_hip_joint");
    this->FLP = this->model->GetJoint("FL_thigh_joint");
    this->FLK = this->model->GetJoint("FL_calf_joint");

    this->rbq3_base_joint = this->model->GetJoint("rbq3_base_joint");
  }


  void STArmPlugin::GetRBQ3JointPosition()
  {
    quad_th[0] = this->HRR->Position(0);
    quad_th[1] = this->HRP->Position(1);
    quad_th[2] = this->HRK->Position(1);
    quad_th[3] = this->HLR->Position(0);
    quad_th[4] = this->HLP->Position(1);
    quad_th[5] = this->HLK->Position(1);
    quad_th[6] = this->FRR->Position(0);
    quad_th[7] = this->FRP->Position(1);
    quad_th[8] = this->FRK->Position(1);
    quad_th[9] = this->FLR->Position(0);
    quad_th[10] = this->FLP->Position(1);
    quad_th[11] = this->FLK->Position(1);

    rbq3_base_rpy[0] = this->rbq3_base_joint->Position(0);
    rbq3_base_rpy[1] = this->rbq3_base_joint->Position(1);
  }


  void STArmPlugin::GetRBQ3JointVelocity()
  {
    quad_th_dot[0] = this->HRR->GetVelocity(0);
    quad_th_dot[1] = this->HRP->GetVelocity(1);
    quad_th_dot[2] = this->HRK->GetVelocity(1);
    quad_th_dot[3] = this->HLR->GetVelocity(0);
    quad_th_dot[4] = this->HLP->GetVelocity(1);
    quad_th_dot[5] = this->HLK->GetVelocity(1);
    quad_th_dot[6] = this->FRR->GetVelocity(0);
    quad_th_dot[7] = this->FRP->GetVelocity(1);
    quad_th_dot[8] = this->FRK->GetVelocity(1);
    quad_th_dot[9] = this->FLR->GetVelocity(0);
    quad_th_dot[10] = this->FLP->GetVelocity(1);
    quad_th_dot[11] = this->FLK->GetVelocity(1);

    rbq3_base_rpy_dot[0] = this->rbq3_base_joint->GetVelocity(0);
    rbq3_base_rpy_dot[1] = this->rbq3_base_joint->GetVelocity(1);
  }


  void STArmPlugin::SetRBQ3JointTorque()
  {
    this->HRR->SetForce(0, quad_joint_torque(0));
    this->HRP->SetForce(1, quad_joint_torque(1));
    this->HRK->SetForce(1, quad_joint_torque(2));
    this->HLR->SetForce(0, quad_joint_torque(3));
    this->HLP->SetForce(1, quad_joint_torque(4));
    this->HLK->SetForce(1, quad_joint_torque(5));
    this->FRR->SetForce(0, quad_joint_torque(6));
    this->FRP->SetForce(1, quad_joint_torque(7));
    this->FRK->SetForce(1, quad_joint_torque(8));
    this->FLR->SetForce(0, quad_joint_torque(9));
    this->FLP->SetForce(1, quad_joint_torque(10));
    this->FLK->SetForce(1, quad_joint_torque(11));

    this->rbq3_base_joint->SetForce(0, rbq3_base_torque(0));
    this->rbq3_base_joint->SetForce(1, rbq3_base_torque(1));
  }


  bool STArmPlugin::InverseSolverUsingJacobian(Vector3d a_target_position, Matrix3d a_target_orientation)
  {
    //solver parameter
    const double lambda = 0.1;
    const int8_t iteration = 80;
    const double tolerance = 0.0001;

    VectorXd l_q = VectorXd::Zero(6);

    for(int8_t i=0; i<6; i++)
    {
      l_q(i) = th(i);
    }

    //delta parameter
    VectorXd pose_difference = VectorXd::Zero(6);
    VectorXd l_delta_q = VectorXd::Zero(6);

    for (int count = 0; count < iteration; count++)
    {
      UpdateJacobian(l_q);

      pose_difference = PoseDifference(a_target_position, a_target_orientation, fk_current_position, fk_current_orientation);

      if (pose_difference.norm() < tolerance)
      {
        // std::cout << "Solved IK" << std::endl;
        std::cout << "Number of iterations: " << count << "     normalized pose difference: " << pose_difference.norm() << std::endl;
        ik_th = l_q;
        return true;
      }
      //get delta angle
      Eigen::ColPivHouseholderQR<MatrixXd> dec(JacobianForIK);
      l_delta_q = lambda * dec.solve(pose_difference);
      for(uint8_t i=0; i<6; i++)
      {
        l_q[i] += l_delta_q[i];
      }
    }
    for(uint8_t i=0; i<6; i++)
    {
      // ik_th(i) = l_q(i);
      ik_current_pose(i) = pose_difference(i);
    }
    return false;
  }


  bool STArmPlugin::InverseSolverUsingSRJacobian(Vector3d a_target_position, Matrix3d a_target_orientation)
  {
    VectorXd l_q = VectorXd::Zero(6);
    VectorXd l_delta_q = VectorXd::Zero(6);

    for(uint8_t i=0; i<6; i++)
    {
      l_q[i] = th[i];
    }

    Matrix4d l_current_pose;

    MatrixXd l_jacobian = MatrixXd::Identity(6, 6);

    //solver parameter
    double lambda = 0.0;
    const double param = 0.002;
    const int8_t iteration = 10;

    const double gamma = 0.5;             //rollback delta

    //sr sovler parameter
    double wn_pos = 1 / 0.3;
    double wn_ang = 1 / (2 * M_PI);
    double pre_Ek = 0.0;
    double new_Ek = 0.0;

    MatrixXd We(6, 6);
    We << wn_pos, 0, 0, 0, 0, 0,
        0, wn_pos, 0, 0, 0, 0,
        0, 0, wn_pos, 0, 0, 0,
        0, 0, 0, wn_ang, 0, 0,
        0, 0, 0, 0, wn_ang, 0,
        0, 0, 0, 0, 0, wn_ang;

    MatrixXd Wn = MatrixXd::Identity(6,6);

    MatrixXd sr_jacobian = MatrixXd::Identity(6, 6);

    //delta parameter
    VectorXd pose_difference = VectorXd::Zero(6);
    VectorXd gerr(6);

    GetJacobians(l_q, l_jacobian, l_current_pose);
    
    pose_difference = PoseDifference(a_target_position, a_target_orientation, l_current_pose);
    pre_Ek = pose_difference.transpose() * We * pose_difference;

    for (int8_t count = 0; count < iteration; count++)
    {
      GetJacobians(l_q, l_jacobian, l_current_pose);

      pose_difference = PoseDifference(a_target_position, a_target_orientation, l_current_pose);
      
      new_Ek = pose_difference.transpose() * We * pose_difference;
      
      lambda = pre_Ek + param;
      
      sr_jacobian = (l_jacobian.transpose() * We * l_jacobian) + (lambda * Wn);     //calculate sr_jacobian (J^T*we*J + lamda*Wn)
      gerr = l_jacobian.transpose() * We * pose_difference;                         //calculate gerr (J^T*we) dx
      Eigen::ColPivHouseholderQR<MatrixXd> dec(sr_jacobian);                        //solving (get dq)
      l_delta_q = dec.solve(gerr);                                                  //(J^T*we) * dx = (J^T*we*J + lamda*Wn) * dq

      l_q = l_q + l_delta_q;

      if (new_Ek < 1E-12)
      {
        std::cout << "Solved IK" << std::endl;
        ik_th = l_q;
        return true;
      }
      else if (new_Ek < pre_Ek)
      {
        pre_Ek = new_Ek;
      }
      else
      {
        l_q = l_q - gamma * l_delta_q;
      }
    }
    ik_th = l_q;
    // std::cout << "Solved IK" << std::endl;
    return false;
  }


  bool STArmPlugin::IK(Vector3d a_target_position, Matrix3d a_target_orientation)
  {
    int it = 0;
    int max_it = 10;
    float tolerance = 0.1;
    float alpha = 0.1;
    float best_norm;
    VectorXd pose_difference = VectorXd::Zero(6);
    VectorXd l_q = VectorXd::Zero(6);
    VectorXd l_best_q = VectorXd::Zero(6);
    VectorXd l_delta_q = VectorXd::Zero(6);
    MatrixXd l_Jacobian = MatrixXd::Zero(6, 6);
    MatrixXd l_Jacobian_inverse = MatrixXd::Zero(6, 6);
    Matrix4d l_current_pose;


    // while ((it == 0 || pose_difference.norm() > tolerance) && it < max_it)
    while ((it == 0 || 1 > tolerance) && it < max_it)
    {
      // GetJacobians(l_q, l_Jacobian, l_current_pose);

      // l_Jacobian_inverse = getDampedPseudoInverse(l_Jacobian, 0);
      l_Jacobian_inverse = l_Jacobian.inverse();

      pose_difference = PoseDifference(a_target_position, a_target_orientation, l_current_pose);

      // l_delta_q = alpha * l_Jacobian_inverse * pose_difference;

      // for(uint8_t i=0; i<6; i++)
      // {
      //   l_q[i] = l_q[i] + l_delta_q[i];
      // }

      // if(it == 0 || pose_difference.norm() < best_norm)
      // {
      //   l_best_q = l_q;
      //   best_norm = pose_difference.norm();
      // }
      it++;
    }
    for(uint8_t i=0; i<6; i++)
    {
      ik_th[i] = l_best_q[i];
    }

    if(it < max_it)
    {
      std::cout << "Did converge, iteration number: " << it << "    pose difference normalized: " << pose_difference.norm() << std::endl;
      return true;
    } 

    std::cout << "Did not converge, iteration number: " << it << "    pose difference normalized: " << (float)pose_difference.norm() << std::endl;

    std::cout << "Here is the matrix J:\n" << l_Jacobian << std::endl;
    std::cout << "\n Here is the matrix J_inv:\n" << l_Jacobian_inverse << std::endl;

    return false;
  }


  VectorXd STArmPlugin::PoseDifference(Vector3d a_desired_position, Matrix3d a_desired_orientation, Vector3d a_present_position, Matrix3d a_present_orientation)
  {
    Vector3d l_position_difference, l_orientation_difference;
    VectorXd l_pose_difference(6);

    l_orientation_difference = Vector3d::Zero(3);

    l_position_difference = PositionDifference(a_desired_position, a_present_position);
    // l_orientation_difference = OrientationDifference(a_desired_orientation, a_present_orientation);
    l_pose_difference << l_position_difference(0), l_position_difference(1), l_position_difference(2),
                        l_orientation_difference(0), l_orientation_difference(1), l_orientation_difference(2);

    return l_pose_difference;
  }


  Vector3d STArmPlugin::PositionDifference(Vector3d desired_position, Vector3d present_position)
  {
    Vector3d position_difference;
    position_difference = desired_position - present_position;

    return position_difference;
  }


  Vector3d STArmPlugin::OrientationDifference(Matrix3d desired_orientation, Matrix3d present_orientation)
  {
    Vector3d orientation_difference;
    orientation_difference = present_orientation * MatrixLogarithm(present_orientation.transpose() * desired_orientation);

    return orientation_difference;
  }


  Vector3d STArmPlugin::MatrixLogarithm(Matrix3d rotation_matrix)
  {
    Matrix3d R = rotation_matrix;
    Vector3d l = Vector3d::Zero();
    Vector3d rotation_vector = Vector3d::Zero();

    double theta = 0.0;
    // double diag = 0.0;
    bool diagonal_matrix = R.isDiagonal();

    l << R(2, 1) - R(1, 2),
        R(0, 2) - R(2, 0),
        R(1, 0) - R(0, 1);
    theta = atan2(l.norm(), R(0, 0) + R(1, 1) + R(2, 2) - 1);
    // diag = R.determinant();

    if (R.isIdentity())
    {
      rotation_vector.setZero(3);
      return rotation_vector;
    }
    
    if (diagonal_matrix == true)
    {
      rotation_vector << R(0, 0) + 1, R(1, 1) + 1, R(2, 2) + 1;
      rotation_vector = rotation_vector * M_PI_2;
    }
    else
    {
      rotation_vector = theta * (l / l.norm());
    }
    return rotation_vector;
  }


  Matrix3d STArmPlugin::skewSymmetricMatrix(Vector3d v)
  {
    Matrix3d skew_symmetric_matrix = Matrix3d::Zero();
    skew_symmetric_matrix << 0,     -v(2),      v(1),
                             v(2),      0,     -v(0),
                            -v(1),   v(0),         0;
    return skew_symmetric_matrix;
  }


  MatrixXd STArmPlugin::getDampedPseudoInverse(MatrixXd Jacobian, float lamda)
  {
    MatrixXd a_Jacobian_Transpose = MatrixXd::Zero(6,6);
    MatrixXd a_damped_psudo_inverse_Jacobian = MatrixXd::Zero(6,6);

    a_Jacobian_Transpose = Jacobian.transpose();

    a_damped_psudo_inverse_Jacobian = (a_Jacobian_Transpose * Jacobian + lamda * lamda * MatrixXd::Identity(6,6)).inverse() * a_Jacobian_Transpose;

    return a_damped_psudo_inverse_Jacobian;
  }

  // a argument   // m member    // l local   //p pointer     //r reference   //
  void STArmPlugin::UpdateJacobian(VectorXd a_th)
  {
    Matrix4d l_HT0, l_HT1, l_HT2, l_HT3, l_HT4, l_HT5, l_HT6;
    Matrix4d l_T00, l_T01, l_T02, l_T03, l_T04, l_T05, l_T06;
    Vector3d l_a0, l_a1, l_a2, l_a3, l_a4, l_a5;
    Vector3d l_P6_P0, l_P6_P1, l_P6_P2, l_P6_P3, l_P6_P4, l_P6_P5;
    VectorXd l_J1(6), l_J2(6), l_J3(6), l_J4(6), l_J5(6), l_J6(6); 

    l_HT0 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;
    l_HT1 << cosf(a_th[0]), 0, -sinf(a_th[0]), 0,
          sinf(a_th[0]), 0, cosf(a_th[0]), 0,
          0, -1, 0, L1,
          0, 0, 0, 1;
    l_HT2 << cosf(a_th[1]), -sinf(a_th[1]), 0, L2*cosf(a_th[1]),
          sinf(a_th[1]), cosf(a_th[1]), 0, L2*sinf(a_th[1]),
          0, 0, 1, 0, 
          0, 0, 0, 1;
    l_HT3 << cosf(a_th[2]), -sinf(a_th[2]), 0, L3*cosf(a_th[2]), 
          sinf(a_th[2]), cosf(a_th[2]), 0, L3*sinf(a_th[2]), 
          0, 0, 1, 0,
          0, 0, 0, 1;
    l_HT4 << sinf(a_th[3]), 0, cosf(a_th[3]), 0,
          -cosf(a_th[3]), 0, sinf(a_th[3]), 0,
          0, -1, 0, 0,
          0, 0, 0, 1;
    l_HT5 << -sinf(a_th[4]), 0, cosf(a_th[4]), 0,
          cosf(a_th[4]), 0, sinf(a_th[4]), 0,
          0, 1, 0, L5,
          0, 0, 0, 1;
    l_HT6 << -sinf(a_th[5]), -cosf(a_th[5]), 0, -L6*sinf(a_th[5]),
          cosf(a_th[5]), -sinf(a_th[5]), 0, L6*cosf(a_th[5]),
          0, 0, 1, 0, 
          0, 0, 0, 1;

    l_T00 = l_HT0;
    l_T01 = l_T00 * l_HT1;
    l_T02 = l_T01 * l_HT2;
    l_T03 = l_T02 * l_HT3;
    l_T04 = l_T03 * l_HT4;
    l_T05 = l_T04 * l_HT5;
    l_T06 = l_T05 * l_HT6;


    l_a0 << l_T00(0,2), l_T00(1,2), l_T00(2,2);
    l_a1 << l_T01(0,2), l_T01(1,2), l_T01(2,2);
    l_a2 << l_T02(0,2), l_T02(1,2), l_T02(2,2);
    l_a3 << l_T03(0,2), l_T03(1,2), l_T03(2,2);
    l_a4 << l_T04(0,2), l_T04(1,2), l_T04(2,2);
    l_a5 << l_T05(0,2), l_T05(1,2), l_T05(2,2);

    l_P6_P0 << l_T06(0,3) - l_T00(0,3), l_T06(1,3) - l_T00(1,3), l_T06(2,3) - l_T00(2,3);
    l_P6_P1 << l_T06(0,3) - l_T01(0,3), l_T06(1,3) - l_T01(1,3), l_T06(2,3) - l_T01(2,3);
    l_P6_P2 << l_T06(0,3) - l_T02(0,3), l_T06(1,3) - l_T02(1,3), l_T06(2,3) - l_T02(2,3);
    l_P6_P3 << l_T06(0,3) - l_T03(0,3), l_T06(1,3) - l_T03(1,3), l_T06(2,3) - l_T03(2,3);
    l_P6_P4 << l_T06(0,3) - l_T04(0,3), l_T06(1,3) - l_T04(1,3), l_T06(2,3) - l_T04(2,3);
    l_P6_P5 << l_T06(0,3) - l_T05(0,3), l_T06(1,3) - l_T05(1,3), l_T06(2,3) - l_T05(2,3);

    l_J1 << l_a0.cross(l_P6_P0), l_a0;
    l_J2 << l_a1.cross(l_P6_P1), l_a1;
    l_J3 << l_a2.cross(l_P6_P2), l_a2;
    l_J4 << l_a3.cross(l_P6_P3), l_a3;
    l_J5 << l_a4.cross(l_P6_P4), l_a4;
    l_J6 << l_a5.cross(l_P6_P5), l_a5;

    JacobianForIK << l_J1, l_J2, l_J3, l_J4, l_J5, l_J6;

    // a_Current_Pose = l_T06;

    fk_current_position << l_T06(0,3), l_T06(1,3), l_T06(2,3);
    fk_current_orientation = l_T06.block<3,3>(0,0);

    // ik_current_pose << a_current_position(0), a_current_position(1), a_current_position(2), a_th(0), a_th(1), a_th(2);
  }



  // // a argument   // m member    // l local   //p pointer     //r reference   //
  // void STArmPlugin::GetJacobians(VectorXd a_th, MatrixXd a_Jacobian, Vector3d a_current_position, Matrix3d a_current_orientation)
  // {
  //   Matrix4d l_HT0, l_HT1, l_HT2, l_HT3, l_HT4, l_HT5, l_HT6;
  //   Matrix4d l_T00, l_T01, l_T02, l_T03, l_T04, l_T05, l_T06;
  //   Vector3d l_a0, l_a1, l_a2, l_a3, l_a4, l_a5;
  //   Vector3d l_P6_P0, l_P6_P1, l_P6_P2, l_P6_P3, l_P6_P4, l_P6_P5;
  //   VectorXd l_J1(6), l_J2(6), l_J3(6), l_J4(6), l_J5(6), l_J6(6); 
  //   l_HT0 << 1, 0, 0, 0,
  //         0, 1, 0, 0,
  //         0, 0, 1, 0,
  //         0, 0, 0, 1;
  //   l_HT1 << cosf(a_th[0]), 0, -sinf(a_th[0]), 0,
  //         sinf(a_th[0]), 0, cosf(a_th[0]), 0,
  //         0, -1, 0, L1,
  //         0, 0, 0, 1;
  //   l_HT2 << cosf(a_th[1]), -sinf(a_th[1]), 0, L2*cosf(a_th[1]),
  //         sinf(a_th[1]), cosf(a_th[1]), 0, L2*sinf(a_th[1]),
  //         0, 0, 1, 0, 
  //         0, 0, 0, 1;
  //   l_HT3 << cosf(a_th[2]), -sinf(a_th[2]), 0, L3*cosf(a_th[2]), 
  //         sinf(a_th[2]), cosf(a_th[2]), 0, L3*sinf(a_th[2]), 
  //         0, 0, 1, 0,
  //         0, 0, 0, 1;
  //   l_HT4 << sinf(a_th[3]), 0, cosf(a_th[3]), 0,
  //         -cosf(a_th[3]), 0, sinf(a_th[3]), 0,
  //         0, -1, 0, 0,
  //         0, 0, 0, 1;
  //   l_HT5 << -sinf(a_th[4]), 0, cosf(a_th[4]), 0,
  //         cosf(a_th[4]), 0, sinf(a_th[4]), 0,
  //         0, 1, 0, L5,
  //         0, 0, 0, 1;
  //   l_HT6 << -sinf(a_th[5]), -cosf(a_th[5]), 0, -L6*sinf(a_th[5]),
  //         cosf(a_th[5]), -sinf(a_th[5]), 0, L6*cosf(a_th[5]),
  //         0, 0, 1, 0, 
  //         0, 0, 0, 1;
  //   l_T00 = l_HT0;
  //   l_T01 = l_T00 * l_HT1;
  //   l_T02 = l_T01 * l_HT2;
  //   l_T03 = l_T02 * l_HT3;
  //   l_T04 = l_T03 * l_HT4;
  //   l_T05 = l_T04 * l_HT5;
  //   l_T06 = l_T05 * l_HT6;
  //   l_a0 << l_T00(0,2), l_T00(1,2), l_T00(2,2);
  //   l_a1 << l_T01(0,2), l_T01(1,2), l_T01(2,2);
  //   l_a2 << l_T02(0,2), l_T02(1,2), l_T02(2,2);
  //   l_a3 << l_T03(0,2), l_T03(1,2), l_T03(2,2);
  //   l_a4 << l_T04(0,2), l_T04(1,2), l_T04(2,2);
  //   l_a5 << l_T05(0,2), l_T05(1,2), l_T05(2,2);
  //   l_P6_P0 << l_T06(0,3) - l_T00(0,3), l_T06(1,3) - l_T00(1,3), l_T06(2,3) - l_T00(2,3);
  //   l_P6_P1 << l_T06(0,3) - l_T01(0,3), l_T06(1,3) - l_T01(1,3), l_T06(2,3) - l_T01(2,3);
  //   l_P6_P2 << l_T06(0,3) - l_T02(0,3), l_T06(1,3) - l_T02(1,3), l_T06(2,3) - l_T02(2,3);
  //   l_P6_P3 << l_T06(0,3) - l_T03(0,3), l_T06(1,3) - l_T03(1,3), l_T06(2,3) - l_T03(2,3);
  //   l_P6_P4 << l_T06(0,3) - l_T04(0,3), l_T06(1,3) - l_T04(1,3), l_T06(2,3) - l_T04(2,3);
  //   l_P6_P5 << l_T06(0,3) - l_T05(0,3), l_T06(1,3) - l_T05(1,3), l_T06(2,3) - l_T05(2,3);
  //   l_J1 << l_a0.cross(l_P6_P0), l_a0;
  //   l_J2 << l_a1.cross(l_P6_P1), l_a1;
  //   l_J3 << l_a2.cross(l_P6_P2), l_a2;
  //   l_J4 << l_a3.cross(l_P6_P3), l_a3;
  //   l_J5 << l_a4.cross(l_P6_P4), l_a4;
  //   l_J6 << l_a5.cross(l_P6_P5), l_a5;
  //   a_Jacobian << l_J1, l_J2, l_J3, l_J4, l_J5, l_J6;
  //   // a_Current_Pose = l_T06;
  //   a_current_position << l_T06(0,3), l_T06(1,3), l_T06(2,3);
  //   a_current_orientation = l_T06.block<3,3>(0,0);
  //   ik_current_pose << a_current_position(0), a_current_position(1), a_current_position(2), a_th(0), a_th(1), a_th(2);
  // }
  // MatrixXd STArmPlugin::jacobian()
  // {
  //   MatrixXd jacobian = MatrixXd::Identity(6, 6);
  //   Vector3d joint_axis = Vector3d::Zero(3);
  //   Vector3d position_changed = Vector3d::Zero(3);
  //   Vector3d orientation_changed = Vector3d::Zero(3);
  //   VectorXd pose_changed = VectorXd::Zero(6);
  //   //////////////////////////////////////////////////////////////////////////////////
  //   int8_t index = 0;
  //   Name my_name =  manipulator->getWorldChildName();
  //   for (int8_t size = 0; size < 6; size++)
  //   {
  //     position_changed = skewSymmetricMatrix(joint_axis) *
  //                       (manipulator->getComponentPositionFromWorld(tool_name) - manipulator->getComponentPositionFromWorld(my_name));
  //     orientation_changed = joint_axis;
  //     pose_changed << position_changed(0),
  //         position_changed(1),
  //         position_changed(2),
  //         orientation_changed(0),
  //         orientation_changed(1),
  //         orientation_changed(2);
  //     jacobian.col(index) = pose_changed;
  //     index++;
  //     my_name = manipulator->getComponentChildName(my_name).at(0); // Get Child name which has active joint
  //   }
  //   return jacobian;
  // }
  // /// <summary>
  // // Takes the current q vector as an input and 
  // //  returns the geometric Position Jacobian J_P, the geometric Rotation Jacobian J_R, 
  // //  the current task-space position r_IE and the current task-space end-effector rotatoin matrix
  // /// </summary>
  // void GetJacobians(float[] q, out Matrix J_P, out Matrix J_R, out Vector I_r_IE_current, out RotationMatrix Rot_M_EE)
  // {
  //     List<Matrix4x4> T_I_k = GetFK(q);
  //     List<RotationMatrix> Rotation_Ms = new List<RotationMatrix>();
  //     List<Vector> position_Vs = new List<Vector>();
  //     foreach (var T in T_I_k)
  //     {
  //         Rotation_Ms.Add(T.GetRotation());
  //         position_Vs.Add(new Vector(T[0, 3], T[1, 3], T[2, 3]));
  //     }
  //     Matrix4x4 HT_EE = T_I_k.Last();
  //     Rot_M_EE = HT_EE.GetRotation();
  //     Vector pos_V_EE = new Vector(HT_EE[0, 3], HT_EE[1, 3], HT_EE[2, 3]);
  //     List<double[]> j = new List<double[]>();
  //     for (int i = 0; i < 6; i++)
  //     {
  //         j.Add(Vector.Cross(Rotation_Ms[i] * vectorZ, pos_V_EE - position_Vs[i]).ToArray());                
  //     }
  //     J_P = new Matrix(j.ToArray());
  //     j = new List<double[]>();
  //     for (int i = 0; i < 6; i++)
  //     {
  //         j.Add((Rotation_Ms[i] * vectorZ).ToArray());
  //     }
  //     J_R = new Matrix(j.ToArray());
  //     I_r_IE_current = pos_V_EE;
  // }
  // VectorXd STArmPlugin::SolveForwardKinematics(VectorXd a_th)
  // {
  //   MatrixXd HT0, HT1, HT2, HT3, HT4, HT5, HT6;
  //   MatrixXd a_T00, a_T01, a_T02, a_T03, a_T04, a_T05, a_T06;
  //   VectorXd a_ee_pose;
  //   HT0 << 1, 0, 0, 0,
  //         0, 1, 0, 0,
  //         0, 0, 1, 0,
  //         0, 0, 0, 1;
  //   HT1 << cos(a_th[0]), 0, -sin(a_th[0]), 0,
  //         sin(a_th[0]), 0, cos(a_th[0]), 0,
  //         0, -1, 0, L1,
  //         0, 0, 0, 1;
  //   HT2 << cos(a_th[1]), -sin(a_th[1]), 0, L2*cos(a_th[1]),
  //         sin(a_th[1]), cos(a_th[1]), 0, L2*sin(a_th[1]),
  //         0, 0, 1, 0, 
  //         0, 0, 0, 1;
  //   HT3 << cos(a_th[2]), -sin(a_th[2]), 0, L3*cos(a_th[2]), 
  //         sin(a_th[2]), cos(a_th[2]), 0, L3*sin(a_th[2]), 
  //         0, 0, 1, 0,
  //         0, 0, 0, 1;
  //   HT4 << sin(a_th[3]), 0, cos(a_th[3]), 0,
  //         -cos(a_th[3]), 0, sin(a_th[3]), 0,
  //         0, -1, 0, 0,
  //         0, 0, 0, 1;
  //   HT5 << -sin(a_th[4]), 0, cos(a_th[4]), 0,
  //         cos(a_th[4]), 0, sin(a_th[4]), 0,
  //         0, 1, 0, L5,
  //         0, 0, 0, 1;
  //   HT6 << -sin(a_th[5]), -cos(a_th[5]), 0, -L6*sin(a_th[5]),
  //         cos(a_th[5]), -sin(a_th[5]), 0, L6*cos(a_th[5]),
  //         0, 0, 1, 0, 
  //         0, 0, 0, 1;
  //   a_T00 = HT0;
  //   a_T01 = a_T00*HT1;
  //   a_T02 = a_T01*HT2;
  //   a_T03 = a_T02*HT3;
  //   a_T04 = a_T03*HT4;
  //   a_T05 = a_T04*HT5;
  //   a_T06 = a_T05*HT6;
  //   a_ee_pose << a_T06(0,3), a_T06(1,3), a_T06(2,3), 0, 0, 0;
  //   return a_ee_pose;
  //   // ee_rotation = a_T06.block<3,3>(0,0);
  // }
  // void STArmPlugin::SolveInverseKinematics()
  // {
  // }

}
