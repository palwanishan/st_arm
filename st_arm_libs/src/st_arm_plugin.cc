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
      RBQ3Motion1();
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

    joint_state_msg.header.stamp = ros::Time::now();
    for (uint8_t i=0; i<NUM_OF_JOINTS_WITH_TOOL; i++)
    {
      joint_state_msg.name.push_back((std::string)joint_names.at(i));
      joint_state_msg.position.push_back((float)(th[i]*RAD2DEG));
      joint_state_msg.velocity.push_back((float)(th_dot[i]));
      joint_state_msg.effort.push_back((float)joint_torque[i]);
    }
    pub_joint_state_deg.publish(joint_state_msg); 

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

    step_time = 6;
    
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
    
    if (cnt<1) initial_ee_position << ee_position(0), ee_position(1), ee_position(2);

    if(cnt_time <= step_time*100)
    { 
      ref_ee_position(0) = initial_ee_position(0) - 0.2*abs(sin(PI/2*(cnt_time/step_time)));
      ref_ee_position(1) = initial_ee_position(1) - 0.3*sin(PI/2*(cnt_time/step_time));
      ref_ee_position(2) = initial_ee_position(2) + 0.1*sin(PI*(cnt_time/step_time));
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

  //	COM based EE orientation controller - not updated
  void STArmPlugin::Motion4()
  { 
    gain_p << 2000, 2000, 2000;     
    gain_d_joint_space << 3, 5, 3, 0.2, 0.1, 0.1;
    gain_w << 10, 10, 10;

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

    shoulder_link_com << 0.5*T01(0,3), 0.5*T01(1,3), 0.5*T01(2,3);
    arm_link_com << T01(0,3) + 0.5*T12(0,3), T01(1,3) + 0.5*T12(1,3), T01(2,3) + 0.5*T12(2,3);
    elbow_link_com << T02(0,3) + 0.5*T23(0,3), T02(1,3) + 0.5*T23(1,3), T02(2,3) + 0.5*T23(2, 3);
    forearm_link_com << T03(0,3), T03(1,3), T03(2,3);
    wrist_link_com << T04(0,3) + 0.5*T45(0,3), T04(1,3) + 0.5*T45(1,3), T04(2,3) + 0.5*T45(2,3);
    endeffector_link_com << T05(0,3) + 0.5*T56(0,3), T05(1,3) + 0.5*T56(1,3), T05(2,3) + 0.5*T56(2,3);

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

    if(cnt_time <= step_time * 100)
    { 
      // ref_com_position(0) = initial_com_position(0) - 0.05 * (1 - cos(PI * (cnt_time/step_time)));
      // ref_com_position(1) = initial_com_position(1) - 0.05 * sin(PI * (cnt_time/step_time));
      // ref_com_position(2) = initial_com_position(2) + 0.02 * sin(PI * (cnt_time/step_time));
      // ref_ee_quaternion.w() = qw; 
			// ref_ee_quaternion.x() = qx; 
			// ref_ee_quaternion.y() = qy; 
			// ref_ee_quaternion.z() = qz; 
      ref_ee_quaternion.w() = 1 - 0.29 * abs(sin(PI * (cnt_time / step_time)));
			ref_ee_quaternion.x() = 0;
			ref_ee_quaternion.y() = 0.71 * sin(PI * (cnt_time / step_time));
			ref_ee_quaternion.z() = 0;
    }

		desired_com_position = initial_com_position;

		for(uint i=0; i<3; i++) virtual_spring_com(i) = gain_p(i) * (desired_com_position(i) - manipulator_com(i));
		
    tau_com = J_CoM.transpose() * virtual_spring_com;
    
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

    virtual_spring_rotational << 0, 0, 0, ee_momentum(0), ee_momentum(1), ee_momentum(2);
    tau_rotational = Jacobian.transpose() * virtual_spring_rotational;

    tau_gravity_compensation[0] = 0.0;
    tau_gravity_compensation[1] = 1.9318*sin(th[1])*sin(th[2]) - 1.9318*cos(th[1])*cos(th[2]) - 2.9498*cos(th[1]) + 0.43025*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43025*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14096*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14096*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    tau_gravity_compensation[2] = 1.9318*sin(th[1])*sin(th[2]) - 1.9318*cos(th[1])*cos(th[2]) + 0.43025*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43025*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14096*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14096*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    tau_gravity_compensation[3] = 0.43025*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43025*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14096*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14096*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    tau_gravity_compensation[4] = 0.14096*cos(th[5] + 1.5708)*sin(th[4] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    tau_gravity_compensation[5] = 0.14096*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14096*cos(th[4] + 1.5708)*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));

    if (cnt<1) last_th = th;
    th_dot = (th - last_th) / dt;
    last_th = th;
    for (int i = 0; i < 6; i++) tau_viscous_damping[i] = gain_d_joint_space[i] * th_dot[i];
    
		joint_torque =  tau_com + tau_rotational + tau_gravity_compensation - tau_viscous_damping;
		
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
    // rbq3_base_range_of_motion << 20, 20, 20;
    rbq3_base_range_of_motion << 0, 0, 0;

    for(uint8_t i=0; i<3; i++)
    {
      rbq3_ref_trajectory[i] = amplitude[i] * sin(2 * PI * frequency[i] * (traj_time - horizontal_translation[i])) + vertical_translation[i];
      rbq3_base_rpy_ref[i] = DEG2RAD * rbq3_base_range_of_motion[i] * rbq3_ref_trajectory[i];
    }

    rbq_base_gain_p = 1000;
    rbq_base_gain_d = 50;

    rbq3_base_torque = rbq_base_gain_p * (rbq3_base_rpy_ref - rbq3_base_rpy) - rbq_base_gain_d * rbq3_base_rpy_dot;

    float quad_js_p = 10;
    float quad_js_d = 0;
    for(uint8_t i=0; i<12; i++)
    {
      quad_joint_torque[i] = quad_js_p * (quad_th_ref[i] - quad_th[i]) - quad_js_d * quad_th_dot[i];
    }
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

}
