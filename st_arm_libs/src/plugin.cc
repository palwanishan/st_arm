#include "plugin.h"



void gazebo::arm_6dof_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) //처음키면 한번 실행되는 함수
{
  RBDL_INIT();
  this->model = _model;
  GetLinks();
  GetJoints();
  InitROSPubSetting();
  joint = new JOINT[6];
  old_joint = new JOINT[6];
  for(int i = 0; i < 3; i++)
  {
    joint[i].torque = 0;
    old_joint[i].torque = 0;
  }
  CONTROL_MODE = IDLE;
  this->last_update_time = this->model->GetWorld()->SimTime();
  this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&arm_6dof_plugin::UpdateAlgorithm, this));
  std::cout << "Load..." << std::endl;
}

void gazebo::arm_6dof_plugin::RBDL_INIT()
{
  //********************RBDL********************//
  rbdl_check_api_version(RBDL_API_VERSION);//check the rdbl version 설치되어있는 RBDL버전과 config.h에 있는 RBDL버전과 다르면 실행이 안됨.

  //********************MODEL********************//
  // Arm model
  Arm.rbdl_model = new Model();//declare Model /new Model()을 Arm.rbdl_model에 동적할당한다.
  Arm.rbdl_model->gravity = Math::Vector3d(0., 0., -9.81);//set gravity

  Arm_Model(Arm);

  rbdl_variable_init(Arm);

  std::cout << "RBDL_INIT()" << std::endl;
}

void gazebo::arm_6dof_plugin::rbdl_variable_init(A_RBDL &rbdl)
{
  //set Q, QDot, QDDot, prevQ, prevQDot // dof_count = num of degree
	rbdl.Q = VectorNd::Zero(rbdl.rbdl_model->dof_count);
	rbdl.QDot = VectorNd::Zero(rbdl.rbdl_model->dof_count);
	rbdl.QDDot = VectorNd::Zero(rbdl.rbdl_model->dof_count);
	rbdl.prevQ = VectorNd::Zero(rbdl.rbdl_model->dof_count);
	rbdl.prevQDot = VectorNd::Zero(rbdl.rbdl_model->dof_count);
  rbdl.Tau = VectorNd::Zero(rbdl.rbdl_model->dof_count); // 엔코더값을 받아서 사용하거나 RBDL함수를 사용하는데 있어서 변수로 사용하는 것들
}

void gazebo::arm_6dof_plugin::Arm_Model(A_RBDL &rbdl)
{
  //********************Arm********************//
  //Inertia of Body
  rbdl.bodyI_LINK1 = Math::Matrix3d(0.00098452, 0, 0, 0, 0.00098452, 0, 0, 0, 0.00108488);
  rbdl.bodyI_LINK2 = Math::Matrix3d(0.00088968, 0, 0, 0, 0.00749656, 0, 0, 0, 0.00692456);
  rbdl.bodyI_LINK3 = Math::Matrix3d(0.00069018, 0, 0, 0, 0.00665911, 0, 0, 0, 0.00622361);
  rbdl.bodyI_LINK4 = Math::Matrix3d(0.00016095, 0, 0, 0, 0.0002175, 0, 0, 0, 0.00016095);
  rbdl.bodyI_LINK5 = Math::Matrix3d(0.00011747, 0, 0, 0, 0.00043797, 0, 0, 0, 0.00043797);
  rbdl.bodyI_LINK6 = Math::Matrix3d(8.091e-05, 0, 0, 0, 0.00017123, 0, 0, 0, 0.00018952);

  //set_body_LINK1
	rbdl.body_LINK1 = Body(m_Link1, Math::Vector3d(0, 0, 0.045285), rbdl.bodyI_LINK1);
	rbdl.joint_LINK1_YAW = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 0, 1));
	rbdl.body_LINK1_id = rbdl.rbdl_model->Model::AddBody(0, Xtrans(Math::Vector3d(0, 0, 0)), rbdl.joint_LINK1_YAW, rbdl.body_LINK1);
	
  //set_body_LINK2
	rbdl.body_LINK2 = Body(m_Link2, Math::Vector3d(0.06503, 0, 0), rbdl.bodyI_LINK2);
	rbdl.joint_LINK2_PITCH = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, -1, 0));
	rbdl.body_LINK2_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_LINK1_id, Xtrans(Math::Vector3d(0, 0, 0.16)), rbdl.joint_LINK2_PITCH, rbdl.body_LINK2);

  //set_body_LINK3
	rbdl.body_LINK3 = Body(m_Link3, Math::Vector3d(0.073438, 0, 0), rbdl.bodyI_LINK3);
	rbdl.joint_LINK3_PITCH = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, -1, 0));
	rbdl.body_LINK3_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_LINK2_id, Xtrans(Math::Vector3d(0.25, 0, 0)), rbdl.joint_LINK3_PITCH, rbdl.body_LINK3);

  //set_body_LINK4
	rbdl.body_LINK4 = Body(m_Link4, Math::Vector3d(0, 0, 0), rbdl.bodyI_LINK4);
	rbdl.joint_LINK4_PITCH = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, -1, 0));
	rbdl.body_LINK4_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_LINK3_id, Xtrans(Math::Vector3d(0.25, 0, 0)), rbdl.joint_LINK4_PITCH, rbdl.body_LINK4);

  //set_body_LINK5
	rbdl.body_LINK5 = Body(m_Link5, Math::Vector3d(0.030698, 0, 0), rbdl.bodyI_LINK5);
	rbdl.joint_LINK5_ROLL = Joint(JointType::JointTypeRevolute, Math::Vector3d(1, 0, 0));
	rbdl.body_LINK5_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_LINK4_id, Xtrans(Math::Vector3d(0, 0, 0)), rbdl.joint_LINK5_ROLL, rbdl.body_LINK5);

  //set_body_LINK6
	rbdl.body_LINK6 = Body(m_Link6, Math::Vector3d(0.01852, 0, 0), rbdl.bodyI_LINK6);
	rbdl.joint_LINK6_YAW = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 0, 1));
	rbdl.body_LINK6_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_LINK5_id, Xtrans(Math::Vector3d(0.1, 0, 0)), rbdl.joint_LINK6_YAW, rbdl.body_LINK6);
}

void gazebo::arm_6dof_plugin::UpdateAlgorithm() // 여러번 실행되는 함수
{
  //************************** Time ********************************//
  current_time = this->model->GetWorld()->SimTime();
  dt = current_time.Double() - this->last_update_time.Double();
  
  EncoderRead(); 

  LinksCoMRead();
  //cout<<"UpdateAlgorithm_check1"<<endl;
  RBDL_variable_update();
  //cout<<"UpdateAlgorithm_check3"<<endl;

  Ref_Ori();

  PostureGeneration(); // PostureGeneration 하위에 Trajectory 하위에 IK푸는것 포함.
  
  jointController();

  ROSMsgPublish();
  
  Print();

  this->last_update_time = current_time;
  //cout<<"UpdateAlgorithm_check4"<<endl;
}

void gazebo::arm_6dof_plugin::GetLinks() 
{
  //LINK DEFINITION

  this->Base  = this->model->GetLink("link1");
  this->Link1 = this->model->GetLink("link2");
  this->Link2 = this->model->GetLink("link3");
  this->Link3 = this->model->GetLink("link4");
  this->Link4 = this->model->GetLink("link5");
  this->Link5 = this->model->GetLink("link6");
  this->Link6 = this->model->GetLink("link7");

}

void gazebo::arm_6dof_plugin::GetJoints()
{
  this->Joint1 = this->model->GetJoint("joint1");
  this->Joint2 = this->model->GetJoint("joint2");
  this->Joint3 = this->model->GetJoint("joint3");
  this->Joint4 = this->model->GetJoint("joint4");
  this->Joint5 = this->model->GetJoint("joint5");
  this->Joint6 = this->model->GetJoint("joint6");
}

void gazebo::arm_6dof_plugin::InitROSPubSetting()
{
  //************************ROS Msg Setting*********************************//
  P_Times = n.advertise<std_msgs::Float64>("times", 1);
  
  P_actual_joint_pos_1 = n.advertise<std_msgs::Float64>("J1_angle", 10);
  P_actual_joint_pos_2 = n.advertise<std_msgs::Float64>("J2_angle", 10);
  P_actual_joint_pos_3 = n.advertise<std_msgs::Float64>("J3_angle", 10);
  P_actual_joint_pos_4 = n.advertise<std_msgs::Float64>("J3_angle", 10);
  P_actual_joint_pos_5 = n.advertise<std_msgs::Float64>("J3_angle", 10);
  P_actual_joint_pos_6 = n.advertise<std_msgs::Float64>("J3_angle", 10);

  P_actual_joint_torque_1 = n.advertise<std_msgs::Float64>("torque_JOINT1", 10);
  P_actual_joint_torque_2 = n.advertise<std_msgs::Float64>("torque_JOINT2", 10);
  P_actual_joint_torque_3 = n.advertise<std_msgs::Float64>("torque_JOINT3", 10);
  P_actual_joint_torque_4 = n.advertise<std_msgs::Float64>("torque_JOINT4", 10);
  P_actual_joint_torque_5 = n.advertise<std_msgs::Float64>("torque_JOINT5", 10);
  P_actual_joint_torque_6 = n.advertise<std_msgs::Float64>("torque_JOINT6", 10);
  
  P_actual_Pos_X = n.advertise<std_msgs::Float64>("act_X", 10);
  P_actual_Pos_Y = n.advertise<std_msgs::Float64>("act_Y", 10);
  P_actual_Pos_Z = n.advertise<std_msgs::Float64>("act_Z", 10);
  P_actual_Pos_Roll = n.advertise<std_msgs::Float64>("act_Roll", 10);
  P_actual_Pos_Pitch = n.advertise<std_msgs::Float64>("act_Pitch", 10);
  P_actual_Pos_Yaw = n.advertise<std_msgs::Float64>("act_Yaw", 10);
  P_ref_Pos_X = n.advertise<std_msgs::Float64>("ref_X", 10);
  P_ref_Pos_Y = n.advertise<std_msgs::Float64>("ref_Y", 10);
  P_ref_Pos_Z = n.advertise<std_msgs::Float64>("ref_Z", 10);
  P_ref_Pos_Roll = n.advertise<std_msgs::Float64>("ref_Roll", 10);
  P_ref_Pos_Pitch = n.advertise<std_msgs::Float64>("ref_Pitch", 10);
  P_ref_Pos_Yaw = n.advertise<std_msgs::Float64>("ref_Yaw", 10);

  P_Arm_CoM_X = n.advertise<std_msgs::Float64>("P_Arm_CoM_X", 10);
  P_Arm_CoM_Y = n.advertise<std_msgs::Float64>("P_Arm_CoM_Y", 10);
  P_Arm_CoM_Z = n.advertise<std_msgs::Float64>("P_Arm_CoM_Z", 10);
  P_Arm_CoM_Roll = n.advertise<std_msgs::Float64>("P_Arm_CoM_Roll", 10);
  P_Arm_CoM_Pitch = n.advertise<std_msgs::Float64>("P_Arm_CoM_Pitch", 10);
  P_Arm_CoM_Yaw = n.advertise<std_msgs::Float64>("P_Arm_CoM_Yaw", 10);
  
  P_ref_CoM_X = n.advertise<std_msgs::Float64>("ref_CoM_X", 10);
  P_ref_CoM_Y = n.advertise<std_msgs::Float64>("ref_CoM_Y", 10);
  P_ref_CoM_Z = n.advertise<std_msgs::Float64>("ref_CoM_Z", 10);
  P_Act_Ori_Vel_X = n.advertise<std_msgs::Float64>("Act_roll_Vel", 10);
  P_Act_Ori_Vel_Y = n.advertise<std_msgs::Float64>("Act_pitch_Vel", 10);
  P_Act_Ori_Vel_Z = n.advertise<std_msgs::Float64>("Act_yaw_Vel", 10);
  P_Act_X_CoM = n.advertise<std_msgs::Float64>("p_Act_X_CoM", 10);
  P_Act_Y_CoM = n.advertise<std_msgs::Float64>("p_Act_Y_CoM", 10);
  P_Act_Z_CoM = n.advertise<std_msgs::Float64>("p_Act_Z_CoM", 10);

  P_ros_msg = n.advertise<std_msgs::Float64MultiArray>("TmpData", 50); // topicname, queue_size = 50
  m_ros_msg.data.resize(50);
  server_sub1 = n.subscribe("Ctrl_mode", 1, &gazebo::arm_6dof_plugin::Callback1, this);

  open_manipulator_joint_states_sub_ = n.subscribe("joint_states", 10, &gazebo::arm_6dof_plugin::jointStatesCallback, this);
}

void gazebo::arm_6dof_plugin::LinksCoMRead() 
{

  double nx, ny, nz, ox, oy, oz, ax, ay, az;
  /*
  A0 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
  A1 << cos(Arm.Q[0]), 0, sin(Arm.Q[0]), 0,
        sin(Arm.Q[0]), 0, -cos(Arm.Q[0]), 0,
        0, 1, 0, L1,
        0, 0, 0, 1;
  A2 << cos(Arm.Q[1]), -sin(Arm.Q[1]), 0, L2*cos(Arm.Q[1]),
        sin(Arm.Q[1]), cos(Arm.Q[1]), 0, L2*sin(Arm.Q[1]),
        0, 0, 1, 0, 
        0, 0, 0, 1;
  A3 << cos(Arm.Q[2]), -sin(Arm.Q[2]), 0, L3*cos(Arm.Q[2]), 
        sin(Arm.Q[2]), cos(Arm.Q[2]), 0, L3*sin(Arm.Q[2]), 
        0, 0, 1, 0,
        0, 0, 0, 1;
  A4 << -sin(Arm.Q[3]), 0, cos(Arm.Q[3]), 0,
        cos(Arm.Q[3]), 0, sin(Arm.Q[3]), 0,
        0, 1, 0, 0,
        0, 0, 0, 1;
  A5 << -sin(Arm.Q[4]), 0, cos(Arm.Q[4]), 0,
        cos(Arm.Q[4]), 0, sin(Arm.Q[4]), 0,
        0, 1, 0, L5,
        0, 0, 0, 1;
  A6 << -sin(Arm.Q[5]), -cos(Arm.Q[5]), 0, -L6*sin(Arm.Q[5]),
        cos(Arm.Q[5]), -sin(Arm.Q[5]), 0, L6*cos(Arm.Q[5]),
        0, 0, 1, 0, 
        0, 0, 0, 1;
  */

  //-----Jinumanip new DH convention-----
  A0 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
  A1 << cos(Arm.Q[0]), 0, -sin(Arm.Q[0]), 0,
        sin(Arm.Q[0]), 0, cos(Arm.Q[0]), 0,
        0, -1, 0, L1,
        0, 0, 0, 1;
  A2 << cos(Arm.Q[1]), -sin(Arm.Q[1]), 0, L2*cos(Arm.Q[1]),
        sin(Arm.Q[1]), cos(Arm.Q[1]), 0, L2*sin(Arm.Q[1]),
        0, 0, 1, 0, 
        0, 0, 0, 1;
  A3 << cos(Arm.Q[2]), -sin(Arm.Q[2]), 0, L3*cos(Arm.Q[2]), 
        sin(Arm.Q[2]), cos(Arm.Q[2]), 0, L3*sin(Arm.Q[2]), 
        0, 0, 1, 0,
        0, 0, 0, 1;
  A4 << sin(Arm.Q[3]), 0, cos(Arm.Q[3]), 0,
        -cos(Arm.Q[3]), 0, sin(Arm.Q[3]), 0,
        0, -1, 0, 0,
        0, 0, 0, 1;
  A5 << -sin(Arm.Q[4]), 0, cos(Arm.Q[4]), 0,
        cos(Arm.Q[4]), 0, sin(Arm.Q[4]), 0,
        0, 1, 0, L5,
        0, 0, 0, 1;
  A6 << -sin(Arm.Q[5]), -cos(Arm.Q[5]), 0, -L6*sin(Arm.Q[5]),
        cos(Arm.Q[5]), -sin(Arm.Q[5]), 0, L6*cos(Arm.Q[5]),
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
  T34 = T04-T03;
  T45 = T05-T04;
  T56 = T06-T05;
  R_act = T06.block<3,3>(0,0);
  R1_act = R_act.block<3,1>(0,0); R2_act = R_act.block<3,1>(0,1); R3_act = R_act.block<3,1>(0,2);


  nx = T06(0,0); ny = T06(1,0); nz = T06(2,0); ox = T06(1,0); oy = T06(1,1); oz = T06(1,2);
  ax = T06(2,0); ay = T06(2,1); az = T06(2,2);

  Act_Pos << T06(0,3), T06(1,3), T06(2,3);
  Act_Pos_Vel = (Act_Pos-Pre_Act_Pos)/inner_dt;
  Pre_Act_Pos = Act_Pos;
  //Get the Euler Angle - pi, theta, psi
  Act_Ori(2) = atan2(ny,nx); // yaw
  Act_Ori(1) = atan2(-nz,cos(Act_Ori(2))*nx + sin(Act_Ori(2))*ny); //pitch
  Act_Ori(0) = atan2(sin(Act_Ori(2))*ax - cos(Act_Ori(2))*ay, -sin(Act_Ori(2))*ox + cos(Act_Ori(2))*oy); //roll

  if(Pre_Act_Ori(0) == 0 && Pre_Act_Ori(1) == 0 && Pre_Act_Ori(2) == 0) Pre_Act_Ori = Act_Ori;
  Act_Ori_Vel = (Act_Ori-Pre_Act_Ori)/inner_dt;
  Pre_Act_Ori = Act_Ori;

  // cout<<"fi : "<<fi*180/PI<<endl;
  // cout<<"theta : "<<theta*180/PI<<endl;
  // cout<<"csi : "<<csi*180/PI<<endl;

  Act_CoM1 << 0.045285/L1*T01(0,3), 0.045285/L1*T01(1,3), 0.045285/L1*T01(2,3);
  Act_CoM2 << T01(0,3)+0.06503/L2*T12(0,3), T01(1,3)+0.06503/L2*T12(1,3), T01(2,3)+0.06503/L2*T12(2,3);
  Act_CoM3 << T02(0,3)+0.073438/L3*T23(0,3), T02(1,3)+0.073438/L3*T23(1,3), T02(2,3)+0.073438/L3*T23(2,3);
  Act_CoM4 << T03(0,3), T03(1,3), T03(2,3);
  Act_CoM5 << T04(0,3)+0.030698/L5*T45(0,3), T04(1,3)+0.030698/L5*T45(1,3), T04(2,3)+0.030698/L5*T45(2,3);
  Act_CoM6 << T05(0,3)+0.01852/L6*T56(0,3), T05(1,3)+0.01852/L6*T56(1,3), T05(2,3)+0.01852/L6*T56(2,3);


  Act_CoM << (m_Link1*Act_CoM1(0)+m_Link2*Act_CoM2(0)+m_Link3*Act_CoM3(0)+m_Link4*Act_CoM4(0)+m_Link5*Act_CoM5(0)+m_Link5*Act_CoM5(0))/m_Arm,
             (m_Link1*Act_CoM1(1)+m_Link2*Act_CoM2(1)+m_Link3*Act_CoM3(1)+m_Link4*Act_CoM4(1)+m_Link5*Act_CoM5(1)+m_Link6*Act_CoM6(1))/m_Arm,
             (m_Link1*Act_CoM1(2)+m_Link2*Act_CoM2(2)+m_Link3*Act_CoM3(2)+m_Link4*Act_CoM4(2)+m_Link5*Act_CoM5(2)+m_Link6*Act_CoM6(2))/m_Arm;

  if(Pre_Act_CoM(0) == 0 && Pre_Act_CoM(1) == 0 && Pre_Act_CoM(2) == 0) Pre_Act_CoM = Act_CoM;
  Act_CoM_Vel = (Act_CoM-Pre_Act_CoM)/inner_dt;
  Pre_Act_CoM = Act_CoM;
}

void gazebo::arm_6dof_plugin::EncoderRead()
{
  //************************** Encoder ********************************//  daunting
  actual_joint_pos[0] = this->Joint1->Position(2);//.Radian();
  actual_joint_pos[1] = this->Joint2->Position(1);//.Radian();
  actual_joint_pos[2] = this->Joint3->Position(1);//.Radian();
  actual_joint_pos[3] = this->Joint4->Position(1);//.Radian();
  actual_joint_pos[4] = this->Joint5->Position(0);//.Radian();
  actual_joint_pos[5] = this->Joint6->Position(2);//.Radian();

/*  actual_joint_pos[0] = this->Joint1->Position(2);//.Radian();
  actual_joint_pos[1] = this->Joint2->Position(1);//.Radian();
  actual_joint_pos[2] = this->Joint3->Position(1);//.Radian();
  actual_joint_pos[3] = this->Joint4->Position(1);//.Radian();
  actual_joint_pos[4] = this->Joint5->Position(0);//.Radian();
  actual_joint_pos[5] = this->Joint6->Position(2);//.Radian();
*/
  //************************** Encoder (dot)********************************//
  for (int i = 0; i < 6; i++)
  {
    actual_joint_vel[i] = (actual_joint_pos[i] - pre_actual_joint_pos[i]) / inner_dt;
    actual_joint_acc[i] = (actual_joint_vel[i] - pre_actual_joint_vel[i]) / inner_dt;

    pre_actual_joint_pos[i] = actual_joint_pos[i];
    pre_actual_joint_vel[i] = actual_joint_vel[i];
  }
}

void gazebo::arm_6dof_plugin::jointController() //limit torque 설정
{
  //Applying torques
  this->Joint1->SetForce(2, joint[0].torque); 
  this->Joint2->SetForce(1, joint[1].torque); 
  this->Joint3->SetForce(1, joint[2].torque);
  this->Joint4->SetForce(1, joint[3].torque);
  this->Joint5->SetForce(0, joint[4].torque); 
  this->Joint6->SetForce(2, joint[5].torque);
}

void gazebo::arm_6dof_plugin::Callback1(const std_msgs::Int32Ptr & msg) {
    cnt = 0;

    if (msg -> data == 0) {
        cnt = 0;
        CONTROL_MODE = IDLE;

    } else if (msg -> data == 1) {
        cnt = 0;
        CONTROL_MODE = CoM_EEOri;

    } else if (msg -> data == 2) {
        cnt = 0;
        CONTROL_MODE = JOINT_Angle;

    } else if (msg -> data == 3) {
        cnt = 0;
        CONTROL_MODE = EE_Pos_Ori;

    } else if (msg -> data == 4) {
        cnt = 0;
        CONTROL_MODE = Jinu_GC;

    } else if (msg -> data == 5) {
        cnt = 0;
        CONTROL_MODE = Jinu_EE;

    } else if (msg -> data == 6) {
        cnt = 0;
        CONTROL_MODE = Jinu_EE_Rot;

    } else if (msg -> data == 7) {
        cnt = 0;
        CONTROL_MODE = Jinu_EE_Rot_Quat;

    } else {
        cnt = 0;
        CONTROL_MODE = IDLE;
    }
}

void gazebo::arm_6dof_plugin::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle; 
  temp_angle.resize(NUM_OF_JOINT_AND_TOOL);
  for(int i = 0; i < msg->name.size(); i++)
  {
         if(!msg->name.at(i).compare("joint1")) temp_angle.at(0) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint2")) temp_angle.at(1) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint3")) temp_angle.at(2) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint4")) temp_angle.at(3) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint5")) temp_angle.at(4) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint6")) temp_angle.at(5) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("gripper")) temp_angle.at(6) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}

void gazebo::arm_6dof_plugin::PostureGeneration()
{
  if (CONTROL_MODE == IDLE)
  {
    Init_Pos_Traj();
  }
  else if (CONTROL_MODE == CoM_EEOri)
  {
    Input_CoM_EEOri();
  }
  else if (CONTROL_MODE == JOINT_Angle) //들고있는발의 중력보상 보행할 때 사용하지 않음.
  {
    Input_Joint_Angle(); 
  }
  else if (CONTROL_MODE == EE_Pos_Ori)
  {
    Input_EE_Pos_Ori();
  }
  else if (CONTROL_MODE == Jinu_GC)
  {
    Jinu_Gravity_Compensation();
  }
  else if (CONTROL_MODE == Jinu_EE)
  {
    Jinu_Input_EE_Pos_Ori();
  }
  else if (CONTROL_MODE == Jinu_EE_Rot)
  {
    Jinu_Input_EE_Pos_Ori_Rot();
  }
  else if (CONTROL_MODE == Jinu_EE_Rot_Quat)
  {
    Jinu_Input_EE_Pos_Ori_Rot_Quat();
  }
  else {
    CONTROL_MODE = IDLE;
  }
}

void gazebo::arm_6dof_plugin::RBDL_variable_update()
{
  // Arm variable
  Arm.Q(0) = actual_joint_pos[0];
  Arm.Q(1) = actual_joint_pos[1];
  Arm.Q(2) = actual_joint_pos[2];
  Arm.Q(3) = actual_joint_pos[3];
  Arm.Q(4) = actual_joint_pos[4];
  Arm.Q(5) = actual_joint_pos[5];

  Arm.QDot = (Arm.Q - Arm.prevQ) / inner_dt;
  Arm.QDDot = (Arm.QDot - Arm.prevQDot) / inner_dt;

  Arm.prevQ = Arm.Q;
  Arm.prevQDot = Arm.QDot;

}

void gazebo::arm_6dof_plugin::Ref_Ori()
{
    qw = 1;
    qx = 0;
    qy = 0;
    qz = 0;
}

void gazebo::arm_6dof_plugin::Init_Pos_Traj()  // 0
{
  Kp_v << 100, 100, 100, 30, 30, 30;
  Kd_v << 1, 1, 1, 0.1, 0.1, 0.1;
  step_time = 4; //주기설정 (초) 변수
  cnt_time = cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  double nx, ny, nz, ox, oy, oz, ax, ay, az;
  double Init_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));
  Math::Matrix3d R, R_Tmp;
  cnt++;
  if(cnt_time <= step_time)
  {
    ref_Arm_th[0] = 0*Init_trajectory*deg2rad;
    ref_Arm_th[1] = -60*Init_trajectory*deg2rad;
    ref_Arm_th[2] = 60*Init_trajectory*deg2rad;
    ref_Arm_th[3] = 0*Init_trajectory*deg2rad;
    ref_Arm_th[4] = 0*Init_trajectory*deg2rad;
    ref_Arm_th[5] = 0*Init_trajectory*deg2rad; 
  }

  for (int i = 0; i < 5; i++) {
    // if(cnt_time <= step_time) joint[i].torque = Kp_v[i]*(ref_Arm_th[i] - actual_joint_pos[i]) + Kd_v[i] * (0 - actual_joint_vel[i]) + Arm.Tau(i)*Init_trajectory; // 기본 PV제어 코드
    // else Arm.Tau(i);
    joint[i].torque = Kp_v[i]*(ref_Arm_th[i] - actual_joint_pos[i]) + Kd_v[i] * (0 - actual_joint_vel[i]); // 기본 PV제어 코드
    old_joint[i].torque = joint[i].torque;
  }
}

void gazebo::arm_6dof_plugin::Input_CoM_EEOri()  // 1
{
  Kp_s << 2000, 2000, 2000; // 10000, 10000, 10000; 800, 800, 800
  Kd_s << 60, 60, 60; // 75, 75, 75;               1, 1, 1
  step_time = 6; //주기설정 (초) 변수
  cnt_time = cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  double Init_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));
  double nx, ny, nz, ox, oy, oz, ax, ay, az;
  Eigen::Vector3d CoM1, CoM2, CoM3, CoM4, CoM5, CoM6, C1, C2, C3, C4, C5, C6, a0, a1, a2, a3, a4, a5, C1_P0, C2_P1 ,C3_P2, C4_P3, C5_P4, C6_P5, J1_CoM, J2_CoM, J3_CoM, J4_CoM, J5_CoM, J6_CoM, F, P6_P0, P6_P1, P6_P2, P6_P3, P6_P4, P6_P5;
  Eigen::MatrixXd J(6,6); Eigen::MatrixXd J_CoM(3,6); Eigen::VectorXd F_Ori(6), J1(6), J2(6), J3(6), J4(6), J5(6), J6(6), T_Ori(6), T_CoM(6);
  Eigen::Vector3d Ori_Err, m_Ori, Kw;

  Kw << 10, 10, 10;
  cnt++;
  if(cnt<2){
    Init_Pose << Act_CoM(0),Act_CoM(1),Act_CoM(2),Act_Ori(0),Act_Ori(1),Act_Ori(2);
    cout<<"Init_Pose\n"<<Init_Pose<<endl;
  }
  if(cnt_time <= step_time*100)
  {
    ref_Pose(0) = Init_Pose(0) - 0.05*(1-cos(PI*(cnt_time/step_time)));
    ref_Pose(1) = Init_Pose(1) + 0.05*sin(PI*(cnt_time/step_time));
    ref_Pose(2) = Init_Pose(2) + 0.02*sin(PI*(cnt_time/step_time));
    ref_Pose(3) = Init_Pose(3) + 30*deg2rad*Init_trajectory;
    ref_Pose(4) = Init_Pose(4) - 30*deg2rad*Init_trajectory;
    ref_Pose(5) = Init_Pose(5) + 30*deg2rad*Init_trajectory;
  }

  // *** 참조 CoM과 E.E의 방위를 넣어서 사용할 때 *** //
  A0 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
  A1 << cos(Arm.Q[0]), 0, sin(Arm.Q[0]), 0,
        sin(Arm.Q[0]), 0, -cos(Arm.Q[0]), 0,
        0, 1, 0, L1,
        0, 0, 0, 1;
  A2 << cos(Arm.Q[1]), -sin(Arm.Q[1]), 0, L2*cos(Arm.Q[1]),
        sin(Arm.Q[1]), cos(Arm.Q[1]), 0, L2*sin(Arm.Q[1]),
        0, 0, 1, 0, 
        0, 0, 0, 1;
  A3 << cos(Arm.Q[2]), -sin(Arm.Q[2]), 0, L3*cos(Arm.Q[2]), 
        sin(Arm.Q[2]), cos(Arm.Q[2]), 0, L3*sin(Arm.Q[2]), 
        0, 0, 1, 0,
        0, 0, 0, 1;
  A4 << -sin(Arm.Q[3]), 0, cos(Arm.Q[3]), 0,
        cos(Arm.Q[3]), 0, sin(Arm.Q[3]), 0,
        0, 1, 0, 0,
        0, 0, 0, 1;
  A5 << -sin(Arm.Q[4]), 0, cos(Arm.Q[4]), 0,
        cos(Arm.Q[4]), 0, sin(Arm.Q[4]), 0,
        0, 1, 0, L5,
        0, 0, 0, 1;
  A6 << -sin(Arm.Q[5]), -cos(Arm.Q[5]), 0, -L6*sin(Arm.Q[5]),
        cos(Arm.Q[5]), -sin(Arm.Q[5]), 0, L6*cos(Arm.Q[5]),
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
  T34 = T04-T03;
  T45 = T05-T04;
  T56 = T06-T05;

  // ref Ori넣어줄 때
  R_ref << cos(ref_Pose(5))*cos(ref_Pose(4)), -sin(ref_Pose(5))*cos(ref_Pose(3))+cos(ref_Pose(5))*sin(ref_Pose(4))*sin(ref_Pose(3)), sin(ref_Pose(5))*sin(ref_Pose(3))+cos(ref_Pose(5))*sin(ref_Pose(4))*cos(ref_Pose(3)),
           sin(ref_Pose(5))*cos(ref_Pose(4)), cos(ref_Pose(5))*cos(ref_Pose(3))+sin(ref_Pose(5))*sin(ref_Pose(4))*sin(ref_Pose(3)), -cos(ref_Pose(5))*sin(ref_Pose(3))+sin(ref_Pose(5))*sin(ref_Pose(4))*sin(ref_Pose(3)),
           -sin(ref_Pose(4)), cos(ref_Pose(4))*sin(ref_Pose(3)), cos(ref_Pose(4))*cos(ref_Pose(3));

  R1_ref = R_ref.block<3,1>(0,0); R2_ref = R_ref.block<3,1>(0,1); R3_ref = R_ref.block<3,1>(0,2);

  Ori_Err = R1_act.cross(R1_ref) + R2_act.cross(R2_ref) + R3_act.cross(R3_ref);
  m_Ori << Kw(0)*Ori_Err(0), Kw(1)*Ori_Err(1), Kw(2)*Ori_Err(2);

  ref_Pos << T06(0,3), T06(1,3), T06(2,3);
  nx = T06(0,0); ny = T06(1,0); nz = T06(2,0); ox = T06(1,0); oy = T06(1,1); oz = T06(1,2);
  ax = T06(2,0); ay = T06(2,1); az = T06(2,2);


  // Get the Euler Angle - pi, theta, psi
  fi = atan2(ny,nx); // yaw
  theta = atan2(-nz,cos(fi)*nx + sin(fi)*ny); //pitch
  csi = atan2(sin(fi)*ax - cos(fi)*ay, -sin(fi)*ox + cos(fi)*oy); //roll
  
  CoM1 << 0.045285/L1*T01(0,3), 0.045285/L1*T01(1,3), 0.045285/L1*T01(2,3);
  CoM2 << T01(0,3)+0.06503/L2*T12(0,3), T01(1,3)+0.06503/L2*T12(1,3), T01(2,3)+0.06503/L2*T12(2,3);
  CoM3 << T02(0,3)+0.073438/L3*T23(0,3), T02(1,3)+0.073438/L3*T23(1,3), T02(2,3)+0.073438/L3*T23(2,3);
  CoM4 << T03(0,3), T03(1,3), T03(2,3);
  CoM5 << T04(0,3)+0.030698/L5*T45(0,3), T04(1,3)+0.030698/L5*T45(1,3), T04(2,3)+0.030698/L5*T45(2,3);
  CoM6 << T05(0,3)+0.01852/L6*T56(0,3), T05(1,3)+0.01852/L6*T56(1,3), T05(2,3)+0.01852/L6*T56(2,3);

  ref_CoM << (m_Link1*CoM1(0)+m_Link2*CoM2(0)+m_Link3*CoM3(0)+m_Link4*CoM4(0)+m_Link5*CoM5(0)+m_Link6*CoM6(0))/m_Arm,
             (m_Link1*CoM1(1)+m_Link2*CoM2(1)+m_Link3*CoM3(1)+m_Link4*CoM4(1)+m_Link5*CoM5(1)+m_Link6*CoM6(1))/m_Arm,
             (m_Link1*CoM1(2)+m_Link2*CoM2(2)+m_Link3*CoM3(2)+m_Link4*CoM4(2)+m_Link5*CoM5(2)+m_Link6*CoM6(2))/m_Arm;  

  C1 << ref_CoM(0), ref_CoM(1), ref_CoM(2);
  C2 << (m_Link2*CoM2(0)+m_Link3*CoM3(0)+m_Link4*CoM4(0)+m_Link5*CoM5(0)+m_Link6*CoM6(0))/M2, (m_Link2*CoM2(1)+m_Link3*CoM3(1)+m_Link4*CoM4(1)+m_Link5*CoM5(1)+m_Link6*CoM6(1))/M2, (m_Link2*CoM2(2)+m_Link3*CoM3(2)+m_Link4*CoM4(2)+m_Link5*CoM5(2)+m_Link6*CoM6(2))/M2;
  C3 << (m_Link3*CoM3(0)+m_Link4*CoM4(0)+m_Link5*CoM5(0)+m_Link6*CoM6(0))/M3, (m_Link3*CoM3(1)+m_Link4*CoM4(1)+m_Link5*CoM5(1)+m_Link6*CoM6(1))/M3, (m_Link3*CoM3(2)+m_Link4*CoM4(2)+m_Link5*CoM5(2)+m_Link6*CoM6(2))/M3;
  C4 << (m_Link4*CoM4(0)+m_Link5*CoM5(0)+m_Link6*CoM6(0))/M4, (m_Link4*CoM4(1)+m_Link5*CoM5(1)+m_Link6*CoM6(1))/M4, (m_Link4*CoM4(2)+m_Link5*CoM5(2)+m_Link6*CoM6(2))/M4;
  C5 << (m_Link5*CoM5(0)+m_Link6*CoM6(0))/M5, (m_Link5*CoM5(1)+m_Link6*CoM6(1))/M5, (m_Link5*CoM5(2)+m_Link6*CoM6(2))/M5;
  C6 << CoM6(0), CoM6(1), CoM6(2);
  
  
  a0 << T00(0,2), T00(1,2), T00(2,2);
  C1_P0 << C1(0)-T00(0,3), C1(1)-T00(1,3), C1(2)-T00(2,3);

  a1 << T01(0,2), T01(1,2), T01(2,2);
  C2_P1 << C2(0)-T01(0,3), C2(1)-T01(1,3), C2(2)-T01(2,3);

  a2 << T02(0,2), T02(1,2), T02(2,2);
  C3_P2 << C3(0)-T02(0,3), C3(1)-T02(1,3), C3(2)-T02(2,3);

  a3 << T03(0,2), T03(1,2), T03(2,2);
  C4_P3 << C4(0)-T03(0,3), C4(1)-T03(1,3), C4(2)-T03(2,3);

  a4 << T04(0,2), T04(1,2), T04(2,2);
  C5_P4 << C5(0)-T04(0,3), C5(1)-T04(1,3), C5(2)-T04(2,3);

  a5 << T05(0,2), T05(1,2), T05(2,2);
  C6_P5 << C6(0)-T05(0,3), C6(1)-T05(1,3), C6(2)-T05(2,3);

  J1_CoM = M1/m_Arm*a0.cross(C1_P0);
  J2_CoM = M2/m_Arm*a1.cross(C2_P1);
  J3_CoM = M3/m_Arm*a2.cross(C3_P2);
  J4_CoM = M4/m_Arm*a3.cross(C4_P3);
  J5_CoM = M5/m_Arm*a4.cross(C5_P4);
  J6_CoM = M6/m_Arm*a5.cross(C6_P5);

  J_CoM << J1_CoM, J2_CoM, J3_CoM, J4_CoM, J5_CoM, J6_CoM;
  F << Kp_s(0)*(ref_Pose(0)-Act_CoM(0))-Kd_s(0)*(Act_CoM_Vel(0)), Kp_s(1)*(ref_Pose(1)-Act_CoM(1))-Kd_s(1)*(Act_CoM_Vel(1)), Kp_s(2)*(ref_Pose(2)-Act_CoM(2))-Kd_s(2)*(Act_CoM_Vel(2));
  T_CoM = J_CoM.transpose()*F;

  InverseDynamics(*Arm.rbdl_model, Arm.Q, VectorXd::Zero(6), VectorXd::Zero(6), Arm.Tau, NULL);

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

  J << J1, J2, J3, J4, J5, J6;

  F_Ori << 0, 0, 0, m_Ori(0), m_Ori(1), m_Ori(2);
  T_Ori = J.transpose()*F_Ori;
  ///////////////토크 입력////////////////
  for (int i = 0; i < 6; i++) {
    //joint[i].torque = T_CoM(i) + T_Ori(i) + Arm.Tau(i);
    joint[i].torque = T_CoM(i) + T_Ori(i);
    old_joint[i].torque = joint[i].torque;
  }
}

void gazebo::arm_6dof_plugin::Input_Joint_Angle() // 2
{
  Kp_v << 100, 100, 100, 30, 30, 30;
  Kd_v << 1, 1, 1, 0.1, 0.1, 0.1;
  step_time = 6; //주기설정 (초) 변수
  cnt_time = cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  cnt++;

  if(cnt_time <= step_time*100)
  {
    for(int i=0; i < 6; i++) 
      {
        ref_Arm_th[i] = present_joint_angle_[i];
      }
  }

  ///////////////토크 입력////////////////
  for (int i = 0; i < 6; i++) {
    joint[i].torque = Kp_v[i]*(ref_Arm_th[i] - actual_joint_pos[i]) + Kd_v[i] * (0 - actual_joint_vel[i]); // 기본 PV제어 코드
    old_joint[i].torque = joint[i].torque;
  }
}

void gazebo::arm_6dof_plugin::Input_EE_Pos_Ori()  // 3
{
  step_time = 6; //주기설정 (초) 변수
  cnt_time = cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  double Init_trajectory = 0.5*(1-cos(2*PI*(cnt_time/step_time)));
  double nx, ny, nz, ox, oy, oz, ax, ay, az;
  Eigen::Vector3d CoM1, CoM2, CoM3, CoM4, CoM5, CoM6, C1, C2, C3, C4, C5, C6, a0, a1, a2, a3, a4, a5, C1_P0, C2_P1 ,C3_P2, C4_P3, C5_P4, C6_P5, J1_CoM, J2_CoM, J3_CoM, J4_CoM, J5_CoM, J6_CoM, P6_P0, P6_P1, P6_P2, P6_P3, P6_P4, P6_P5;
  Eigen::MatrixXd J(6,6); Eigen::MatrixXd J_CoM(3,6); Eigen::VectorXd F(6), J1(6), J2(6), J3(6), J4(6), J5(6), J6(6), T(6);
  Eigen::Vector3d Ori_Err, m_Ori, f_Pos, Kw;

  Kp_s << 2000, 2000, 2000;
  Kw << 10, 10, 10;
  cnt++;
  if(cnt<2){
    Init_Pose << Act_Pos(0), Act_Pos(1), Act_Pos(2), Act_Ori(0), Act_Ori(1), Act_Ori(2);
    cout<<"Init_Pose\n"<<Init_Pose<<endl;
  }
  if(cnt_time <= step_time*100)
  { 

    ref_Pose(0) = Init_Pose(0) - 0.0*Init_trajectory;
    ref_Pose(1) = Init_Pose(1) - 0.2*sin(PI/2*(cnt_time/step_time));
    ref_Pose(2) = Init_Pose(2) + 0.1*sin(PI*(cnt_time/step_time));
    ref_Pose(3) = Init_Pose(3) - 0*deg2rad*sin(2*PI*(cnt_time/step_time));
    ref_Pose(4) = Init_Pose(4) + 0*deg2rad*sin(2*PI*(cnt_time/step_time));
    ref_Pose(5) = Init_Pose(5) - 0*deg2rad; // *sin(2*PI*(cnt_time/step_time));

    // if(int(cnt_time / step_time) == 0 ){
    //   ref_Pose(0) = Init_Pose(0) - 0.1*Init_trajectory;
    //   ref_Pose(1) = Init_Pose(1) + 0.0*Init_trajectory;
    //   ref_Pose(2) = Init_Pose(2) + 0.0*Init_trajectory;
    //   ref_Pose(3) = Init_Pose(3) + 0*deg2rad*Init_trajectory;
    //   ref_Pose(4) = Init_Pose(4) + 0*deg2rad*Init_trajectory;
    //   ref_Pose(5) = Init_Pose(5) + 0*deg2rad*Init_trajectory;
    // }
    // if(int(cnt_time / step_time) == 1 ){
    //   ref_Pose(0) = Init_Pose(0) - 0.0*Init_trajectory;
    //   ref_Pose(1) = Init_Pose(1) + 0.1*Init_trajectory;
    //   ref_Pose(2) = Init_Pose(2) + 0.0*Init_trajectory;
    //   ref_Pose(3) = Init_Pose(3) + 0*deg2rad*Init_trajectory;
    //   ref_Pose(4) = Init_Pose(4) + 0*deg2rad*Init_trajectory;
    //   ref_Pose(5) = Init_Pose(5) + 0*deg2rad*Init_trajectory;
    // }
    // if(int(cnt_time / step_time) == 2 ){
    //   ref_Pose(0) = Init_Pose(0) - 0.0*Init_trajectory;
    //   ref_Pose(1) = Init_Pose(1) + 0.0*Init_trajectory;
    //   ref_Pose(2) = Init_Pose(2) + 0.1*Init_trajectory;
    //   ref_Pose(3) = Init_Pose(3) + 0*deg2rad*Init_trajectory;
    //   ref_Pose(4) = Init_Pose(4) + 0*deg2rad*Init_trajectory;
    //   ref_Pose(5) = Init_Pose(5) + 0*deg2rad*Init_trajectory;
    // }
    // if(int(cnt_time / step_time) == 3 ){
    //   ref_Pose(0) = Init_Pose(0) - 0.05*Init_trajectory;
    //   ref_Pose(1) = Init_Pose(1) + 0.0*Init_trajectory;
    //   ref_Pose(2) = Init_Pose(2) + 0.0*Init_trajectory;
    //   ref_Pose(3) = Init_Pose(3) + 360*deg2rad*Init_trajectory;
    //   ref_Pose(4) = Init_Pose(4) + 0*deg2rad*Init_trajectory;
    //   ref_Pose(5) = Init_Pose(5) + 0*deg2rad*Init_trajectory;
    // }
    // if(int(cnt_time / step_time) == 4 ){
    //   ref_Pose(0) = Init_Pose(0) - 0.1*Init_trajectory;
    //   ref_Pose(1) = Init_Pose(1) + 0.0*Init_trajectory;
    //   ref_Pose(2) = Init_Pose(2) + 0.0*Init_trajectory;
    //   ref_Pose(3) = Init_Pose(3) + 0*deg2rad*Init_trajectory;
    //   ref_Pose(4) = Init_Pose(4) + 45*deg2rad*Init_trajectory;
    //   ref_Pose(5) = Init_Pose(5) + 0*deg2rad*Init_trajectory;
    // }
    // if(int(cnt_time / step_time) == 5 ){
    //   ref_Pose(0) = Init_Pose(0) - 0.1*Init_trajectory;
    //   ref_Pose(1) = Init_Pose(1) + 0.0*Init_trajectory;
    //   ref_Pose(2) = Init_Pose(2) + 0.0*Init_trajectory;
    //   ref_Pose(3) = Init_Pose(3) + 0*deg2rad*Init_trajectory;
    //   ref_Pose(4) = Init_Pose(4) + 0*deg2rad*Init_trajectory;
    //   ref_Pose(5) = Init_Pose(5) + 360*deg2rad*Init_trajectory;
    // }
  }

  // *** 참조 CoM과 E.E의 방위를 넣어서 사용할 때 *** //
  
  A0 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
  A1 << cos(Arm.Q[0]), 0, sin(Arm.Q[0]), 0,
        sin(Arm.Q[0]), 0, -cos(Arm.Q[0]), 0,
        0, 1, 0, L1,
        0, 0, 0, 1;
  A2 << cos(Arm.Q[1]), -sin(Arm.Q[1]), 0, L2*cos(Arm.Q[1]),
        sin(Arm.Q[1]), cos(Arm.Q[1]), 0, L2*sin(Arm.Q[1]),
        0, 0, 1, 0, 
        0, 0, 0, 1;
  A3 << cos(Arm.Q[2]), -sin(Arm.Q[2]), 0, L3*cos(Arm.Q[2]), 
        sin(Arm.Q[2]), cos(Arm.Q[2]), 0, L3*sin(Arm.Q[2]), 
        0, 0, 1, 0,
        0, 0, 0, 1;
  A4 << -sin(Arm.Q[3]), 0, cos(Arm.Q[3]), 0,
        cos(Arm.Q[3]), 0, sin(Arm.Q[3]), 0,
        0, 1, 0, 0,
        0, 0, 0, 1;
  A5 << -sin(Arm.Q[4]), 0, cos(Arm.Q[4]), 0,
        cos(Arm.Q[4]), 0, sin(Arm.Q[4]), 0,
        0, 1, 0, L5,
        0, 0, 0, 1;
  A6 << -sin(Arm.Q[5]), -cos(Arm.Q[5]), 0, -L6*sin(Arm.Q[5]),
        cos(Arm.Q[5]), -sin(Arm.Q[5]), 0, L6*cos(Arm.Q[5]),
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
  T34 = T04-T03;
  T45 = T05-T04;
  T56 = T06-T05;

  

  // ref Ori넣어줄 때
  R_ref << cos(ref_Pose(5))*cos(ref_Pose(4)), -sin(ref_Pose(5))*cos(ref_Pose(3))+cos(ref_Pose(5))*sin(ref_Pose(4))*sin(ref_Pose(3)), sin(ref_Pose(5))*sin(ref_Pose(3))+cos(ref_Pose(5))*sin(ref_Pose(4))*cos(ref_Pose(3)),
           sin(ref_Pose(5))*cos(ref_Pose(4)), cos(ref_Pose(5))*cos(ref_Pose(3))+sin(ref_Pose(5))*sin(ref_Pose(4))*sin(ref_Pose(3)), -cos(ref_Pose(5))*sin(ref_Pose(3))+sin(ref_Pose(5))*sin(ref_Pose(4))*sin(ref_Pose(3)),
           -sin(ref_Pose(4)), cos(ref_Pose(4))*sin(ref_Pose(3)), cos(ref_Pose(4))*cos(ref_Pose(3));

  R1_ref = R_ref.block<3,1>(0,0); R2_ref = R_ref.block<3,1>(0,1); R3_ref = R_ref.block<3,1>(0,2);

  Ori_Err = R1_act.cross(R1_ref) + R2_act.cross(R2_ref) + R3_act.cross(R3_ref);
  m_Ori << Kw(0)*Ori_Err(0), Kw(1)*Ori_Err(1), Kw(2)*Ori_Err(2);
  f_Pos << Kp_s(0)*(ref_Pose(0)-Act_Pos(0)), Kp_s(1)*(ref_Pose(1)-Act_Pos(1)), Kp_s(2)*(ref_Pose(2)-Act_Pos(2)); 
  ref_Pos << T06(0,3), T06(1,3), T06(2,3);
  nx = T06(0,0); ny = T06(1,0); nz = T06(2,0); ox = T06(1,0); oy = T06(1,1); oz = T06(1,2);
  ax = T06(2,0); ay = T06(2,1); az = T06(2,2);

  // Get the Euler Angle - pi, theta, psi
  fi = atan2(ny,nx); // yaw
  theta = atan2(-nz,cos(fi)*nx + sin(fi)*ny); //pitch
  csi = atan2(sin(fi)*ax - cos(fi)*ay, -sin(fi)*ox + cos(fi)*oy); //roll
  
  a0 << T00(0,2), T00(1,2), T00(2,2);
  a1 << T01(0,2), T01(1,2), T01(2,2);
  a2 << T02(0,2), T02(1,2), T02(2,2);
  a3 << T03(0,2), T03(1,2), T03(2,2);
  a4 << T04(0,2), T04(1,2), T04(2,2);
  a5 << T05(0,2), T05(1,2), T05(2,2);

  InverseDynamics(*Arm.rbdl_model, Arm.Q, VectorXd::Zero(6), VectorXd::Zero(6), Arm.Tau, NULL);

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

  J << J1, J2, J3, J4, J5, J6;

  F << f_Pos(0), f_Pos(1), f_Pos(2), m_Ori(0), m_Ori(1), m_Ori(2);
  T = J.transpose()*F;
  ///////////////토크 입력////////////////
  for (int i = 0; i < 6; i++) {
    //joint[i].torque = Arm.Tau(i);
    joint[i].torque = T(i);
    old_joint[i].torque = joint[i].torque;
  }
}

void gazebo::arm_6dof_plugin::Jinu_Gravity_Compensation()  // 4
{
  /*
  Kp_v << 100, 100, 100, 30, 30, 30;
  Kd_v << 1, 1, 1, 0.1, 0.1, 0.1;
  step_time = 4; //주기설정 (초) 변수
  cnt_time = cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  double nx, ny, nz, ox, oy, oz, ax, ay, az;
  double Init_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));
  Math::Matrix3d R, R_Tmp;
  cnt++;
  if(cnt_time <= step_time)
  {
    ref_Arm_th[0] = 0*Init_trajectory*deg2rad;
    ref_Arm_th[1] = -60*Init_trajectory*deg2rad;
    ref_Arm_th[2] = 60*Init_trajectory*deg2rad;
    ref_Arm_th[3] = 30*Init_trajectory*deg2rad;
    ref_Arm_th[4] = 0*Init_trajectory*deg2rad;
    ref_Arm_th[5] = 0*Init_trajectory*deg2rad; 
  }

  A0 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
  A1 << cos(ref_Arm_th[0]), 0, sin(ref_Arm_th[0]), 0,
        sin(ref_Arm_th[0]), 0, -cos(ref_Arm_th[0]), 0,
        0, 1, 0, L1,
        0, 0, 0, 1;
  A2 << cos(ref_Arm_th[1]), -sin(ref_Arm_th[1]), 0, L2*cos(ref_Arm_th[1]),
        sin(ref_Arm_th[1]), cos(ref_Arm_th[1]), 0, L2*sin(ref_Arm_th[1]),
        0, 0, 1, 0, 
        0, 0, 0, 1;
  A3 << cos(ref_Arm_th[2]), -sin(ref_Arm_th[2]), 0, L3*cos(ref_Arm_th[2]), 
        sin(ref_Arm_th[2]), cos(ref_Arm_th[2]), 0, L3*sin(ref_Arm_th[2]), 
        0, 0, 1, 0,
        0, 0, 0, 1;
  A4 << -sin(ref_Arm_th[3]), 0, cos(ref_Arm_th[3]), 0,
        cos(ref_Arm_th[3]), 0, sin(ref_Arm_th[3]), 0,
        0, 1, 0, 0,
        0, 0, 0, 1;
  A5 << -sin(ref_Arm_th[4]), 0, cos(ref_Arm_th[4]), 0,
        cos(ref_Arm_th[4]), 0, sin(ref_Arm_th[4]), 0,
        0, 1, 0, L5,
        0, 0, 0, 1;
  A6 << -sin(ref_Arm_th[5]), -cos(ref_Arm_th[5]), 0, -L6*sin(ref_Arm_th[5]),
        cos(ref_Arm_th[5]), -sin(ref_Arm_th[5]), 0, L6*cos(ref_Arm_th[5]),
        0, 0, 1, 0, 
        0, 0, 0, 1;
  T00 = A0;
  T01 = T00*A1;
  T02 = T01*A2;
  T03 = T02*A3;
  T04 = T03*A4;
  T05 = T04*A5;
  T06 = T05*A6;

  ref_Pos << T06(0,3), T06(1,3), T06(2,3);
  nx = T06(0,0); ny = T06(1,0); nz = T06(2,0); ox = T06(1,0); oy = T06(1,1); oz = T06(1,2);
  ax = T06(2,0); ay = T06(2,1); az = T06(2,2);

  // Get the Euler Angle - pi, theta, psi

  fi = atan2(ny,nx); // yaw
  theta = atan2(-nz,cos(fi)*nx + sin(fi)*ny); //pitch
  csi = atan2(sin(fi)*ax - cos(fi)*ay, -sin(fi)*ox + cos(fi)*oy); //roll

  //InverseDynamics(*Arm.rbdl_model, Arm.Q, VectorXd::Zero(6), VectorXd::Zero(6), Arm.Tau, NULL);
  */

  /////////////// 토크 입력 ////////////////
  joint[0].torque = (0);

  joint[1].torque = (1.5698*sin(actual_joint_pos[1])*sin(actual_joint_pos[2]) - 1.5698*cos(actual_joint_pos[1])*cos(actual_joint_pos[2]) - 2.9226*cos(actual_joint_pos[1]) + 0.21769*cos(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*sin(actual_joint_pos[2]) + cos(actual_joint_pos[2])*sin(actual_joint_pos[1])) + 0.21769*sin(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*cos(actual_joint_pos[2]) - 1.0*sin(actual_joint_pos[1])*sin(actual_joint_pos[2])) + 0.056114*sin(actual_joint_pos[5] + 1.5708)*(cos(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*sin(actual_joint_pos[2]) + cos(actual_joint_pos[2])*sin(actual_joint_pos[1])) + sin(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*cos(actual_joint_pos[2]) - 1.0*sin(actual_joint_pos[1])*sin(actual_joint_pos[2]))) + 0.056114*cos(actual_joint_pos[4] + 1.5708)*cos(actual_joint_pos[5] + 1.5708)*(1.0*sin(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*sin(actual_joint_pos[2]) + cos(actual_joint_pos[2])*sin(actual_joint_pos[1])) - cos(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*cos(actual_joint_pos[2]) - 1.0*sin(actual_joint_pos[1])*sin(actual_joint_pos[2]))));

  joint[2].torque = (1.5698*sin(actual_joint_pos[1])*sin(actual_joint_pos[2]) - 1.5698*cos(actual_joint_pos[1])*cos(actual_joint_pos[2]) + 0.21769*cos(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*sin(actual_joint_pos[2]) + cos(actual_joint_pos[2])*sin(actual_joint_pos[1])) + 0.21769*sin(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*cos(actual_joint_pos[2]) - 1.0*sin(actual_joint_pos[1])*sin(actual_joint_pos[2])) + 0.056114*sin(actual_joint_pos[5] + 1.5708)*(cos(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*sin(actual_joint_pos[2]) + cos(actual_joint_pos[2])*sin(actual_joint_pos[1])) + sin(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*cos(actual_joint_pos[2]) - 1.0*sin(actual_joint_pos[1])*sin(actual_joint_pos[2]))) + 0.056114*cos(actual_joint_pos[4] + 1.5708)*cos(actual_joint_pos[5] + 1.5708)*(1.0*sin(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*sin(actual_joint_pos[2]) + cos(actual_joint_pos[2])*sin(actual_joint_pos[1])) - cos(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*cos(actual_joint_pos[2]) - 1.0*sin(actual_joint_pos[1])*sin(actual_joint_pos[2]))));

  joint[3].torque = (0.21769*cos(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*sin(actual_joint_pos[2]) + cos(actual_joint_pos[2])*sin(actual_joint_pos[1])) + 0.21769*sin(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*cos(actual_joint_pos[2]) - 1.0*sin(actual_joint_pos[1])*sin(actual_joint_pos[2])) + 0.056114*sin(actual_joint_pos[5] + 1.5708)*(cos(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*sin(actual_joint_pos[2]) + cos(actual_joint_pos[2])*sin(actual_joint_pos[1])) + sin(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*cos(actual_joint_pos[2]) - 1.0*sin(actual_joint_pos[1])*sin(actual_joint_pos[2]))) + 0.056114*cos(actual_joint_pos[4] + 1.5708)*cos(actual_joint_pos[5] + 1.5708)*(1.0*sin(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*sin(actual_joint_pos[2]) + cos(actual_joint_pos[2])*sin(actual_joint_pos[1])) - cos(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*cos(actual_joint_pos[2]) - 1.0*sin(actual_joint_pos[1])*sin(actual_joint_pos[2]))));

  joint[4].torque = (0.056114*cos(actual_joint_pos[5] + 1.5708)*sin(actual_joint_pos[4] + 1.5708)*(cos(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*sin(actual_joint_pos[2]) + cos(actual_joint_pos[2])*sin(actual_joint_pos[1])) + sin(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*cos(actual_joint_pos[2]) - 1.0*sin(actual_joint_pos[1])*sin(actual_joint_pos[2]))));

  joint[5].torque = (0.056114*cos(actual_joint_pos[5] + 1.5708)*(1.0*sin(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*sin(actual_joint_pos[2]) + cos(actual_joint_pos[2])*sin(actual_joint_pos[1])) - cos(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*cos(actual_joint_pos[2]) - 1.0*sin(actual_joint_pos[1])*sin(actual_joint_pos[2]))) + 0.056114*cos(actual_joint_pos[4] + 1.5708)*sin(actual_joint_pos[5] + 1.5708)*(cos(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*sin(actual_joint_pos[2]) + cos(actual_joint_pos[2])*sin(actual_joint_pos[1])) + sin(actual_joint_pos[3] - 1.5708)*(cos(actual_joint_pos[1])*cos(actual_joint_pos[2]) - 1.0*sin(actual_joint_pos[1])*sin(actual_joint_pos[2]))));

  for (int i = 0; i < 6; i++) {
    old_joint[i].torque = joint[i].torque;
  }
}

void gazebo::arm_6dof_plugin::Jinu_Input_EE_Pos_Ori()  // 5
{
  step_time = 6; //주기설정 (초) 변수
  cnt_time = cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  double Init_trajectory = 0.5*(1-cos(2*PI*(cnt_time/step_time)));
  double nx, ny, nz, ox, oy, oz, ax, ay, az;
  Eigen::Vector3d CoM1, CoM2, CoM3, CoM4, CoM5, CoM6, C1, C2, C3, C4, C5, C6, a0, a1, a2, a3, a4, a5, C1_P0, C2_P1 ,C3_P2, C4_P3, C5_P4, C6_P5, J1_CoM, J2_CoM, J3_CoM, J4_CoM, J5_CoM, J6_CoM, P6_P0, P6_P1, P6_P2, P6_P3, P6_P4, P6_P5;
  Eigen::MatrixXd J(6,6); Eigen::MatrixXd J_CoM(3,6); Eigen::VectorXd F(6), J1(6), J2(6), J3(6), J4(6), J5(6), J6(6), T(6);
  Eigen::MatrixXd B(3,3); MatrixXd J_temp(6,6); MatrixXd J_temp2(6,6); MatrixXd B_inv(3,3); MatrixXd Ja(6,6);// jinu added
  Eigen::Vector3d Ori_Err, m_Ori, f_Pos, Kw;

  Kp_s << 500, 500, 500;
  Kd_s << 1, 10, 10;
  Kw << 100, 100, 100;
  cnt++;
  if(cnt<2){
    Init_Pose << Act_Pos(0), Act_Pos(1), Act_Pos(2), Act_Ori(0), Act_Ori(1), Act_Ori(2);
    //cout<<"Init_Pose\n"<<Init_Pose<<endl;
  }
  /*------temp-----
  //if(cnt_time <= step_time*100)
  if(cnt_time < step_time*3 )
  { 
    ref_Pose(0) = Init_Pose(0);
    ref_Pose(1) = Init_Pose(1) - 0.2*sin(PI/2*(cnt_time/step_time));
    ref_Pose(2) = Init_Pose(2) + 0.1*sin(PI*(cnt_time/step_time));
    ref_Pose(3) = 0;
    ref_Pose(4) = 0;
    ref_Pose(5) = 0;
  }
  if((cnt_time / step_time) == 3 )
  {
    Init_Pose << Act_Pos(0), Act_Pos(1), Act_Pos(2), Act_Ori(0), Act_Ori(1), Act_Ori(2);
    //cout<<"Init_Pose\n"<<Init_Pose<<endl;
  }
  if(int(cnt_time / step_time) == 3 )
  {
    ref_Pose(0) = Init_Pose(0) - 0.1*Init_trajectory;
    ref_Pose(1) = Init_Pose(1);
    ref_Pose(2) = Init_Pose(2);
    ref_Pose(3) = Init_Pose(3);
    ref_Pose(4) = Init_Pose(4);
    ref_Pose(5) = Init_Pose(5);
  }
*/


    if(int(cnt_time / step_time) == 0 ){
      ref_Pose(0) = Init_Pose(0) - 0.1*Init_trajectory;
      ref_Pose(1) = Init_Pose(1) + 0.0*Init_trajectory;
      ref_Pose(2) = Init_Pose(2) + 0.0*Init_trajectory;
      ref_Pose(3) = Init_Pose(3) + 0*deg2rad*Init_trajectory;
      ref_Pose(4) = Init_Pose(4) + 0*deg2rad*Init_trajectory;
      ref_Pose(5) = Init_Pose(5) + 0*deg2rad*Init_trajectory;
    }
    if(int(cnt_time / step_time) == 1 ){
      ref_Pose(0) = Init_Pose(0) - 0.0*Init_trajectory;
      ref_Pose(1) = Init_Pose(1) + 0.1*Init_trajectory;
      ref_Pose(2) = Init_Pose(2) + 0.0*Init_trajectory;
      ref_Pose(3) = Init_Pose(3) + 0*deg2rad*Init_trajectory;
      ref_Pose(4) = Init_Pose(4) + 0*deg2rad*Init_trajectory;
      ref_Pose(5) = Init_Pose(5) + 0*deg2rad*Init_trajectory;
    }
    if(int(cnt_time / step_time) == 2 ){
      ref_Pose(0) = Init_Pose(0) - 0.0*Init_trajectory;
      ref_Pose(1) = Init_Pose(1) + 0.0*Init_trajectory;
      ref_Pose(2) = Init_Pose(2) + 0.1*Init_trajectory;
      ref_Pose(3) = Init_Pose(3) + 0*deg2rad*Init_trajectory;
      ref_Pose(4) = Init_Pose(4) + 0*deg2rad*Init_trajectory;
      ref_Pose(5) = Init_Pose(5) + 0*deg2rad*Init_trajectory;
    }
    if(int(cnt_time / step_time) == 3 ){
      ref_Pose(0) = Init_Pose(0) - 0.05*Init_trajectory;
      ref_Pose(1) = Init_Pose(1) + 0.0*Init_trajectory;
      ref_Pose(2) = Init_Pose(2) + 0.0*Init_trajectory;
      ref_Pose(3) = Init_Pose(3) + 360*deg2rad*Init_trajectory;
      ref_Pose(4) = Init_Pose(4) + 0*deg2rad*Init_trajectory;
      ref_Pose(5) = Init_Pose(5) + 0*deg2rad*Init_trajectory;
    }
    if(int(cnt_time / step_time) == 4 ){
      ref_Pose(0) = Init_Pose(0) - 0.1*Init_trajectory;
      ref_Pose(1) = Init_Pose(1) + 0.0*Init_trajectory;
      ref_Pose(2) = Init_Pose(2) + 0.0*Init_trajectory;
      ref_Pose(3) = Init_Pose(3) + 0*deg2rad*Init_trajectory;
      ref_Pose(4) = Init_Pose(4) + 45*deg2rad*Init_trajectory;
      ref_Pose(5) = Init_Pose(5) + 0*deg2rad*Init_trajectory;
    }
    if(int(cnt_time / step_time) == 5 ){
      ref_Pose(0) = Init_Pose(0) - 0.1*Init_trajectory;
      ref_Pose(1) = Init_Pose(1) + 0.0*Init_trajectory;
      ref_Pose(2) = Init_Pose(2) + 0.0*Init_trajectory;
      ref_Pose(3) = Init_Pose(3) + 0*deg2rad*Init_trajectory;
      ref_Pose(4) = Init_Pose(4) + 0*deg2rad*Init_trajectory;
      ref_Pose(5) = Init_Pose(5) + 360*deg2rad*Init_trajectory;
     }


  //-----Jinumanip new DH convention-----
  A0 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
  A1 << cos(Arm.Q[0]), 0, -sin(Arm.Q[0]), 0,
        sin(Arm.Q[0]), 0, cos(Arm.Q[0]), 0,
        0, -1, 0, L1,
        0, 0, 0, 1;
  A2 << cos(Arm.Q[1]), -sin(Arm.Q[1]), 0, L2*cos(Arm.Q[1]),
        sin(Arm.Q[1]), cos(Arm.Q[1]), 0, L2*sin(Arm.Q[1]),
        0, 0, 1, 0, 
        0, 0, 0, 1;
  A3 << cos(Arm.Q[2]), -sin(Arm.Q[2]), 0, L3*cos(Arm.Q[2]), 
        sin(Arm.Q[2]), cos(Arm.Q[2]), 0, L3*sin(Arm.Q[2]), 
        0, 0, 1, 0,
        0, 0, 0, 1;
  A4 << sin(Arm.Q[3]), 0, cos(Arm.Q[3]), 0,
        -cos(Arm.Q[3]), 0, sin(Arm.Q[3]), 0,
        0, -1, 0, 0,
        0, 0, 0, 1;
  A5 << -sin(Arm.Q[4]), 0, cos(Arm.Q[4]), 0,
        cos(Arm.Q[4]), 0, sin(Arm.Q[4]), 0,
        0, 1, 0, L5,
        0, 0, 0, 1;
  A6 << -sin(Arm.Q[5]), -cos(Arm.Q[5]), 0, -L6*sin(Arm.Q[5]),
        cos(Arm.Q[5]), -sin(Arm.Q[5]), 0, L6*cos(Arm.Q[5]),
        0, 0, 1, 0, 
        0, 0, 0, 1;
        
  /* //original DH convention   
  A0 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
  A1 << cos(Arm.Q[0]), 0, sin(Arm.Q[0]), 0,
        sin(Arm.Q[0]), 0, -cos(Arm.Q[0]), 0,
        0, 1, 0, L1,
        0, 0, 0, 1;
  A2 << cos(Arm.Q[1]), -sin(Arm.Q[1]), 0, L2*cos(Arm.Q[1]),
        sin(Arm.Q[1]), cos(Arm.Q[1]), 0, L2*sin(Arm.Q[1]),
        0, 0, 1, 0, 
        0, 0, 0, 1;
  A3 << cos(Arm.Q[2]), -sin(Arm.Q[2]), 0, L3*cos(Arm.Q[2]), 
        sin(Arm.Q[2]), cos(Arm.Q[2]), 0, L3*sin(Arm.Q[2]), 
        0, 0, 1, 0,
        0, 0, 0, 1;
  A4 << -sin(Arm.Q[3]), 0, cos(Arm.Q[3]), 0,
        cos(Arm.Q[3]), 0, sin(Arm.Q[3]), 0,
        0, 1, 0, 0,
        0, 0, 0, 1;
  A5 << -sin(Arm.Q[4]), 0, cos(Arm.Q[4]), 0,
        cos(Arm.Q[4]), 0, sin(Arm.Q[4]), 0,
        0, 1, 0, L5,
        0, 0, 0, 1;
  A6 << -sin(Arm.Q[5]), -cos(Arm.Q[5]), 0, -L6*sin(Arm.Q[5]),
        cos(Arm.Q[5]), -sin(Arm.Q[5]), 0, L6*cos(Arm.Q[5]),
        0, 0, 1, 0, 
        0, 0, 0, 1;
  */
        
  T00 = A0;
  T01 = T00*A1;
  T02 = T01*A2;
  T03 = T02*A3;
  T04 = T03*A4;
  T05 = T04*A5;
  T06 = T05*A6;

  T12 = T02-T01;
  T23 = T03-T02;
  T34 = T04-T03;
  T45 = T05-T04;
  T56 = T06-T05;

  // ref Ori넣어줄 때
  /*
  R_ref << cos(ref_Pose(5))*cos(ref_Pose(4)), -sin(ref_Pose(5))*cos(ref_Pose(3))+cos(ref_Pose(5))*sin(ref_Pose(4))*sin(ref_Pose(3)), sin(ref_Pose(5))*sin(ref_Pose(3))+cos(ref_Pose(5))*sin(ref_Pose(4))*cos(ref_Pose(3)),
           sin(ref_Pose(5))*cos(ref_Pose(4)), cos(ref_Pose(5))*cos(ref_Pose(3))+sin(ref_Pose(5))*sin(ref_Pose(4))*sin(ref_Pose(3)), -cos(ref_Pose(5))*sin(ref_Pose(3))+sin(ref_Pose(5))*sin(ref_Pose(4))*sin(ref_Pose(3)),
           -sin(ref_Pose(4)), cos(ref_Pose(4))*sin(ref_Pose(3)), cos(ref_Pose(4))*cos(ref_Pose(3));
  
  R1_ref = R_ref.block<3,1>(0,0); R2_ref = R_ref.block<3,1>(0,1); R3_ref = R_ref.block<3,1>(0,2);
  

  Ori_Err = R1_act.cross(R1_ref) + R2_act.cross(R2_ref) + R3_act.cross(R3_ref);
  m_Ori << Kw(0)*Ori_Err(0), Kw(1)*Ori_Err(1), Kw(2)*Ori_Err(2);
  */
  f_Pos << Kp_s(0)*(ref_Pose(0)-Act_Pos(0)) - Kd_s(0)*(Act_Pos_Vel(0)), Kp_s(1)*(ref_Pose(1)-Act_Pos(1))-Kd_s(1)*(Act_Pos_Vel(1)), Kp_s(2)*(ref_Pose(2)-Act_Pos(2))-Kd_s(2)*(Act_Pos_Vel(2));
  
  ref_Pos << T06(0,3), T06(1,3), T06(2,3);
  nx = T06(0,0); ox = T06(0,1);  ax = T06(0,2);
  ny = T06(1,0); oy = T06(1,1);  ay = T06(1,2);
  nz = T06(2,0); oz = T06(2,1);  az = T06(2,2);


  // Get the Euler Angle 
  //-----new fi theta csi-----
  fi = atan2(ny,nx); // yaw
  theta = atan2(-nz,cos(fi)*nx + sin(fi)*ny); //pitch
  csi = atan2(sin(fi)*ax - cos(fi)*ay, -sin(fi)*ox + cos(fi)*oy); //roll

  m_Ori << Kw(0)*(ref_Pose(3)-csi), Kw(1)*(ref_Pose(4)-theta), Kw(2)*(ref_Pose(5)-fi); 
  ref_Pos << T06(0,3), T06(1,3), T06(2,3);
 
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

  J << J1, J2, J3, J4, J5, J6;
  B << cos(csi)*cos(theta), -sin(csi), 0,
       sin(csi)*cos(theta), cos(csi), 0,
       -sin(theta), 0, 1;  

  B_inv = B.inverse();
  J_temp.topLeftCorner(3,3) = MatrixXd::Identity(3,3);
  J_temp.topRightCorner(3,3) = MatrixXd::Zero(3,3);
  J_temp.bottomLeftCorner(3,3) = MatrixXd::Zero(3,3);
  J_temp.bottomRightCorner(3,3) = B.inverse();

  F << f_Pos(0), f_Pos(1), f_Pos(2), m_Ori(0), m_Ori(1), m_Ori(2);
  
  Ja=J_temp*J;
  T = Ja.transpose()*F;
  //std::cout << "J" << J << "Ja"<< Ja;
  
  for (int i = 0; i < 6; i++) {
    joint[i].torque = T(i);
    old_joint[i].torque = joint[i].torque; 
  }
}

void gazebo::arm_6dof_plugin::Jinu_Input_EE_Pos_Ori_Rot()  // 6
{
  step_time = 6; //주기설정 (초) 변수
  cnt_time = cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  double Init_trajectory = 0.5*(1-cos(2*PI*(cnt_time/step_time)));
  double nx, ny, nz, ox, oy, oz, ax, ay, az;
  Eigen::Vector3d CoM1, CoM2, CoM3, CoM4, CoM5, CoM6, C1, C2, C3, C4, C5, C6, a0, a1, a2, a3, a4, a5, C1_P0, C2_P1 ,C3_P2, C4_P3, C5_P4, C6_P5, J1_CoM, J2_CoM, J3_CoM, J4_CoM, J5_CoM, J6_CoM, P6_P0, P6_P1, P6_P2, P6_P3, P6_P4, P6_P5;
  Eigen::MatrixXd J(6,6); Eigen::MatrixXd J_CoM(3,6); Eigen::VectorXd F(6), J1(6), J2(6), J3(6), J4(6), J5(6), J6(6), T(6);
  Eigen::MatrixXd B(3,3); MatrixXd J_temp(6,6); MatrixXd J_temp2(6,6); MatrixXd B_inv(3,3); MatrixXd Ja(6,6); Eigen::Quaterniond q;// jinu added
  Eigen::Vector3d Ori_Err, m_Ori, f_Pos, Kw;

  Kp_s << 500, 500, 500;
  Kd_s << 1, 10, 10;
  Kw << 10, 10, 10;
  cnt++;
  if(cnt<2){
    Init_Pose << Act_Pos(0), Act_Pos(1), Act_Pos(2), Act_Ori(0), Act_Ori(1), Act_Ori(2);
    //cout<<"Init_Pose\n"<<Init_Pose<<endl;
  }
if(cnt_time <= step_time*100)
  { 

    ref_Pose(0) = Init_Pose(0) - 0.0*Init_trajectory;
    ref_Pose(1) = Init_Pose(1) - 0.2*sin(PI/2*(cnt_time/step_time));
    ref_Pose(2) = Init_Pose(2) + 0.1*sin(PI*(cnt_time/step_time));
    float qw = 0;
    float qx = 1;
    float qy = 0;
    float qz = 0;
    q.w() = qw;
    q.x() = qx;
    q.y() = qy;
    q.z() = qz;

  }

/*
    if(int(cnt_time / step_time) == 0 ){
      ref_Pose(0) = Init_Pose(0) - 0.1*Init_trajectory;
      ref_Pose(1) = Init_Pose(1) + 0.0*Init_trajectory;
      ref_Pose(2) = Init_Pose(2) + 0.0*Init_trajectory;
      ref_Pose(3) = Init_Pose(3) + 0*deg2rad*Init_trajectory;
      ref_Pose(4) = Init_Pose(4) + 0*deg2rad*Init_trajectory;
      ref_Pose(5) = Init_Pose(5) + 0*deg2rad*Init_trajectory;
    }
    if(int(cnt_time / step_time) == 1 ){
      ref_Pose(0) = Init_Pose(0) - 0.0*Init_trajectory;
      ref_Pose(1) = Init_Pose(1) + 0.1*Init_trajectory;
      ref_Pose(2) = Init_Pose(2) + 0.0*Init_trajectory;
      ref_Pose(3) = Init_Pose(3) + 0*deg2rad*Init_trajectory;
      ref_Pose(4) = Init_Pose(4) + 0*deg2rad*Init_trajectory;
      ref_Pose(5) = Init_Pose(5) + 0*deg2rad*Init_trajectory;
    }
    if(int(cnt_time / step_time) == 2 ){
      ref_Pose(0) = Init_Pose(0) - 0.0*Init_trajectory;
      ref_Pose(1) = Init_Pose(1) + 0.0*Init_trajectory;
      ref_Pose(2) = Init_Pose(2) + 0.1*Init_trajectory;
      ref_Pose(3) = Init_Pose(3) + 0*deg2rad*Init_trajectory;
      ref_Pose(4) = Init_Pose(4) + 0*deg2rad*Init_trajectory;
      ref_Pose(5) = Init_Pose(5) + 0*deg2rad*Init_trajectory;
    }
    if(int(cnt_time / step_time) == 3 ){
      ref_Pose(0) = Init_Pose(0) - 0.05*Init_trajectory;
      ref_Pose(1) = Init_Pose(1) + 0.0*Init_trajectory;
      ref_Pose(2) = Init_Pose(2) + 0.0*Init_trajectory;
      ref_Pose(3) = Init_Pose(3) + 360*deg2rad*Init_trajectory;
      ref_Pose(4) = Init_Pose(4) + 0*deg2rad*Init_trajectory;
      ref_Pose(5) = Init_Pose(5) + 0*deg2rad*Init_trajectory;
    }
    if(int(cnt_time / step_time) == 4 ){
      ref_Pose(0) = Init_Pose(0) - 0.1*Init_trajectory;
      ref_Pose(1) = Init_Pose(1) + 0.0*Init_trajectory;
      ref_Pose(2) = Init_Pose(2) + 0.0*Init_trajectory;
      ref_Pose(3) = Init_Pose(3) + 0*deg2rad*Init_trajectory;
      ref_Pose(4) = Init_Pose(4) + 45*deg2rad*Init_trajectory;
      ref_Pose(5) = Init_Pose(5) + 0*deg2rad*Init_trajectory;
    }
    if(int(cnt_time / step_time) == 5 ){
      ref_Pose(0) = Init_Pose(0) - 0.2*Init_trajectory;
      ref_Pose(1) = Init_Pose(1) + 0.0*Init_trajectory;
      ref_Pose(2) = Init_Pose(2) + 0.0*Init_trajectory;
      ref_Pose(3) = Init_Pose(3) + 0*deg2rad*Init_trajectory;
      ref_Pose(4) = Init_Pose(4) + 0*deg2rad*Init_trajectory;
      ref_Pose(5) = Init_Pose(5) + 360*deg2rad*Init_trajectory;
     }
*/


  //-----Jinumanip new DH convention-----
  A0 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
  A1 << cos(Arm.Q[0]), 0, -sin(Arm.Q[0]), 0,
        sin(Arm.Q[0]), 0, cos(Arm.Q[0]), 0,
        0, -1, 0, L1,
        0, 0, 0, 1;
  A2 << cos(Arm.Q[1]), -sin(Arm.Q[1]), 0, L2*cos(Arm.Q[1]),
        sin(Arm.Q[1]), cos(Arm.Q[1]), 0, L2*sin(Arm.Q[1]),
        0, 0, 1, 0, 
        0, 0, 0, 1;
  A3 << cos(Arm.Q[2]), -sin(Arm.Q[2]), 0, L3*cos(Arm.Q[2]), 
        sin(Arm.Q[2]), cos(Arm.Q[2]), 0, L3*sin(Arm.Q[2]), 
        0, 0, 1, 0,
        0, 0, 0, 1;
  A4 << sin(Arm.Q[3]), 0, cos(Arm.Q[3]), 0,
        -cos(Arm.Q[3]), 0, sin(Arm.Q[3]), 0,
        0, -1, 0, 0,
        0, 0, 0, 1;
  A5 << -sin(Arm.Q[4]), 0, cos(Arm.Q[4]), 0,
        cos(Arm.Q[4]), 0, sin(Arm.Q[4]), 0,
        0, 1, 0, L5,
        0, 0, 0, 1;
  A6 << -sin(Arm.Q[5]), -cos(Arm.Q[5]), 0, -L6*sin(Arm.Q[5]),
        cos(Arm.Q[5]), -sin(Arm.Q[5]), 0, L6*cos(Arm.Q[5]),
        0, 0, 1, 0, 
        0, 0, 0, 1;
        
  /* //original DH convention   
  A0 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
  A1 << cos(Arm.Q[0]), 0, sin(Arm.Q[0]), 0,
        sin(Arm.Q[0]), 0, -cos(Arm.Q[0]), 0,
        0, 1, 0, L1,
        0, 0, 0, 1;
  A2 << cos(Arm.Q[1]), -sin(Arm.Q[1]), 0, L2*cos(Arm.Q[1]),
        sin(Arm.Q[1]), cos(Arm.Q[1]), 0, L2*sin(Arm.Q[1]),
        0, 0, 1, 0, 
        0, 0, 0, 1;
  A3 << cos(Arm.Q[2]), -sin(Arm.Q[2]), 0, L3*cos(Arm.Q[2]), 
        sin(Arm.Q[2]), cos(Arm.Q[2]), 0, L3*sin(Arm.Q[2]), 
        0, 0, 1, 0,
        0, 0, 0, 1;
  A4 << -sin(Arm.Q[3]), 0, cos(Arm.Q[3]), 0,
        cos(Arm.Q[3]), 0, sin(Arm.Q[3]), 0,
        0, 1, 0, 0,
        0, 0, 0, 1;
  A5 << -sin(Arm.Q[4]), 0, cos(Arm.Q[4]), 0,
        cos(Arm.Q[4]), 0, sin(Arm.Q[4]), 0,
        0, 1, 0, L5,
        0, 0, 0, 1;
  A6 << -sin(Arm.Q[5]), -cos(Arm.Q[5]), 0, -L6*sin(Arm.Q[5]),
        cos(Arm.Q[5]), -sin(Arm.Q[5]), 0, L6*cos(Arm.Q[5]),
        0, 0, 1, 0, 
        0, 0, 0, 1;
  */
        
  T00 = A0;
  T01 = T00*A1;
  T02 = T01*A2;
  T03 = T02*A3;
  T04 = T03*A4;
  T05 = T04*A5;
  T06 = T05*A6;

  T12 = T02-T01;
  T23 = T03-T02;
  T34 = T04-T03;
  T45 = T05-T04;
  T56 = T06-T05;

/*
  R_ref << cos(ref_Pose(5))*cos(ref_Pose(4)), -sin(ref_Pose(5))*cos(ref_Pose(3))+cos(ref_Pose(5))*sin(ref_Pose(4))*sin(ref_Pose(3)), sin(ref_Pose(5))*sin(ref_Pose(3))+cos(ref_Pose(5))*sin(ref_Pose(4))*cos(ref_Pose(3)),
           sin(ref_Pose(5))*cos(ref_Pose(4)), cos(ref_Pose(5))*cos(ref_Pose(3))+sin(ref_Pose(5))*sin(ref_Pose(4))*sin(ref_Pose(3)), -cos(ref_Pose(5))*sin(ref_Pose(3))+sin(ref_Pose(5))*sin(ref_Pose(4))*sin(ref_Pose(3)),
           -sin(ref_Pose(4)), cos(ref_Pose(4))*sin(ref_Pose(3)), cos(ref_Pose(4))*cos(ref_Pose(3));
*/

  Eigen::Matrix3d R_ref = q.normalized().toRotationMatrix();
  //cout<<R<<endl;

  R1_ref = R_ref.block<3,1>(0,0); R2_ref = R_ref.block<3,1>(0,1); R3_ref = R_ref.block<3,1>(0,2);
  

  Ori_Err = R1_act.cross(R1_ref) + R2_act.cross(R2_ref) + R3_act.cross(R3_ref);
  m_Ori << Kw(0)*Ori_Err(0), Kw(1)*Ori_Err(1), Kw(2)*Ori_Err(2);
  

  f_Pos << Kp_s(0)*(ref_Pose(0)-Act_Pos(0)) - Kd_s(0)*(Act_Pos_Vel(0)), Kp_s(1)*(ref_Pose(1)-Act_Pos(1))-Kd_s(1)*(Act_Pos_Vel(1)), Kp_s(2)*(ref_Pose(2)-Act_Pos(2))-Kd_s(2)*(Act_Pos_Vel(2));
  
  ref_Pos << T06(0,3), T06(1,3), T06(2,3);
  nx = T06(0,0); ox = T06(0,1);  ax = T06(0,2);
  ny = T06(1,0); oy = T06(1,1);  ay = T06(1,2);
  nz = T06(2,0); oz = T06(2,1);  az = T06(2,2);

  /*
  // Get the Euler Angle 
  //-----new fi theta csi-----
  fi = atan2(ny,nx); // yaw
  theta = atan2(-nz,cos(fi)*nx + sin(fi)*ny); //pitch
  csi = atan2(sin(fi)*ax - cos(fi)*ay, -sin(fi)*ox + cos(fi)*oy); //roll

  m_Ori << Kw(0)*(ref_Pose(3)-csi), Kw(1)*(ref_Pose(4)-theta), Kw(2)*(ref_Pose(5)-fi); 
  */
  ref_Pos << T06(0,3), T06(1,3), T06(2,3);
 
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

  J << J1, J2, J3, J4, J5, J6;
  // B << cos(csi)*cos(theta), -sin(csi), 0,
  //      sin(csi)*cos(theta), cos(csi), 0,
  //      -sin(theta), 0, 1;  

  // B_inv = B.inverse();
  // J_temp.topLeftCorner(3,3) = MatrixXd::Identity(3,3);
  // J_temp.topRightCorner(3,3) = MatrixXd::Zero(3,3);
  // J_temp.bottomLeftCorner(3,3) = MatrixXd::Zero(3,3);
  // J_temp.bottomRightCorner(3,3) = B.inverse();

  F << f_Pos(0), f_Pos(1), f_Pos(2), m_Ori(0), m_Ori(1), m_Ori(2);
  
  // Ja=J_temp*J;
  T = J.transpose()*F;
  //std::cout << "J" << J << "Ja"<< Ja;
  
  for (int i = 0; i < 6; i++) {
    joint[i].torque = T(i);
    old_joint[i].torque = joint[i].torque; 
  }
}

void gazebo::arm_6dof_plugin::Jinu_Input_EE_Pos_Ori_Rot_Quat()  // 7
{
  step_time = 6; //주기설정 (초) 변수
  cnt_time = cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  double Init_trajectory = 0.5*(1-cos(2*PI*(cnt_time/step_time)));
  double nx, ny, nz, ox, oy, oz, ax, ay, az;
  Eigen::Vector3d CoM1, CoM2, CoM3, CoM4, CoM5, CoM6, C1, C2, C3, C4, C5, C6, a0, a1, a2, a3, a4, a5, C1_P0, C2_P1 ,C3_P2, C4_P3, C5_P4, C6_P5, J1_CoM, J2_CoM, J3_CoM, J4_CoM, J5_CoM, J6_CoM, P6_P0, P6_P1, P6_P2, P6_P3, P6_P4, P6_P5;
  Eigen::MatrixXd J(6,6); Eigen::MatrixXd J_CoM(3,6); Eigen::VectorXd F(6), J1(6), J2(6), J3(6), J4(6), J5(6), J6(6), T(6);
  Eigen::MatrixXd B(3,3); MatrixXd J_temp(6,6); MatrixXd J_temp2(6,6); MatrixXd B_inv(3,3); MatrixXd Ja(6,6); Eigen::Quaterniond q;// jinu added
  Eigen::Vector3d Ori_Err, m_Ori, f_Pos, Kw;

  Kp_s << 500, 500, 500;
  Kd_s << 1, 10, 10;
  Kw << 10, 10, 10;
  cnt++;
  if(cnt<2){
    Init_Pose << Act_Pos(0), Act_Pos(1), Act_Pos(2), Act_Ori(0), Act_Ori(1), Act_Ori(2);
    //cout<<"Init_Pose\n"<<Init_Pose<<endl;
  }
if(cnt_time <= step_time*100)
  { 

    ref_Pose(0) = Init_Pose(0) - 0.0*Init_trajectory;
    ref_Pose(1) = Init_Pose(1) - 0.2*sin(PI/2*(cnt_time/step_time));
    ref_Pose(2) = Init_Pose(2) + 0.1*sin(PI*(cnt_time/step_time));
    q.w() = qw; q.x() = qx; q.y() = qy; q.z() = qz;
    
  }

/*
    if(int(cnt_time / step_time) == 0 ){
      ref_Pose(0) = Init_Pose(0) - 0.1*Init_trajectory;
      ref_Pose(1) = Init_Pose(1) + 0.0*Init_trajectory;
      ref_Pose(2) = Init_Pose(2) + 0.0*Init_trajectory;
      ref_Pose(3) = Init_Pose(3) + 0*deg2rad*Init_trajectory;
      ref_Pose(4) = Init_Pose(4) + 0*deg2rad*Init_trajectory;
      ref_Pose(5) = Init_Pose(5) + 0*deg2rad*Init_trajectory;
    }
    if(int(cnt_time / step_time) == 1 ){
      ref_Pose(0) = Init_Pose(0) - 0.0*Init_trajectory;
      ref_Pose(1) = Init_Pose(1) + 0.1*Init_trajectory;
      ref_Pose(2) = Init_Pose(2) + 0.0*Init_trajectory;
      ref_Pose(3) = Init_Pose(3) + 0*deg2rad*Init_trajectory;
      ref_Pose(4) = Init_Pose(4) + 0*deg2rad*Init_trajectory;
      ref_Pose(5) = Init_Pose(5) + 0*deg2rad*Init_trajectory;
    }
    if(int(cnt_time / step_time) == 2 ){
      ref_Pose(0) = Init_Pose(0) - 0.0*Init_trajectory;
      ref_Pose(1) = Init_Pose(1) + 0.0*Init_trajectory;
      ref_Pose(2) = Init_Pose(2) + 0.1*Init_trajectory;
      ref_Pose(3) = Init_Pose(3) + 0*deg2rad*Init_trajectory;
      ref_Pose(4) = Init_Pose(4) + 0*deg2rad*Init_trajectory;
      ref_Pose(5) = Init_Pose(5) + 0*deg2rad*Init_trajectory;
    }
    if(int(cnt_time / step_time) == 3 ){
      ref_Pose(0) = Init_Pose(0) - 0.05*Init_trajectory;
      ref_Pose(1) = Init_Pose(1) + 0.0*Init_trajectory;
      ref_Pose(2) = Init_Pose(2) + 0.0*Init_trajectory;
      ref_Pose(3) = Init_Pose(3) + 360*deg2rad*Init_trajectory;
      ref_Pose(4) = Init_Pose(4) + 0*deg2rad*Init_trajectory;
      ref_Pose(5) = Init_Pose(5) + 0*deg2rad*Init_trajectory;
    }
    if(int(cnt_time / step_time) == 4 ){
      ref_Pose(0) = Init_Pose(0) - 0.1*Init_trajectory;
      ref_Pose(1) = Init_Pose(1) + 0.0*Init_trajectory;
      ref_Pose(2) = Init_Pose(2) + 0.0*Init_trajectory;
      ref_Pose(3) = Init_Pose(3) + 0*deg2rad*Init_trajectory;
      ref_Pose(4) = Init_Pose(4) + 45*deg2rad*Init_trajectory;
      ref_Pose(5) = Init_Pose(5) + 0*deg2rad*Init_trajectory;
    }
    if(int(cnt_time / step_time) == 5 ){
      ref_Pose(0) = Init_Pose(0) - 0.2*Init_trajectory;
      ref_Pose(1) = Init_Pose(1) + 0.0*Init_trajectory;
      ref_Pose(2) = Init_Pose(2) + 0.0*Init_trajectory;
      ref_Pose(3) = Init_Pose(3) + 0*deg2rad*Init_trajectory;
      ref_Pose(4) = Init_Pose(4) + 0*deg2rad*Init_trajectory;
      ref_Pose(5) = Init_Pose(5) + 360*deg2rad*Init_trajectory;
     }
*/


  //-----Jinumanip new DH convention-----
  A0 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
  A1 << cos(Arm.Q[0]), 0, -sin(Arm.Q[0]), 0,
        sin(Arm.Q[0]), 0, cos(Arm.Q[0]), 0,
        0, -1, 0, L1,
        0, 0, 0, 1;
  A2 << cos(Arm.Q[1]), -sin(Arm.Q[1]), 0, L2*cos(Arm.Q[1]),
        sin(Arm.Q[1]), cos(Arm.Q[1]), 0, L2*sin(Arm.Q[1]),
        0, 0, 1, 0, 
        0, 0, 0, 1;
  A3 << cos(Arm.Q[2]), -sin(Arm.Q[2]), 0, L3*cos(Arm.Q[2]), 
        sin(Arm.Q[2]), cos(Arm.Q[2]), 0, L3*sin(Arm.Q[2]), 
        0, 0, 1, 0,
        0, 0, 0, 1;
  A4 << sin(Arm.Q[3]), 0, cos(Arm.Q[3]), 0,
        -cos(Arm.Q[3]), 0, sin(Arm.Q[3]), 0,
        0, -1, 0, 0,
        0, 0, 0, 1;
  A5 << -sin(Arm.Q[4]), 0, cos(Arm.Q[4]), 0,
        cos(Arm.Q[4]), 0, sin(Arm.Q[4]), 0,
        0, 1, 0, L5,
        0, 0, 0, 1;
  A6 << -sin(Arm.Q[5]), -cos(Arm.Q[5]), 0, -L6*sin(Arm.Q[5]),
        cos(Arm.Q[5]), -sin(Arm.Q[5]), 0, L6*cos(Arm.Q[5]),
        0, 0, 1, 0, 
        0, 0, 0, 1;
        
  /* //original DH convention   
  A0 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
  A1 << cos(Arm.Q[0]), 0, sin(Arm.Q[0]), 0,
        sin(Arm.Q[0]), 0, -cos(Arm.Q[0]), 0,
        0, 1, 0, L1,
        0, 0, 0, 1;
  A2 << cos(Arm.Q[1]), -sin(Arm.Q[1]), 0, L2*cos(Arm.Q[1]),
        sin(Arm.Q[1]), cos(Arm.Q[1]), 0, L2*sin(Arm.Q[1]),
        0, 0, 1, 0, 
        0, 0, 0, 1;
  A3 << cos(Arm.Q[2]), -sin(Arm.Q[2]), 0, L3*cos(Arm.Q[2]), 
        sin(Arm.Q[2]), cos(Arm.Q[2]), 0, L3*sin(Arm.Q[2]), 
        0, 0, 1, 0,
        0, 0, 0, 1;
  A4 << -sin(Arm.Q[3]), 0, cos(Arm.Q[3]), 0,
        cos(Arm.Q[3]), 0, sin(Arm.Q[3]), 0,
        0, 1, 0, 0,
        0, 0, 0, 1;
  A5 << -sin(Arm.Q[4]), 0, cos(Arm.Q[4]), 0,
        cos(Arm.Q[4]), 0, sin(Arm.Q[4]), 0,
        0, 1, 0, L5,
        0, 0, 0, 1;
  A6 << -sin(Arm.Q[5]), -cos(Arm.Q[5]), 0, -L6*sin(Arm.Q[5]),
        cos(Arm.Q[5]), -sin(Arm.Q[5]), 0, L6*cos(Arm.Q[5]),
        0, 0, 1, 0, 
        0, 0, 0, 1;
  */
        
  T00 = A0;
  T01 = T00*A1;
  T02 = T01*A2;
  T03 = T02*A3;
  T04 = T03*A4;
  T05 = T04*A5;
  T06 = T05*A6;

  T12 = T02-T01;
  T23 = T03-T02;
  T34 = T04-T03;
  T45 = T05-T04;
  T56 = T06-T05;

/*
  R_ref << cos(ref_Pose(5))*cos(ref_Pose(4)), -sin(ref_Pose(5))*cos(ref_Pose(3))+cos(ref_Pose(5))*sin(ref_Pose(4))*sin(ref_Pose(3)), sin(ref_Pose(5))*sin(ref_Pose(3))+cos(ref_Pose(5))*sin(ref_Pose(4))*cos(ref_Pose(3)),
           sin(ref_Pose(5))*cos(ref_Pose(4)), cos(ref_Pose(5))*cos(ref_Pose(3))+sin(ref_Pose(5))*sin(ref_Pose(4))*sin(ref_Pose(3)), -cos(ref_Pose(5))*sin(ref_Pose(3))+sin(ref_Pose(5))*sin(ref_Pose(4))*sin(ref_Pose(3)),
           -sin(ref_Pose(4)), cos(ref_Pose(4))*sin(ref_Pose(3)), cos(ref_Pose(4))*cos(ref_Pose(3));
*/

  Eigen::Matrix3d R_ref = q.normalized().toRotationMatrix();
  //cout<<R<<endl;

  R1_ref = R_ref.block<3,1>(0,0); R2_ref = R_ref.block<3,1>(0,1); R3_ref = R_ref.block<3,1>(0,2);
  

  Ori_Err = R1_act.cross(R1_ref) + R2_act.cross(R2_ref) + R3_act.cross(R3_ref);
  m_Ori << Kw(0)*Ori_Err(0), Kw(1)*Ori_Err(1), Kw(2)*Ori_Err(2);
  

  f_Pos << Kp_s(0)*(ref_Pose(0)-Act_Pos(0)) - Kd_s(0)*(Act_Pos_Vel(0)), Kp_s(1)*(ref_Pose(1)-Act_Pos(1))-Kd_s(1)*(Act_Pos_Vel(1)), Kp_s(2)*(ref_Pose(2)-Act_Pos(2))-Kd_s(2)*(Act_Pos_Vel(2));
  
  ref_Pos << T06(0,3), T06(1,3), T06(2,3);
  nx = T06(0,0); ox = T06(0,1);  ax = T06(0,2);
  ny = T06(1,0); oy = T06(1,1);  ay = T06(1,2);
  nz = T06(2,0); oz = T06(2,1);  az = T06(2,2);

  /*
  // Get the Euler Angle 
  //-----new fi theta csi-----
  fi = atan2(ny,nx); // yaw
  theta = atan2(-nz,cos(fi)*nx + sin(fi)*ny); //pitch
  csi = atan2(sin(fi)*ax - cos(fi)*ay, -sin(fi)*ox + cos(fi)*oy); //roll

  m_Ori << Kw(0)*(ref_Pose(3)-csi), Kw(1)*(ref_Pose(4)-theta), Kw(2)*(ref_Pose(5)-fi); 
  */
  ref_Pos << T06(0,3), T06(1,3), T06(2,3);
 
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

  J << J1, J2, J3, J4, J5, J6;
  // B << cos(csi)*cos(theta), -sin(csi), 0,
  //      sin(csi)*cos(theta), cos(csi), 0,
  //      -sin(theta), 0, 1;  

  // B_inv = B.inverse();
  // J_temp.topLeftCorner(3,3) = MatrixXd::Identity(3,3);
  // J_temp.topRightCorner(3,3) = MatrixXd::Zero(3,3);
  // J_temp.bottomLeftCorner(3,3) = MatrixXd::Zero(3,3);
  // J_temp.bottomRightCorner(3,3) = B.inverse();

  F << f_Pos(0), f_Pos(1), f_Pos(2), m_Ori(0), m_Ori(1), m_Ori(2);
  
  // Ja=J_temp*J;
  T = J.transpose()*F;
  //std::cout << "J" << J << "Ja"<< Ja;
  
  for (int i = 0; i < 6; i++) {
    joint[i].torque = T(i);
    old_joint[i].torque = joint[i].torque; 
  }
}

void gazebo::arm_6dof_plugin::Print() // 한 싸이클 돌때마다 데이터 플로팅
{
  if(CONTROL_MODE != IDLE)
  {
    // cout << "Control Mode Num: " << CONTROL_MODE << endl;
  }
}

void gazebo::arm_6dof_plugin::ROSMsgPublish()
{
  //****************** msg에 데이터 저장 *****************//
  m_Act_X_CoM.data = Act_CoM[0];
  m_Act_Y_CoM.data = Act_CoM[1];
  m_Act_Z_CoM.data = Act_CoM[2];

  m_ref_CoM_X.data = ref_CoM(0);
  m_ref_CoM_Y.data = ref_CoM(1);
  m_ref_CoM_Z.data = ref_CoM(2);

  m_Act_Ori_Vel_X.data = Act_Ori_Vel[0];
  m_Act_Ori_Vel_Y.data = Act_Ori_Vel[1];
  m_Act_Ori_Vel_Z.data = Act_Ori_Vel[2];

  m_Arm_CoM_X.data = Arm_CoM(0);
  m_Arm_CoM_Y.data = Arm_CoM(1);
  m_Arm_CoM_Z.data = Arm_CoM(2);
  m_Arm_CoM_Roll.data = Arm_CoM(3);
  m_Arm_CoM_Pitch.data = Arm_CoM(4);
  m_Arm_CoM_Yaw.data = Arm_CoM(5);

  m_actual_joint_pos_1.data = actual_joint_pos(0)*rad2deg;
  m_actual_joint_pos_2.data = actual_joint_pos(1)*rad2deg;
  m_actual_joint_pos_3.data = actual_joint_pos(2)*rad2deg;
  m_actual_joint_pos_4.data = actual_joint_pos(3)*rad2deg;
  m_actual_joint_pos_5.data = actual_joint_pos(4)*rad2deg;
  m_actual_joint_pos_6.data = actual_joint_pos(5)*rad2deg;

  m_actual_joint_torque_1.data = joint[0].torque;
  m_actual_joint_torque_2.data = joint[1].torque;
  m_actual_joint_torque_3.data = joint[2].torque;
  m_actual_joint_torque_4.data = joint[3].torque;
  m_actual_joint_torque_5.data = joint[4].torque;
  m_actual_joint_torque_6.data = joint[5].torque;


  m_ref_Pos_X.data = ref_Pose(0);
  m_ref_Pos_Y.data = ref_Pose(1);
  m_ref_Pos_Z.data = ref_Pose(2);
/*
  m_ref_Pos_X.data = ref_Pose(0);
  m_ref_Pos_Y.data = ref_Pose(1);
  m_ref_Pos_Z.data = ref_Pose(2);
*/
  m_ref_Roll.data = csi*rad2deg;
  m_ref_Pitch.data = theta*rad2deg;
  m_ref_Yaw.data = fi*rad2deg;

  m_actual_Pos_X.data = Act_Pos(0);
  m_actual_Pos_Y.data = Act_Pos(1);
  m_actual_Pos_Z.data = Act_Pos(2);

  m_actual_Roll.data = Act_Ori(0)*rad2deg;
  m_actual_Pitch.data = Act_Ori(1)*rad2deg;
  m_actual_Yaw.data = Act_Ori(2)*rad2deg;

   //******************* 퍼블리시 요청 ********************//
  P_ref_CoM_X.publish(m_ref_CoM_X);
  P_ref_CoM_Y.publish(m_ref_CoM_Y);
  P_ref_CoM_Z.publish(m_ref_CoM_Z);
  P_Act_Ori_Vel_X.publish(m_Act_Ori_Vel_X);
  P_Act_Ori_Vel_Y.publish(m_Act_Ori_Vel_Y);
  P_Act_Ori_Vel_Z.publish(m_Act_Ori_Vel_Z);
  P_Act_X_CoM.publish(m_Act_X_CoM);
  P_Act_Y_CoM.publish(m_Act_Y_CoM);
  P_Act_Z_CoM.publish(m_Act_Z_CoM);

  P_actual_joint_pos_1.publish(m_actual_joint_pos_1);
  P_actual_joint_pos_2.publish(m_actual_joint_pos_2);
  P_actual_joint_pos_3.publish(m_actual_joint_pos_3);
  P_actual_joint_pos_4.publish(m_actual_joint_pos_4);
  P_actual_joint_pos_5.publish(m_actual_joint_pos_5);
  P_actual_joint_pos_6.publish(m_actual_joint_pos_6);
  
  P_actual_joint_torque_1.publish(m_actual_joint_torque_1);
  P_actual_joint_torque_2.publish(m_actual_joint_torque_2);
  P_actual_joint_torque_3.publish(m_actual_joint_torque_3);
  P_actual_joint_torque_4.publish(m_actual_joint_torque_4);
  P_actual_joint_torque_5.publish(m_actual_joint_torque_5);
  P_actual_joint_torque_6.publish(m_actual_joint_torque_6);

  P_actual_Pos_X.publish(m_actual_Pos_X);
  P_actual_Pos_Y.publish(m_actual_Pos_Y);
  P_actual_Pos_Z.publish(m_actual_Pos_Z);
  P_actual_Pos_Roll.publish(m_actual_Roll);
  P_actual_Pos_Pitch.publish(m_actual_Pitch);
  P_actual_Pos_Yaw.publish(m_actual_Yaw);
  P_ref_Pos_X.publish(m_ref_Pos_X);
  P_ref_Pos_Y.publish(m_ref_Pos_Y);
  P_ref_Pos_Z.publish(m_ref_Pos_Z);
  P_ref_Pos_Roll.publish(m_ref_Roll);
  P_ref_Pos_Pitch.publish(m_ref_Pitch);
  P_ref_Pos_Yaw.publish(m_ref_Yaw);

  P_Arm_CoM_X.publish(m_Arm_CoM_X);
  P_Arm_CoM_Y.publish(m_Arm_CoM_Y);
  P_Arm_CoM_Z.publish(m_Arm_CoM_Z);
  P_Arm_CoM_Roll.publish(m_Arm_CoM_Roll);
  P_Arm_CoM_Pitch.publish(m_Arm_CoM_Pitch);
  P_Arm_CoM_Yaw.publish(m_Arm_CoM_Yaw);
  
  P_ros_msg.publish(m_ros_msg);
}
