#include "ostrich.h"


namespace gazebo
{
    void Ostrich_simple::Load(physics::ModelPtr _model, sdf::ElementPtr)
    {
        this->model = _model;
        RBDL_INIT();
        GetLinks();
        GetJoints();
        SensorSetting();
        InitROSPubSetting();
        CONTROL_MODE = IDLE;
        joint = new JOINT[12];
        this->last_update_time = this->model->GetWorld()->SimTime();
        this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Ostrich_simple::UpdateAlgorithm, this));
        std::cout << "Load..." << std::endl;
        Torque_data1 = fopen("/home/gnte/catkin_ws/src/ostrich/motion1_Torque.dat","w");
        Torque_data2 = fopen("/home/gnte/catkin_ws/src/ostrich/motion2_Torque.dat","w");
        Torque_data3 = fopen("/home/gnte/catkin_ws/src/ostrich/motion3_Torque.dat","w");
        Torque_data4 = fopen("/home/gnte/catkin_ws/src/ostrich/motion4_Torque.dat","w");
        // std::cout << "file open success." << std::endl;        
    }

    void Ostrich_simple::RBDL_INIT()
    {
       
        rbdl_check_api_version(RBDL_API_VERSION);//check the rdbl version 설치되어있는 RBDL버전과 config.h에 있는 RBDL버전과 다르면 실행이 안됨.
        std::cout << "Check RBDL API VERSION" << std::endl;

        
        A_L.rbdl_model = new Model();
        A_L.rbdl_model->gravity = Math::Vector3d(0., 0., -9.81);
        A_R.rbdl_model = new Model();
        A_R.rbdl_model->gravity = Math::Vector3d(0., 0., -9.81);

    
        L_Air_Model(A_L);
        R_Air_Model(A_R);
        //std::cout << "RBDL_INIT check 2" << std::endl;
    
        rbdl_variable_init(A_L);
        cout<<"DoF of A_L : "<<A_L.rbdl_model->dof_count<<endl;
        rbdl_variable_init(A_R);
        cout<<"DoF of A_R : "<<A_R.rbdl_model->dof_count<<endl;
        std::cout << "RBDL_INIT()" << std::endl;
    }

    void Ostrich_simple::rbdl_variable_init(A_RBDL &rbdl)
    {
        //set Q, QDot, QDDot, prevQ, prevQDot // dof_count = num of degree
        rbdl.Q = VectorNd::Zero(rbdl.rbdl_model->dof_count);
        rbdl.QDot = VectorNd::Zero(rbdl.rbdl_model->dof_count);
        rbdl.QDDot = VectorNd::Zero(rbdl.rbdl_model->dof_count);
        rbdl.prevQ = VectorNd::Zero(rbdl.rbdl_model->dof_count);
        rbdl.prevQDot = VectorNd::Zero(rbdl.rbdl_model->dof_count);
        rbdl.Tau = VectorNd::Zero(rbdl.rbdl_model->dof_count); // 엔코더값을 받아서 사용하거나 RBDL함수를 사용하는데 있어서 변수로 사용하는 것들

        rbdl.Foot_Pos = VectorNd::Zero(6);
        rbdl.Foot_Pos_dot = VectorNd::Zero(6);
        rbdl.A_Jacobian = MatrixNd::Zero(6,6);
        rbdl.prev_A_Jacobian = MatrixNd::Zero(6,6);
        rbdl.A_Jacobian_dot = MatrixNd::Zero(6,6);
        rbdl.Inv_A_Jacobian = MatrixNd::Zero(6,6);
        rbdl.Des_X = VectorNd::Zero(6);
        rbdl.Des_XDot = VectorNd::Zero(6);
        rbdl.Des_XDDot = VectorNd::Zero(6);
        rbdl.torque_CTC = VectorNd::Zero(6);
        rbdl.Old_Des_X = VectorNd::Zero(6);
        rbdl.Old_Des_XDot = VectorNd::Zero(6);
        rbdl.Old_Des_XDDot = VectorNd::Zero(6);
        rbdl.New_Des_X = VectorNd::Zero(6);
        rbdl.New_Des_XDot = VectorNd::Zero(6);
        rbdl.New_Des_XDDot = VectorNd::Zero(6); // 계산한 결과값들을 저장하는 변수
        rbdl.Friction_Torque = VectorNd::Zero(6);
        //std::cout << "rbdl_variable_init check 2" << std::endl;
        rbdl.Kp = VectorNd::Zero(6);
        rbdl.Kv = VectorNd::Zero(6); // 계산한 결과값들을 저장하는 변수
    }
  
    void Ostrich_simple::L_Air_Model(A_RBDL &rbdl)
    {
        
        //********************LEFT_LEG********************//
        //Inertia of Body
        rbdl.bodyI_hip_y = Math::Matrix3d(0.0010518,  4.2724e-11,  -4.8365e-06, 
                                          4.2724e-11, 0.00070148,  -4.6403e-11, 
                                          -4.8365e-06,-4.6403e-11, 0.00080534);

        rbdl.bodyI_hip_r = Math::Matrix3d(0.00062466, 1.9311e-06 ,-1.13e-08, 
                                          1.9311e-06,0.0010077 ,-2.4235e-06, 
                                          -1.13e-08, -2.4235e-06,0.00063231);

        rbdl.bodyI_hip_p = Math::Matrix3d(0.011545, -1.4992e-06, -0.00015607,
                                          -2.8384e-09,-1.4992e-06,0.018715,
                                          -0.00015607,-7.2676e-05 ,0.0076612);

        rbdl.bodyI_knee_p = Math::Matrix3d(0.0014033, -2.8384e-09 ,1.6076e-06,
                                           -2.8384e-09, 0.0021396 ,-8.0318e-06,
                                           1.6076e-06,-8.0318e-06,0.0010902);

        rbdl.bodyI_ankle_p = Math::Matrix3d(0.00013273 ,-2.8963e-08 ,-1.1807e-06, 
                                            -2.8963e-08,0.00019407, -7.5079e-07,
                                            -1.1807e-06,-7.5079e-07, 0.00012766);

        rbdl.bodyI_ankle_r = Math::Matrix3d(0.00098453 ,-1.7901e-06, -3.0542e-05, 
                                            -1.7901e-06, 0.00075959, 2.1039e-06, 
                                            -3.0542e-05,2.1039e-06,0.00075474);

        rbdl.bodyI_foot = Math::Matrix3d(0,0,0,
                                         0,0,0,
                                         0,0,0);
        //set body hip y 
            rbdl.body_hip_y = Body(0.95179, Math::Vector3d(0.050716,-8.8e-05,-0.063512), rbdl.bodyI_hip_y); //기존 무게중심 위치
            rbdl.joint_HIP_YAW = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 0, 1)); // pelvis yaw
            rbdl.body_hip_y_id = rbdl.rbdl_model->Model::AddBody(0, Xtrans(Math::Vector3d(0, 0.1, 0)), rbdl.joint_HIP_YAW, rbdl.body_hip_y);
        
        //set_body_hip_r
            rbdl.body_hip_r = Body(0.87236, Math::Vector3d(5.6e-05,0.080624,-0.000409), rbdl.bodyI_hip_r); // 기존
            rbdl.joint_HIP_ROLL = Joint(JointType::JointTypeRevolute, Math::Vector3d(1, 0, 0)); // pelvis roll
            rbdl.body_hip_r_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_hip_y_id, Xtrans(Math::Vector3d(0, 0, -0.0815)), rbdl.joint_HIP_ROLL, rbdl.body_hip_r);

        //set_body_hip_p
            rbdl.body_hip_p = Body(2.2935, Math::Vector3d(-0.026841,0.05898,-0.22522), rbdl.bodyI_hip_p); // 기존
            rbdl.joint_HIP_PITCH = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0)); // pelvis pitch
            rbdl.body_hip_p_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_hip_r_id, Xtrans(Math::Vector3d(0, 0.111, 0)), rbdl.joint_HIP_PITCH, rbdl.body_hip_p);
        
        //set_body_knee_p
            rbdl.body_knee_p = Body(1.5368, Math::Vector3d(-8.7e-05,-0.053596,-0.02678), rbdl.bodyI_knee_p); // 기존
            rbdl.joint_KNEE_PITCH = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0));
            rbdl.body_knee_p_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_hip_p_id, Xtrans(Math::Vector3d(0, 0, -0.27)), rbdl.joint_KNEE_PITCH, rbdl.body_knee_p);

        //set_body_ankle_p
            rbdl.body_ankle_p = Body(0.36811, Math::Vector3d(0.001874,0.011551,0.000985), rbdl.bodyI_ankle_p); // 기존
            rbdl.joint_ANKLE_PITCH = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0));
            rbdl.body_ankle_p_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_knee_p_id, Xtrans(Math::Vector3d(0, 0.0474, -0.272)), rbdl.joint_ANKLE_PITCH, rbdl.body_ankle_p);

        //set_body_ankle_r
            rbdl.body_ankle_r = Body(1.226, Math::Vector3d(0.029986,0.004847,-0.006262), rbdl.bodyI_ankle_r);
            rbdl.joint_ANKLE_ROLL = Joint(JointType::JointTypeRevolute, Math::Vector3d(1, 0, 0));
            rbdl.body_ankle_r_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_ankle_p_id, Xtrans(Math::Vector3d(0.039, 0, 0)), rbdl.joint_ANKLE_ROLL, rbdl.body_ankle_r);

        //set_body_foot
            rbdl.body_foot = Body(0, Math::Vector3d(0, 0, 0), rbdl.bodyI_foot);
            rbdl.joint_FOOT = Joint(JointType::JointTypeFixed);
            rbdl.body_foot_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_ankle_r_id, Xtrans(Math::Vector3d(0, 0, -0.0815)), rbdl.joint_FOOT, rbdl.body_foot);
    } 

    void Ostrich_simple::R_Air_Model(A_RBDL &rbdl)
    {
        // ///////////////// 골반중심 CTC 풀때 ////////////////////
        // rbdl.bodyI_Base = Math::Matrix3d(0.836611, 1.63e-06, 9.5e-07, 1.63e-06, 2.7426, -5.3e-07, 9.5e-07, -5.3e-07, 2.72415); //상체무게중심에서 측정한 관성메트릭스
            // rbdl.body_Base = Body(0, Math::Vector3d(0, 0, 0), rbdl.bodyI_Base);
            // rbdl.joint_Base = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 0, 0));
        // rbdl.body_Base_id = rbdl.rbdl_model->Model::AddBody(0, Xtrans(Math::Vector3d(0, 0, 0)), rbdl.joint_Base, rbdl.body_Base);

        //********************LEFT_LEG********************//
        //Inertia of Body
        rbdl.bodyI_hip_y = Math::Matrix3d(0.0010518,4.4734e-11,-4.8365e-06,
                                          4.4734e-11 ,0.00070148, -4.6503e-11,
                                          -4.8365e-06,-4.6503e-11, 0.00080534);

        rbdl.bodyI_hip_r = Math::Matrix3d(0.00062466, -1.9311e-06, -1.1207e-08, 
                                          -1.9311e-06, 0.0010077,2.4235e-06, 
                                          -1.1207e-08,2.4235e-06,0.00063231);

        rbdl.bodyI_hip_p = Math::Matrix3d( 0.011545, 3.859e-07, -0.00015608, 
                                           3.859e-07,0.018716, -6.1747e-06,
                                          -0.00015608,-6.1747e-06, 0.0076607);

        rbdl.bodyI_knee_p = Math::Matrix3d(0.0014033,2.8239e-09,1.6076e-06,
                                           2.8239e-09, 0.0021396,8.0318e-06,
                                           1.6076e-06 ,8.0318e-06, 0.0010902);

        rbdl.bodyI_ankle_p = Math::Matrix3d(0.00013272, 2.8963e-08, -1.1807e-06, 
                                            2.8963e-08, 0.00019405, 7.5077e-07,
                                            -1.1807e-06, 7.5077e-07, 0.00012765);

        rbdl.bodyI_ankle_r = Math::Matrix3d(0.00098453, 1.7899e-06, -3.0542e-05, 
                                            1.7899e-06, 0.00075959, -2.1041e-06, 
                                            -3.0542e-05, -2.1041e-06, 0.00075474);

        rbdl.bodyI_foot = Math::Matrix3d(0,0,0,
                                         0,0,0,
                                         0,0,0);

        //set_body_hip_y
            rbdl.body_hip_y = Body(0.95179, Math::Vector3d(0.050716,-8.8e-05,-0.063167), rbdl.bodyI_hip_y); //기존 무게중심 위치
            rbdl.joint_HIP_YAW = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 0, 1)); // pelvis yaw-2.1041e-06
            rbdl.body_hip_y_id = rbdl.rbdl_model->Model::AddBody(0, Xtrans(Math::Vector3d(0, -0.1, 0)), rbdl.joint_HIP_YAW, rbdl.body_hip_y);
        //rbdl.body_hip_y_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_Base_id, Xtrans(Math::Vector3d(0, 0.07, 0)), rbdl.joint_HIP_YAW, rbdl.body_hip_y);
        
        //set_body_hip_r
            rbdl.body_hip_r = Body(0.87236, Math::Vector3d(5.6e-05,-0.080624,-0.000409), rbdl.bodyI_hip_r); // 기존
            rbdl.joint_HIP_ROLL = Joint(JointType::JointTypeRevolute, Math::Vector3d(1, 0, 0)); // pelvis roll
            rbdl.body_hip_r_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_hip_y_id, Xtrans(Math::Vector3d(0, 0, -0.0815)), rbdl.joint_HIP_ROLL, rbdl.body_hip_r);

        //set_body_hip_p
            rbdl.body_hip_p = Body(2.2935, Math::Vector3d(-0.026841,-0.060585,-0.2248), rbdl.bodyI_hip_p); // 기존
            rbdl.joint_HIP_PITCH = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0)); // pelvis pitch
            rbdl.body_hip_p_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_hip_r_id, Xtrans(Math::Vector3d(0, -0.111, 0)), rbdl.joint_HIP_PITCH, rbdl.body_hip_p);
        
            //set_body_knee_p
            rbdl.body_knee_p = Body(1.5368, Math::Vector3d(-8.7e-05,0.053596,-0.02678), rbdl.bodyI_knee_p); // 기존
            rbdl.joint_KNEE_PITCH = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0));
            rbdl.body_knee_p_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_hip_p_id, Xtrans(Math::Vector3d(0, 0, -0.27)), rbdl.joint_KNEE_PITCH, rbdl.body_knee_p);

            //set_body_ankle_p
            rbdl.body_ankle_p = Body(0.36739, Math::Vector3d(0.001827,-0.011645,0.000965), rbdl.bodyI_ankle_p); // 기존
            rbdl.joint_ANKLE_PITCH = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0));
            rbdl.body_ankle_p_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_knee_p_id, Xtrans(Math::Vector3d(0, 0.0474, -0.27)), rbdl.joint_ANKLE_PITCH, rbdl.body_ankle_p);

            //set_body_ankle_r
            rbdl.body_ankle_r = Body(1.226, Math::Vector3d(0.029986,-0.004847,-0.006262), rbdl.bodyI_ankle_r);
            rbdl.joint_ANKLE_ROLL = Joint(JointType::JointTypeRevolute, Math::Vector3d(1, 0, 0));
            rbdl.body_ankle_r_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_ankle_p_id, Xtrans(Math::Vector3d(0.039, 0, 0)), rbdl.joint_ANKLE_ROLL, rbdl.body_ankle_r);

        //set_body_foot
            rbdl.body_foot = Body(0, Math::Vector3d(0, 0, 0), rbdl.bodyI_foot);
            rbdl.joint_FOOT = Joint(JointType::JointTypeFixed);
            rbdl.body_foot_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_ankle_r_id, Xtrans(Math::Vector3d(0, 0, -0.0815)), rbdl.joint_FOOT, rbdl.body_foot);
    }

    void Ostrich_simple::UpdateAlgorithm()
    {
        current_time = this->model->GetWorld()->SimTime();
        dt = current_time.Double() - this->last_update_time.Double();
        this->last_update_time = current_time;
         
        // std::cout << "update algo start." << std::endl;
        // IMUSensorRead();
        // std::cout << "Imu sensor read success." << std::endl;
        EncoderRead();               
        // std::cout << "Encoder read success." << std::endl;
        RBDL_variable_update();
        VelocityRead();
        ROSMsgPublish();
        // std::cout << "ros msg published." << std::endl;
        Calculate_CoM();
        PostureGeneration();
        // std::cout << "Posture generation success." << std::endl;
        JointController();   
        // std::cout << "update algo finish." << std::endl;    
    }

    void Ostrich_simple::EncoderRead()
    {           
        actual_joint_pos[0] = this->L_HIP_Y_JOINT->Position(2);
        actual_joint_pos[1] = this->L_HIP_R_JOINT->Position(0);
        actual_joint_pos[2] = this->L_HIP_P_JOINT->Position(1);        
        actual_joint_pos[3] = this->L_KNEE_P_JOINT->Position(1);
        actual_joint_pos[4] = this->L_ANKLE_P_JOINT->Position(1);
        actual_joint_pos[5] = this->L_ANKLE_R_JOINT->Position(0);
        
        // for (uint8_t i = 0; i<6; i ++) th[i] = actual_joint_pos[i];

        actual_joint_pos[6] = this->R_HIP_Y_JOINT->Position(2);
        actual_joint_pos[7] = this->R_HIP_R_JOINT->Position(0);
        actual_joint_pos[8] = this->R_HIP_P_JOINT->Position(1);        
        actual_joint_pos[9] = this->R_KNEE_P_JOINT->Position(1);
        actual_joint_pos[10] = this->R_ANKLE_P_JOINT->Position(1);
        actual_joint_pos[11] = this->R_ANKLE_R_JOINT->Position(0);


        
    }

    void Ostrich_simple::VelocityRead()
    {           
        actual_joint_vel[0] = this->L_HIP_Y_JOINT->GetVelocity(2);
        actual_joint_vel[1] = this->L_HIP_R_JOINT->GetVelocity(0);
        actual_joint_vel[2] = this->L_HIP_P_JOINT->GetVelocity(1);        
        actual_joint_vel[3] = this->L_KNEE_P_JOINT->GetVelocity(1);
        actual_joint_vel[4] = this->L_ANKLE_P_JOINT->GetVelocity(1);
        actual_joint_vel[5] = this->L_ANKLE_R_JOINT->GetVelocity(0);

        actual_joint_vel[6] = this->R_HIP_Y_JOINT->GetVelocity(2);
        actual_joint_vel[7] = this->R_HIP_R_JOINT->GetVelocity(0);
        actual_joint_vel[8] = this->R_HIP_P_JOINT->GetVelocity(1);        
        actual_joint_vel[9] = this->R_KNEE_P_JOINT->GetVelocity(1);
        actual_joint_vel[10] = this->R_ANKLE_P_JOINT->GetVelocity(1);
        actual_joint_vel[11] = this->R_ANKLE_R_JOINT->GetVelocity(0);
    }

    // void Ostrich_simple::IMUSensorRead()
    // {
    //     base_info = this->model->WorldPose();
    //     body_quat = this->BODY_IMU->Orientation();
    //     body_roll = body_quat.Euler()[0];
    //     body_pitch = body_quat.Euler()[1];
    // }

    void Ostrich_simple::GetJoints()
    {
        //JOINT DEFINITION
        //BODY//
        //this->BASE_JOINT = this->model->GetJoint("WORLD_JOINT");
        //this->IMU_JOINT = this->model->GetJoint("IMU_JOINT");
        //this->PELVIS_JOINT = this->model->GetJoint("PELVIS_JOINT");
        //this->LF_WHEEL_JOINT = this->model->GetJoint("LF_WHEEL_JOINT");
        //this->RF_WHEEL_JOINT = this->model->GetJoint("RF_WHEEL_JOINT");
        /////////    
        ////LEFT_LEG_JOINTS////////
        this->L_ANKLE_P_JOINT = this->model->GetJoint("L_ANKLE_P_JOINT");
        this->L_ANKLE_R_JOINT = this->model->GetJoint("L_ANKLE_R_JOINT");
        this->L_HIP_P_JOINT = this->model->GetJoint("L_HIP_P_JOINT");
        this->L_HIP_R_JOINT = this->model->GetJoint("L_HIP_R_JOINT");
        this->L_HIP_Y_JOINT = this->model->GetJoint("L_HIP_Y_JOINT");
        this->L_KNEE_P_JOINT = this->model->GetJoint("L_KNEE_P_JOINT");
        this->L_MAINFOOT_P_JOINT = this->model->GetJoint("L_MAINFOOT_P_JOINT");
        this->L_SUBFOOT_P_JOINT = this->model->GetJoint("L_SUBFOOT_P_JOINT");
        
        ///////////
        //////RIGHT_LEG_JOINTS///////
        this->R_ANKLE_P_JOINT = this->model->GetJoint("R_ANKLE_P_JOINT");
        this->R_ANKLE_R_JOINT = this->model->GetJoint("R_ANKLE_R_JOINT");
        this->R_HIP_P_JOINT = this->model->GetJoint("R_HIP_P_JOINT");
        this->R_HIP_R_JOINT = this->model->GetJoint("R_HIP_R_JOINT");
        this->R_HIP_Y_JOINT = this->model->GetJoint("R_HIP_Y_JOINT");
        this->R_KNEE_P_JOINT = this->model->GetJoint("R_KNEE_P_JOINT");
        this->R_MAINFOOT_P_JOINT = this->model->GetJoint("R_MAINFOOT_P_JOINT");
        this->R_SUBFOOT_P_JOINT = this->model->GetJoint("R_SUBFOOT_P_JOINT");
        
    }

    void Ostrich_simple::GetLinks()
    {
        //BODY//
        //this->BASE_LINK = this->model->GetLink("BASE_LINK");
        //this->IMU_LINK = this->model->GetLink("IMU_LINK");
        //this->PELVIS_LINK = this->model->GetLink("PELVIS_LINK");
        //this->LF_WHEEL_LINK = this->model->GetLink("LF_WHEEL_LINK");
        //this->RF_WHEEL_LINK = this->model->GetLink("RF_WHEEL_LINK");
        /////////    
        ////LEFT_LEG_LINKS////////
        this->L_ANKLE_P_LINK = this->model->GetLink("L_ANKLE_P_LINK");
        this->L_ANKLE_R_LINK = this->model->GetLink("L_ANKLE_R_LINK");
        this->L_HIP_P_LINK = this->model->GetLink("L_HIP_P_LINK");
        this->L_HIP_R_LINK = this->model->GetLink("L_HIP_R_LINK");
        this->L_HIP_Y_LINK = this->model->GetLink("L_HIP_Y_LINK");
        this->L_KNEE_P_LINK = this->model->GetLink("L_KNEE_P_LINK");
        this->L_MAINFOOT_P_LINK = this->model->GetLink("L_MAINFOOT_P_LINK");
        this->L_SUBFOOT_P_LINK = this->model->GetLink("L_SUBFOOT_P_LINK");
        ///////////
        //////RIGHT_LEG_LINKS///////
        this->R_ANKLE_P_LINK = this->model->GetLink("R_ANKLE_P_LINK");
        this->R_ANKLE_R_LINK = this->model->GetLink("R_ANKLE_R_LINK");
        this->R_HIP_P_LINK = this->model->GetLink("R_HIP_P_LINK");
        this->R_HIP_R_LINK = this->model->GetLink("R_HIP_R_LINK");
        this->R_HIP_Y_LINK = this->model->GetLink("R_HIP_Y_LINK");
        this->R_KNEE_P_LINK = this->model->GetLink("R_KNEE_P_LINK");
        this->R_MAINFOOT_P_LINK = this->model->GetLink("R_MAINFOOT_P_LINK");
        this->R_SUBFOOT_P_LINK = this->model->GetLink("R_SUBFOOT_P_LINK");
    }

    void Ostrich_simple::SensorSetting()
    {
        this->Sensor = sensors::get_sensor("BODY_IMU");
        this->BODY_IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor);
    }

    void Ostrich_simple::InitROSPubSetting()
    {        
        p_joint_state = node_handle.advertise<sensor_msgs::JointState>("ostrich/joint_states", 100);
        p_ee_pose = node_handle.advertise<geometry_msgs::Pose>("STOB/R_EE_Pose", 100);
        p_ee_pose_ref = node_handle.advertise<geometry_msgs::Pose>("STOB/R_EE_Pose_ref", 100);
        p_imu = node_handle.advertise<sensor_msgs::Imu>("ostrich/imu", 100);  
        p_ros_msg = node_handle.advertise<std_msgs::Float64MultiArray>("TmpData", 50); // topicname, queue_size = 50
        m_ros_msg.data.resize(50);  
        server_sub1 = node_handle.subscribe("Ctrl_mode", 1, &gazebo::Ostrich_simple::SwitchMode, this); 
        p_torque_L_Knee_P_J = node_handle.advertise<std_msgs::Float64>("torque_L_Knee_P_J", 10);
        s_gain_p = node_handle.subscribe("STOB/gain_p", 1, &gazebo::Ostrich_simple::SwitchGainP, this);
        s_gain_d = node_handle.subscribe("STOB/gain_d", 1, &gazebo::Ostrich_simple::SwitchGainD, this);
        s_gain_w = node_handle.subscribe("STOB/gain_w", 1, &gazebo::Ostrich_simple::SwitchGainW, this);
        p_com = node_handle.advertise<geometry_msgs::Pose>("STOB/robot_act_com",100);
    }

    void Ostrich_simple::SwitchMode(const std_msgs::Int32Ptr & msg) 
    {
        cnt = 0;
        if (msg -> data == 0) CONTROL_MODE = IDLE;
        else if(msg -> data == 1) CONTROL_MODE = walk_ready;
        else if (msg -> data == 2) CONTROL_MODE = motion_TS_PD;
        else if(msg -> data == 3) CONTROL_MODE = gravitiy_compensation_air;
        else CONTROL_MODE = IDLE;
    }

    void Ostrich_simple::SwitchGainP(const std_msgs::Float32MultiArrayPtr &msg){
        gain_p[0] = msg -> data.at(0);
        gain_p[1] = msg -> data.at(1);
        gain_p[2] = msg -> data.at(2);
    }

    void Ostrich_simple::SwitchGainW(const std_msgs::Float32MultiArrayPtr &msg){
        gain_w[0] = msg -> data.at(0);
        gain_w[1] = msg -> data.at(1);
        gain_w[2] = msg -> data.at(2);
    }

    void Ostrich_simple::SwitchGainD(const std_msgs::Float32MultiArrayPtr &msg){
        gain_d[0] = msg -> data.at(0);
        gain_d[1] = msg -> data.at(1);
        gain_d[2] = msg -> data.at(2);
        gain_d[3] = msg -> data.at(3);
        gain_d[4] = msg -> data.at(4);
        gain_d[5] = msg -> data.at(5);
    }

    void Ostrich_simple::ROSMsgPublish(){
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.header.stamp = ros::Time::now();  

        for (uint8_t i = 0; i<12; i ++)
        {
            joint_state_msg.name.push_back((std::string)joint_names.at(i));
            joint_state_msg.position.push_back((float)(actual_joint_pos[i]*rad2deg));
            joint_state_msg.velocity.push_back((float)(actual_joint_vel[i]));
            joint_state_msg.effort.push_back((float)joint[i].torque);
        }
        p_joint_state.publish(joint_state_msg); 

        geometry_msgs::Pose r_ee_pose_msg;
        r_ee_pose_msg.position.x = (float)(ee_position[0]);
        r_ee_pose_msg.position.y = (float)(ee_position[1]);
        r_ee_pose_msg.position.z = (float)(ee_position[2]);
        p_ee_pose.publish(r_ee_pose_msg);

        geometry_msgs::Pose r_ee_pose_ref_msg;
        r_ee_pose_ref_msg.position.x = (float)(ref_ee_position[0]);
        r_ee_pose_ref_msg.position.y = (float)(ref_ee_position[1]);
        r_ee_pose_ref_msg.position.z = (float)(ref_ee_position[2]);
        p_ee_pose_ref.publish(r_ee_pose_ref_msg);

        geometry_msgs::Pose com_pose_msg;
        com_pose_msg.position.x = (float)(Robot_Pos_CoM[0]);
        com_pose_msg.position.y = (float)(Robot_Pos_CoM[1]);
        com_pose_msg.position.z = (float)(Robot_Pos_CoM[2]);
        p_com.publish(com_pose_msg);
    }

    void Ostrich_simple::JointController() 
    {
        // Left Leg ankle roll
        if (joint[5].torque >= 21.6) joint[5].torque = 21.6;
        if (joint[5].torque <= -21.6) joint[5].torque = -21.6;
        // Left Leg ankle pitch
        if (joint[4].torque >= 31.2) joint[4].torque = 31.2;
        if (joint[4].torque <= -31.2) joint[4].torque = -31.2;
        // right Leg ankle roll
        if (joint[11].torque >= 21.6)  joint[11].torque = 21.6;
        if (joint[11].torque <= -21.6)  joint[11].torque = -21.6;
        // right Leg ankle pitch
        if (joint[10].torque >= 31.2) joint[10].torque = 31.2;
        if (joint[10].torque <= -31.2)   joint[10].torque = -31.2;
        // left Leg knee pitch
        if (joint[3].torque >= 40) joint[3].torque = 40;
        if (joint[3].torque <= -40) joint[3].torque = -40;
        // right Leg knee pitch
        if (joint[9].torque >= 40) joint[9].torque = 40;
        if (joint[9].torque <= -40) joint[9].torque = -40;
        // left Leg hip yaw
        if (joint[0].torque >= 8.4)  joint[0].torque = 8.4;
        if (joint[0].torque <= -8.4) joint[0].torque = -8.4;
        // right Leg hip yaw
        if (joint[6].torque >= 8.4) joint[6].torque = 8.4;
        if (joint[6].torque <= -8.4) joint[6].torque = -8.4;
        // left hip roll
        if (joint[1].torque >= 21.6) joint[1].torque = 21.6;
        if (joint[1].torque <= -21.6) joint[1].torque = -21.6;
        // right hip roll
        if (joint[7].torque >= 21.6) joint[7].torque = 21.6;
        if (joint[7].torque <= -21.6) joint[7].torque = -21.6;
        // left hip pitch
        if (joint[2].torque >= 21.6) joint[2].torque = 21.6;
        if (joint[2].torque <= -21.6) joint[2].torque = -21.6;
        // right hip pitch
        if (joint[8].torque >= 21.6) joint[8].torque = 21.6;
        if (joint[8].torque <= -21.6) joint[8].torque = -21.6;

        this->L_HIP_Y_JOINT->SetForce(2, joint[0].torque); 
        this->L_HIP_R_JOINT->SetForce(0, joint[1].torque); 
        this->L_HIP_P_JOINT->SetForce(1, joint[2].torque); 
        this->L_KNEE_P_JOINT->SetForce(1, joint[3].torque); 
        this->L_ANKLE_P_JOINT->SetForce(1, joint[4].torque); 
        this->L_ANKLE_R_JOINT->SetForce(0, joint[5].torque); 
      
        this->R_HIP_Y_JOINT->SetForce(2, joint[6].torque); 
        this->R_HIP_R_JOINT->SetForce(0, joint[7].torque); 
        this->R_HIP_P_JOINT->SetForce(1, joint[8].torque); 
        this->R_KNEE_P_JOINT->SetForce(1, joint[9].torque); 
        this->R_ANKLE_P_JOINT->SetForce(1, joint[10].torque); 
        this->R_ANKLE_R_JOINT->SetForce(0, joint[11].torque); 
    }

  
    void Ostrich_simple::PostureGeneration()
    {
        if (CONTROL_MODE == IDLE) Init_Pos_Traj();
        else if(CONTROL_MODE == walk_ready) Walk_ready();
        else if (CONTROL_MODE == motion_TS_PD) Task_Space_PD_both_legs();
        else if (CONTROL_MODE == gravitiy_compensation_air) Gravity_Compensation_air();
        else Init_Pos_Traj();        
    }

    void Ostrich_simple::RBDL_variable_update()
    {
        // Air variable
        A_L.Q(0) = actual_joint_pos[0];
        A_L.Q(1) = actual_joint_pos[1];
        A_L.Q(2) = actual_joint_pos[2];
        A_L.Q(3) = actual_joint_pos[3];
        A_L.Q(4) = actual_joint_pos[4];
        A_L.Q(5) = actual_joint_pos[5];
        //A_R.Q(6) = 0;

        A_R.Q(0) = actual_joint_pos[6];
        A_R.Q(1) = actual_joint_pos[7];
        A_R.Q(2) = actual_joint_pos[8];
        A_R.Q(3) = actual_joint_pos[9];
        A_R.Q(4) = actual_joint_pos[10];
        A_R.Q(5) = actual_joint_pos[11];
        //A_L.Q(6) = 0;

        A_L.QDot = (A_L.Q - A_L.prevQ) / inner_dt;
        A_L.QDDot = (A_L.QDot - A_L.prevQDot) / inner_dt;
        A_R.QDot = (A_R.Q - A_R.prevQ) / inner_dt;
        A_R.QDDot = (A_R.QDot - A_R.prevQDot) / inner_dt;

        A_L.prevQ = A_L.Q;
        A_L.prevQDot = A_L.QDot;
        A_R.prevQ = A_R.Q;
        A_R.prevQDot = A_R.QDot;
    }

    void Ostrich_simple::Init_Pos_Traj()
    {
        Kp_s << 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300;

        Kd_s << 1, 1, 1, 1, 1, 1;

        step_time = 0.5; //주기설정 (초) 변수
        cnt_time = cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
        cnt++;

        double Init_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));

        if(cnt == 1){
            for(int i = 0; i<12; i++){
            Init_th[i] = actual_joint_pos[i];
            }
        }

        if(cnt_time <= step_time)
        {
           

            Theo_LL_th[0] = Init_th[0] + 0*Init_trajectory*deg2rad;    //hipyaw
            Theo_LL_th[1] = Init_th[1] + 0*Init_trajectory*deg2rad;    //hiproll
            Theo_LL_th[2] = Init_th[2] + 30*Init_trajectory*deg2rad;    //hippitch
            Theo_LL_th[3] = Init_th[3] - 60*Init_trajectory*deg2rad;    //kneepitch
            Theo_LL_th[4] = Init_th[4] + 30*Init_trajectory*deg2rad;    //anklepitch    
            Theo_LL_th[5] = Init_th[5] + 0*Init_trajectory*deg2rad;    //ankleroll
   
            Theo_RL_th[0] = Init_th[6] + 0*Init_trajectory*deg2rad;    //hipyaw
            Theo_RL_th[1] = Init_th[7] + 0*Init_trajectory*deg2rad;    //hiproll
            Theo_RL_th[2] = Init_th[8] + 30*Init_trajectory*deg2rad;    //hippitc
            Theo_RL_th[3] = Init_th[9] - 60*Init_trajectory*deg2rad;    //kneepitch
            Theo_RL_th[4] = Init_th[10] + 30*Init_trajectory*deg2rad;   //anklepitch   
            Theo_RL_th[5] = Init_th[11] + 0*Init_trajectory*deg2rad;   //ankleroll
           
        }
        NonlinearEffects(*A_L.rbdl_model, A_L.Q, A_L.QDot, A_L.Tau, NULL);
        NonlinearEffects(*A_R.rbdl_model, A_R.Q, A_R.QDot, A_R.Tau, NULL);
    
         for (int i = 0; i < 6; i++) {
            joint[i].torque = Kp_s[i]*(Theo_LL_th[i] - actual_joint_pos[i]) 
                              + Kd_s[i]* (0 - actual_joint_vel[i])
                              +A_L.Tau(i); // 기본 PV제어 코드

            joint[i+6].torque = Kp_s[i+6]*(Theo_RL_th[i] - actual_joint_pos[i+6]) 
                                + Kd_s[i] * (0 - actual_joint_vel[i+6])
                                +A_R.Tau(i); // 기본 PV제어 코드
        }
                
        

    }

    /**void Ostrich_simple::Walk_ready()
    {
        Kp_s << 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300;
        Kd_s << 1, 1, 1, 1, 1, 1;

        step_time = 0.5; //주기설정 (초) 변수
        cnt_time = cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
        cnt++;

        double Init_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));
        if(cnt == 1)
        {
            for(int i = 0; i<12; i++){
            Init_th[i] = actual_joint_pos[i];
            }
        }

        if(cnt_time <= step_time)
        {
           

            Theo_LL_th[0] = Init_th[0] + 0*Init_trajectory*deg2rad;    //hipyaw
            Theo_LL_th[1] = Init_th[1] + 0*Init_trajectory*deg2rad;    //hiproll
            Theo_LL_th[2] = Init_th[2] + 30*Init_trajectory*deg2rad;    //hippitch
            Theo_LL_th[3] = Init_th[3] - 60*Init_trajectory*deg2rad;    //kneepitch
            Theo_LL_th[4] = Init_th[4] + 30*Init_trajectory*deg2rad;    //anklepitch    
            Theo_LL_th[5] = Init_th[5] + 0*Init_trajectory*deg2rad;    //ankleroll      

            Theo_RL_th[0] = Init_th[6]  + 0*Init_trajectory*deg2rad;    //hipyaw
            Theo_RL_th[1] = Init_th[7]  + 0*Init_trajectory*deg2rad;    //hiproll
            Theo_RL_th[2] = Init_th[8]  + 30*Init_trajectory*deg2rad;    //hippitc
            Theo_RL_th[3] = Init_th[9]  - 60*Init_trajectory*deg2rad;    //kneepitch
            Theo_RL_th[4] = Init_th[10] + 30*Init_trajectory*deg2rad;   //anklepitch   
            Theo_RL_th[5] = Init_th[11] + 0*Init_trajectory*deg2rad;   //ankleroll
           
        }
        NonlinearEffects(*A_L.rbdl_model, A_L.Q, A_L.QDot, A_L.Tau, NULL);
        NonlinearEffects(*A_R.rbdl_model, A_R.Q, A_R.QDot, A_R.Tau, NULL);
    
       
        ///////////////토크 입력////////////////
        for (int i = 0; i < 6; i++) {
            joint[i].torque = Kp_s[i]*(Theo_LL_th[i] - actual_joint_pos[i]) 
                              + Kd_s[i]* (0 - actual_joint_vel[i])
                              +A_L.Tau(i); // 기본 PV제어 코드

            joint[i+6].torque = Kp_s[i+6]*(Theo_RL_th[i] - actual_joint_pos[i+6]) 
                                + Kd_s[i] * (0 - actual_joint_vel[i+6])
                                +A_R.Tau(i); // 기본 PV제어 코드
        }
       
    }*/

   // both legs
    void Ostrich_simple::Task_Space_PD_both_legs(){
        
        for (uint8_t i = 0; i<12; i ++) th[i] = actual_joint_pos[i];
        //for (uint8_t i = 0; i<12; i ++) joint[i].torque = 0;
        
        step_time = 3;
        cnt_time = cnt*inner_dt;   

        A0 << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

        A1 <<  0, -1, 0, 0,
               1, 0, 0, L8,
               0, 0, 1, 0,
               0, 0, 0, 1;
        
        A2 <<  cos(th[0]),     0,     sin(th[0]),      0,
               sin(th[0]),     0,    -cos(th[0]),      0,
                        0,     1,              0,    -L9,
                        0,     0,              0,      1;
        
        A3 <<  -sin(th[1]),    0,       cos(th[1]),    0,
                cos(th[1]),    0,       sin(th[1]),    0,
                         0,    1,                0,    0,
                         0,    0,                0,    1;

        A4 <<            1,    0,                0,    0,
                         0,    1,                0,    0,
                         0,    0,                1,  L10,
                         0,    0,                0,    1;

        A5 <<   cos(th[2]), -sin(th[2]),  0,   -L11*cos(th[2]),
                sin(th[2]),  cos(th[2]),  0,   -L11*sin(th[2]),
                         0,           0,  1,                 0,
                         0,           0,  0,                 1;

        A6 <<   cos(th[3]), -sin(th[3]),  0,   -L13*cos(th[3]),
                sin(th[3]),  cos(th[3]),  0,   -L13*sin(th[3]),
                          0,          0,  1,              -L12,
                          0,          0,  0,                 1;     

        A7 <<   cos(th[4]),   0,    -sin(th[4]),        0,
                sin(th[4]),   0,     cos(th[4]),        0,
                         0,  -1,              0,        0,
                         0,   0,              0,        1;

        A8 <<   0,1,0,0,
                0,1,0,0,
                0,0,1,L14,
                0,0,0,1;

        A9 <<   cos(th[11]),   0,     -sin(th[11]),      -L15*cos(th[11]),
                sin(th[11]),   0,      cos(th[11]),       -L15*sin(th[11]),
                         0,  -1,               0,       0,
                         0,   0,               0,       1;

        A10 <<  0,0,1,0,
               -1,0,0,0,
                0,-1,0,0,
                0,0,0,1;

        // std::cout << "HT matrix half success." << std::endl;    

        T00 = A0*A1*A2*A3*A4;
        T01 = T00*A5;
        T02 = T01*A6;
        T03 = T02*A7*A8*A9*A10;
    
        a0 << T00(0,2), T00(1,2), T00(2,2);  //z axis unit vector  
        a1 << T01(0,2), T01(1,2), T01(2,2);
        a2 << T02(0,2), T02(1,2), T02(2,2);
        a3 << T03(0,2), T03(1,2), T03(2,2);

        // std::cout << "HT matrix full success." << std::endl;    

        P3_P0 << T03(0,3)-T00(0,3), T03(1,3)-T00(1,3), T03(2,3)-T00(2,3); // 0 bun gwa E.E sai vector 
        P3_P1 << T03(0,3)-T01(0,3), T03(1,3)-T01(1,3), T03(2,3)-T01(2,3);
        P3_P2 << T03(0,3)-T02(0,3), T03(1,3)-T02(1,3), T03(2,3)-T02(2,3);

        // std::cout << "FK success." << std::endl;    

        J1 << a0.cross(P3_P0), a0;
        J2 << a1.cross(P3_P1), a1;
        J3 << a2.cross(P3_P2), a2;

        // std::cout << "Jacobian half success." << std::endl;    

        Jacobian << J1, J2, J3;

        // std::cout << "Jacobian success." << std::endl;    

        ee_position << T03(0,3), T03(1,3), T03(2,3);

        if (cnt<1) initial_ee_position = ee_position;    

        if(cnt_time <= step_time*100)
        { 
            // ref_ee_position(0) = 0.1;
            ref_ee_position(0) = Robot_Pos_CoM[0];
            ref_ee_position(1) = initial_ee_position(1);
            ref_ee_position(2) = initial_ee_position(2) + 0.12 * (1 - cos(PI * (cnt_time / step_time)));
            
        }
        NonlinearEffects(*A_L.rbdl_model, A_L.Q, A_L.QDot, A_L.Tau, NULL);
        ref_ee_quaternion.w() = qw; ref_ee_quaternion.x() = qx; ref_ee_quaternion.y() = qy; ref_ee_quaternion.z() = qz;      

        // std::cout << "Trajectory success." << std::endl;          

        ee_force(0) = gain_p(0) * (ref_ee_position(0) - ee_position(0));
        ee_force(1) = gain_p(1) * (ref_ee_position(1) - ee_position(1));
        ee_force(2) = gain_p(2) * (ref_ee_position(2) - ee_position(2));

        ee_rotation = T03.block<3,3>(0,0);
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
    
        tau = Jacobian.transpose()*virtual_spring;
        
        for (uint8_t i = 0; i<6; i ++) tau_viscous_damping[i] = gain_d[i] * actual_joint_vel[i]; 

        joint[2].torque = tau[0] - tau_viscous_damping[2] + A_L.Tau(2);
        joint[3].torque = tau[1] - tau_viscous_damping[3] + A_L.Tau(3);
        joint[4].torque = tau[2] - tau_viscous_damping[4] + A_L.Tau(4);

    
        // Right leg start

        A11 <<  0, -1, 0, 0,
               1, 0, 0, -L8,
               0, 0, 1, 0,
               0, 0, 0, 1;
        
        A12 <<  cos(th[6]),     0,     sin(th[6]),      0,
               sin(th[6]),     0,    -cos(th[6]),      0,
                        0,     1,              0,    -L9,
                        0,     0,              0,      1;
        
        A13 <<  -sin(th[7]),    0,       cos(th[7]),    0,
                cos(th[7]),    0,        sin(th[7]),    0,
                         0,    1,                0,    0,
                         0,    0,                0,    1;

        A14 <<            1,    0,                0,    0,
                         0,    1,                0,    0,
                         0,    0,                1, -L10,
                         0,    0,                0,    1;

        A15 <<   cos(th[8]), -sin(th[8]),  0,   -L11*cos(th[8]),
                sin(th[8]),  cos(th[8]),  0,   -L11*sin(th[8]),
                         0,           0,  1,                 0,
                         0,           0,  0,                 1;

        A16 <<   cos(th[9]), -sin(th[9]),  0,   -L13*cos(th[9]),
                sin(th[9]),  cos(th[9]),  0,   -L13*sin(th[9]),
                          0,          0,  1,               L12,
                          0,          0,  0,                 1;     

        A17 <<   cos(th[10]),   0,    -sin(th[10]),        0,
                sin(th[10]),   0,     cos(th[10]),        0,
                         0,  -1,              0,        0,
                         0,   0,              0,        1;

        A18 <<   0,1,0,0,
               0,1,0,0,
                0,0,1,L14,
                0,0,0,1;

        A19 <<   cos(th[11]),   0,     -sin(th[11]),      -L15*cos(th[11]),
                sin(th[11]),   0,      cos(th[11]),       -L15*sin(th[11]),
                         0,  -1,               0,       0,
                         0,   0,               0,       1;

        A20 <<  0,0,1,0,
               -1,0,0,0,
                0,-1,0,0,
                0,0,0,1;

        T10 = A0*A11*A12*A13*A14;
        T11 = T10*A15;
        T12 = T11*A16;
        T13 = T12*A17*A18*A19*A20;
    
        a11 << T11(0,2), T11(1,2), T11(2,2);
        a12 << T12(0,2), T12(1,2), T12(2,2);
        a13 << T13(0,2), T13(1,2), T13(2,2);  

        P13_P0 <<  T13(0,3)-T00(0,3), T13(1,3)-T00(1,3), T13(2,3)-T00(2,3);
        P13_P11 << T13(0,3)-T11(0,3), T13(1,3)-T11(1,3), T13(2,3)-T11(2,3);
        P13_P12 << T13(0,3)-T12(0,3), T13(1,3)-T12(1,3), T13(2,3)-T12(2,3);

        J11 << a0.cross(P13_P0), a0;
        J12 << a11.cross(P13_P11), a11;
        J13 << a12.cross(P13_P12), a12;

        Jacobian_R << J11, J12, J13;

        r_ee_position << T13(0,3), T13(1,3), T13(2,3);   
        
        if (cnt<1) r_initial_ee_position = r_ee_position;    

        if(cnt_time <= step_time*100)
        { 
            r_ref_ee_position(0) = Robot_Pos_CoM[0];          
            r_ref_ee_position(1) = r_initial_ee_position(1);
            r_ref_ee_position(2) = r_initial_ee_position(2) + 0.12* (1 - cos(PI * (cnt_time / step_time)));

        }
         NonlinearEffects(*A_R.rbdl_model, A_R.Q, A_R.QDot, A_R.Tau, NULL);

        r_ref_ee_quaternion.w() = qw; r_ref_ee_quaternion.x() = qx; r_ref_ee_quaternion.y() = qy; r_ref_ee_quaternion.z() = qz;      

        r_ee_force(0) = gain_p(0) * (r_ref_ee_position(0) - r_ee_position(0));
        r_ee_force(1) = gain_p(1) * (r_ref_ee_position(1) - r_ee_position(1));
        r_ee_force(2) = gain_p(2) * (r_ref_ee_position(2) - r_ee_position(2));

        r_ee_rotation = T13.block<3,3>(0,0);
        r_ee_rotation_x = r_ee_rotation.block<3,1>(0,0); 
        r_ee_rotation_y = r_ee_rotation.block<3,1>(0,1); 
        r_ee_rotation_z = r_ee_rotation.block<3,1>(0,2);

        r_ref_ee_rotation = r_ref_ee_quaternion.normalized().toRotationMatrix();    

        r_ref_ee_rotation_x = r_ref_ee_rotation.block<3,1>(0,0); 
        r_ref_ee_rotation_y = r_ref_ee_rotation.block<3,1>(0,1); 
        r_ref_ee_rotation_z = r_ref_ee_rotation.block<3,1>(0,2);
        r_ee_orientation_error = r_ee_rotation_x.cross(r_ref_ee_rotation_x) + r_ee_rotation_y.cross(r_ref_ee_rotation_y) + r_ee_rotation_z.cross(r_ref_ee_rotation_z);
        r_ee_momentum << gain_w(0) * r_ee_orientation_error(0), gain_w(1) * r_ee_orientation_error(1), gain_w(2) * r_ee_orientation_error(2);

        r_virtual_spring << r_ee_force(0), r_ee_force(1), r_ee_force(2), r_ee_momentum(0), r_ee_momentum(1), r_ee_momentum(2);
    
        r_tau = Jacobian_R.transpose() * r_virtual_spring;
        for (uint8_t i = 0; i<6; i ++) r_tau_viscous_damping[i] = gain_d[i] * actual_joint_vel[i+6]; 
        

        joint[8].torque =  r_tau[0] - r_tau_viscous_damping[2] + A_R.Tau(2);
        joint[9].torque =  r_tau[1] - r_tau_viscous_damping[3] + A_R.Tau(3);
        joint[10].torque = r_tau[2] - r_tau_viscous_damping[4] + A_R.Tau(4);

        // for (uint8_t i = 0; i<12; i ++) joint[i].torque = 0;
        cnt++;  

    }

    void Ostrich_simple::Calculate_CoM(){

        for (uint8_t i = 0; i<12; i ++) th[i] = actual_joint_pos[i];
        
        A0 << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

        L_A1 <<  0, -1, 0, 0,
                1, 0, 0, L8,
                0, 0, 1, 0,
                0, 0, 0, 1;
        
        L_A2 <<  cos(th[0]),     0,     sin(th[0]),      0,
                sin(th[0]),     0,    -cos(th[0]),      0,
                        0,     1,              0,    -L9,
                        0,     0,              0,      1;
        
        L_A3 <<  -sin(th[1]),    0,       cos(th[1]),    0,
                cos(th[1]),    0,       sin(th[1]),    0,
                            0,    1,                0,    0,
                            0,    0,                0,    1;

        L_A4 <<            1,    0,                0,    0,
                            0,    1,                0,    0,
                            0,    0,                1,  L10,
                            0,    0,                0,    1;

        L_A5 <<   cos(th[2]), -sin(th[2]),  0,   -L11*cos(th[2]),
                sin(th[2]),  cos(th[2]),  0,   -L11*sin(th[2]),
                            0,           0,  1,                 0,
                            0,           0,  0,                 1;

        L_A6 <<   cos(th[3]), -sin(th[3]),  0,   -L13*cos(th[3]),
                sin(th[3]),  cos(th[3]),  0,   -L13*sin(th[3]),
                            0,          0,  1,              -L12,
                            0,          0,  0,                 1;     

        L_A7 <<   cos(th[4]),   0,    -sin(th[4]),        0,
                sin(th[4]),   0,     cos(th[4]),        0,
                            0,  -1,              0,        0,
                            0,   0,              0,        1;

        L_A8 <<   1,0,0,0,
                0,1,0,0,
                0,0,1,L14,
                0,0,0,1;

        L_A9 <<   cos(th[5]),   0,     -sin(th[5]),       -L15*cos(th[5]),
                sin(th[5]),   0,      cos(th[5]),       -L15*sin(th[5]),
                            0,  -1,               0,       0,
                            0,   0,               0,       1;

        L_A10 <<  0,0,1,0,
                -1,0,0,0,
                0,-1,0,0,
                0,0,0,1;

        R_A1 <<  0, -1, 0, 0,
                1, 0, 0, -L8,
                0, 0, 1, 0,
                0, 0, 0, 1;
        
        R_A2 <<  cos(th[6]),     0,     sin(th[6]),      0,
                sin(th[6]),     0,    -cos(th[6]),      0,
                        0,     1,              0,    -L9,
                        0,     0,              0,      1;
        
        R_A3 <<  -sin(th[7]),    0,       cos(th[7]),    0,
                cos(th[7]),    0,         sin(th[7]),    0,
                            0,    1,                0,    0,
                            0,    0,                0,    1;

        R_A4 <<            1,    0,                0,    0,
                            0,    1,                0,    0,
                            0,    0,                1, -L10,
                            0,    0,                0,    1;

        R_A5 <<   cos(th[8]), -sin(th[8]),  0,   -L11*cos(th[8]),
                sin(th[8]),  cos(th[8]),  0,   -L11*sin(th[8]),
                            0,           0,  1,                 0,
                            0,           0,  0,                 1;

        R_A6 <<   cos(th[9]), -sin(th[9]),  0,   -L13*cos(th[9]),
                sin(th[9]),  cos(th[9]),  0,   -L13*sin(th[9]),
                            0,          0,  1,               L12,
                            0,          0,  0,                 1;     

        R_A7 <<   cos(th[10]),   0,    -sin(th[10]),        0,
                sin(th[10]),   0,     cos(th[10]),        0,
                            0,  -1,              0,        0,
                            0,   0,              0,        1;

        R_A8 << 1,0,0,0,
                0,1,0,0,
                0,0,1,L14,
                0,0,0,1;

        R_A9 <<   cos(th[11]),   0,     -sin(th[11]),       -L15*cos(th[11]),
                sin(th[11]),   0,      cos(th[11]),       -L15*sin(th[11]),
                            0,  -1,               0,       0,
                            0,   0,               0,       1;

        R_A10 <<  0,0,1,0,
                -1,0,0,0,
                0,-1,0,0,
                0,0,0,1;

        L_T00 = A0 * L_A1;
        L_T01 = L_T00 * L_A2;
        L_T02 = L_T01 * L_A3;   
        L_T03 = L_T02 * L_A4 * L_A5;
        L_T04 = L_T03 * L_A6;   
        L_T05 = L_T04 * L_A7;   
        L_T06 = L_T05 * L_A8 * L_A9 * L_A10;   

        R_T00 = A0 * R_A1;
        R_T01 = R_T00 * R_A2;
        R_T02 = R_T01 * R_A3 ;   
        R_T03 = R_T02 * R_A4 * R_A5;   
        R_T04 = R_T03 * R_A6;   
        R_T05 = R_T04 * R_A7;   
        R_T06 = R_T05 * R_A8 * R_A9 * R_A10;   

    
        L_T00_rot = L_T00.block<3,3>(0,0);
        L_T01_rot = L_T01.block<3,3>(0,0);
        L_T02_rot = L_T02.block<3,3>(0,0);
        L_T03_rot = L_T03.block<3,3>(0,0);
        L_T04_rot = L_T04.block<3,3>(0,0);
        L_T05_rot = L_T05.block<3,3>(0,0);
        L_T06_rot = L_T06.block<3,3>(0,0);

    
        R_T00_rot = R_T00.block<3,3>(0,0);
        R_T01_rot = R_T01.block<3,3>(0,0);
        R_T02_rot = R_T02.block<3,3>(0,0);
        R_T03_rot = R_T03.block<3,3>(0,0);
        R_T04_rot = R_T04.block<3,3>(0,0);
        R_T05_rot = R_T05.block<3,3>(0,0);
        R_T06_rot = R_T06.block<3,3>(0,0);

        L_T00_pos = L_T00.block<3,1>(0,3);
        L_T01_pos = L_T01.block<3,1>(0,3);
        L_T02_pos = L_T02.block<3,1>(0,3);
        L_T03_pos = L_T03.block<3,1>(0,3);
        L_T04_pos = L_T04.block<3,1>(0,3);
        L_T05_pos = L_T05.block<3,1>(0,3);

        R_T00_pos = R_T00.block<3,1>(0,3);
        R_T01_pos = R_T01.block<3,1>(0,3);
        R_T02_pos = R_T02.block<3,1>(0,3);
        R_T03_pos = R_T03.block<3,1>(0,3);
        R_T04_pos = R_T04.block<3,1>(0,3);
        R_T05_pos = R_T05.block<3,1>(0,3);

        // L_Pos_HIP_Y_CoM << 0, 0.0508, -0.0632;   //  0.0508 0 -0.0632
        L_Pos_HIP_Y_CoM << 0, -0.0508, -0.0632;   //  0.0508 0 -0.0632
        L_Pos_HIP_Y_CoM = L_T00_pos + L_T00_rot * L_Pos_HIP_Y_CoM;
        // L_Pos_HIP_Y_CoM = L_T00_pos + L_T01_rot * L_Pos_HIP_Y_CoM;

        // L_Pos_HIP_R_CoM << -0.0004 , 0.0007, 0.0814;   //  0.0007 0.0814 -0.0004 
        L_Pos_HIP_R_CoM << 0.0007 , -0.0004, -0.0814;   //  0.0007 0.0814 -0.0004 
        L_Pos_HIP_R_CoM = L_T01_pos + L_T01_rot * L_Pos_HIP_R_CoM;
        // L_Pos_HIP_R_CoM = L_T01_pos + L_T02_rot * L_Pos_HIP_R_CoM;

        // L_Pos_HIP_P_CoM << 0.0626, -0.227, -0.0282;   //  -0.0282 0.0626 -0.227
        L_Pos_HIP_P_CoM << -0.227, -0.0282, 0.0626;   //  -0.0282 0.0626 -0.227
        L_Pos_HIP_P_CoM = L_T02_pos + L_T02_rot * L_Pos_HIP_P_CoM;
        // L_Pos_HIP_P_CoM = L_T02_pos + L_T03_rot * L_Pos_HIP_P_CoM;

        // L_Pos_KNEE_P_CoM << -0.0536, -0.0265, -0.0001;  //  -0.0001 -0.0536 -0.0265
        L_Pos_KNEE_P_CoM << -0.0265, -0.0001, -0.0536;  //  -0.0001 -0.0536 -0.0265
        L_Pos_KNEE_P_CoM = L_T03_pos + L_T03_rot * L_Pos_KNEE_P_CoM;
        // L_Pos_KNEE_P_CoM = L_T03_pos + L_T04_rot * L_Pos_KNEE_P_CoM;

        // L_Pos_ANKLE_P_CoM << 0.0117, 0.00097427, 0.0018477; //  0.0018477 0.0117 0.00097427
        L_Pos_ANKLE_P_CoM << 0.00097427, 0.0018477, 0.0117; //  0.0018477 0.0117 0.00097427
        L_Pos_ANKLE_P_CoM = L_T04_pos + L_T04_rot * L_Pos_ANKLE_P_CoM;
        // L_Pos_ANKLE_P_CoM = L_T04_pos + L_T05_rot * L_Pos_ANKLE_P_CoM;

        // L_Pos_ANKLE_R_CoM << -0.0159, 0.0425, 0.0095; //  0.0425 0.0095 -0.0159
        L_Pos_ANKLE_R_CoM << 0.0095, -0.0159, 0.0425; //  0.0425 0.0095 -0.0159
        L_Pos_ANKLE_R_CoM = L_T05_pos + L_T05_rot * L_Pos_ANKLE_R_CoM;
        // L_Pos_ANKLE_R_CoM = L_T05_pos + L_T06_rot * L_Pos_ANKLE_R_CoM;

        // R_Pos_HIP_Y_CoM << 0, 0.0507, -0.0632;   //0.0507 0 -0.0632
        R_Pos_HIP_Y_CoM << 0, -0.0507, -0.0632;   //0.0507 0 -0.0632
        R_Pos_HIP_Y_CoM = R_T00_pos + R_T00_rot * R_Pos_HIP_Y_CoM;
        // R_Pos_HIP_Y_CoM = R_T00_pos + R_T01_rot * R_Pos_HIP_Y_CoM;

        // R_Pos_HIP_R_CoM << -0.0003, 0.0007, -0.0814;  // 0.0007 -0.0814 -0.0003
        R_Pos_HIP_R_CoM << 0.0007 , -0.0004, 0.0814;  // 0.0007 -0.0814 -0.0003
        R_Pos_HIP_R_CoM = R_T01_pos + R_T01_rot * R_Pos_HIP_R_CoM;
        // R_Pos_HIP_R_CoM = R_T01_pos + R_T02_rot * R_Pos_HIP_R_CoM;

        // R_Pos_HIP_P_CoM << -0.0626, -0.227, -0.0228;  //-0.0228 -0.0626 -0.227
        R_Pos_HIP_P_CoM << -0.227, -0.0282, -0.0626;  //-0.0228 -0.0626 -0.227
        R_Pos_HIP_P_CoM = R_T02_pos + R_T02_rot * R_Pos_HIP_P_CoM;
        // R_Pos_HIP_P_CoM = R_T02_pos + R_T03_rot * R_Pos_HIP_P_CoM;

        // R_Pos_KNEE_P_CoM << 0.0536, -0.0265, -0.0265; //-0.0001 0.0536 -0.0265
        R_Pos_KNEE_P_CoM << -0.0265, -0.0001, 0.0536; //-0.0001 0.0536 -0.0265
        R_Pos_KNEE_P_CoM = R_T03_pos + R_T03_rot * R_Pos_KNEE_P_CoM;
        // R_Pos_KNEE_P_CoM = R_T03_pos + R_T04_rot * R_Pos_KNEE_P_CoM;

        // R_Pos_ANKLE_P_CoM << -0.01173, 0.00095457, 0.0018003; //0.0018003 -0.01173 0.00095457
        R_Pos_ANKLE_P_CoM << 0.00097427, 0.0018477, -0.0117; //0.0018003 -0.01173 0.00095457
        R_Pos_ANKLE_P_CoM = R_T04_pos + R_T04_rot * R_Pos_ANKLE_P_CoM;
        // R_Pos_ANKLE_P_CoM = R_T04_pos + R_T05_rot * R_Pos_ANKLE_P_CoM;

        // R_Pos_ANKLE_R_CoM << -0.0159, 0.0425, -0.0095; //0.0425 -0.0095 -0.0159
        R_Pos_ANKLE_R_CoM << -0.0095, -0.0159, 0.0425; //0.0425 -0.0095 -0.0159
        R_Pos_ANKLE_R_CoM = R_T05_pos + R_T05_rot * R_Pos_ANKLE_R_CoM;
        // R_Pos_ANKLE_R_CoM = R_T05_pos + R_T06_rot * R_Pos_ANKLE_R_CoM;

        Base_Pos_CoM << 0.15633 ,0 ,-0.028929; // 0.15633 0 -0.028929

        Robot_Pos_CoM[0] = (Base_Pos_CoM[0] * M_base
                            + L_Pos_HIP_Y_CoM[0] * M_L_HIP_Y
                            + L_Pos_HIP_R_CoM[0] * M_L_HIP_R
                            + L_Pos_HIP_P_CoM[0] * M_L_HIP_P
                            + L_Pos_KNEE_P_CoM[0] * M_L_KNEE_P
                            + L_Pos_ANKLE_P_CoM[0] * M_L_ANKLE_P
                            + L_Pos_ANKLE_R_CoM[0] * M_L_ANKLE_R
                            + R_Pos_HIP_Y_CoM[0] * M_R_HIP_Y
                            + R_Pos_HIP_R_CoM[0] * M_R_HIP_R
                            + R_Pos_HIP_P_CoM[0] * M_R_HIP_P
                            + R_Pos_KNEE_P_CoM[0] * M_R_KNEE_P
                            + R_Pos_ANKLE_P_CoM[0] * M_R_ANKLE_P
                            + R_Pos_ANKLE_R_CoM[0] * M_R_ANKLE_R) / M_Robot;

        Robot_Pos_CoM[1] = (Base_Pos_CoM[1] * M_base
                            + L_Pos_HIP_Y_CoM[1] * M_L_HIP_Y
                            + L_Pos_HIP_R_CoM[1] * M_L_HIP_R
                            + L_Pos_HIP_P_CoM[1] * M_L_HIP_P
                            + L_Pos_KNEE_P_CoM[1] * M_L_KNEE_P
                            + L_Pos_ANKLE_P_CoM[1] * M_L_ANKLE_P
                            + L_Pos_ANKLE_R_CoM[1] * M_L_ANKLE_R
                            + R_Pos_HIP_Y_CoM[1] * M_R_HIP_Y
                            + R_Pos_HIP_R_CoM[1] * M_R_HIP_R
                            + R_Pos_HIP_P_CoM[1] * M_R_HIP_P
                            + R_Pos_KNEE_P_CoM[1] * M_R_KNEE_P
                            + R_Pos_ANKLE_P_CoM[1] * M_R_ANKLE_P
                            + R_Pos_ANKLE_R_CoM[1] * M_R_ANKLE_R) / M_Robot;

        Robot_Pos_CoM[2] = (Base_Pos_CoM[2] * M_base
                            + L_Pos_HIP_Y_CoM[2] * M_L_HIP_Y
                            + L_Pos_HIP_R_CoM[2] * M_L_HIP_R
                            + L_Pos_HIP_P_CoM[2] * M_L_HIP_P
                            + L_Pos_KNEE_P_CoM[2] * M_L_KNEE_P
                            + L_Pos_ANKLE_P_CoM[2] * M_L_ANKLE_P
                            + L_Pos_ANKLE_R_CoM[2] * M_L_ANKLE_R
                            + R_Pos_HIP_Y_CoM[2] * M_R_HIP_Y
                            + R_Pos_HIP_R_CoM[2] * M_R_HIP_R
                            + R_Pos_HIP_P_CoM[2] * M_R_HIP_P
                            + R_Pos_KNEE_P_CoM[2] * M_R_KNEE_P
                            + R_Pos_ANKLE_P_CoM[2] * M_R_ANKLE_P
                            + R_Pos_ANKLE_R_CoM[2] * M_R_ANKLE_R) / M_Robot;
    }

    void Ostrich_simple::Gravity_Compensation_air()
    {
        step_time = 1; //주기설정 (초) 변수
        cnt_time = cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
        
        if(cnt == 1){
            for(int i = 0; i < 6; i++){
            cout<<"A_L.Q("<<i<<") : "<<A_L.Q(i)*rad2deg<<endl;
            }
            for(int i = 0; i < 6; i++){
            cout<<"A_R.Q("<<i<<") : "<<A_R.Q(i)*rad2deg<<endl;
            }
        }
        
        NonlinearEffects(*A_L.rbdl_model, A_L.Q, A_L.QDot, A_L.Tau, NULL);
        NonlinearEffects(*A_R.rbdl_model, A_R.Q, A_R.QDot, A_R.Tau, NULL);

        
            joint[2].torque = A_L.Tau(2);
            joint[3].torque = A_L.Tau(3);
            joint[4].torque = A_L.Tau(4);
            

            joint[8].torque = A_R.Tau(3);
            joint[9].torque = A_R.Tau(4);
            joint[10].torque = A_R.Tau(5);

        cnt++;
    }

    

}