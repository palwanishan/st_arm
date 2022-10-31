#include "dynamics.h"

extern Motor_Controller motor_ctrl;

namespace Dynamics
{
    JMDynamics::JMDynamics(){}


    void JMDynamics::Loop()
    {
        SetTheta(motor_ctrl.GetJointTheta());
        SetThetaDotSMAF(motor_ctrl.GetThetaDotSMAF());
        PostureGeneration();
        motor_ctrl.SetTorque(GetTorque());
    }


    void JMDynamics::PostureGeneration()
    {
        switch(control_mode)
        {
            case gravity_compensation:
                GenerateTorqueGravityCompensation();
                break;
            case manipulation_mode:
                GenerateTorqueManipulationMode();
                break;
            case vision_mode:
                GenerateTorqueVisionMode();
                break;
            case draw_infinity:
                GenerateTrajectory();
                GenerateTorqueManipulationMode();
                break;
            default:
                GenerateTorqueGravityCompensation();
        }
    }


    void JMDynamics::SwitchMode(const std_msgs::Int32ConstPtr & msg)
    {
        cnt = 0;
        if      (msg -> data == 0) control_mode = gravity_compensation;
        else if (msg -> data == 1) control_mode = manipulation_mode;
        else if (msg -> data == 2) control_mode = vision_mode;  
        else if (msg -> data == 3) control_mode = draw_infinity;  
        else                       control_mode = gravity_compensation;    
    }


    void JMDynamics::SetTheta(VectorXd a_theta)
    {
        for(uint8_t i=0; i<7; i++) th[i] = a_theta[i];
    }


    void JMDynamics::SetThetaDot(VectorXd a_theta_dot)
    {
        for(uint8_t i=0; i<6; i++) th_dot[i] = a_theta_dot[i]; 
    }


    void JMDynamics::SetThetaDotSMAF(VectorXd a_theta_dot)
    {
        for(uint8_t i=0; i<7; i++) th_dot_sma_filtered[i] = a_theta_dot[i];
    }


    void JMDynamics::SetThetaDotEst(VectorXd a_theta_dot_est)
    {
        for(uint8_t i=0; i<4; i++) th_dot_estimated[i+3] = a_theta_dot_est[i];
    }


    void JMDynamics::SetGripperValue(float a_desired_gripper_theta)
    {
        ref_th[6] = a_desired_gripper_theta;
    }


    VectorXd JMDynamics::GetTorque()
    {   
        return joint_torque;
    }


    void JMDynamics::GenerateTorqueTaskSpacePD(){
        // gain_p << 0, 0, 0;
        gain_p = gain_p_task_space;
        gain_w = gain_w_task_space;
        // gain_d << 5, 5, 5;
        // gain_w << 1, 1, 1;

        // cnt_time = cnt*inner_dt;   

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

        ee_force(0) = gain_p(0) * (om_ee_position(0) - ee_position(0)); 
        ee_force(1) = gain_p(1) * (om_ee_position(1) - ee_position(1)); 
        ee_force(2) = gain_p(2) * (om_ee_position(2) - ee_position(2)); 

        ee_rotation = T06.block<3,3>(0,0);
        ee_rotation_x = ee_rotation.block<3,1>(0,0); 
        ee_rotation_y = ee_rotation.block<3,1>(0,1); 
        ee_rotation_z = ee_rotation.block<3,1>(0,2);

        ref_ee_rotation_x = om_ee_rotation.block<3,1>(0,0); 
        ref_ee_rotation_y = om_ee_rotation.block<3,1>(0,1); 
        ref_ee_rotation_z = om_ee_rotation.block<3,1>(0,2);

        ee_orientation_error  = ee_rotation_x.cross(ref_ee_rotation_x) 
                              + ee_rotation_y.cross(ref_ee_rotation_y) 
                              + ee_rotation_z.cross(ref_ee_rotation_z);

        ee_momentum << gain_w(0) * ee_orientation_error(0), 
                        gain_w(1) * ee_orientation_error(1), 
                        gain_w(2) * ee_orientation_error(2);
        
        // tau_gravity_compensation[0] = 0;
        // tau_gravity_compensation[1] = 1.5698*sin(th[1])*sin(th[2]) - 1.5698*cos(th[1])*cos(th[2]) - 2.9226*cos(th[1]) + 0.21769*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.21769*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.056114*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.056114*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
        // tau_gravity_compensation[2] = 1.5698*sin(th[1])*sin(th[2]) - 1.5698*cos(th[1])*cos(th[2]) + 0.21769*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.21769*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.056114*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.056114*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
        // tau_gravity_compensation[3] = 0.21769*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.21769*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.056114*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.056114*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
        // tau_gravity_compensation[4] = 0.056114*cos(th[5] + 1.5708)*sin(th[4] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
        // tau_gravity_compensation[5] = 0.056114*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.056114*cos(th[4] + 1.5708)*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));

        // tau_gravity_compensation[0] = 0;
        // tau_gravity_compensation[1] = 1.5698*sin(th_joint[1])*sin(th_joint[2]) - 1.5698*cos(th_joint[1])*cos(th_joint[2]) - 2.9226*cos(th_joint[1]) + 0.21769*cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) + 0.21769*sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2])) + 0.056114*sin(th_joint[5] + 1.5708)*(cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) + sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2]))) + 0.056114*cos(th_joint[4] + 1.5708)*cos(th_joint[5] + 1.5708)*(1.0*sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) - cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2])));
        // tau_gravity_compensation[2] = 1.5698*sin(th_joint[1])*sin(th_joint[2]) - 1.5698*cos(th_joint[1])*cos(th_joint[2]) + 0.21769*cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) + 0.21769*sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2])) + 0.056114*sin(th_joint[5] + 1.5708)*(cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) + sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2]))) + 0.056114*cos(th_joint[4] + 1.5708)*cos(th_joint[5] + 1.5708)*(1.0*sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) - cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2])));
        // tau_gravity_compensation[3] = 0.21769*cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) + 0.21769*sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2])) + 0.056114*sin(th_joint[5] + 1.5708)*(cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) + sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2]))) + 0.056114*cos(th_joint[4] + 1.5708)*cos(th_joint[5] + 1.5708)*(1.0*sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) - cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2])));
        // tau_gravity_compensation[4] = 0.056114*cos(th_joint[5] + 1.5708)*sin(th_joint[4] + 1.5708)*(cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) + sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2])));
        // tau_gravity_compensation[5] = 0.056114*cos(th_joint[5] + 1.5708)*(1.0*sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) - cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2]))) + 0.056114*cos(th_joint[4] + 1.5708)*sin(th_joint[5] + 1.5708)*(cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) + sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2])));


        virtual_spring << ee_force(0), ee_force(1), ee_force(2), ee_momentum(0), ee_momentum(1), ee_momentum(2);

        joint_torque = Jacobian.transpose() * virtual_spring;// + tau_gravity_compensation;
        // joint_torque = tau_gravity_compensation;

    }


    void JMDynamics::GenerateTrajectory(){
        float count_time = count * dt;
        count++;   
        trajectory = PI * (1 - cos(PI * (count_time/step_time)));

        // hmd_position(0) = 0.3;
        // hmd_position(1) = 0 - 0.3*sin(PI/2*(count_time/step_time));
        // hmd_position(2) = 0.35 + 0.15*sin(PI*(count_time/step_time));
    }


    void JMDynamics::GenerateTorqueJointSpacePD()
    {
        ref_th << 0, 0, 0, 0, 0, 0;
        for (uint8_t i = 3; i < 6; i++) 
        {
            ref_th[i] = 1 * trajectory;
            joint_torque[i] = gain_p_joint_space[i] * (ref_th[i] - th[i]) - gain_d_joint_space[i] * th_dot_sma_filtered[i]; 
        }
    }


    void JMDynamics::CalculateRefEEPose()
    {
        om_A0 << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        om_A1 << cos(om_th[0]), 0, -sin(om_th[0]), 0,
                sin(om_th[0]), 0, cos(om_th[0]), 0,
                0, -1, 0, L1,
                0, 0, 0, 1;
        om_A2 << cos(om_th[1]), -sin(om_th[1]), 0, L2*cos(om_th[1]),
                sin(om_th[1]), cos(om_th[1]), 0, L2*sin(om_th[1]),
                0, 0, 1, 0, 
                0, 0, 0, 1;
        om_A3 << cos(om_th[2]), -sin(om_th[2]), 0, L3*cos(om_th[2]), 
                sin(om_th[2]), cos(om_th[2]), 0, L3*sin(om_th[2]), 
                0, 0, 1, 0,
                0, 0, 0, 1;
        om_A4 << sin(om_th[3]), 0, cos(om_th[3]), 0,
                -cos(om_th[3]), 0, sin(om_th[3]), 0,
                0, -1, 0, 0,
                0, 0, 0, 1;
        om_A5 << -sin(om_th[4]), 0, cos(om_th[4]), 0,
                cos(om_th[4]), 0, sin(om_th[4]), 0,
                0, 1, 0, L5,
                0, 0, 0, 1;
        om_A6 << -sin(om_th[5]), -cos(om_th[5]), 0, -L6*sin(om_th[5]),
                cos(om_th[5]), -sin(om_th[5]), 0, L6*cos(om_th[5]),
                0, 0, 1, 0, 
                0, 0, 0, 1;          
            
        om_T06 = om_A0*om_A1*om_A2*om_A3*om_A4*om_A5*om_A6;

        om_ee_position = om_T06.block<3,1>(0,3);
        om_ee_rotation = om_T06.block<3,3>(0,0);
    }


    void JMDynamics::GenerateTorqueGravityCompensation(){

        tau_gravity_compensation[0] = 0;
        tau_gravity_compensation[1] = 1.5698*sin(th_joint[1])*sin(th_joint[2]) - 1.5698*cos(th_joint[1])*cos(th_joint[2]) - 2.9226*cos(th_joint[1]) + 0.21769*cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) + 0.21769*sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2])) + 0.056114*sin(th_joint[5] + 1.5708)*(cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) + sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2]))) + 0.056114*cos(th_joint[4] + 1.5708)*cos(th_joint[5] + 1.5708)*(1.0*sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) - cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2])));
        tau_gravity_compensation[2] = 1.5698*sin(th_joint[1])*sin(th_joint[2]) - 1.5698*cos(th_joint[1])*cos(th_joint[2]) + 0.21769*cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) + 0.21769*sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2])) + 0.056114*sin(th_joint[5] + 1.5708)*(cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) + sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2]))) + 0.056114*cos(th_joint[4] + 1.5708)*cos(th_joint[5] + 1.5708)*(1.0*sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) - cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2])));
        tau_gravity_compensation[3] = 0.21769*cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) + 0.21769*sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2])) + 0.056114*sin(th_joint[5] + 1.5708)*(cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) + sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2]))) + 0.056114*cos(th_joint[4] + 1.5708)*cos(th_joint[5] + 1.5708)*(1.0*sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) - cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2])));
        tau_gravity_compensation[4] = 0.056114*cos(th_joint[5] + 1.5708)*sin(th_joint[4] + 1.5708)*(cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) + sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2])));
        tau_gravity_compensation[5] = 0.056114*cos(th_joint[5] + 1.5708)*(1.0*sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) - cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2]))) + 0.056114*cos(th_joint[4] + 1.5708)*sin(th_joint[5] + 1.5708)*(cos(th_joint[3] - 1.5708)*(cos(th_joint[1])*sin(th_joint[2]) + cos(th_joint[2])*sin(th_joint[1])) + sin(th_joint[3] - 1.5708)*(cos(th_joint[1])*cos(th_joint[2]) - 1.0*sin(th_joint[1])*sin(th_joint[2])));

        joint_torque = tau_gravity_compensation;
    }


    void JMDynamics::CalculateJointTheta(){
        joint_gear_reduction << 6, -8, 6, 1, 1, 1;
        th_incremental = th - last_th;
        last_th = th;
        for (int i = 0; i < 6; i++) {
            if (th_incremental[i] > 4) th_incremental[i] -= 6.28319;
            else if (th_incremental[i] < -4) th_incremental[i] += 6.28319;
            th_joint[i] += th_incremental[i] / joint_gear_reduction[i];
        }        
        th_dot_joint = (th_joint - last_th_joint) / 0.002;
        last_th_joint = th_joint;
    }

    //  Manipulation mode
    void JMDynamics::GenerateTorqueManipulationMode(){
        gain_p = gain_p_task_space;
        gain_d = gain_d_task_space;
        gain_w = gain_w_task_space; 
        // gain_r << 1, 1.5, 1.5, 1, 1, 1; //adjust GC intensity

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

        ee_velocity = (ee_position - pre_ee_position) * 500; //  500 = 1/0.002 
        pre_ee_position = ee_position;

        ref_ee_position = hmd_position;

        ee_force(0) = gain_p(0) * (ref_ee_position(0) - ee_position(0)) - gain_d(0) * ee_velocity(0);
        ee_force(1) = gain_p(1) * (ref_ee_position(1) - ee_position(1)) - gain_d(1) * ee_velocity(1);
        ee_force(2) = gain_p(2) * (ref_ee_position(2) - ee_position(2)) - gain_d(2) * ee_velocity(2);

        // std::cout << ee_velocity(0) << "    " << ee_velocity(1) << "    " << ee_velocity(2) << std::endl;

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

        // tau_gravity_compensation[0] = 0.0;
        // tau_gravity_compensation[1] = 1.9076*sin(th[1])*sin(th[2]) - 1.9076*cos(th[1])*cos(th[2]) - 2.7704*cos(th[1]) + 0.43376*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43376*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14068*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14068*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
        // tau_gravity_compensation[2] = 1.9076*sin(th[1])*sin(th[2]) - 1.9076*cos(th[1])*cos(th[2]) + 0.43376*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43376*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14068*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14068*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
        // tau_gravity_compensation[3] = 0.43376*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43376*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14068*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14068*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
        // tau_gravity_compensation[4] = 0.14068*cos(th[5] + 1.5708)*sin(th[4] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
        // tau_gravity_compensation[5] = 0.14068*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14068*cos(th[4] + 1.5708)*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));

        tau_gravity_compensation[0] = 0.0;
        tau_gravity_compensation[1] = 1.9318*sin(th[1])*sin(th[2]) - 1.9318*cos(th[1])*cos(th[2]) - 2.9498*cos(th[1]) + 0.43025*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43025*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14096*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14096*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
        tau_gravity_compensation[2] = 1.9318*sin(th[1])*sin(th[2]) - 1.9318*cos(th[1])*cos(th[2]) + 0.43025*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43025*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14096*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14096*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
        tau_gravity_compensation[3] = 0.43025*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43025*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14096*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14096*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
        tau_gravity_compensation[4] = 0.14096*cos(th[5] + 1.5708)*sin(th[4] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
        tau_gravity_compensation[5] = 0.14096*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14096*cos(th[4] + 1.5708)*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    
        virtual_spring << ee_force(0), ee_force(1), ee_force(2), ee_momentum(0), ee_momentum(1), ee_momentum(2);

        for(uint8_t i=0; i<6; i++) {
            // tau_viscous_damping[i] = gain_d_joint_space[i] * th_dot[i]; 
            tau_viscous_damping[i] = gain_d_joint_space[i] * th_dot_sma_filtered[i]; 
            tau_gravity_compensation[i] = tau_gravity_compensation[i] * gain_r[i];
        }

        tau = Jacobian.transpose() * virtual_spring + tau_gravity_compensation - tau_viscous_damping;
        for(int i=0; i<6; i++){
            if(th(i) > joint_limit(0,i) - threshold(i) && tau(i) > 0 || th(i) < joint_limit(1,i) + threshold(i) && tau(i) < 0)
                joint_torque[i] = tau_gravity_compensation[i] - tau_viscous_damping[i];    //we can add more damping here
            else joint_torque[i] = tau[i]; 
        }
    }

    //  Vision mode
    void JMDynamics::GenerateTorqueVisionMode()
    {
        gain_p = gain_p_task_space;
        gain_w = gain_w_task_space;

        threshold << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1; 
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

        // initial_com_position << 0.05,0,0.25;
        initial_com_position << 0.1,0,0.25;

        desired_com_position = initial_com_position;

        for(uint8_t i=0; i<3; i++) virtual_spring_com(i) = gain_p(i) * (desired_com_position(i) - manipulator_com(i));
        
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

        // tau_gravity_compensation[0] = 0.0;
        // tau_gravity_compensation[1] = 1.6535*sin(th[1])*sin(th[2]) - 1.6535*cos(th[1])*cos(th[2]) - 2.5977*cos(th[1]) + 0.34176*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.34176*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.10301*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.10301*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
        // tau_gravity_compensation[2] = 1.6535*sin(th[1])*sin(th[2]) - 1.6535*cos(th[1])*cos(th[2]) + 0.34176*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.34176*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.10301*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.10301*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
        // tau_gravity_compensation[3] = 0.34176*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.34176*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.10301*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.10301*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
        // tau_gravity_compensation[4] = 0.10301*cos(th[5] + 1.5708)*sin(th[4] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
        // tau_gravity_compensation[5] = 0.10301*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.10301*cos(th[4] + 1.5708)*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));   

        tau_gravity_compensation[0] = 0.0;
        tau_gravity_compensation[1] = 1.9318*sin(th[1])*sin(th[2]) - 1.9318*cos(th[1])*cos(th[2]) - 2.9498*cos(th[1]) + 0.43025*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43025*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14096*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14096*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
        tau_gravity_compensation[2] = 1.9318*sin(th[1])*sin(th[2]) - 1.9318*cos(th[1])*cos(th[2]) + 0.43025*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43025*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14096*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14096*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
        tau_gravity_compensation[3] = 0.43025*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43025*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14096*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14096*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
        tau_gravity_compensation[4] = 0.14096*cos(th[5] + 1.5708)*sin(th[4] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
        tau_gravity_compensation[5] = 0.14096*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14096*cos(th[4] + 1.5708)*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    
        for(uint8_t i=0; i<6; i++) {
            tau_viscous_damping[i] = gain_d_joint_space[i] * th_dot[i]; 
            tau_gravity_compensation[i] = tau_gravity_compensation[i] * gain_r[i];
        }

        tau =  tau_com + tau_rotational + tau_gravity_compensation - tau_viscous_damping;
        for(int i=0; i<6; i++){
            if(th(i) > joint_limit(0,i) - threshold(i) && tau(i) > 0 || th(i) < joint_limit(1,i) + threshold(i) && tau(i) < 0)
                joint_torque[i] = tau_gravity_compensation[i] - tau_viscous_damping[i];    //we can add more damping here
            else joint_torque[i] = tau[i]; 
        }
    }

    // a argument   // m member    // l local   //p pointer     //r reference   //
    void JMDynamics::GenerateGripperTorque()
    {
        const float maximum_value = 1;
        const float minimum_value = -1;
        const float max_torque = 1;

        float l_torque{0};

        float Kp = gain_p_joint_space[6];
        float Kd = gain_d_joint_space[6];

        float desired_torque;
        float desired_value = ref_th[6];

        if(desired_value > maximum_value) desired_value = maximum_value;
        else if(desired_value < minimum_value) desired_value = minimum_value;

        l_torque = Kp * (desired_value - th[6]) - Kd * (th_dot_sma_filtered[6]);

        if(l_torque > max_torque) l_torque = max_torque;
        else if(l_torque < -1 * max_torque) l_torque = -1 * max_torque;

        joint_torque[6] = l_torque;
    }
}