#include "rmd_motor.h"

CAN_msg rmd_motor::ref_msg;

rmd_motor::rmd_motor()
{
    
}

void rmd_motor::UpdateRxData(void) 
{
    joint_temperature = (int)(torque_data[1]);
    
    int temp_torque = (int)(torque_data[2] | (torque_data[3]<<8));
    joint_torque = temp_torque / torque_to_data;
    
    int temp_speed = (int)(torque_data[4] | (torque_data[5]<<8));
    if(temp_speed > 30000) temp_speed -= 65535;
    joint_velocity = 0.01 * temp_speed * actuator_direction;

    int temp_encoder = (int)(torque_data[6] | (torque_data[7]<<8));
    float temp_theta = temp_encoder / data_to_radian;
    float incremental_theta{0};
    if(initialize_position){
        joint_theta = joint_initial_position;
        motor_theta_last = temp_theta;
        initialize_position = false;
    }
    else{
        incremental_theta = temp_theta - motor_theta_last;
        motor_theta_last = temp_theta;
    }    
    if (incremental_theta > 4) incremental_theta -= 6.28319;
    else if (incremental_theta < -4) incremental_theta += 6.28319;
    joint_theta += incremental_theta / actuator_gear_ratio * actuator_direction;
}

void rmd_motor::SetRefDataFFTorqueMode(float p_des, float v_des, float t_ff, float kp, float kd)
{
    long p_des_value = 0;
    long v_des_value;
    long kp_value;
    long kd_value;
    long t_ff_value;

    ref_msg.id = (0x400 + mc_id);

    ref_msg.data[0] = (p_des_value     ) & 0xFF;
    ref_msg.data[1] = (p_des_value >> 8) & 0xFF;
    ref_msg.data[2] = 0x00 & 0xFF;
    ref_msg.data[3] = 0x00 & 0xFF;
    ref_msg.data[4] = 0x00 & 0xFF;
    ref_msg.data[5] = 0x00 & 0xFF;
    ref_msg.data[6] = 0x00 & 0xFF;
    ref_msg.data[7] = 0x00 & 0xFF;
}

void rmd_motor::SetRefDataFeedForwardTorqueControlMode(float a_torque_des)
{
    if (a_torque_des > actuator_torque_limit) a_torque_des = actuator_torque_limit;
    else if (a_torque_des < -1 * actuator_torque_limit) a_torque_des = -1 * actuator_torque_limit;

    long torque_des_value = a_torque_des * torque_to_data * actuator_direction;

    ref_msg.id = 0x140 + mc_id;

    ref_msg.data[0] = 0x73;
    ref_msg.data[1] = 0x00 & 0xFF;
    ref_msg.data[2] = (torque_des_value     ) & 0xFF;
    ref_msg.data[3] = (torque_des_value >> 8) & 0xFF;
    ref_msg.data[4] = 0x00 & 0xFF;
    ref_msg.data[5] = 0x00 & 0xFF;
    ref_msg.data[6] = 0x00 & 0xFF;
    ref_msg.data[7] = 0x00 & 0xFF;
}

void rmd_motor::SetRefDataTorqueControlMode(float a_torque_des)
{
    if (a_torque_des > actuator_torque_limit) a_torque_des = actuator_torque_limit;
    else if (a_torque_des < -1 * actuator_torque_limit) a_torque_des = -1 * actuator_torque_limit;

    long torque_des_value = a_torque_des * torque_to_data * actuator_direction;

    ref_msg.id = 0x140 + mc_id;

    ref_msg.data[0] = 0xA1;
    ref_msg.data[1] = 0x00 & 0xFF;
    ref_msg.data[2] = 0x00 & 0xFF;
    ref_msg.data[3] = 0x00 & 0xFF;    
    ref_msg.data[4] = (torque_des_value     ) & 0xFF;
    ref_msg.data[5] = (torque_des_value >> 8) & 0xFF;
    ref_msg.data[6] = 0x00 & 0xFF;
    ref_msg.data[7] = 0x00 & 0xFF;
}

void rmd_motor::SetEnableMotor()
{
    ref_msg.dlc = 8;
    ref_msg.id = 0x140 + mc_id;

    ref_msg.data[0] = 0x88;
    ref_msg.data[1] = 0x00 & 0xFF;
    ref_msg.data[2] = 0x00 & 0xFF;
    ref_msg.data[3] = 0x00 & 0xFF;
    ref_msg.data[4] = 0x00 & 0xFF;
    ref_msg.data[5] = 0x00 & 0xFF;
    ref_msg.data[6] = 0x00 & 0xFF;
    ref_msg.data[7] = 0x00 & 0xFF;

    is_comm_enabled = true;
}

void rmd_motor::SetEnableFilter()
{
    ref_msg.id = (0x140 + mc_id);

    ref_msg.data[0] = 0x20 & 0xFF;
    ref_msg.data[1] = 0x02 & 0xFF;
    ref_msg.data[2] = 0x00 & 0xFF;
    ref_msg.data[3] = 0x00 & 0xFF;
    ref_msg.data[4] = 0x01 & 0xFF;
    ref_msg.data[5] = 0x00 & 0xFF;
    ref_msg.data[6] = 0x00 & 0xFF;
    ref_msg.data[7] = 0x00 & 0xFF;
}

void rmd_motor::SetCanShieldChannel(int a_channel_num)
{
    switch(a_channel_num)
    {
        case 0:                         // CHANNEL A
            can_shield_channel = 0;
            ref_msg.header = 0x89;
            break;
        case 1:                         // CHANNEL B
            can_shield_channel = 1;
            ref_msg.header = 0x77;
            break;
        case 2:                         // CHANNEL C
            can_shield_channel = 2;
            ref_msg.header = 0x89;
            break;
        case 3:                         // CHANNEL D
            can_shield_channel = 3;
            ref_msg.header = 0x77;
            break;
        default:
            can_shield_channel = 0;
            ref_msg.header = 0x89;
    }
}

void rmd_motor::SetMotorControllerId(int a_id_num)
{
    mc_id = a_id_num;
}

float rmd_motor::GetTheta() 
{
    return joint_theta;
}

// V3 0x92
float rmd_motor::GetThetaV3() 
{
    int temp_enc = (int)(enc_data[4] | (enc_data[5]<<8) | (enc_data[6]<<16) | (enc_data[7]<<24));
    float temp_theta = temp_enc * 0.0174533 * 0.01 * actuator_direction;
    if(initialize_position)
    {
        joint_theta_offset_92 = temp_theta - joint_initial_position; 
        initialize_position = false;
    }
    joint_theta_92 = temp_theta - joint_theta_offset_92;
    return joint_theta_92;
}


float rmd_motor::GetThetaDot() 
{
    return joint_velocity;
}


float rmd_motor::GetTorque()
{
    return joint_torque;
}


