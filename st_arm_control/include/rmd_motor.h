#ifndef RMD_MOTOR_H
#define RMD_MOTOR_H

#include <linux/types.h>
#include <math.h>
// #include "rmd_can.h"
#include "rt_utils.h"


typedef struct{
    unsigned char header;
    unsigned char dlc;
    unsigned short id;
    unsigned char data[8];
}CAN_msg;



class rmd_motor
{
public:
    rmd_motor();

    static CAN_msg  ref_msg;

    unsigned char ref_data[8];
    unsigned char ref_data2[8];
    unsigned char enc_data[8];
    unsigned char torque_data[8];

    int     count;
    int     count_92;
    int     count_A1;
    int     unknown_value;

    int     actuator_gear_ratio;
    int     actuator_direction;
    float   actuator_torque_limit;
    float   joint_initial_position;
    bool    initialize_position = true;
    float   torque_to_data;
    float   torque_to_current;
    float   data_to_radian;
    bool    is_comm_enabled;

    void    UpdateRxData(void);
    void    SetTorqueData(void);
    float   GetTheta();
    float   GetThetaV3();
    float   GetThetaDot();
    float   GetTorque();

    void    SetCanShieldChannel(int);
    void    SetMotorControllerId(int);
    void    SetRefDataFFTorqueMode(float p_des, float v_des, float t_ff, float kp, float kd);
    void    SetRefDataFeedForwardTorqueControlMode(float reference_torque);
    void    SetRefDataTorqueControlMode(float reference_torque);
    void    SetEnableMotor();
    void    SetDisableMotor();
    void    SetEnableFilter();

    int     mc_id;
    int     can_shield_channel;


private:
    float   joint_velocity;
    float   joint_theta;
    float   joint_torque;
    float   joint_temperature;

    float   motor_theta_last;

    float   joint_theta_92;
    float   joint_theta_offset_92;


};

#endif // RMD_MOTOR_H
