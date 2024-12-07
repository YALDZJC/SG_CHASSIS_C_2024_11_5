#include "Dji_Motor.hpp"
#include "dr16.hpp"
#include "PID.hpp"
#include "Wheel.hpp"
#include "Dm_Motor.hpp"
#include "State.hpp"
#include "Tools.hpp"
#include "PM01.hpp"
#include "PowerTask.hpp"

// 发送id
#define SEND_MOTOR_ID_2006 (0x200)
#define SEND_MOTOR_ID_3508 (0x200)
#define SEND_MOTOR_ID_6020 (0x1FF)
#define SEND_MOTOR_CurrentID_6020 (0x1FE)

//获取设置id
#define Get_MOTOR_SET_ID_2006(x) (x - 0x200)
#define Get_MOTOR_SET_ID_3508(x) (x - 0x200)
#define Get_MOTOR_SET_ID_6020(x) (x - 0x204)

//获取设置stdid
#define Get_MOTOR_SET_STDID_2006(x) (x + 0x200)
#define Get_MOTOR_SET_STDID_3508(x) (x + 0x200)
#define Get_MOTOR_SET_STDID_6020(x) (x + 0x204)

#define Get_InitID_3508(x) (x + 0x201)
#define Get_InitID_6020(x) (x + 0x205)
//数量
#define _Motor2006_SIZE 1
#define _Motor3508_SIZE 4
#define _Motor6020_SIZE 4
#define _DM4310_SIZE 1

// ID号
#define L_Forward_6020_ID   0x205
#define L_Back_6020_ID      0x206
#define R_Back_6020_ID      0x207
#define R_Forward_6020_ID   0x208

// ID号
#define L_Forward_3508_ID   0x201
#define L_Back_3508_ID      0x202
#define R_Back_3508_ID      0x203
#define R_Forward_3508_ID   0x204

#define Chassis_angle_Init_0x205 7344
#define Chassis_angle_Init_0x206 5424 + 4096
#define Chassis_angle_Init_0x207 3141
#define Chassis_angle_Init_0x208 5316 + 4096

extern uint32_t Send_ms;

extern Motor_send_data_t msd_6020;
extern Motor_send_data_t msd_3508_2006;

extern Dji_Motor Motor2006;
extern Dji_Motor Motor3508;
extern Dji_Motor Motor6020;
extern DM_Motor  Motor4310;

// PID角度环设置
extern Kpid_t Kpid_6020_angle;
extern PID pid_angle_0x205;
extern PID pid_angle_0x206;
extern PID pid_angle_0x207;
extern PID pid_angle_0x208;
extern Kpid_t Kpid_6020_vel;
extern PID pid_vel_0x205;
extern PID pid_vel_0x206;
extern PID pid_vel_0x207;
extern PID pid_vel_0x208;

// PID速度环设置
extern Kpid_t Kpid_3508_vel;
extern PID pid_vel_0x201;
extern PID pid_vel_0x202;
extern PID pid_vel_0x203;
extern PID pid_vel_0x204;

// 力矩控制
extern Kpid_t Kpid_6020_T;
extern PID pid_T_0x207;
extern Kpid_t Kpid_3508_T;
extern PID pid_T_0x201;

extern TD td6020_torque;
extern TD td3508_torque;

extern TD td_6020_1;
extern TD td_6020_2;
extern TD td_6020_3;
extern TD td_6020_4;
extern TD td_6020_angle_1;

extern TD td_3508_1;
extern TD td_3508_2;
extern TD td_3508_3;
extern TD td_3508_4;

extern TD tar_vw;
extern TD tar_vx;
extern TD tar_vy;
extern TD td_FF_Tar;

// 前馈
extern FeedTar feed_6020_1;
extern FeedTar feed_6020_2;
extern FeedTar feed_6020_3;
extern FeedTar feed_6020_4;

extern RM_Clicker dr16;

extern Wheel_t<SG> Wheel;

extern Chassis_task chassis_task;
extern Tools_t Tools;
extern Chassis_Data_t Chassis_Data;

extern PM01 pm01;

extern SGPowerControl::PowerTask_t PowerControl;
