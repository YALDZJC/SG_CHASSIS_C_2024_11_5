#include "Dji_Motor.hpp"
#include "dr16.hpp"
#include "PID.hpp"
#include "Wheel.hpp"
#include "Dm_Motor.hpp"
#include "State.hpp"
#include "Tools.hpp"

// 发送id
#define SEND_MOTOR_ID_2006 (0x200)
#define SEND_MOTOR_ID_3508 (0x200)
#define SEND_MOTOR_ID_6020 (0x1FF)

//获取设置id
#define Get_MOTOR_SET_ID_2006(x) (x - 0x200)
#define Get_MOTOR_SET_ID_3508(x) (x - 0x200)
#define Get_MOTOR_SET_ID_6020(x) (x - 0x204)

//获取设置stdid
#define Get_MOTOR_SET_STDID_2006(x) (x + 0x200)
#define Get_MOTOR_SET_STDID_3508(x) (x + 0x200)
#define Get_MOTOR_SET_STDID_6020(x) (x + 0x204)

//数量
#define _Motor2006_SIZE 1
#define _Motor3508_SIZE 1
#define _Motor6020_SIZE 4
#define _DM4310_SIZE 1

// ID号
#define L_Forward_ID 0x205
#define L_Back_ID    0x206
#define R_Back_ID    0x207
#define R_Forward_ID 0x208

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

extern TD td_6020_1;
extern TD td_6020_2;
extern TD td_6020_3;
extern TD td_6020_4;

extern TD td_3508_1;
extern TD td_3508_2;
extern TD td_3508_3;
extern TD td_3508_4;

extern TD tar_vw;
extern TD tar_vx;
extern TD tar_vy;
extern TD td_zero;
extern RM_Clicker dr16;

extern Wheel_t<SG> Wheel;

extern Chassis_task chassis_task;
extern Tools_t Tools;
extern Chassis_Data_t Chassis_Data;



