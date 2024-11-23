#include "Dji_Motor.hpp"
#include "dr16.hpp"
#include "PID.hpp"
#include "Wheel.hpp"
#include "Dm_Motor.hpp"
#include "State.hpp"

//发送id
#define SEND_MOTOR_ID_2006 (0x200)
#define SEND_MOTOR_ID_3508 (0x200)
#define SEND_MOTOR_ID_6020 (0x1FF)

// //获取设置id
// #define Get_MOTOR_SET_ID_2006(x) (x - 0x200)
// #define Get_MOTOR_SET_ID_3508(x) (x - 0x200)
// #define Get_MOTOR_SET_ID_6020(x) (x - 0x204)

// //获取设置stdid
// #define Get_MOTOR_SET_STDID_2006(x) (x + 0x200)
// #define Get_MOTOR_SET_STDID_3508(x) (x + 0x200)
// #define Get_MOTOR_SET_STDID_6020(x) (x + 0x204)

//数量
#define _Motor2006_SIZE 1
#define _Motor3508_SIZE 1
#define _Motor6020_SIZE 1
#define _DM4310_SIZE 1

// ID号
#define L_Forward_ID 0x205
#define L_Back_ID    0x206
#define R_Back_ID    0x207
#define R_Forward_ID 0x208

extern Dji_Motor Motor2006;
extern Dji_Motor Motor3508;
extern Dji_Motor Motor6020;
extern DM_Motor Motor4310;
//提供一个默认数据变量
extern Motor_send_data_t msd_6020;
extern Motor_send_data_t msd_3508_2006;

extern Kpid_t K_DEMO_6020;
extern PID DEMO_6020;

extern Kpid_t K_DEMO_6020_IN;
extern PID DEMO_6020_IN;

extern TD td_speed;
extern TD tar_vw;
extern TD tar_vx;
extern TD tar_vy;
extern TD td_zero;
extern RM_Clicker dr16;

extern Wheel_t<SG> Wheel;

//发送6020数据
void Send_6020_CAN();


extern Chassis_task chassis_task;

