#include "Variable.hpp"
#include "My_hal.hpp"

uint32_t Send_ms;

//存储发送的数据
Motor_send_data_t msd_6020;
Motor_send_data_t msd_3508_2006;

//设置电机数量，ID号
Motor_t _Motor2006_[_Motor2006_SIZE] = { 0 };uint8_t _Motor2006_ID_[_Motor2006_SIZE] = { 0 };
Motor_t _Motor3508_[_Motor3508_SIZE] = { 0 };uint8_t _Motor3508_ID_[_Motor3508_SIZE] = { 1, 2, 3, 4 };
Motor_t _Motor6020_[_Motor6020_SIZE] = { 0 };uint8_t _Motor6020_ID_[_Motor6020_SIZE] = { 1, 2, 3, 4 };
Dji_Motor Motor2006(0x200, _Motor2006_SIZE, _Motor2006_, _Motor2006_ID_);
Dji_Motor Motor3508(0x200, _Motor3508_SIZE, _Motor3508_, _Motor3508_ID_);
Dji_Motor Motor6020(0x204, _Motor6020_SIZE, _Motor6020_, _Motor6020_ID_);

//PID角度环设置
Kpid_t Kpid_6020_angle(0.15, 0, 0);
PID pid_angle_0x205;
PID pid_angle_0x206;
PID pid_angle_0x207;
PID pid_angle_0x208;
// PID角度环设置
Kpid_t Kpid_6020_vel(80, 0, 0);
PID pid_vel_0x205;
PID pid_vel_0x206;
PID pid_vel_0x207;
PID pid_vel_0x208;

// PID速度环设置
Kpid_t Kpid_3508_vel(5, 0, 0);
PID pid_vel_0x201;
PID pid_vel_0x202;
PID pid_vel_0x203;
PID pid_vel_0x204;

//尖括号里填底盘类型
Wheel_t<SG> Wheel;

//6020速度滤波
TD td_6020_1(100);
TD td_6020_2(100);
TD td_6020_3(100);
TD td_6020_4(100);

//3508速度滤波
TD td_3508_1(100);
TD td_3508_2(100);
TD td_3508_3(100);
TD td_3508_4(100);

//期望值滤波
TD tar_vw(10);
TD tar_vx(40);
TD tar_vy(40);
TD td_zero(80);

//前馈
FeedTar feed_6020_1(200, 10);
FeedTar feed_6020_2(200, 10);
FeedTar feed_6020_3(200, 10);
FeedTar feed_6020_4(200, 10);

// 创建遥控器实例
RM_Clicker dr16;

//创建底盘任务实例
Chassis_task chassis_task;

// 创建工具实例
Tools_t Tools;

// 创建底盘变量实例
Chassis_Data_t Chassis_Data;
