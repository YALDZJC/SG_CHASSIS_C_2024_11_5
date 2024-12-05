#include "Variable.hpp"
#include "My_hal.hpp"
uint32_t Send_ms;

// 存储发送的数据
Motor_send_data_t msd_6020;
Motor_send_data_t msd_3508_2006;

// 设置电机数量，ID号
Dji_Motor_Data _Motor2006_[_Motor2006_SIZE] = {0};  uint8_t _Motor2006_ID_[_Motor2006_SIZE] = {0};
Dji_Motor_Data _Motor3508_[_Motor3508_SIZE] = {0};  uint8_t _Motor3508_ID_[_Motor3508_SIZE] = {1, 2, 3, 4};
Dji_Motor_Data _Motor6020_[_Motor6020_SIZE] = {0};  uint8_t _Motor6020_ID_[_Motor6020_SIZE] = {1, 2, 3, 4};

Dji_Motor Motor2006(0x200, _Motor2006_SIZE, _Motor2006_, _Motor2006_ID_);
Dji_Motor Motor3508(0x200, _Motor3508_SIZE, _Motor3508_, _Motor3508_ID_);
Dji_Motor Motor6020(0x204, _Motor6020_SIZE, _Motor6020_, _Motor6020_ID_);

// PID角度环设置
Kpid_t Kpid_6020_angle(0.35, 0, 0);
PID pid_angle_0x205;
PID pid_angle_0x206;
PID pid_angle_0x207;
PID pid_angle_0x208;
// PID角度环设置
Kpid_t Kpid_6020_vel(110, 0, 0);
PID pid_vel_0x205;
PID pid_vel_0x206;
PID pid_vel_0x207;
PID pid_vel_0x208;

// PID速度环设置
Kpid_t Kpid_3508_vel(8, 0.1, 0);
PID pid_vel_0x201(1000, 2000);
PID pid_vel_0x202(1000, 2000);
PID pid_vel_0x203(1000, 2000);
PID pid_vel_0x204(1000, 2000);

// 力矩控制
Kpid_t Kpid_6020_T(0, 0, 0);
PID pid_T_0x207;
Kpid_t Kpid_3508_T(0, 0, 0);
PID pid_T_0x201;

// 尖括号里填底盘类型
Wheel_t<SG> Wheel;

// 6020速度滤波
TD td_6020_1(400);
TD td_6020_2(400);
TD td_6020_3(400);
TD td_6020_4(400);

// 3508速度滤波
TD td_3508_1(300);
TD td_3508_2(300);
TD td_3508_3(300);
TD td_3508_4(300);

// 力矩滤波
TD td6020_torque(100);
TD td3508_torque(100);

// 期望值滤波
TD tar_vw(30);
TD tar_vx(30);
TD tar_vy(30);
TD td_FF_Tar(100);

// 前馈
FeedTar feed_6020_1(200, 8);
FeedTar feed_6020_2(200, 8);
FeedTar feed_6020_3(200, 8);
FeedTar feed_6020_4(200, 8);

// 创建遥控器实例
RM_Clicker dr16;

// 创建底盘任务实例
Chassis_task chassis_task;

// 创建工具实例
Tools_t Tools;

// 创建底盘变量实例
Chassis_Data_t Chassis_Data;

PM01 pm01;
