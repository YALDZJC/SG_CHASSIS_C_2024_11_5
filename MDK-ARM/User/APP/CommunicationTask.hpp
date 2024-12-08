#pragma once

#include "stdxxx.hpp"
#include "dr16.hpp"
#include "EvenTask.hpp"
#define L_MODE1 (RM_Clicker::RC_Ctl.rc.s1 == 1) // 底盘动作
#define L_MODE2 (RM_Clicker::RC_Ctl.rc.s1 == 3) // 底盘动作
#define L_MODE3 (RM_Clicker::RC_Ctl.rc.s1 == 2) // 底盘动作

#define R_MODE1 (RM_Clicker::RC_Ctl.rc.s2 == 1) // 云台动作
#define R_MODE2 (RM_Clicker::RC_Ctl.rc.s2 == 3) // 云台动作
#define R_MODE3 (RM_Clicker::RC_Ctl.rc.s2 == 2) // 云台动作

/*遥控器信号源切换*/
/*
	CONTROL_SIG 0 遥控器
	CONTROL_SIG 1 上下板
*/
#define CONTROL_SIG 0
#if CONTROL_SIG == 0
// 模式切换
#define Universal ((L_MODE1 && R_MODE1) || (L_MODE1 && R_MODE2) || (L_MODE1 && R_MODE3)) // （1）万向模式
#define Follow (L_MODE2 && R_MODE1) || (L_MODE2 && R_MODE3)								 // （2）底盘跟随
#define Rotating (L_MODE3 && R_MODE1) || (L_MODE3 && R_MODE2)							 // （3）小陀螺
#define KeyBoard (L_MODE2 && R_MODE2)													 // （4键鼠模式）
#define Stop (L_MODE3 && R_MODE3)														 // （5停止模式）

// 期望值切换
#define TAR_LX RC_LX
#define TAR_LY RC_LY
#define TAR_RX RC_RX
#define TAR_RY RC_RY

#elif CONTROL_SIG == 1
// 模式切换
#define Universal (Gimbal_to_Chassis_Data.Universal_t) // （1）万向模式
#define Follow (Gimbal_to_Chassis_Data.Follow_t)	   // （2）底盘跟随
#define Rotating (Gimbal_to_Chassis_Data.Rotating_t)   // （3）小陀螺
// #define KeyBoard (Gimbal_to_Chassis_Data.KeyBoard_t)											 // （4键鼠模式）
#define Stop (Gimbal_to_Chassis_Data.stop)			   // （5停止模式）

// 期望值切换
#define TAR_LX Gimbal_to_Chassis_Data.int16_RC_LX
#define TAR_LY Gimbal_to_Chassis_Data.int16_RC_LY
#define TAR_RX Gimbal_to_Chassis_Data.int16_RC_RX
#define TAR_RY Gimbal_to_Chassis_Data.int16_RC_RY

#endif

class Communicat_Data
{
public:
	Communicat_Data(uint16_t size)
	{
		size_ = size;
		data_ = new uint16_t[size_];
	}
	~Communicat_Data()
	{
		delete[] data_;
	}

protected:
private:
	uint16_t *data_;
	uint16_t size_;
};

struct Gimbal_to_Chassis_Data_t
{
	uint8_t head; // 帧头

	bool notation_RC_LX; // RC_LX符号位
	uint8_t _RC_LX;		 // RC_LX

	bool notation_RC_LY; // RC_LY符号位
	uint8_t _RC_LY;		 // RC_LY

	int16_t yaw_encoder_angle_e; // 底盘跟随云台误差
	int16_t yaw_encoder_e;

	int16_t int16_RC_LX; // 转换之后的RC_LX
	int16_t int16_RC_LY; // 转换之后的RC_LY

	bool notation_ER; // ER符号位，小陀螺+-速度
	uint8_t _ER;	  // ER

	int8_t int8_ER; // ER

	bool notation_ZX; // ZX符号位，底盘平移+-速度
	uint8_t _ZX;	  // ZX

	int8_t int8_ZX; // ZX

	bool notation_pitch_cai; // pitch符号位，小陀螺+-数度
	uint8_t _pitch_cai;		 // pitch

	int8_t int8_pitch_cai; // pitch

	bool Follow_t;	  // 底盘跟随云台
	bool Universal_t; // 主云台模式
	bool Rotating_t;  // 小陀螺
	bool up_ui;		  // 刷新ui
	bool stop;		  // 停止
	bool MCL_of;	  // 摩擦轮是否开启
	bool CM_of;		  // 仓门是否开启
	bool v_of;		  // 视觉自瞄是否开启
	bool bp_of;		  // 拨盘是否开启
	uint8_t pData[50];
	bool dir;
};
// 数据
extern Gimbal_to_Chassis_Data_t Gimbal_to_Chassis_Data;

// 将RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif

	void CommunicationTask(void *argument);

#ifdef __cplusplus
}
#endif
