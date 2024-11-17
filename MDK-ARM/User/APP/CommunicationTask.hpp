#pragma once

#include "stdxxx.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void CommunicationTask(void* argument);

#ifdef __cplusplus
}
#endif

/***************************云台到底盘数据*********************************/
struct Gimbal_to_Chassis_Data_t
{
	uint8_t head;//帧头
	
	bool notation_RC_LX;//RC_LX符号位
	uint8_t _RC_LX;//RC_LX
	
	bool notation_RC_LY;//RC_LY符号位
	uint8_t _RC_LY;//RC_LY
	
	int16_t yaw_encoder_angle_e;//底盘跟随云台误差
	int16_t yaw_encoder_e;
	
	int16_t int16_RC_LX;//转换之后的RC_LX
	int16_t int16_RC_LY;//转换之后的RC_LY
	
	bool notation_ER;//ER符号位，小陀螺+-速度
	uint8_t _ER;//ER
	
	int8_t int8_ER;//ER
	
	bool notation_ZX;//ZX符号位，底盘平移+-速度
	uint8_t _ZX;//ZX
	
	int8_t int8_ZX;//ZX
	
	bool notation_pitch_cai;//pitch符号位，小陀螺+-数度
	uint8_t _pitch_cai;//pitch
	
	int8_t int8_pitch_cai;//pitch
	
	bool Follow_t;    //底盘跟随云台
	bool Universal_t; //主云台模式
	bool Rotating_t;  //小陀螺
	bool up_ui;//刷新ui
	bool stop;//停止
	bool MCL_of;//摩擦轮是否开启
	bool CM_of;//仓门是否开启
	bool v_of;//视觉自瞄是否开启
	bool bp_of;//拨盘是否开启
	uint8_t pData[50];
	bool dir;
};
//数据
Gimbal_to_Chassis_Data_t Gimbal_to_Chassis_Data; 
