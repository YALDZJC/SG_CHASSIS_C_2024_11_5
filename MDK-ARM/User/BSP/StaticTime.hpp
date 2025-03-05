#pragma once
#include "stm32f4xx_hal.h"
#include "stdxxx.hpp"
#include "Key.hpp"

class RM_StaticTime
{
public:
	uint32_t lastTime;	//上一时刻
	void UpLastTime();//更新上一时刻
	// bool ISOne(uint32_t targetTime);//判断单次信号
	// bool ISGL(uint32_t targetTime, uint8_t percentage = 50/*百分比占比*/);//判断连续信号
	bool ISDir(uint32_t dirTime);//定时器死亡
	bool ISFromOne(uint64_t nowTime, uint64_t targetTime);//自定义判断单次信号
	bool ISFromGL(uint64_t nowTime, uint64_t targetTime, uint8_t percentage = 50/*百分比占比*/);//自定义判断连续信号
};

inline void RM_StaticTime::UpLastTime()
{
	this->lastTime = HAL_GetTick();
}

inline bool RM_StaticTime::ISDir(uint32_t dirTime)
{
	if(HAL_GetTick() - this->lastTime >= dirTime)
		return true;
		
  	return false;
}
