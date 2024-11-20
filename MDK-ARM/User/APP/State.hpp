#pragma once

#include "stdxxx.hpp"
#include "CommunicationTask.hpp"

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
#define Universal ((L_MODE1 && R_MODE1) || (L_MODE1 && R_MODE2) || (L_MODE1 && R_MODE3)) // （1）万向模式
#define Follow (L_MODE2 && R_MODE1) || (L_MODE2 && R_MODE3)								 // （2）底盘跟随
#define Rotating (L_MODE3 && R_MODE1) || (L_MODE3 && R_MODE2)							 // （3）小陀螺
#define KeyBoard (L_MODE2 && R_MODE2)													 // （4键鼠模式）
#define Stop (L_MODE3 && R_MODE3)														 // （5停止模式）

#elif CONTROL_SIG == 1
#define Universal (Gimbal_to_Chassis_Data.Universal_t) // （1）万向模式
#define Follow (Gimbal_to_Chassis_Data.Follow_t)	   // （2）底盘跟随
#define Rotating (Gimbal_to_Chassis_Data.Rotating_t)   // （3）小陀螺
// #define KeyBoard (Gimbal_to_Chassis_Data.KeyBoard_t)											 // （4键鼠模式）
#define Stop (Gimbal_to_Chassis_Data.stop)			   // （5停止模式）
#endif

class Task
{
public:
	virtual void upData() = 0;
	virtual ~Task() = default;
};

class Universal_mode : public Task
{
public:
	void upData() ;
};

class Follow_mode : public Task
{
public:
	void upData() ;
};

class Rotating_mode : public Task
{
public:
	void upData() ;
};

class Stop_mode : public Task
{
public:
	void upData();
};

class Chassis_task : public Task
{
private:
	static const uint8_t MAX_TASK = 10;
	Task *task[MAX_TASK] = {nullptr};

	uint8_t taskCount = 0;

public:
	bool AddTask(Task *newTask)
	{
		if (taskCount < MAX_TASK)
		{
			this->task[taskCount] = newTask;
			taskCount++;
			return true;
		}
		return false;
	}
	void upData()
	{
		if (Gimbal_to_Chassis_Data.Universal_t == true)
		{
			task[0]->upData();
		}
		if (Gimbal_to_Chassis_Data.Follow_t == true)
		{
			task[1]->upData();
		}
		if (Gimbal_to_Chassis_Data.Rotating_t == true)
		{
			task[2]->upData();
		}
		if (Gimbal_to_Chassis_Data.Rotating_t == true)
		{
			task[3]->upData();
		}
	}
};
