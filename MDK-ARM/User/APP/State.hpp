#pragma once

#include <memory>
#include "stdxxx.hpp"
#include "CommunicationTask.hpp"
#include <map>
#include <string>
#include "dr16.hpp"
#include "PID.hpp"

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

#define MAX_TASK 6 // 任务最大数量

enum State
{
	Universal_State = 0,
	Follow_State,
	Rotating_State,
	Stop_State,
};

class ChassisState
{
public:
	// 期望值
	float tar_AGV_angle[4];
	float tar_AGV_speed[4];
	float Zero_crossing[4];

	void Wheel_UpData();

	void Tar_Updata();

	void TD_Updata();

	void PID_Updata();

	virtual void upData() = 0;
	virtual ~ChassisState() = default;
};

class Universal_mode : public ChassisState
{
public:
	void upData();
};

class Follow_mode : public ChassisState
{
public:
	void upData();
};

class Rotating_mode : public ChassisState
{
public:
	void upData();
};

class Stop_mode : public ChassisState
{
public:
	void upData();
};


class Chassis_task
{
private:
	uint8_t taskCount = 0;
	State curTask;
	ChassisState *curState;
	std::map<uint8_t, ChassisState *> stateMap;

public:
	State GetState()
	{
		if (Universal)
			return Universal_State;
		else if (Follow)
			return Follow_State;
		else if (Rotating)
			return Rotating_State;
		else if (Stop)
			return Stop_State;
		else
			return Stop_State;
	}
	bool AddState(uint8_t &taskName, ChassisState *newTask)
	{
		if (taskCount >= MAX_TASK)
		 	return false;

		stateMap[taskName] = newTask;
		taskCount++;

		return true;
	}
	void setState(State newState)
	{
		auto it = stateMap.find(newState);
		if (it != stateMap.end())
		{
			curState = it->second; // 直接赋值，不释放当前对象
		}
	}

	void upData()
	{
		curTask = GetState();
		setState(curTask); // 根据当前状态选择对应的处理函数
		if (curState != nullptr)
		{
			curState->upData();
		}
	}
};