#pragma once

#include "stdxxx.hpp"
#include "CommunicationTask.hpp"
#include "EvenTask.hpp"

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
	void Tar_Updata();

	void Wheel_UpData();

	void Filtering();

	void PID_Updata();

	void CAN_Setting();

	void CAN_Send();

	void Base_UpData()
	{
		Tar_Updata();
		Wheel_UpData();
		Filtering();

		PID_Updata();

		CAN_Setting();
		CAN_Send();
	}


	virtual void upData() = 0;
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
	State curState;
	ChassisState *curTask;	//当前状态
	ChassisState *stateArray[MAX_TASK] = {nullptr}; // 存储状态对象

public:

	State GetState();// 判断当前状态的名称


	bool AddState(const State &taskName, ChassisState *newTask) // 设置状态的名称与对应的状态对象
	{
		if (taskCount >= MAX_TASK)
			return false;

		stateArray[taskName] = newTask;
		taskCount++;

		return true;
	}

	void setState(State newState) // 设置当前状态
	{
		curTask = stateArray[newState];
	}

	void upData()
	{
		curState = GetState();
		setState(curState); // 根据当前状态选择对应的处理函数

		if (curTask != nullptr) // 当前状态不为空
		{
			curTask->upData(); // 一键更新
		}
	}
};

extern Chassis_task chassis_task;
