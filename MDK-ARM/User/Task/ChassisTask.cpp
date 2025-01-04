#include "ChassisTask.hpp"
#include "cmsis_os2.h"
#include "HAL.hpp"
#include "State.hpp"
#include "Variable.hpp"

Universal_mode Universal_t;
Follow_mode Follow_t;
Rotating_mode Rotating_t;
Stop_mode stop;

void ChassisTask(void *argument)
{
	chassis_task.AddState(Universal_State,	&Universal_t);
	chassis_task.AddState(Follow_State, 	&Follow_t);
	chassis_task.AddState(Rotating_State,	&Rotating_t);
	chassis_task.AddState(Stop_State,		&stop);
	
	for (;;)
	{
		chassis_task.upData();
		osDelay(2);
	}
}
 