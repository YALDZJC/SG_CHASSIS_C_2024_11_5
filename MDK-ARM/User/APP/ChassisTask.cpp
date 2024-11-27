#include "ChassisTask.hpp"
#include "cmsis_os2.h"
#include "HAL.hpp"
#include "State.hpp"
#include "Variable.hpp"

void ChassisTask(void *argument)
{
	for (;;)
	{
		chassis_task.upData();
		osDelay(2);
	}
}
