#include "ChassisTask.hpp"
#include "cmsis_os2.h"
#include "HAL.hpp"
#include "State.hpp"
#include "Variable.hpp"

float tar_angle;

float pos;
float speed;
float pi = 3.1415926;
float hz;

float tar_speed[4];
float getMinPos[4];
float Zero_cross[4];
void ChassisTask(void *argument)
{
	for (;;)
	{
		chassis_task.upData();

		osDelay(1);
	}
}
