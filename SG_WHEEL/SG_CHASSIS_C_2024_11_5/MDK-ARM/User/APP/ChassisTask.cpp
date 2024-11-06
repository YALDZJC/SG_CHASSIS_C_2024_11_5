#include "ChassisTask.hpp"
#include "cmsis_os2.h"
#include "dr16.hpp"

void ChassisTask(void* argument) 
{
  for (;;) 
	{
//		dr16.Parse(&huart3, 18);

		osDelay(1);
  }
}
