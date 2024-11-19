#include "RefeeTask.hpp"
#include "cmsis_os2.h"
#include "HAL.hpp"
void RefeeTask(void* argument) {
  for (;;) 
	{
		
		HAL::osDelay(1);
  }
}
