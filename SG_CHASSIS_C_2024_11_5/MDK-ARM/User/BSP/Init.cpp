#include "Init.hpp"
#include "My_hal.hpp"
#include "dr16.hpp"
#include "tim.h"

void Init() 
{
	dr16.Init();
	
	HAL_TIM_Base_Start_IT(&htim7);
}
