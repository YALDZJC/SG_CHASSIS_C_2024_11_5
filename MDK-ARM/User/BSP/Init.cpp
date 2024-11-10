#include "Init.hpp"
#include "My_hal.hpp"
#include "tim.h"
#include "BSP_Can.hpp"
#include "Variable.hpp"

void Init() 
{
	dr16.Init();
	Can_Init();
	
	HAL_TIM_Base_Start_IT(&htim7);
}
