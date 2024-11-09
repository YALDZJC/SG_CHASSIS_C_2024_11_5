#include "Init.hpp"
#include "My_hal.hpp"
#include "dr16.hpp"
#include "tim.h"
#include "BSP_Can.hpp"

void Init() 
{
	dr16.Init();
	CAN.BSP_CAN_Init();
	
	HAL_TIM_Base_Start_IT(&htim7);
}
