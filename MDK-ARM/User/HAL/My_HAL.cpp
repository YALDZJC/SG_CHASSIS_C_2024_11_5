#include "My_HAL.hpp"
#include "arm_math.h"
#include "cmsis_os2.h"

#include "BSP_Can.hpp"
#include "tim.h"
#include "Variable.hpp"

//初始化
void My_hal::_timer_init()
{
    HAL_TIM_Base_Start_IT(&htim7);
}

void My_hal::_dr16_init()
{
    dr16.Init();
}

void My_hal::_can_init()
{
    Can_Init();
}

void My_hal::_referee_init()
{

}

void My_hal::_darw_graphic_init() 
{

}

void My_hal::_capactal_init()
{

}

void My_hal::_delay(unsigned long _mill)
{
    HAL_Delay(_mill);
}

void My_hal::_osDelay(unsigned long _mill)
{
    osDelay(_mill);
}

unsigned long My_hal::_GetTick()
{
    return HAL_GetTick();
}

float My_hal::_sinf(float x)
{
    return arm_sin_f32(x);
}

float My_hal::_cosf(float x)
{
    return arm_cos_f32(x);
}

float My_hal::_sqrt(float in, float pOut)
{
    return arm_sqrt_f32(in, &pOut);
}




