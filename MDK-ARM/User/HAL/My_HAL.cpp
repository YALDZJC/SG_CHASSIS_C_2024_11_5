#include "My_HAL.hpp"
#include "arm_math.h"
void My_hal::_delay(unsigned long _mill)
{
    HAL_Delay(_mill);
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
