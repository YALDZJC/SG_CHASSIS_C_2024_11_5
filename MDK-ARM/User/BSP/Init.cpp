#include "Init.hpp"
#include "My_hal.hpp"
#include "BSP_Can.hpp"
#include "Variable.hpp"

void Init() 
{
	HAL::inject(new My_hal);
}
