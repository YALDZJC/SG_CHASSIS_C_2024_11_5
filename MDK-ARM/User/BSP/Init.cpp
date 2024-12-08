#include "Init.hpp"
#include "My_hal.hpp"
#include "BSP_Can.hpp"
#include "State.hpp"

Universal_mode Universal_t;
Follow_mode Follow_t;
Rotating_mode Rotating_t;
Stop_mode stop;
//Defalut_t Defalut_t_t;

void Init()
{
	chassis_task.AddState(Universal_State,	&Universal_t);
	chassis_task.AddState(Follow_State, 	&Follow_t);
	chassis_task.AddState(Rotating_State,	&Rotating_t);
	chassis_task.AddState(Stop_State,		&stop);

	HAL::inject(new My_hal);
}
