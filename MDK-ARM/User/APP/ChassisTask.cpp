#include "ChassisTask.hpp"
#include "cmsis_os2.h"
#include "Bsp_Can.hpp"
#include "Variable.hpp"

float tar_angle;

float pos;
void ChassisTask(void* argument) 
{
  for (;;) 
	{
	tar_angle += RC_LX*0.01;
		
	tar_angle = Motor6020.Zero_crossing_processing(tar_angle, Motor6020.GetEquipData(0x205, Angle), 8191);

	td_speed.Calc(Motor6020.GetEquipData(0x205, Speed));
		
   DEMO_6020.GetPidPos(K_DEMO_6020, tar_angle, Motor6020.GetEquipData(0x205, Angle), 30000);
   DEMO_6020_IN.GetPidPos(K_DEMO_6020_IN, DEMO_6020.pid.cout, td_speed.x1, 30000);

	setMSD(&msd_6020, DEMO_6020_IN.pid.cout, 1);
		pos = Motor6020.GetEquipData(0x205, Angle);
	Send_6020_CAN();
    
		osDelay(1);
  }
}
