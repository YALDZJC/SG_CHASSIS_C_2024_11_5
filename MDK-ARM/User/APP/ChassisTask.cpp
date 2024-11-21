#include "ChassisTask.hpp"
#include "cmsis_os2.h"
#include "Bsp_Can.hpp"
#include "Variable.hpp"
#include "HAL.hpp"
#include "State.hpp"

float tar_angle;

float pos;
float speed;
float pi = 3.1415926;
float hz;
void ChassisTask(void* argument) 
{
  for (;;) 
	{
//		mecanumWheel.WheelType.UpDate(RC_LX, RC_LY, RC_RX, 8191);
//		speed = mecanumWheel.WheelType.speed[0];
//		
//		tar_angle += RC_LX*0.01;
		Motor4310.ON(&hcan1);
		Motor4310.ctrl_Motor(&hcan1, 0, 0, 0, 0, speed);
		// tar_angle = Motor6020.Zero_crossing_processing(tar_angle, Motor6020.GetEquipData(0x205, Angle), 8191);

		// td_speed.Calc(Motor6020.GetEquipData(0x205, Speed));
		
		// DEMO_6020.GetPidPos(K_DEMO_6020, tar_angle, Motor6020.GetEquipData(0x205, Angle), 30000);
		// DEMO_6020_IN.GetPidPos(K_DEMO_6020_IN, DEMO_6020.pid.cout, td_speed.x1, 30000);

		// setMSD(&msd_6020, DEMO_6020_IN.pid.cout, 1);
		// pos = Motor6020.GetEquipData(0x205, Angle);



		pos = HAL::sinf(2*pi*hz);
		hz+=0.001;

		Send_6020_CAN();
    
		osDelay(1);
  }
}
