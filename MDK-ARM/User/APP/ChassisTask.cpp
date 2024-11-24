#include "ChassisTask.hpp"
#include "cmsis_os2.h"
#include "Bsp_Can.hpp"
#include "Variable.hpp"
#include "HAL.hpp"
#include "State.hpp"
#include "cmsis_os2.h"

float tar_angle;

float pos;
float speed;
float pi = 3.1415926;
float hz;

float tar_speed[4];
float getMinPos[4];
float Zero_cross[4];
void ChassisTask(void *argument)
{
	for (;;)
	{
		//		mecanumWheel.WheelType.UpDate(RC_LX, RC_LY, RC_RX, 8191);
		//		speed = mecanumWheel.WheelType.speed[0];
		//
		//		tar_angle += RC_LX*0.01;

		// tar_angle = Motor6020.Zero_crossing_processing(tar_angle, Motor6020.GetEquipData(0x205, Angle), 8191);

		// td_speed.Calc(Motor6020.GetEquipData(0x205, Speed));

		// DEMO_6020.GetPidPos(K_DEMO_6020, tar_angle, Motor6020.GetEquipData(0x205, Angle), 30000);
		// DEMO_6020_IN.GetPidPos(K_DEMO_6020_IN, DEMO_6020.pid.cout, td_speed.x1, 30000);

		// setMSD(&msd_6020, DEMO_6020_IN.pid.cout, 1);
		// pos = Motor6020.GetEquipData(0x205, Angle);

//	    tar_vx.Calc(TAR_LX);
//    tar_vy.Calc(TAR_LY);

//    if (CONTROL_SIG == 0)
//        tar_vw.Calc(TAR_RX);
//	
//    // 对轮子进行运动学变换
//    Wheel.WheelType.UpDate(tar_vx.x1, tar_vy.x1, tar_vw.x1, 8191);

//    // 储存最小角判断的速度
//    tar_speed[0] = Wheel.WheelType.speed[0];
//    tar_speed[1] = Wheel.WheelType.speed[1];
//    tar_speed[2] = Wheel.WheelType.speed[2];
//    tar_speed[3] = Wheel.WheelType.speed[3];
//    // 储存最小角判断的角度
//    getMinPos[0] = Motor6020.MinPosHelm(tar_vx.x1, Motor6020.GetEquipData(L_Forward_ID, Dji_Speed), &tar_speed[0], 8191, 8191);
//    getMinPos[1] = Motor6020.MinPosHelm(tar_vx.x1, Motor6020.GetEquipData(L_Back_ID   , Dji_Speed), &tar_speed[1], 8191, 8191);
//    getMinPos[2] = Motor6020.MinPosHelm(tar_vx.x1, Motor6020.GetEquipData(R_Back_ID   , Dji_Speed), &tar_speed[2], 8191, 8191);
//    getMinPos[3] = Motor6020.MinPosHelm(tar_vx.x1, Motor6020.GetEquipData(R_Forward_ID, Dji_Speed), &tar_speed[3], 8191, 8191);
////    //过零处理
//		Zero_cross[0] = Motor6020.Zero_crossing_processing(getMinPos[0], Motor6020.GetEquipData(L_Forward_ID, Dji_Angle), 8191);
//		Zero_cross[1] = Motor6020.Zero_crossing_processing(getMinPos[1], Motor6020.GetEquipData(L_Back_ID   , Dji_Angle), 8191);
//		Zero_cross[2] = Motor6020.Zero_crossing_processing(getMinPos[2], Motor6020.GetEquipData(R_Back_ID   , Dji_Angle), 8191);
//		Zero_cross[3] = Motor6020.Zero_crossing_processing(getMinPos[3], Motor6020.GetEquipData(R_Forward_ID, Dji_Angle), 8191);
		
//	    tar_vx.Calc(TAR_LX);
//    tar_vy.Calc(TAR_LY);

//    if (CONTROL_SIG == 0)
//        tar_vw.Calc(TAR_RX);
//    // 对轮子进行运动学变换
//    Wheel.WheelType.UpDate(tar_vx.x1, tar_vy.x1, tar_vw.x1, 8191);

//    // 储存最小角判断的速度
//    tar_speed[0] = Wheel.WheelType.speed[0];
//    tar_speed[1] = Wheel.WheelType.speed[1];
//    tar_speed[2] = Wheel.WheelType.speed[2];
//    tar_speed[3] = Wheel.WheelType.speed[3];
//    // 储存最小角判断的角度
//    getMinPos[0] = Motor6020.MinPosHelm(tar_vx.x1, Motor6020.GetEquipData(L_Forward_ID, Dji_Speed), &tar_speed[0], 8191, 8191);
//    getMinPos[1] = Motor6020.MinPosHelm(tar_vx.x1, Motor6020.GetEquipData(L_Back_ID   , Dji_Speed), &tar_speed[1], 8191, 8191);
//    getMinPos[2] = Motor6020.MinPosHelm(tar_vx.x1, Motor6020.GetEquipData(R_Back_ID   , Dji_Speed), &tar_speed[2], 8191, 8191);
//    getMinPos[3] = Motor6020.MinPosHelm(tar_vx.x1, Motor6020.GetEquipData(R_Forward_ID, Dji_Speed), &tar_speed[3], 8191, 8191);
////    //过零处理
//		Zero_cross[0] = Motor6020.Zero_crossing_processing(getMinPos[0], Motor6020.GetEquipData(L_Forward_ID, Dji_Angle), 8191);
//		Zero_cross[1] = Motor6020.Zero_crossing_processing(getMinPos[1], Motor6020.GetEquipData(L_Back_ID   , Dji_Angle), 8191);
//		Zero_cross[2] = Motor6020.Zero_crossing_processing(getMinPos[2], Motor6020.GetEquipData(R_Back_ID   , Dji_Angle), 8191);
//		Zero_cross[3] = Motor6020.Zero_crossing_processing(getMinPos[3], Motor6020.GetEquipData(R_Forward_ID, Dji_Angle), 8191);
		
		chassis_task.upData();
//		tar_angle = chassis_task.GetState();
//		pos = HAL::sinf(2 * pi * hz);
//		hz += 0.001;

//		Tools.vofaSend(Motor6020.GetEquipData(0x205, Dji_Angle), pos, 0, 0, 0, 0);

//		Send_6020_CAN();

		osDelay(1);
	}
}
