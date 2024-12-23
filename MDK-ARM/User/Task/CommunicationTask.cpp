#include "CommunicationTask.hpp"
#include "cmsis_os2.h"
// #include "Variable.hpp"
// #include "State.hpp"

#define SIZE 8
Gimbal_to_Chassis_Data_t Gimbal_to_Chassis_Data;

uint8_t format[8] = {0XAA};
void CommunicationTask(void *argument)
{
	for (;;)
	{
//		format[0] = 0XAA;
//		format[1] = ((RC_LY) < 0 ? 1 : 0) << 7 | (abs((RC_LY) / 6));
//		format[2] = ((RC_LX) < 0 ? 1 : 0) << 7 | (abs((RC_LX) / 6));

//		format[SIZE - 1] = 0XAA;

//		/*在组合运动里面6和4等于1,5代表着是小陀螺测试*/
		// format[5] = ((Stop) << 7 /*保留给停止位*/ |
		// 			 (Universal) << 6 /*万向模式*/ |
		// 			 (Follow) << 5 /*底盘跟随*/ |
		// 			 (Rotating) << 4 /*小陀螺*/ |
		// 			 (0) << 3 /*速度方向*/ |
		// 			 (0) << 2 /*冲刺模式*/ |
		// 			 (0) << 1 /*ctrl按键*/); // 停止模式

//		if (format[0] == 0XAA && format[SIZE - 1] == 0XAA)
//		{
//			Gimbal_to_Chassis_Data.notation_RC_LY = format[1] >> 7;					// 符号位
//			Gimbal_to_Chassis_Data._RC_LY = format[1] & 0x7f;						// 数据
//			Gimbal_to_Chassis_Data.int16_RC_LY = Gimbal_to_Chassis_Data._RC_LY * 6; //RC_LY解算
//			if (Gimbal_to_Chassis_Data.notation_RC_LY == 0)
//				Gimbal_to_Chassis_Data.int16_RC_LY *= -1; // 符号位解算

//			Gimbal_to_Chassis_Data.notation_RC_LX = format[2] >> 7;					// 符号位
//			Gimbal_to_Chassis_Data._RC_LX = format[2] & 0x7f;						// 数据
//			Gimbal_to_Chassis_Data.int16_RC_LX = Gimbal_to_Chassis_Data._RC_LX * 6; //RC_LX解算
//			if (Gimbal_to_Chassis_Data.notation_RC_LX == 0)
//				Gimbal_to_Chassis_Data.int16_RC_LX *= -1; // 符号位解算

//			Gimbal_to_Chassis_Data.stop = (bool)(format[5] & 0x80);									 // 停止
//			Gimbal_to_Chassis_Data.Universal_t = (bool)(format[5] & 0x40);							 // 底盘跟随云台
//			Gimbal_to_Chassis_Data.Follow_t = (bool)(format[5] & 0x20);								 // 主云台
//			Gimbal_to_Chassis_Data.Rotating_t = (bool)(format[5] & 0x10) | (bool)(format[5] & 0x08); // 小陀螺
//		}
		osDelay(1);
	}
}
