#pragma once

#include "stdxxx.hpp"

#define L_MODE1 (RM_Clicker::RC_Ctl.rc.s1 == 1)//底盘动作
#define L_MODE2 (RM_Clicker::RC_Ctl.rc.s1 == 3)//底盘动作
#define L_MODE3 (RM_Clicker::RC_Ctl.rc.s1 == 2)//底盘动作

#define R_MODE1 (RM_Clicker::RC_Ctl.rc.s2 == 1)//云台动作
#define R_MODE2 (RM_Clicker::RC_Ctl.rc.s2 == 3)//云台动作
#define R_MODE3 (RM_Clicker::RC_Ctl.rc.s2 == 2)//云台动作

/*遥控器信号源切换*/
/*
	CONTROL_SIG 0 遥控器
	CONTROL_SIG 1 上下板
*/
#define CONTROL_SIG 0
#if CONTROL_SIG == 0	
	#define Universal   ((L_MODE1 && R_MODE1) || (L_MODE1 && R_MODE2) || (L_MODE1 && R_MODE3))//（1）万向模式
	#define Follow      (L_MODE2 && R_MODE1) || (L_MODE2 && R_MODE3)//（2）底盘跟随
	#define Rotating    (L_MODE3 && R_MODE1) || (L_MODE3 && R_MODE2)//（3）小陀螺
    #define KeyBoard    (L_MODE2 && L_MODE2)    //（4键鼠模式）
    #define Stop        (L_MODE3 && L_MODE3)    //（5停止模式）

#elif CONTROL_SIG == 1
	#define stop_mode (Gimbal_to_Chassis_Data.stop)//（9）
#endif



//对于底盘的模式来说，只有输入方式的不同，于是可以把键盘输入与遥控器输入做区分
class ChassisState{
public :

    void UpData();
};

