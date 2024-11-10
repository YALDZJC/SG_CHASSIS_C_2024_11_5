#include "Variable.hpp"
#include "My_hal.hpp"

//发送6020数据
void Send_6020_CAN()
{
	//发送
	RM_FDorCAN_Send(&hcan1,SEND_MOTOR_ID_6020,msd_6020.Data,CAN_TX_MAILBOX1);
}

//提供一个默认数据变量
Motor_send_data_t msd_6020;
Motor_send_data_t msd_3508_2006;
Motor_t _Motor2006_[_Motor2006_SIZE] = { 0 };uint8_t _Motor2006_ID_[_Motor2006_SIZE] = { 0 };
Motor_t _Motor3508_[_Motor3508_SIZE] = { 0 };uint8_t _Motor3508_ID_[_Motor3508_SIZE] = { 1,2,3,4 };
Motor_t _Motor6020_[_Motor6020_SIZE] = { 0 };uint8_t _Motor6020_ID_[_Motor6020_SIZE] = { 1 };
Dji_Motor Motor2006(0x200, _Motor2006_SIZE, _Motor2006_, _Motor2006_ID_);
Dji_Motor Motor3508(0x200, _Motor3508_SIZE, _Motor3508_, _Motor3508_ID_);
Dji_Motor Motor6020(0x204, _Motor6020_SIZE, _Motor6020_, _Motor6020_ID_);

Kpid_t K_DEMO_6020(0, 0, 0);
PID DEMO_6020;

Kpid_t K_DEMO_6020_IN(0, 0, 0);
PID DEMO_6020_IN;

TD td_speed(100);

RM_Clicker dr16;


