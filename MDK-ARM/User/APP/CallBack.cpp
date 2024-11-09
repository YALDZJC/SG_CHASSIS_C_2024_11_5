#include "CallBack.hpp"
#include "My_hal.hpp"
#include "dr16.hpp"
#include "Dji_Motor.hpp"

//数量
#define _Motor2006_SIZE 1
#define _Motor3508_SIZE 4
#define _Motor6020_SIZE 1

Motor_t _Motor2006_[_Motor2006_SIZE] = { 0 };uint8_t _Motor2006_ID_[_Motor2006_SIZE] = { 0 };
Motor_t _Motor3508_[_Motor3508_SIZE] = { 0 };uint8_t _Motor3508_ID_[_Motor3508_SIZE] = { 1,2,3,4 };
Motor_t _Motor6020_[_Motor6020_SIZE] = { 0 };uint8_t _Motor6020_ID_[_Motor6020_SIZE] = { 1 };
Dji_Motor Motor2006(0x200, _Motor2006_SIZE, _Motor2006_, _Motor2006_ID_);
Dji_Motor Motor3508(0x200, _Motor3508_SIZE, _Motor3508_, _Motor3508_ID_);
Dji_Motor Motor6020(0x204, _Motor6020_SIZE, _Motor6020_, _Motor6020_ID_);

//can_filo0中断接收
CAN_RxHeaderTypeDef RxHeader;	//can接收数据
uint8_t RxHeaderData[8] = { 0 };
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//接受信息 
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader,RxHeaderData);
	if(hcan == &hcan1)
	{
		Motor3508.Parse(RxHeader,RxHeaderData);
		Motor6020.Parse(RxHeader,RxHeaderData);
	}
	if(hcan == &hcan2)
	{

	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	dr16.Parse(huart, Size);
}
