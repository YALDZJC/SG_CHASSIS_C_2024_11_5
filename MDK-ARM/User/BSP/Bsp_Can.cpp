#include "Bsp_Can.hpp"
#include "My_hal.hpp"

void CAN_Filter_Init()
{
  	CAN_FilterTypeDef Filter;
	Filter.FilterActivation = CAN_FILTER_ENABLE;//ʹ�ܹ�����
	Filter.FilterBank = 0;//ͨ��
	Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;//������
	Filter.FilterIdHigh = 0x0;//��16
	Filter.FilterIdLow = 0x0;//��16
	Filter.FilterMaskIdHigh = 0x0;//��16
	Filter.FilterMaskIdLow = 0x0;//��16
	Filter.FilterMode = CAN_FILTERMODE_IDMASK;//����
	Filter.FilterScale = CAN_FILTERSCALE_32BIT;
	Filter.SlaveStartFilterBank = 14;	
	HAL_CAN_ConfigFilter(&hcan1,&Filter);
	Filter.FilterBank = 14;//ͨ��
	Filter.SlaveStartFilterBank = 28;	
	HAL_CAN_ConfigFilter(&hcan2,&Filter);
}
 
void Can_Init()
{
	CAN_Filter_Init();
	//����can1
	HAL_CAN_Start(&hcan1);
	//�����ж�
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	//����can2
	HAL_CAN_Start(&hcan2);
	//�����ж�
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
}
void Can_Send(CAN_HandleTypeDef* han,uint32_t StdId,uint8_t* s_data,uint32_t pTxMailbox)
{
	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.DLC = 8;//����
	TxHeader.ExtId = 0;//��չid
	TxHeader.IDE = CAN_ID_STD;//��׼
	TxHeader.RTR = CAN_RTR_DATA;//����֡
	TxHeader.StdId = StdId;//id
	TxHeader.TransmitGlobalTime = DISABLE;
	
	if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0)
	{
		//��������
		HAL_CAN_AddTxMessage(han,&TxHeader,s_data,&pTxMailbox);
	}
}
