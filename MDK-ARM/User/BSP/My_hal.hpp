#pragma once

#define IS_F4 (0x00)  //ʹ��can
#define IS_H7 (0x01)//ʹ��fdcan

#define RM_IS_Fx (IS_F4)//����ʹ��facan����can

#if (RM_IS_Fx == IS_F4)
	#include "stm32f4xx_hal.h"
	#include "can.h"
#elif (RM_IS_Fx == IS_H7)
	#include "stm32h7xx_hal.h"
	#include "stm32h7xx_hal_fdcan.h"
	#include "fdcan.h"
#endif

#include "main.h"
#include "usart.h"
#include "gpio.h"
/***********************fdcan����canʹ�ö���********************************/
//#include "RM_FDCan.h"

/*fdcan��can�����л�*/
#define RM_IS_HAL_CAN (0x00)  //ʹ��can
#define RM_IS_HAL_FDCAN (0x01)//ʹ��fdcan

#define RM_IS_HAL_FDorCAN (RM_IS_HAL_CAN)//����ʹ��facan����can
/*
RM_FDorCAN_HandleTypeDef ԭcan����facan�Ľṹ��
RM_FDorCAN_RxHeaderTypeDef ԭcan����facan�Ľ��սṹ��
RM_FDorCAN_TxHeaderTypeDef ԭcan����facan�ķ��ͽṹ��
FDorCAN_ID ԭcan����facan��id
RM_FDorCAN_Filter_Init ����������
RM_FDorCAN_Init ��ʼ��
RM_FDorCAN_Send ���ͺ���
RM_FDorCAN_RxFifo0PendingCallback ԭcan����facan�Ľ��ջص�������ע�⣺����������һ��!!!
*/

/*ʵ��fdcan��can�޷��л�*/
#if (RM_IS_HAL_FDorCAN == RM_IS_HAL_CAN)
 #define RM_FDorCAN_HandleTypeDef           CAN_HandleTypeDef
 #define RM_FDorCAN_RxHeaderTypeDef         CAN_RxHeaderTypeDef
 #define RM_FDorCAN_TxHeaderTypeDef         CAN_TxHeaderTypeDef
 #define FDorCAN_ID(x) (x.StdId)
	#define RM_FDorCAN_Filter_Init             RM_CAN_Filter_Init()
 #define RM_FDorCAN_Init(x)                    			RM_Can_Init()
 #define RM_FDorCAN_Send(x,ID,s_data,pTxMailbox)          RM_Can_Send(x,ID,s_data,pTxMailbox)
#elif (RM_IS_HAL_FDorCAN == RM_IS_HAL_FDCAN)
 #define RM_FDorCAN_HandleTypeDef           FDCAN_HandleTypeDef
 #define RM_FDorCAN_RxHeaderTypeDef         FDCAN_RxHeaderTypeDef
 #define RM_FDorCAN_TxHeaderTypeDef         FDCAN_TxHeaderTypeDef
 #define FDorCAN_ID(x) (x.Identifier)
	#define RM_FDorCAN_Filter_Init(x)             RM_FDCAN_Filter_Init(x)
 #define RM_FDorCAN_Init(x)                    RM_FDCan_Init(x)
 #define RM_FDorCAN_Send(x,ID,s_data)          RM_FDCan_Send(x,ID,s_data)
#endif

/**************************************************************************/
