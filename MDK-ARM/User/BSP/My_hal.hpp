#pragma once

#define IS_F4 (0x00)  //使用can
#define IS_F7 (0x01)//使用fdcan

#define RM_IS_Fx (IS_F4)//定义使用facan还是can

#if (RM_IS_Fx == IS_F4)
	#include "stm32f4xx_hal.h"
	#include "can.h"
#elif (RM_IS_Fx == IS_F7)
	#include "stm32h7xx_hal.h"
	#include "stm32h7xx_hal_fdcan.h"
	#include "fdcan.h"
#endif

#include "main.h"
#include "usart.h"
#include "gpio.h"
/***********************fdcan或者can使用定义********************************/
//#include "RM_FDCan.h"

/*fdcan与can来回切换*/
#define RM_IS_HAL_CAN (0x00)  //使用can
#define RM_IS_HAL_FDCAN (0x01)//使用fdcan

#define RM_IS_HAL_FDorCAN (RM_IS_HAL_CAN)//定义使用facan还是can
/*
RM_FDorCAN_HandleTypeDef 原can或者facan的结构体
RM_FDorCAN_RxHeaderTypeDef 原can或者facan的接收结构体
RM_FDorCAN_TxHeaderTypeDef 原can或者facan的发送结构体
FDorCAN_ID 原can或者facan的id
RM_FDorCAN_Filter_Init 过滤器配置
RM_FDorCAN_Init 初始化
RM_FDorCAN_Send 发送函数
RM_FDorCAN_RxFifo0PendingCallback 原can或者facan的接收回调函数，注意：参数数量不一样!!!
*/

/*实现fdcan与can无缝切换*/
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
