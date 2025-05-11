#include "CallBack.hpp"
#include "../APP/Referee/RM_RefereeSystem.h"
#include "../BSP/Dbus.hpp"
#include "../BSP/Power/PM01.hpp"
#include "../BSP/SuperCap/SuperCap.hpp"
#include "../Task/CommunicationTask.hpp"
#include "My_hal.hpp"
#include "Variable.hpp"

// can_filo0中断接收
CAN_RxHeaderTypeDef RxHeader; // can接收数据
uint8_t RxHeaderData[8] = {0};

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 接受信息
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxHeaderData);
    if (hcan == &hcan1)
    {
        Motor3508.Parse(RxHeader, RxHeaderData);
        Motor6020.Parse(RxHeader, RxHeaderData);
    }
    if (hcan == &hcan2)
    {
        //		pm01.PM01Parse(RxHeader,RxHeaderData);
        // MeterPower.Parse(RxHeader, RxHeaderData);
        // BSP::Power::pm01.PM01Parse(RxHeader, RxHeaderData);
        BSP::Power::pm01.PM01Parse(RxHeader,RxHeaderData);

        BSP::SuperCap::cap.Parse(RxHeader, RxHeaderData);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    BSP::Remote::dr16.Parse(huart, Size);
}

// UART中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    Gimbal_to_Chassis_Data.Data_receive(huart);
    RM_RefereeSystem::RM_RefereeSystemParse(huart);
}
