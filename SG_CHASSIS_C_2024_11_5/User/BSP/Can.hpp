#pragma once
#include "My_hal.hpp"

class BSP_CAN
{
public:

    virtual void Filter_Init() = 0;
    virtual void Can_Init() = 0;

    virtual void Can_Send(CAN_HandleTypeDef* han,uint32_t StdId,uint8_t* s_data,uint32_t pTxMailbox) = 0;
    void BSP_CAN_Init()
    {
        Filter_Init();
        Can_Init();
    }
};

class F4_CAN : public BSP_CAN
{
public:
    void Filter_Init();
    void Can_Init();
    void Can_Send(CAN_HandleTypeDef* han,uint32_t StdId,uint8_t* s_data,uint32_t pTxMailbox);
};

inline void BSP_CAN_Init(BSP_CAN *can)
{
	can->BSP_CAN_Init();
}

