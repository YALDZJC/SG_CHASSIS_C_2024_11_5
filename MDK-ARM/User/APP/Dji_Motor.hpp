#pragma once

#include "BSP_Motor.hpp"
#include "My_hal.hpp"
#include "stdxxx.hpp"

class Dji_Motor : public RM_Motor 
{
public:
    Dji_Motor(int16_t address, uint8_t MotorSize, Motor_t* MotorAddress,uint8_t* idxs);
    //数据解析
    void Parse(CAN_RxHeaderTypeDef  RxHeader,uint8_t RxHeaderData[]);
    float Zero_crossing_processing(float expectations, float feedback, float maxpos );
};

void setMSD(Motor_send_data_t* msd,int16_t data,int id);
