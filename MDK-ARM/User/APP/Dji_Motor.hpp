#pragma once

#include "BSP_Motor.hpp"
#include "My_hal.hpp"
#include "stdxxx.hpp"

//电机反馈数据枚举，分别是转角，速度，转矩，温度，外加一个停止模式
enum Jji_Data
{
	Angle = 0x00,
	Speed = 0x01,
	Torque = 0x02,
	Temperature = 0x03,
	Stop = 0x04,
};

class Dji_Motor : public RM_Motor 
{
public:
    Dji_Motor(int16_t address, uint8_t MotorSize, Motor_t* MotorAddress,uint8_t* idxs);
    //数据解析
    void Parse(CAN_RxHeaderTypeDef RxHeader,uint8_t RxHeaderData[]);
    float Zero_crossing_processing(float expectations, float feedback, float maxpos );

    float GetEquipData(int16_t address, Jji_Data DataType) 
		{
        return this->motorData[this->GET_Motor_ID_ADDRESS_BIND_(address)].Data[DataType];
    }

    float GetEquipLastData(int16_t address, Jji_Data DataType) 
		{
        return this->motorData[this->GET_Motor_ID_ADDRESS_BIND_(address)].LastData[DataType];
    }
};

void setMSD(Motor_send_data_t* msd,int16_t data,int id);
