#pragma once

#include "BSP_Motor.hpp"
#include "My_hal.hpp"
#include "stdxxx.hpp"

//发送id
#define SEND_MOTOR_ID_2006 (0x200)
#define SEND_MOTOR_ID_3508 (0x200)
#define SEND_MOTOR_ID_6020 (0x1FF)

//获取设置id
#define Get_MOTOR_SET_ID_2006(x) (x - 0x200)
#define Get_MOTOR_SET_ID_3508(x) (x - 0x200)
#define Get_MOTOR_SET_ID_6020(x) (x - 0x204)

//获取设置stdid
#define Get_MOTOR_SET_STDID_2006(x) (x + 0x200)
#define Get_MOTOR_SET_STDID_3508(x) (x + 0x200)
#define Get_MOTOR_SET_STDID_6020(x) (x + 0x204)

class Dji_Motor : protected RM_Motor 
{
public:
    Dji_Motor(int16_t address, uint8_t MotorSize, Motor_t* MotorAddress,uint8_t* idxs);
    //数据解析
    void Parse(CAN_RxHeaderTypeDef  RxHeader,uint8_t RxHeaderData[]);

    //获取数据
    float GetMotorData(int16_t address, Motor_Data_Type DataType)
    {
        return this->motorData[GET_Motor_ID_ADDRESS_BIND_(address)].Data[DataType];
    }

	//获取类型
	inline Motor_t *GetMotor(int16_t address)
    {
        return &this->motorData[GET_Motor_ID_ADDRESS_BIND_(address)];
    }
};




