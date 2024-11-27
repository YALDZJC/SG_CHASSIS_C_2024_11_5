#pragma once

#include "BSP_Motor.hpp"
#include "My_hal.hpp"
#include "stdxxx.hpp"



// 电机反馈数据枚举，分别是转角，速度，转矩，温度，外加一个停止模式
enum Dji_Data
{
    Dji_Angle = 0x00,
    Dji_Speed = 0x01,
    Dji_Torque = 0x02,
    Dji_Temperature = 0x03,
    Dji_Stop = 0x04,
};

class Dji_Motor : public RM_Motor
{
public:
    Dji_Motor(int16_t address, uint8_t MotorSize, Motor_t *MotorAddress, uint8_t *idxs);
    // 数据解析
    void Parse(CAN_RxHeaderTypeDef RxHeader, uint8_t RxHeaderData[]);
    // 过零处理
    float Zero_crossing_processing(float expectations, float feedback, float maxpos);
    // 最小角判断
    double MinPosHelm(float expectations, float feedback, float *speed, float maxspeed, float maxpos);
    // 设置id
    void setMSD(Motor_send_data_t *msd, int16_t data, int id);
    // 发送id
    void Send_CAN_MAILBOX0(Motor_send_data_t *msd, uint16_t SendID);
    void Send_CAN_MAILBOX1(Motor_send_data_t *msd, uint16_t SendID);

    // 得到当前值
    inline float GetEquipData(int16_t address, Dji_Data DataType)
    {
        return this->motorData[this->GET_Motor_ID_ADDRESS_BIND_(address)].Data[DataType];
    }

    // 得到上一次值
    inline float GetEquipLastData(int16_t address, Dji_Data DataType)
    {
        return this->motorData[this->GET_Motor_ID_ADDRESS_BIND_(address)].LastData[DataType];
    }
};
