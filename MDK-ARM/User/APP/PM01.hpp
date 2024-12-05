#pragma once

#include "BSP_Motor.hpp"
#include "My_hal.hpp"
#include "stdxxx.hpp"

// 电机反馈数据枚举，分别是转角，速度，转矩，温度，外加一个停止模式
enum PM01_Data
{
    Power = 0x00,
    Voltage = 0x01,
    Ampere = 0x02,
};

struct PM10_State_t
{
    /* data */
    uint16_t Mod_state;
    uint16_t Err_state;
};

struct PM01_Data_t
{
    /* data */
    float Out_Data[3];
    float In_Data[3];
};

struct PM01_Time_t
{
    /* data */
    float Add_Time;
    float Cur_Time;
};


class PM01
{
private:
    CAN_TxHeaderTypeDef TxHeader;
    RM_StaticTime time;

public:
    PM10_State_t PM10_State;
    PM01_Time_t PM01_Time;
    PM01_Data_t PM01_Data;

    bool Dir;
    float Temperature;
    // 解析
    void PM01Parse(CAN_RxHeaderTypeDef RxHeader, uint8_t RxHeaderData[]);

    // 发送远程帧查看
    void PM01SendRemote(uint32_t StdId);

    void PM01SendData(uint32_t StdId, uint16_t Data);
};
