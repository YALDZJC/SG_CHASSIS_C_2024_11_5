#pragma once

#include "StaticTime.hpp"
#include "stdxxx.hpp"

//获取下标
#define GET_Motor_ID_IDX_BIND_(_motor_,id) (_motor_.id_idx_bind.idxs[id])
#define _Motor_ID_IDX_BIND_SIZE_ 5

//电机反馈数据枚举，分别是转角，速度，转矩，温度，外加一个停止模式
enum Motor_Data_Type
{
	Angle = 0x00,
	Speed = 0x01,
	Torque = 0x02,
	Temperature = 0x03,
	Stop = 0x04,
};

//电机发送数据
typedef struct 
{
	uint8_t Data[8];
}Motor_send_data_t;

typedef struct 
{
    int16_t address;//地址
	float Data[4];//数据
	float LastData[4];//历史数据
	float AddData;//累加数据
	int16_t InitData;//初始化数据
	bool  InitFlag;//初始化标记
	bool  DirFlag;//死亡标记
	RM_StaticTime dirTime;//运行时间
}Motor_t;	//电机

class RM_Motor
{
protected:
	int16_t init_address;//首地址
    uint8_t MotorSize;	 //电机数量
    uint8_t idxs[_Motor_ID_IDX_BIND_SIZE_];	//电机最大个数

	void _Motor_ID_IDX_BIND_(uint8_t* ids,uint8_t size);	//电机绑定
  	uint8_t ISDir();	//断连
	uint8_t Motor_Data;

public:
	//电机数据
    Motor_t* motorData;	
	//获取对应下标
	int GET_Motor_ID_ADDRESS_BIND_(int address);	
	//数据解析
    virtual void Parse(CAN_RxHeaderTypeDef  RxHeader,uint8_t RxHeaderData[]) = 0;
    // 通用获取电机数据的方法
    virtual float GetMotorData(int16_t address, Motor_Data_Type DataType) = 0;
	//获取类型
	virtual inline Motor_t *GetMotor(int16_t address) = 0;
};




