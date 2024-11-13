#pragma once

#include "StaticTime.hpp"
#include "stdxxx.hpp"

//获取下标
#define GET_Motor_ID_IDX_BIND_(_motor_,id) (_motor_.id_idx_bind.idxs[id])
#define _Motor_ID_IDX_BIND_SIZE_ 5

//为抽象类定义的枚举，让枚举作为传参，在子类中实现具体枚举
//这是C++中的重载特性
enum Equip_Data_Type
{

};

//电机发送数据
typedef struct 
{
	uint8_t Data[8];
}Motor_send_data_t;

class Motor_t
{
public:
  int16_t address;//地址
	float Data[4];//数据
	float LastData[4];//历史数据
	float AddData;//累加数据
	int16_t InitData;//初始化数据
	bool  InitFlag;//初始化标记
	bool  DirFlag;//死亡标记
	RM_StaticTime dirTime;//运行时间
};	//电机

class RM_Motor
{
protected:
	int16_t init_address;//首地址
    uint8_t MotorSize;	 //电机数量
    uint8_t idxs[_Motor_ID_IDX_BIND_SIZE_];	//电机最大个数

	uint8_t Motor_Data;
	void _Motor_ID_IDX_BIND_(uint8_t* ids,uint8_t size);	//电机绑定

public:
	//电机数据
    Motor_t* motorData;	
	Motor_send_data_t* Motor_send_data;

  	uint8_t ISDir();	//断连

	//获取对应下标
	int GET_Motor_ID_ADDRESS_BIND_(int address);	
	//获取数据
    float GetEquipData(int16_t address, Equip_Data_Type DataType)
	{
		return this->motorData[this->GET_Motor_ID_ADDRESS_BIND_(address)].Data[DataType];
	}
	//获取上一数据
    float GetEquipLastData(int16_t address, Equip_Data_Type DataType)
	{
        return this->motorData[this->GET_Motor_ID_ADDRESS_BIND_(address)].LastData[DataType];
	}
	
	//数据解析
    virtual void Parse(CAN_RxHeaderTypeDef  RxHeader,uint8_t RxHeaderData[]) = 0;
};




