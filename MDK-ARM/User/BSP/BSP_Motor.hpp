#pragma once

#include "StaticTime.hpp"
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

//获取下标
#define GET_Motor_ID_IDX_BIND_(_motor_,id) (_motor_.id_idx_bind.idxs[id])
#define _Motor_ID_IDX_BIND_SIZE_ 5

//获取对应的下标
class RM_Motor;
int GET_Motor_ID_ADDRESS_BIND_(RM_Motor* _motor_,int address);

//电机发送数据
typedef struct 
{
	uint8_t Data[8];
}Motor_send_data_t;

//电机反馈数据枚举，分别是转角，速度，转矩，温度，外加一个停止模式
enum Motor_Data_Type
{
	Motor_Data_Angle = 0x00,
	Motor_Data_Speed = 0x01,
	Motor_Data_Torque = 0x02,
	Motor_Data_Temperature = 0x03,
	Motor_Data_Stop = 0x04,
};

//电机绑定
struct _Motor_ID_IDX_BIND_
{
	uint8_t idxs[_Motor_ID_IDX_BIND_SIZE_];
	_Motor_ID_IDX_BIND_(uint8_t* ids,uint8_t size)
	{
		for (uint8_t i = 0; i < _Motor_ID_IDX_BIND_SIZE_; i++)//标记
		{
			this->idxs[i] = 0xff;
		}
		for (uint8_t i = 0; i < size; i++)//绑定
		{
			this->idxs[ids[i]] = i;
		}
	}
};

typedef struct 
{
  int16_t address;//地址
	int16_t Data[4];//数据
	int16_t LastData[4];//历史数据
	int32_t AddData;//累加数据
	int16_t InitData;//初始化数据
	bool  InitFlag;//初始化标记
	bool  DirFlag;//死亡标记
	RM_StaticTime dirTime;//运行时间
}Motor_t;	//电机

class RM_Motor
{
public:
	int16_t init_address;//首地址
    Motor_t* motorData;
    uint8_t MotorSize;
	_Motor_ID_IDX_BIND_ id_idx_bind;
	//初始化
    RM_Motor(int16_t address, uint8_t MotorSize, Motor_t* MotorAddress,uint8_t* idxs);
	//数据解析
    virtual void Parse(CAN_RxHeaderTypeDef  RxHeader,uint8_t RxHeaderData[]) = 0;
	//断链
  	uint8_t ISDir();
	//获取速度
	inline int GetMotorDataSpeed(int16_t address)
    {
	    return this->motorData[GET_Motor_ID_ADDRESS_BIND_((this),address)].Data[Motor_Data_Speed];
    }
	//获取位置
	inline int GetMotorDataPos(int16_t address)
    {
        return this->motorData[GET_Motor_ID_ADDRESS_BIND_((this),address)].Data[Motor_Data_Angle];
    }
	//获取转矩
	inline int GetMotorDataTorque(int16_t address)
    {
        return this->motorData[GET_Motor_ID_ADDRESS_BIND_((this),address)].Data[Motor_Data_Torque];
    }
	//获取类型
	inline Motor_t *GetMotor(int16_t address)
    {
        return &this->motorData[GET_Motor_ID_ADDRESS_BIND_((this),address)];
    }
};




