#include "Dji_Motor.hpp"

//初始化电机信息
Dji_Motor::Dji_Motor(int16_t address, uint8_t MotorSize, Motor_t* MotorAddress,uint8_t* idxs)
{
	this->_Motor_ID_IDX_BIND_(idxs, MotorSize);

	this->motorData = MotorAddress;
	this->init_address = address;
	for (uint8_t i = 0; i < MotorSize; i++)
	{		
    this->motorData[i].LastData[0] = -1;
		this->motorData[i].address = address + idxs[i];		
	}
	
  	this->MotorSize = MotorSize;
}

//初始数据解算
void Dji_Motor::Parse(RM_FDorCAN_RxHeaderTypeDef  RxHeader, uint8_t RxHeaderData[])
{
    if(!(FDorCAN_ID(RxHeader) >= 0x200 && FDorCAN_ID(RxHeader) <= 0x208) || this->MotorSize == 0)return;	
	int idx = GET_Motor_ID_ADDRESS_BIND_(FDorCAN_ID(RxHeader));
	if(idx == -1)return;//如果超越数组大小，或者不存在id

	//数据解析
	this->motorData[idx].Data[Motor_Data_Angle] = (int16_t)(RxHeaderData[0]) << 8 | RxHeaderData[1];
	//转子速度
	this->motorData[idx].Data[Motor_Data_Speed] = (int16_t)(RxHeaderData[2]) << 8 | RxHeaderData[3];
	this->motorData[idx].Data[Motor_Data_Torque] = (int16_t)(RxHeaderData[4]) << 8 | RxHeaderData[5];
    //温度
    this->motorData[idx].Data[Motor_Data_Temperature] = (int16_t)(RxHeaderData[6]);
    //数据累加
	if(this->motorData[idx].LastData[Motor_Data_Angle] != this->motorData[idx].Data[Motor_Data_Angle] && this->motorData[idx].LastData[Motor_Data_Angle] != -1)
	{
		int lastData = this->motorData[idx].LastData[Motor_Data_Angle];
		int Data = this->motorData[idx].Data[Motor_Data_Angle];
		if(Data - lastData < -4000)//正转
			this->motorData[idx].AddData += (8191 - lastData + Data);
		else if(Data - lastData > 4000)//反转
			this->motorData[idx].AddData += -(8191 - Data + lastData);
		else 
			this->motorData[idx].AddData += (Data - lastData);
	}

	//数据上一次更新
	//数据解析
	this->motorData[idx].LastData[Motor_Data_Angle] = this->motorData[idx].Data[Motor_Data_Angle];
	//转子速度
	this->motorData[idx].LastData[Motor_Data_Speed] = this->motorData[idx].Data[Motor_Data_Speed];
	this->motorData[idx].LastData[Motor_Data_Torque] = this->motorData[idx].Data[Motor_Data_Torque];
	//初始化数据
	if(this->motorData[idx].InitFlag == 0)
	{
		this->motorData[idx].InitData = this->motorData[idx].Data[Motor_Data_Angle];
		this->motorData[idx].InitFlag = 1;
	}
    //更新时间
    this->motorData[idx].dirTime.UpLastTime();
}


