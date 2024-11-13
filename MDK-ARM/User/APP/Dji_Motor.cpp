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
    if(!(FDorCAN_ID(RxHeader) >= 0x200 && FDorCAN_ID(RxHeader) <= 0x208) || this->MotorSize == 0)
		return;	

	int idx = GET_Motor_ID_ADDRESS_BIND_(FDorCAN_ID(RxHeader));

	if(idx == -1)
		return;//如果超越数组大小，或者不存在id
		
	this->motorData[idx].DirFlag = this->motorData[idx].dirTime.ISDir(10);

	// 数据解析
	this->motorData[idx].Data[Angle] = (float)((int16_t)(RxHeaderData[0] << 8 | RxHeaderData[1]));

	// 转子速度
	this->motorData[idx].Data[Speed] = (float)((int16_t)(RxHeaderData[2] << 8 | RxHeaderData[3]));
	this->motorData[idx].Data[Torque] = (float)((int16_t)(RxHeaderData[4] << 8 | RxHeaderData[5]));

	// 温度
	this->motorData[idx].Data[Temperature] = (float)((int16_t)(RxHeaderData[6]));

    //数据累加
	if(this->motorData[idx].LastData[Angle] != this->motorData[idx].Data[Angle] && this->motorData[idx].LastData[Angle] != -1)
	{
		int lastData = this->motorData[idx].LastData[Angle];
		int Data = this->motorData[idx].Data[Angle];
		if(Data - lastData < -4000)//正转
			this->motorData[idx].AddData += (8191 - lastData + Data);
		else if(Data - lastData > 4000)//反转
			this->motorData[idx].AddData += -(8191 - Data + lastData);
		else 
			this->motorData[idx].AddData += (Data - lastData);
	}

	//数据上一次更新
	//数据解析
	this->motorData[idx].LastData[Angle] = this->motorData[idx].Data[Angle];
	//转子速度
	this->motorData[idx].LastData[Speed] = this->motorData[idx].Data[Speed];
	this->motorData[idx].LastData[Torque] = this->motorData[idx].Data[Torque];
	//初始化数据
	if(this->motorData[idx].InitFlag == 0)
	{
		this->motorData[idx].InitData = this->motorData[idx].Data[Angle];
		this->motorData[idx].InitFlag = 1;
	}
    //更新时间
    this->motorData[idx].dirTime.UpLastTime();
}

//过零处理
float Dji_Motor::Zero_crossing_processing(float expectations, float feedback, float maxpos )
{
	double tempcin = expectations;
	if(maxpos != 0)
	{
		tempcin = fmod(expectations,maxpos);
		double x1 = feedback;
		if(tempcin < 0)
			x1 -= maxpos;
		//过0处理
		if(tempcin - feedback < -maxpos/2)
			tempcin += maxpos;			
		if(tempcin - feedback > maxpos/2)
			tempcin -= maxpos;
	}
	return tempcin;
}

//设置发送数据
void setMSD(Motor_send_data_t* msd,int16_t data,int id)
{
	msd->Data[(id - 1) * 2] = data >> 8;
  msd->Data[(id - 1) * 2 + 1] = data << 8 >> 8;	
}

