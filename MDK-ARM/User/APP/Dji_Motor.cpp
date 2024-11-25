#include "Dji_Motor.hpp"

// 初始化电机信息
/**
 * @brief Dji电机进行ID设置
 *
 *
 * @param address
 * @param MotorSize
 * @param MotorAddress
 * @param idxs
 */
Dji_Motor::Dji_Motor(int16_t address, uint8_t MotorSize, Motor_t *MotorAddress, uint8_t *idxs)
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

/**
 * @brief 对Dji电机进行数据解析
 *
 * @param RxHeader
 * @param RxHeaderData
 */
void Dji_Motor::Parse(RM_FDorCAN_RxHeaderTypeDef RxHeader, uint8_t RxHeaderData[])
{
	if (!(FDorCAN_ID(RxHeader) >= this->init_address && FDorCAN_ID(RxHeader) <= this->init_address + 10) || this->MotorSize == 0)
		return;

	int idx = GET_Motor_ID_ADDRESS_BIND_(FDorCAN_ID(RxHeader));

	if (idx == -1)
		return; // 如果超越数组大小，或者不存在id

	this->motorData[idx].DirFlag = this->motorData[idx].dirTime.ISDir(10);

	// 数据解析
	this->motorData[idx].Data[Dji_Angle] = (float)((int16_t)(RxHeaderData[0] << 8 | RxHeaderData[1]));

	// 转子速度
	this->motorData[idx].Data[Dji_Speed] = (float)((int16_t)(RxHeaderData[2] << 8 | RxHeaderData[3]));
	this->motorData[idx].Data[Dji_Torque] = (float)((int16_t)(RxHeaderData[4] << 8 | RxHeaderData[5]));

	// 温度
	this->motorData[idx].Data[Dji_Temperature] = (float)((int16_t)(RxHeaderData[6]));

	// 数据累加
	if (this->motorData[idx].LastData[Dji_Angle] != this->motorData[idx].Data[Dji_Angle] && this->motorData[idx].LastData[Dji_Angle] != -1)
	{
		int lastData = this->motorData[idx].LastData[Dji_Angle];
		int Data = this->motorData[idx].Data[Dji_Angle];

		if (Data - lastData < -4000) // 正转
			this->motorData[idx].AddData += (8191 - lastData + Data);
		else if (Data - lastData > 4000) // 反转
			this->motorData[idx].AddData += -(8191 - Data + lastData);
		else
			this->motorData[idx].AddData += (Data - lastData);
	}

	// 数据上一次更新
	// 数据解析
	this->motorData[idx].LastData[Dji_Angle] = this->motorData[idx].Data[Dji_Angle];
	// 转子速度
	this->motorData[idx].LastData[Dji_Speed] = this->motorData[idx].Data[Dji_Speed];
	this->motorData[idx].LastData[Dji_Torque] = this->motorData[idx].Data[Dji_Torque];
	// 初始化数据
	if (this->motorData[idx].InitFlag == 0)
	{
		this->motorData[idx].InitData = this->motorData[idx].Data[Dji_Angle];
		this->motorData[idx].InitFlag = 1;
	}
	// 更新时间
	this->motorData[idx].dirTime.UpLastTime();
}



// 设置发送数据
void Dji_Motor::setMSD(Motor_send_data_t *msd, int16_t data, int id)
{
	msd->Data[(id - 1) * 2] = data >> 8;
	msd->Data[(id - 1) * 2 + 1] = data << 8 >> 8;
}

void Dji_Motor::Send_CAN(Motor_send_data_t *msd, uint16_t SendID)
{
	// 发送
	RM_FDorCAN_Send(&hcan1, SendID, msd->Data, CAN_TX_MAILBOX1);
}
