#include "BSP_Motor.hpp"

//获取对应的下标
int GET_Motor_ID_ADDRESS_BIND_(RM_Motor* _motor_,int address)
{
	int idx = address - (_motor_->init_address);
	if(idx < 0)return -1;
	if(idx >= _Motor_ID_IDX_BIND_SIZE_)return -1;
	if(_motor_->id_idx_bind.idxs[idx] == 0xff)return -1;
	return _motor_->id_idx_bind.idxs[idx];
}

RM_Motor::RM_Motor(int16_t address, uint8_t MotorSize, Motor_t* MotorAddress,uint8_t* idxs) 
        :id_idx_bind(_Motor_ID_IDX_BIND_(idxs,MotorSize))
{
	this->motorData = MotorAddress;
	this->init_address = address;
	for (uint8_t i = 0; i < MotorSize; i++)
    {		
        this->motorData[i].LastData[0] = -1;
		this->motorData[i].address = address + idxs[i];		
    }
    this->MotorSize = MotorSize;
}

