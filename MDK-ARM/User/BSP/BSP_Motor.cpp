#include "BSP_Motor.hpp"

void RM_Motor::_Motor_ID_IDX_BIND_(uint8_t* ids,uint8_t size)
{
	uint8_t idxs[_Motor_ID_IDX_BIND_SIZE_];

	for (uint8_t i = 0; i < _Motor_ID_IDX_BIND_SIZE_; i++)//标记
	{
		this->idxs[i] = 0xff;
	}
	for (uint8_t i = 0; i < size; i++)//绑定
	{
		this->idxs[ids[i]] = i;
	}
}

//获取对应的下标
int RM_Motor::GET_Motor_ID_ADDRESS_BIND_(int address)
{
	int idx = address - (this->init_address);
	if(idx < 0)return -1;
	if(idx >= _Motor_ID_IDX_BIND_SIZE_)return -1;
	if(this->idxs[idx] == 0xff)return -1;

	return this->idxs[idx];
}

uint8_t RM_Motor::ISDir()
{
	bool is_dir = 0;
  for (int i = 0; i < this->MotorSize; i++)
  {
    is_dir |= this->motorData[GET_Motor_ID_ADDRESS_BIND_(this->motorData[i].address)].DirFlag = 
			this->motorData[GET_Motor_ID_ADDRESS_BIND_(this->motorData[i].address)].dirTime.ISDir(10);
  }
  return is_dir;
}
