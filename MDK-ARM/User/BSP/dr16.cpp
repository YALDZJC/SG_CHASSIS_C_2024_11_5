#include "dr16.hpp"
#include "My_hal.hpp"

RC_Ctl_t RM_Clicker::RC_Ctl = { 0 };//初始化
RM_StaticTime RM_Clicker::dirTime = { 0 };//初始化
uint8_t RM_Clicker::pData[18] = { 0 };//初始化

void RM_Clicker::ClearORE(UART_HandleTypeDef* huart,uint8_t* pData,int Size)
{
	if( __HAL_UART_GET_FLAG( huart, UART_FLAG_ORE ) != RESET )
	{
		__HAL_UART_CLEAR_OREFLAG( huart );
		HAL_UARTEx_ReceiveToIdle_DMA( huart, pData, Size );
	}
}

void RM_Clicker::Init()
{
	HAL_UARTEx_ReceiveToIdle_DMA(&ClickerHuart,RM_Clicker::pData,sizeof(RM_Clicker::pData));
}

RC_Ctl_t RM_Clicker::ParseData()
{
	RC_Ctl_t RC_CtrlData = { 0 };    
	RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
	RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;
	RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
	RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
	RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);
	RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
	RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8); 
	RC_CtrlData.mouse.press_l = pData[12];
	RC_CtrlData.mouse.press_r = pData[13];
	RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);
	
	return RC_CtrlData;
}

RM_Clicker::RM_Clicker()
{
	RM_Clicker::Init();
}

bool RM_Clicker::ISDir()
{
  bool Dir = false;
	Dir |= (RC_Ctl.rc.ch0 >= 364) && (RC_Ctl.rc.ch0 <= 1684);
	Dir |= (RC_Ctl.rc.ch1 >= 364) && (RC_Ctl.rc.ch1 <= 1684);
	Dir |= (RC_Ctl.rc.ch2 >= 364) && (RC_Ctl.rc.ch2 <= 1684);
	Dir |= (RC_Ctl.rc.ch3 >= 364) && (RC_Ctl.rc.ch3 <= 1684);
	RC_Ctl.Dir = RM_Clicker::dirTime.ISDir(50) || Dir;
	if(RC_Ctl.Dir)
		RM_Clicker::ClearORE(&ClickerHuart,RM_Clicker::pData,sizeof(RM_Clicker::pData));
	
  return RC_Ctl.Dir;
}

void RM_Clicker::Parse(UART_HandleTypeDef *huart, int Size)
{
  //本机遥控器
	if(huart == &ClickerHuart && Size == sizeof(RM_Clicker::pData))
	{
		RM_Clicker::RC_Ctl = RM_Clicker::ParseData();
		RM_Clicker::dirTime.UpLastTime();
	}
	HAL_UARTEx_ReceiveToIdle_DMA(&ClickerHuart,RM_Clicker::pData,sizeof(RM_Clicker::pData));
}


