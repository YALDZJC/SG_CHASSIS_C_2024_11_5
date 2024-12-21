#pragma once

#include "My_hal.hpp"
#include "StaticTime.hpp"
#define KEY_PRESSED_OFFSET_W (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 0 & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_S (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 1 & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_A (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 2 & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_D (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 3 & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_SHIFT (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 4 & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_CTRL (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 5 & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_Q (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 6 & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_E (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 7 & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_R (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 8 & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_F (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 9 & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_G (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 10 & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_Z (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 11 & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_X (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 12 & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_C (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 13 & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_V (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 14 & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_B (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 15 & (uint8_t)0x01)
#define MOUSE_RDOWN (uint8_t)(RM_Clicker::RC_Ctl.mouse.press_r)
#define MOUSE_LDOWN (uint8_t)(RM_Clicker::RC_Ctl.mouse.press_l)
#define MOUSE_X										 (float)((RM_Clicker::RC_Ctl.mouse.x)
#define MOUSE_Y										 (float)((RM_Clicker::RC_Ctl.mouse.y)
#define RC_LX (int16_t)(RM_Clicker::RC_Ctl.rc.ch2 - 1024)
#define RC_LY (int16_t)(RM_Clicker::RC_Ctl.rc.ch3 - 1024)
#define RC_RX (int16_t)(RM_Clicker::RC_Ctl.rc.ch0 - 1024)
#define RC_RY (int16_t)(RM_Clicker::RC_Ctl.rc.ch1 - 1024)
#define RC_SW (int16_t)(RM_Clicker::RC_Ctl.rc.sw - 1024)

#define ClickerHuart huart3
// 结构体
typedef struct
{
	struct
	{
		uint16_t ch0;
		uint16_t ch1;
		uint16_t ch2;
		uint16_t ch3;
		uint8_t s1;
		uint8_t s2;
		uint16_t sw;
	} rc;
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	} mouse;
	struct
	{
		uint16_t v;
	} key;
	char Dir;
} RC_Ctl_t;

class RM_Clicker
{

private:
	// 遥控器数据解析
	static RC_Ctl_t ParseData();

public:
	RM_Clicker();
	static RC_Ctl_t RC_Ctl;
	static RM_StaticTime dirTime;
	static uint8_t pData[18];
	// 解除ore
	static void ClearORE(UART_HandleTypeDef *huart, uint8_t *pData, int Size);
	// 遥控器初始化
	static void Init();
	// 判断遥控器断连
	static bool ISDir();
	// 遥控器数据解析
	static void Parse(UART_HandleTypeDef *huart, int Size);
};
