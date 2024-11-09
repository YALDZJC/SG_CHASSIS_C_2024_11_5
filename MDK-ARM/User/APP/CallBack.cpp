#include "CallBack.hpp"
#include "My_hal.hpp"
#include "dr16.hpp"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	dr16.Parse(huart, Size);
}
