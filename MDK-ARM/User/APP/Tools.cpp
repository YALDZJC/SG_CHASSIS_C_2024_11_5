#include "Tools.hpp"
#include "usart.h"

void Tools_t::vofaSend(float x1, float x2, float x3, float x4, float x5, float x6)
{
    const uint8_t sendSize = 4;

    *((float *)&send_str2[sendSize * 0]) = x1;
    *((float *)&send_str2[sendSize * 1]) = x2;
    *((float *)&send_str2[sendSize * 2]) = x3;
    *((float *)&send_str2[sendSize * 3]) = x4;
    *((float *)&send_str2[sendSize * 4]) = x5;
    *((float *)&send_str2[sendSize * 5]) = x6;

    *((uint32_t *)&send_str2[sizeof(float) * (7)]) = 0x7f800000;
    HAL_UART_Transmit_DMA(&huart1, send_str2, sizeof(float) * (7 + 1));
}

