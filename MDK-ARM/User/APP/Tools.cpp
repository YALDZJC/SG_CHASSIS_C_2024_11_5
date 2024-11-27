#include "Tools.hpp"
#include "usart.h"

/**
 * @brief 用于vofa发送波形数据
 * 
 * @param x1 
 * @param x2 
 * @param x3 
 * @param x4 
 * @param x5 
 * @param x6 
 */
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

// 过零处理
/**
 * @brief 用于处理6020的零点
 *
 * @param expectations
 * @param feedback
 * @param maxpos
 * @return float
 */
float Tools_t::Zero_crossing_processing(float expectations, float feedback, float maxpos)
{
    double tempcin = expectations;
    if (maxpos != 0)
    {
        tempcin = fmod(expectations, maxpos);
        double x1 = feedback;
        if (tempcin < 0)
            x1 -= maxpos;
        // 过0处理
        if (tempcin - feedback < -maxpos / 2)
            tempcin += maxpos;
        if (tempcin - feedback > maxpos / 2)
            tempcin -= maxpos;
    }
    return tempcin;
}

/**
 * @brief 对最小角进行判断
 *
 * @param expectations
 * @param feedback
 * @param speed
 * @param maxspeed
 * @param maxpos
 * @return double
 */
double Tools_t::MinPosHelm(float expectations, float feedback, float *speed, float maxspeed, float maxpos)
{
    // x1当前位置
    // x2当前位置的对半位置
    // x3反馈位置
    double x1 = 0, x2 = 0, x3 = 0;
    double tempcin = fmod(expectations, maxpos);
    x1 = tempcin;
    x2 = tempcin + 8191 / 2;
    x3 = feedback;
    if (tempcin < 0)
        x3 -= maxpos;
    // 过0处理
    if (x1 - x3 < -maxpos / 2)
        x1 += maxpos;

    if (x1 - x3 > maxpos / 2)
        x1 -= maxpos;

    if (x2 - x3 < -maxpos / 2)
        x2 += maxpos;

    if (x2 - x3 > maxpos / 2)
        x2 -= maxpos;

    // 两个最小角度
    int minangle1 = 0, minangle2 = 0;
    minangle1 = fabs(x1 - x3);
    minangle2 = fabs(x2 - x3);
    // 角度比较，看选择哪个角度
    if (minangle1 <= minangle2)
    {
        tempcin = x1;
        *speed = *speed;
    }
    else
    {
        tempcin = x2;
        *speed = -*speed;
    }
    return tempcin;
}
