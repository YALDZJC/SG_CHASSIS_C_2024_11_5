#include "CommunicationTask.hpp"
#include "cmsis_os2.h"
// #include "Variable.hpp"
// #include "State.hpp"
#include "tim.h"

#include "usart.h"

#define SIZE 8

void CommunicationTask(void *argument)
{
    for (;;)
    {
        // Gimbal_to_Chassis_Data.Data_receive(&huart1);

        osDelay(5);
    }
}
Communicat::Gimbal_to_Chassis Gimbal_to_Chassis_Data;

namespace Communicat
{
void Gimbal_to_Chassis::Init()
{
    HAL_UART_Receive_IT(&huart1, pData, sizeof(pData));
}

void Gimbal_to_Chassis::Data_receive(UART_HandleTypeDef *huart)
{
    const uint8_t EXPECTED_HEAD = 0xA5; // 根据发送端设置的头字节
    const uint8_t EXPECTED_LEN = 1 + sizeof(Direction) + sizeof(ChassisMode) + sizeof(UiList);


    // 校验长度和头字节
    if (pData[0] != EXPECTED_HEAD)
    {
        SlidingWindowRecovery();
        return;
    }

    auto ptr = pData + 1; // 跳过头字节

    std::memcpy(&direction, ptr, sizeof(direction));
    ptr += sizeof(direction);

    std::memcpy(&chassis_mode, ptr, sizeof(chassis_mode));
    ptr += sizeof(chassis_mode);

    std::memcpy(&ui_list, ptr, sizeof(ui_list));
    ptr += sizeof(ui_list);

    dirTime.UpLastTime();

    HAL_UART_Receive_IT(&huart1, pData, sizeof(pData));
}

void Gimbal_to_Chassis::SlidingWindowRecovery()
{
    const int window_size = sizeof(pData);
    int found_pos = -1;

    // 遍历整个缓冲区寻找有效头
    for (int i = 0; i < window_size; i++) // 修正循环条件
    {
        if (pData[i] == 0xA5)
        {
            found_pos = i;
            break;
        }
    }

    if (found_pos > 0)
    {
        // 使用memmove处理可能重叠的内存区域
        std::memmove(pData, &pData[found_pos], window_size - found_pos);

        // 可选：重新配置DMA接收剩余空间
        int remaining_space = window_size - found_pos;
        HAL_UART_Receive_DMA(&huart1, pData + remaining_space, found_pos);
    }
}
bool Gimbal_to_Chassis::ISDir()
{
    char Dir = 0;

    is_dir = dirTime.ISDir(50) | Dir;
    return is_dir;
}

}; // namespace Communicat