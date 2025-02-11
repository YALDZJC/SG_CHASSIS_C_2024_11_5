#include "CommunicationTask.hpp"
#include "cmsis_os2.h"
// #include "Variable.hpp"
// #include "State.hpp"
#include "tim.h"

#define SIZE 8
Gimbal_to_Chassis_Data_t Gimbal_to_Chassis_Data;

uint8_t format[8] = {0XAA};

void CommunicationTask(void *argument)
{
	for (;;)
	{



		osDelay(1);
	}
}

namespace Communicat
{
    void Gimbal_to_Chassis::Data_receive(uint8_t *pData)
    {
        const uint8_t EXPECTED_HEAD = 0xA5; // 根据发送端设置的头字节
        const uint8_t EXPECTED_LEN  = 1 + sizeof(Direction) + sizeof(ChassisMode) + sizeof(UiList);

        // 校验长度和头字节
        if (pData[0] != EXPECTED_HEAD) {
            return; // 数据错误，丢弃
        }

        uint8_t *ptr = pData + 1; // 跳过头字节

        std::memcpy(&direction, ptr, sizeof(direction));
		ptr += sizeof(direction);

        std::memcpy(&chassis_mode, ptr, sizeof(chassis_mode));
		ptr += sizeof(chassis_mode);

        std::memcpy(&ui_list, ptr, sizeof(ui_list));
		ptr += sizeof(ui_list);


    }
};