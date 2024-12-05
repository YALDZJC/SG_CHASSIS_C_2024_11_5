#include "PowerTask.hpp"
#include "cmsis_os2.h"
#include "Variable.hpp"
using namespace SGPowerControl;

uint32_t pm01_ms = 0;
void PowerTask(void *argument)
{

    for (;;)
    {
        PowerControl.SGPowerFromCurr();
        osDelay(1);
    }
}

void RLSTask(void *argument)
{
    for (;;)
    {

        osDelay(1);
    }
}
void PowerTask_t::SGPowerFromCurr()
{
    for(int i = 0; i < 4; i++)
    {
        PowerData._3508Torque[i] = Motor3508.GetTorque_3508(Motor3508.GetEquipData(Get_InitID_3508(i), Dji_Torque));
    }

    for (int i = 0; i < 4; i++)
    {
        PowerData._3508Power[i] = Tools.GetMachinePower(PowerData._3508Torque[i], Motor3508.GetEquipData(Get_InitID_3508(i), Dji_Speed));
    }

    for (int i = 0; i < 4; i++)
    {
        PowerData._6020Power[i] = Tools.GetMachinePower(Motor6020.GetTorque_6020(Motor6020.GetEquipData(0x205, Dji_Torque)), Motor6020.GetEquipData(0x205, Dji_Speed));
    }

    PowerData._6020Power[1] = Tools.GetMachinePower(Motor6020.GetTorque_6020(Motor6020.GetEquipData(0x206, Dji_Torque)), Motor6020.GetEquipData(0x206, Dji_Speed));
    PowerData._6020Power[2] = Tools.GetMachinePower(Motor6020.GetTorque_6020(Motor6020.GetEquipData(0x207, Dji_Torque)), Motor6020.GetEquipData(0x207, Dji_Speed));
    PowerData._6020Power[3] = Tools.GetMachinePower(Motor6020.GetTorque_6020(Motor6020.GetEquipData(0x208, Dji_Torque)), Motor6020.GetEquipData(0x208, Dji_Speed));

    PowerData.Wheel_Power[0] = PowerData._3508Power[0] + PowerData._6020Power[0];
    PowerData.Wheel_Power[1] = PowerData._3508Power[1] + PowerData._6020Power[1];
    PowerData.Wheel_Power[2] = PowerData._3508Power[2] + PowerData._6020Power[2];
    PowerData.Wheel_Power[3] = PowerData._3508Power[3] + PowerData._6020Power[3];

    PowerData.ALL_Power = PowerData.Wheel_Power[0] + PowerData.Wheel_Power[1] + PowerData.Wheel_Power[2] + PowerData.Wheel_Power[3];

    PowerData.Energy += PowerData.ALL_Power * 0.001;
}
