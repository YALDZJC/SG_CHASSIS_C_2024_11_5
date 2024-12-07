#include "PowerTask.hpp"
#include "cmsis_os2.h"
#include "Variable.hpp"
using namespace SGPowerControl;

uint32_t pm01_ms = 0;
void PowerTask(void *argument)
{

    for (;;)
    {
        PowerControl.UpSteerData();
        PowerControl.UpWheelData();
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
void PowerTask_t::UpSteerData()
{
    for (int i = 0; i < 4; i++)
    {
        _6020_PowerData.Cur_Torque[i] = Motor6020.GetTorque_6020(Motor6020.GetEquipData(Get_InitID_6020(i), Dji_Torque));
    }

    for (int i = 0; i < 4; i++)
    {
        _6020_PowerData.Cur_Power[i] = Tools.GetMachinePower(_6020_PowerData.Cur_Torque[i], Motor6020.GetEquipData(Get_InitID_6020(i), Dji_Speed));
    }

    for (int i = 0; i < 4; i++)
    {
        _6020_PowerData.Cmd_Torque[i] = Motor6020.GetTorque_6020(Chassis_Data.final_6020_Out[i]);
    }

    for (int i = 0; i < 4; i++)
    {
        _6020_PowerData.Cmd_Power[i] = Tools.GetMachinePower(_6020_PowerData.Cmd_Torque[i], Motor6020.GetEquipData(Get_InitID_6020(i), Dji_Speed));
    }


    _6020_PowerData.Cmd_ALL_Power = _6020_PowerData.Cmd_Power[0] + _6020_PowerData.Cmd_Power[1] + _6020_PowerData.Cmd_Power[2] + _6020_PowerData.Cmd_Power[3];

    _6020_PowerData.Cur_ALL_Power = _6020_PowerData.Cur_Power[0] + _6020_PowerData.Cur_Power[1] + _6020_PowerData.Cur_Power[2] + _6020_PowerData.Cur_Power[3];
    

    _6020_PowerData.Energy += _6020_PowerData.Cur_ALL_Power * 0.001;
}

void PowerTask_t::UpWheelData()
{
    for (int i = 0; i < 4; i++)
    {
        _3508_PowerData.Cur_Torque[i] = Motor3508.GetTorque_3508(Motor3508.GetEquipData(Get_InitID_3508(i), Dji_Torque));
    }

    for (int i = 0; i < 4; i++)
    {
        _3508_PowerData.Cur_Power[i] = Tools.GetMachinePower(_3508_PowerData.Cur_Torque[i], Motor3508.GetEquipData(Get_InitID_3508(i), Dji_Speed));
    }

    for (int i = 0; i < 4; i++)
    {
        _3508_PowerData.Cmd_Torque[i] = Motor3508.GetTorque_3508(Chassis_Data.final_3508_Out[i]);
    }

    for (int i = 0; i < 4; i++)
    {
        _3508_PowerData.Cmd_Power[i] = Tools.GetMachinePower(_3508_PowerData.Cmd_Torque[i], Motor3508.GetEquipData(Get_InitID_3508(i), Dji_Speed));
    }

    _3508_PowerData.Cmd_ALL_Power = _3508_PowerData.Cmd_Power[0] + _3508_PowerData.Cmd_Power[1] + _3508_PowerData.Cmd_Power[2] + _3508_PowerData.Cmd_Power[3];

    _3508_PowerData.Cur_ALL_Power = _3508_PowerData.Cur_Power[0] + _3508_PowerData.Cur_Power[1] + _3508_PowerData.Cur_Power[2] + _3508_PowerData.Cur_Power[3];

    _3508_PowerData.Energy += _3508_PowerData.Cur_ALL_Power * 0.001;

    ALL_Power = _3508_PowerData.Cur_ALL_Power + _6020_PowerData.Cur_ALL_Power;
    Energy += ALL_Power * 0.001;
}
