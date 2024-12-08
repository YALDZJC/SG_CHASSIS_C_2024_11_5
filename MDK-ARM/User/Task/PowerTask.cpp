#include "PowerTask.hpp"
#include "cmsis_os2.h"
#include "Variable.hpp"
#include "Tools.hpp"

using namespace SGPowerControl;

uint32_t pm01_ms = 0;

Math::RLS<2> rls(1e-5f, 0.99999f);

void PowerTask(void *argument)
{

    for (;;)
    {
        PowerControl.UpSteerData();
        PowerControl.UpWheelData();
        osDelay(1);
    }
}

float W2, T2;
float EffectivePower_t;
void RLSTask(void *argument)
{
    static Matrixf<2, 1> samples;
    static Matrixf<2, 1> params;

    for (;;)
    {
        //        for(int i = 0; i < 4; i++)
        //        {
        //            PowerControl._6020_PowerData.EffectivePower += Motor6020.GetEquipData(Get_InitID_6020(i), Dji_Speed) * Motor6020.GetEquipData(Get_InitID_6020(i), Dji_Torque);
        //            samples[0][0] += fabsf(rpm2av(Motor6020.GetEquipData(Get_InitID_6020(i), Dji_Speed)));
        //            samples[1][0] += PowerControl._6020_PowerData.Cur_Torque[i] * PowerControl._6020_PowerData.Cur_Torque[i];
        //        }

        //        params = rls.update(samples, PowerControl._6020_PowerData.Cmd_ALL_Power);
        //        PowerControl._6020_PowerData.k1 = fmax(params[0][0], 1e-5f); // In case the k1 diverge to negative number
        //        PowerControl._6020_PowerData.k2 = fmax(params[1][0], 1e-5f); // In case the k2

        PowerControl._6020_PowerData.EstimatedPower = PowerControl._6020_PowerData.k1 * samples[0][0] + PowerControl._6020_PowerData.k2 * samples[1][0] + PowerControl._6020_PowerData.k3;

        PowerControl._3508_PowerData.EffectivePower = 0;
				EffectivePower_t = 0;
        samples[0][0] = 0;
        samples[1][0] = 0;
        for (int i = 0; i < 4; i++)
        {
            PowerControl._3508_PowerData.EffectivePower += (rpm2av(Motor3508.GetEquipData(Get_InitID_3508(i), Dji_Speed)) * PowerControl._3508_PowerData.Cur_Torque[i]);
						EffectivePower_t += (rpm2av(Motor3508.GetEquipData(Get_InitID_3508(i), Dji_Speed)) * PowerControl._3508_PowerData.Cmd_Torque[i]);
            samples[0][0] += fabsf(rpm2av(Motor3508.GetEquipData(Get_InitID_3508(i), Dji_Speed)));
            samples[1][0] += PowerControl._3508_PowerData.Cmd_Torque[i] * PowerControl._3508_PowerData.Cmd_Torque[i];
        }
        //        }
        
        W2 = samples[0][0];
        T2 = samples[1][0];

        params = rls.update(samples, PowerControl._3508_PowerData.Cur_ALL_Power - EffectivePower_t);
        PowerControl._3508_PowerData.k1 = fmax(params[0][0], 1e-5f); // In case the k1 diverge to negative number
        PowerControl._3508_PowerData.k2 = fmax(params[1][0], 1e-5f); // In case the k2

        PowerControl._3508_PowerData.EstimatedPower = PowerControl._3508_PowerData.k1 * samples[0][0] + PowerControl._3508_PowerData.k2 * samples[1][0] + PowerControl._3508_PowerData.EffectivePower + PowerControl._3508_PowerData.k3;

        osDelay(1);
    }
}
void PowerTask_t::UpSteerData()
{
    // 更新反馈力矩
    for (int i = 0; i < 4; i++)
    {
        _6020_PowerData.Cur_Torque[i] = Motor6020.GetTorque_6020(Motor6020.GetEquipData(Get_InitID_6020(i), Dji_Torque));
    }
    // 从反馈值算出功率
    for (int i = 0; i < 4; i++)
    {
        _6020_PowerData.Cur_Power[i] = Tools.GetMachinePower(_6020_PowerData.Cur_Torque[i], Motor6020.GetEquipData(Get_InitID_6020(i), Dji_Speed));
    }
    // 更新目标力矩
    for (int i = 0; i < 4; i++)
    {
        _6020_PowerData.Cmd_Torque[i] = Motor6020.GetTorque_6020(Chassis_Data.final_6020_Out[i]);
    }
    // 从目标值算出功率
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
